import threading
import time
from typing import Optional

import cv2
import rclpy
import tf2_ros
from apriltag_interfaces.action import StartTracking
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image

from apriltag.control import LQRTracker, TrajectoryPlanner
from apriltag.perception.tag_perception import (
    build_detector,
    draw_detections_and_collect_targets,
)
from apriltag.ros.ros_io import make_camera_intrinsics, msg_to_cv2, publish_image
from apriltag.runtime.target_flow import choose_best_target, process_targets_pipeline

class AprilTagRosNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        # Low-latency image QoS: best effort + keep last 1.
        self.image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # initialize AprilTag detector
        self.detector = build_detector(tag_family="tag36h11")

        # subscribe to image topic
        self.sub = self.create_subscription(Image,
                                            '/camera/camera/color/image_raw',
                                            self.image_callback,
                                            self.image_qos)

        # subscribe to camera_info to get intrinsics
        self.info_sub = self.create_subscription(CameraInfo,
                                                 '/camera/camera/color/camera_info',
                                                 self.info_callback,
                                                 10)

        # camera intrinsics (filled by camera_info)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.cam_width = None
        self.cam_height = None
        self.camera_frame = 'camera_link'

        # publisher for PoseStamped and TF broadcaster
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag_pose', 10)
        self.traj_pub = self.create_publisher(Path, '/apriltag_trajectory', 10)
        self.image_pub = self.create_publisher(Image, '/apriltag/marked_image', self.image_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        ## AprilTag physical size and distance parameters
        # meters, adjust to your tag's real size
        self.tag_size = 0.0635
        # two-stage desired forward distances to tag
        self.stage1_distance = 0.50   # 第一階段：50 cm 對齊
        self.stage2_distance = 0.28   # 第二階段：28.5 cm 對齊
        self.desired_distance = self.stage1_distance
        # camera lateral offset in AMR control frame (+left / -right)
        # self.camera_y_offset = 0.036
        self.y_offset = -0.01
        self.camera_y_offset = 0.036 + self.y_offset

        ## control-related state and parameters
        # trajectory planner and LQR tracker
        # smooth_tau=0.5: slower reference prevents y overshoot/oscillation
        self.trajectory_planner = TrajectoryPlanner(smooth_tau=0.5)
        # r_weights: R_vx=3.0 > R_vy=1.0 → K_vy > K_vx (lateral faster than forward)
        self.lqr_tracker = LQRTracker(r_weights=(3.0, 1.0, 3.0))
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._last_time = None
        self.max_vx = 0.5
        self.max_vy = 0.5
        self.max_vw = 0.5
        self.max_dt = 0.2
        self.latest_plan = None
        self.lost_target_timeout = 1.0
        self._last_target_time = None
        self._stopped_on_target_loss = False
        self._control_lock = threading.Lock()
        self._control_enabled = False
        self._has_sent_stop_while_paused = False
        self._error_zero_since = None
        self._alignment_stage = 1
        self.stop_hold_seconds = 1.0
        self.stop_x_error_tolerance = 0.05
        self.stop_y_error_tolerance = 0.05
        self.stop_yaw_error_tolerance = 0.05
        self._tracking_done = threading.Event()
        self._tracking_result: dict = {}
        self._latest_best_target: dict = {}
        self._tracking_start_time: Optional[float] = None
        self._action_cb_group = ReentrantCallbackGroup()
        self.start_tracking_action_server = ActionServer(
            self,
            StartTracking,
            'start_tracking',
            execute_callback=self._execute_start_tracking,
            goal_callback=self._start_tracking_goal_callback,
            cancel_callback=self._start_tracking_cancel_callback,
            callback_group=self._action_cb_group,
        )
        self.get_logger().info(
            'Control is paused. Send the start_tracking action to start.'
        )

    def _publish_stop_command(self) -> None:
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

    def _start_control(self, reason: str) -> None:
        with self._control_lock:
            self._control_enabled = True
            self._has_sent_stop_while_paused = False
            self._error_zero_since = None
            self._alignment_stage = 1
            self.desired_distance = self.stage1_distance
            self._last_time = time.monotonic()
            self._tracking_start_time = self._last_time
            self._last_target_time = None
            self._stopped_on_target_loss = False
            self._latest_best_target = {}
            self.trajectory_planner.reset()
            self.lqr_tracker.reset()
        self.get_logger().info(reason)

    def _start_tracking_goal_callback(self, goal_request: StartTracking.Goal):
        return GoalResponse.ACCEPT

    def _start_tracking_cancel_callback(self, goal_handle):
        _ = goal_handle
        return CancelResponse.ACCEPT

    def _signal_tracking_complete(self, success: bool, message: str) -> None:
        elapsed = (
            time.monotonic() - self._tracking_start_time
            if self._tracking_start_time is not None
            else float('nan')
        )
        timed_message = f'{message} elapsed={elapsed:.2f}s'
        self.get_logger().info(f'[Tracking] elapsed={elapsed:.2f}s')
        print(f'[Tracking] elapsed={elapsed:.2f}s')
        with self._control_lock:
            self._tracking_result = {'success': success, 'message': timed_message}
        self._tracking_done.set()

    def _execute_start_tracking(self, goal_handle):
        result = StartTracking.Result()

        if not goal_handle.request.start:
            self._pause_control('Tracking stopped from action server.')
            # 喚醒卡在 while loop 的 start=True thread，避免 thread pool 耗盡
            if not self._tracking_done.is_set():
                with self._control_lock:
                    self._tracking_result = {'success': False, 'message': 'Tracking stopped.'}
                self._tracking_done.set()
            result.success = True
            result.message = 'Tracking stopped.'
            goal_handle.succeed()
            return result

        # start=True: block until aligned or failed
        self._tracking_done.clear()
        self._tracking_result = {}
        self._start_control('Control started from action server. Tracking target...')

        feedback = StartTracking.Feedback()
        while not self._tracking_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                self._pause_control('Tracking cancelled by client.')
                result.success = False
                result.message = 'Tracking cancelled.'
                goal_handle.canceled()
                return result

            t = self._latest_best_target
            feedback.status = (
                f"tracking: x_err={t.get('x_error', float('nan')):.3f} "
                f"y_err={t.get('y_error', float('nan')):.3f} "
                f"yaw_err={t.get('yaw_error', float('nan')):.3f}"
            )
            goal_handle.publish_feedback(feedback)

        with self._control_lock:
            res = self._tracking_result.copy()

        result.success = res.get('success', False)
        result.message = res.get('message', '')

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _is_control_enabled(self) -> bool:
        with self._control_lock:
            return self._control_enabled

    def _pause_control(self, reason: str):
        with self._control_lock:
            self._control_enabled = False
            self._has_sent_stop_while_paused = False
            self._error_zero_since = None
            self._last_target_time = None
            self._stopped_on_target_loss = False
        self.trajectory_planner.reset()
        self.lqr_tracker.reset()
        self._publish_stop_command()
        self.get_logger().info(
            f'{reason} Send the start_tracking action to start again.'
        )

    def _error_is_zero(self, target) -> bool:
        if target is None:
            return False
        return (
            abs(float(target.get('x_error', 0.0))) <= self.stop_x_error_tolerance
            and abs(float(target.get('y_error', 0.0))) <= self.stop_y_error_tolerance
            and abs(float(target.get('yaw_error', 0.0))) <= self.stop_yaw_error_tolerance
        )

    def _publish_zero_once_if_paused(self):
        with self._control_lock:
            should_publish = not self._has_sent_stop_while_paused
            if should_publish:
                self._has_sent_stop_while_paused = True

        if should_publish:
            self._publish_stop_command()

    # callback to receive camera intrinsics
    def info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.cam_width = msg.width
        self.cam_height = msg.height
        self.camera_frame = msg.header.frame_id if msg.header and msg.header.frame_id else self.camera_frame
        self.get_logger().info(f'Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
        # Unsubscribe after getting intrinsics
        if self.info_sub is not None:
            self.destroy_subscription(self.info_sub)
            self.info_sub = None

    # callback to process incoming images and detect AprilTags
    def image_callback(self, msg: Image):
        if not self._is_control_enabled():
            self._publish_zero_once_if_paused()
            return

        frame = msg_to_cv2(msg)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        intrinsics = make_camera_intrinsics(
            self.fx,
            self.fy,
            self.cx,
            self.cy,
            self.cam_width,
            self.cam_height,
        )

        has_camera_intrinsics = intrinsics.fx > 0.0 and intrinsics.fy > 0.0
        if has_camera_intrinsics:
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy),
                tag_size=self.tag_size,
            )
        else:
            detections = self.detector.detect(gray)

        vis, targets = draw_detections_and_collect_targets(
            frame.copy(),
            detections,
            intrinsics,
            self.detector,
            self.tag_size,
            self.desired_distance,
            self.camera_y_offset,
            self.get_logger(),
        )

        self._latest_best_target = choose_best_target(targets) or {}

        now = time.monotonic()
        was_stopped_on_target_loss = self._stopped_on_target_loss
        self._last_target_time, self._stopped_on_target_loss, self._last_time, latest_plan = process_targets_pipeline(
            targets=targets,
            now=now,
            last_target_time=self._last_target_time,
            stopped_on_target_loss=self._stopped_on_target_loss,
            last_time=self._last_time,
            max_dt=self.max_dt,
            trajectory_planner=self.trajectory_planner,
            lqr_tracker=self.lqr_tracker,
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_vw=self.max_vw,
            logger=self.get_logger(),
            cmd_pub=self.cmd_pub,
            clock=self.get_clock(),
            camera_frame=self.camera_frame,
            pose_pub=self.pose_pub,
            traj_pub=self.traj_pub,
            tf_broadcaster=self.tf_broadcaster,
            lost_target_timeout=self.lost_target_timeout,
        )

        if self._stopped_on_target_loss and not was_stopped_on_target_loss:
            lost_msg = f'AprilTag lost for {self.lost_target_timeout:.2f}s. Tracking stopped.'
            self._pause_control(lost_msg)
            self._signal_tracking_complete(False, lost_msg)
            publish_image(self.image_pub, vis, header=msg.header, resize_to=(640, 360))
            return

        if latest_plan is not None:
            self.latest_plan = latest_plan
            best_target = choose_best_target(targets)
            if self._error_is_zero(best_target):
                if self._error_zero_since is None:
                    self._error_zero_since = now

                hold_time = now - self._error_zero_since
                if hold_time < self.stop_hold_seconds:
                    publish_image(self.image_pub, vis, header=msg.header, resize_to=(640, 360))
                    return

                if self._alignment_stage == 1:
                    self.get_logger().info(
                        f'Stage 1 aligned at {self.stage1_distance:.2f}m. '
                        f'Advancing to Stage 2 ({self.stage2_distance:.2f}m).'
                    )
                    with self._control_lock:
                        self._alignment_stage = 2
                        self.desired_distance = self.stage2_distance
                        self._error_zero_since = None
                    self.trajectory_planner.reset()
                    self.lqr_tracker.reset()
                else:
                    x_error = float(best_target.get('x_error', 0.0))
                    y_error = float(best_target.get('y_error', 0.0))
                    yaw_error = float(best_target.get('yaw_error', 0.0))
                    stop_msg = Twist()
                    self.cmd_pub.publish(stop_msg)
                    aligned_msg = (
                        f'Stage 2 aligned at {self.stage2_distance:.2f}m. '
                        f'Error converged within tol: '
                        f'x_error={x_error:.6f} (<= {self.stop_x_error_tolerance:.3f}), '
                        f'y_error={y_error:.6f} (<= {self.stop_y_error_tolerance:.3f}), '
                        f'yaw_error={yaw_error:.6f} (<= {self.stop_yaw_error_tolerance:.3f}), '
                        f'held_for={hold_time:.2f}s (>= {self.stop_hold_seconds:.2f}s).'
                    )
                    self._pause_control(aligned_msg)
                    self._signal_tracking_complete(True, aligned_msg)
            else:
                self._error_zero_since = None

        publish_image(self.image_pub, vis, header=msg.header, resize_to=(640, 360))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagRosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
