import threading
import time
from typing import Optional

import numpy as np
import rclpy
from apriltag_interfaces.action import StartTracking
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool

from apriltag.control import LQRTracker, TrajectoryPlanner
from apriltag.domain.math_utils import optical_to_control_error, quaternion_to_rotation_matrix
from apriltag.ros.ros_io import publish_trajectory, publish_twist
from apriltag.runtime.control_flow import publish_control
from apriltag.runtime.safety_guard import handle_target_lost


class AprilTagControlNode(Node):
    """Orchestrate AprilTag tracking and drive the robot to the tag.

    Owns the start_tracking action. On a start goal it enables apriltag_detection,
    tracks /apriltag_pose with TrajectoryPlanner + LQR and publishes /cmd_vel_nav.
    Detection is disabled again whenever the goal succeeds, fails, is cancelled
    or is stopped.
    """

    def __init__(self):
        super().__init__('apriltag_control')

        self.pose_sub = self.create_subscription(PoseStamped,
                                                 'apriltag_pose',
                                                 self.pose_callback,
                                                 1)
        self.traj_pub = self.create_publisher(Path, 'apriltag_trajectory', 10)

        ## AprilTag distance parameters
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
        # capture time (s) of the last pose used for control, for dt
        self._last_stamp = None
        self.max_vx = 0.5
        self.max_vy = 0.5
        self.max_vw = 0.5
        self.max_dt = 0.2
        self.latest_plan = None
        self.lost_target_timeout = 1.0
        self.lost_check_period = 0.05
        self._last_target_time = None
        self._control_lock = threading.Lock()
        self._control_enabled = False
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

        # pose_callback and the lost-target timer share the default
        # (mutually exclusive) group, so they never run concurrently.
        self.lost_check_timer = self.create_timer(self.lost_check_period, self._check_target_lost)

        # detection on/off client; own group so its response can be handled
        # while the action execute thread is waiting on it
        self.detection_service_timeout = 2.0
        self._client_cb_group = MutuallyExclusiveCallbackGroup()
        self.detection_enable_client = self.create_client(
            SetBool,
            'apriltag_detection/enable',
            callback_group=self._client_cb_group,
        )

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

    def _safe_stop(self, reset_planner: bool = False) -> None:
        publish_twist(self.cmd_pub, 0.0, 0.0, 0.0)
        if reset_planner:
            self.trajectory_planner.reset()
            self.lqr_tracker.reset()

    def _set_detection_enabled(self, enabled: bool) -> bool:
        """Call apriltag_detection's enable service; return True on success."""
        action = 'enable' if enabled else 'disable'
        if not self.detection_enable_client.wait_for_service(timeout_sec=self.detection_service_timeout):
            self.get_logger().error(
                f'Cannot {action} detection: service {self.detection_enable_client.srv_name} not available.'
            )
            return False

        request = SetBool.Request()
        request.data = enabled
        future = self.detection_enable_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.detection_service_timeout):
            self.get_logger().error(f'Cannot {action} detection: service call timed out.')
            return False

        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(f'Cannot {action} detection: service call failed.')
            return False
        return True

    def _start_control(self, reason: str) -> None:
        with self._control_lock:
            self._control_enabled = True
            self._error_zero_since = None
            self._alignment_stage = 1
            self.desired_distance = self.stage1_distance
            self._last_stamp = None
            self._tracking_start_time = time.monotonic()
            self._last_target_time = None
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
        with self._control_lock:
            self._tracking_result = {'success': success, 'message': timed_message}
        self._tracking_done.set()

    def _execute_start_tracking(self, goal_handle):
        result = StartTracking.Result()

        if not goal_handle.request.start:
            self._pause_control('Tracking stopped from action server.')
            self._set_detection_enabled(False)
            # 喚醒卡在 while loop 的 start=True thread，避免 thread pool 耗盡
            if not self._tracking_done.is_set():
                with self._control_lock:
                    self._tracking_result = {'success': False, 'message': 'Tracking stopped.'}
                self._tracking_done.set()
            result.success = True
            result.message = 'Tracking stopped.'
            goal_handle.succeed()
            return result

        # start=True: enable detection, then block until aligned or failed
        self._tracking_done.clear()
        self._tracking_result = {}
        if not self._set_detection_enabled(True):
            self._set_detection_enabled(False)
            result.success = False
            result.message = 'Failed to enable apriltag_detection.'
            goal_handle.abort()
            return result
        self._start_control('Control started from action server. Tracking target...')

        feedback = StartTracking.Feedback()
        while not self._tracking_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                self._pause_control('Tracking cancelled by client.')
                self._set_detection_enabled(False)
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

        # goal finished (aligned, target lost or stopped): turn detection off
        self._set_detection_enabled(False)

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
            self._error_zero_since = None
            self._last_target_time = None
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

    # watchdog: detection publishes nothing when no tag is visible, so target
    # loss must be checked on a timer rather than in pose_callback
    def _check_target_lost(self):
        if not self._is_control_enabled():
            return

        self._last_target_time, lost = handle_target_lost(
            now=time.monotonic(),
            last_target_time=self._last_target_time,
            lost_target_timeout=self.lost_target_timeout,
            stopped_on_target_loss=False,
            safe_stop_fn=self._safe_stop,
            logger=self.get_logger(),
        )
        if lost:
            lost_msg = f'AprilTag lost for {self.lost_target_timeout:.2f}s. Tracking stopped.'
            self._pause_control(lost_msg)
            self._signal_tracking_complete(False, lost_msg)

    # callback to convert tag pose into control errors and publish cmd_vel
    def pose_callback(self, msg: PoseStamped):
        if not self._is_control_enabled():
            return

        now = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        p = msg.pose.position
        q = msg.pose.orientation
        t_vec = np.array([p.x, p.y, p.z], dtype=float)
        r_mat = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        state = optical_to_control_error(t_vec, self.desired_distance, r_mat, self.camera_y_offset)
        best_target = {
            'x_error': state.x_error,
            'y_error': state.y_error,
            'yaw_error': state.yaw_error,
        }
        self._latest_best_target = best_target
        self._last_target_time = now

        publish_trajectory(self.traj_pub, msg)

        try:
            self._last_stamp, latest_plan = publish_control(
                best_target=best_target,
                now=stamp,
                last_time=self._last_stamp,
                max_dt=self.max_dt,
                trajectory_planner=self.trajectory_planner,
                lqr_tracker=self.lqr_tracker,
                max_vx=self.max_vx,
                max_vy=self.max_vy,
                max_vw=self.max_vw,
                publish_twist_fn=lambda vx, vy, vw: publish_twist(self.cmd_pub, vx, vy, vw),
                logger=self.get_logger(),
            )
        except Exception as exc:
            self.get_logger().warn(f"Control publish failed: {exc}")
            self._safe_stop(reset_planner=True)
            return

        self.latest_plan = latest_plan
        if not self._error_is_zero(best_target):
            self._error_zero_since = None
            return

        if self._error_zero_since is None:
            self._error_zero_since = now

        hold_time = now - self._error_zero_since
        if hold_time < self.stop_hold_seconds:
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
            self._publish_stop_command()
            aligned_msg = (
                f'Stage 2 aligned at {self.stage2_distance:.2f}m. '
                f'Error converged within tol: '
                f'x_error={state.x_error:.6f} (<= {self.stop_x_error_tolerance:.3f}), '
                f'y_error={state.y_error:.6f} (<= {self.stop_y_error_tolerance:.3f}), '
                f'yaw_error={state.yaw_error:.6f} (<= {self.stop_yaw_error_tolerance:.3f}), '
                f'held_for={hold_time:.2f}s (>= {self.stop_hold_seconds:.2f}s).'
            )
            self._pause_control(aligned_msg)
            self._signal_tracking_complete(True, aligned_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagControlNode()
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
