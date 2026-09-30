import signal
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
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_srvs.srv import Empty, SetBool, Trigger

from apriltag.control import LQRTracker, TrajectoryPlanner
from apriltag.domain.math_utils import optical_to_control_error, quaternion_to_rotation_matrix
from apriltag.ros.ros_io import publish_trajectory, publish_twist
from apriltag.runtime.control_flow import publish_control
from apriltag.runtime.safety_guard import handle_target_lost

# high-rate messages (cmd_vel_nav, action feedback) are logged at most once per this period
LOG_THROTTLE_SEC = 1.0


def msg_to_str(msg) -> str:
    """Compact 'field=value, ...' text of a ROS message; '{}' if it has no fields."""
    fields = msg.get_fields_and_field_types()
    if not fields:
        return '{}'
    return ', '.join(f'{name}={getattr(msg, name)!r}' for name in fields)


class AprilTagControlNode(Node):
    """Orchestrate AprilTag tracking and drive the robot to the tag.

    Owns the start_tracking action. On a start goal it closes AMCL, enables
    apriltag_detection, tracks /apriltag_pose with TrajectoryPlanner + LQR and
    publishes /cmd_vel_nav. Before Stage 2 it stops and disables the PLC lidar
    safety field. Whenever the goal ends (succeeded, failed, cancelled or
    stopped) it stops the robot, disables detection and restores whatever it
    changed: lidar safety first, then AMCL. With manage_amcl_and_lidar_safety
    set to false it skips all AMCL and lidar safety calls.

    AMCL and lidar safety are external services. This node only calls them at
    the right time and checks their responses; what the providers actually do
    with a request is outside this package.
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
        self.max_vx = 0.05
        self.max_vy = 0.05
        self.max_vw = 0.05
        self.max_dt = 0.2
        self.latest_plan = None
        self.lost_target_timeout = 1.0
        self.lost_check_period = 0.05
        self._last_target_time = None
        self._control_lock = threading.Lock()
        self._control_enabled = False
        self._error_zero_since = None
        self._alignment_stage = 1
        # Stage 1 converged and the robot is stopped; the action execute thread
        # disables lidar safety before Stage 2 starts
        self._stage2_pending = False
        self.stop_hold_seconds = 1.0
        self.stop_x_error_tolerance = 0.05
        self.stop_y_error_tolerance = 0.05
        self.stop_yaw_error_tolerance = 0.05
        self._tracking_done = threading.Event()
        self._tracking_result: dict = {}
        self._latest_best_target: dict = {}
        self._tracking_start_time: Optional[float] = None

        # only one start=True goal at a time; set while no goal is running
        self._goal_active = False
        self._goal_idle = threading.Event()
        self._goal_idle.set()

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

        ## external services (AMCL, PLC lidar safety)
        # False: never call the AMCL / lidar safety services, so tracking is
        # not blocked by them; True: close AMCL and disable lidar safety as below
        self.manage_amcl_and_lidar_safety = bool(self.declare_parameter(
            'manage_amcl_and_lidar_safety', True).value)
        # check: success=True means AMCL is running
        self.amcl_check_service = self.declare_parameter(
            'amcl_check_service', '/check_mcl_if_trigger').value
        self.amcl_close_service = self.declare_parameter(
            'amcl_close_service', '/close_amcl').value
        self.amcl_open_service = self.declare_parameter(
            'amcl_open_service', '/open_amcl').value
        # SetBool data=True disables the lidar safety field; success is the write result
        self.lidar_safety_service = self.declare_parameter(
            'lidar_safety_service', '/g7_plc/set_disable_lidar_safety').value
        self.external_service_timeout = float(self.declare_parameter(
            'external_service_timeout', 5.0).value)

        self.amcl_check_client = self.create_client(
            Trigger, self.amcl_check_service, callback_group=self._client_cb_group)
        self.amcl_close_client = self.create_client(
            Empty, self.amcl_close_service, callback_group=self._client_cb_group)
        self.amcl_open_client = self.create_client(
            Empty, self.amcl_open_service, callback_group=self._client_cb_group)
        self.lidar_safety_client = self.create_client(
            SetBool, self.lidar_safety_service, callback_group=self._client_cb_group)

        # what this node changed and must restore; kept across goals if a
        # restore fails, so the next goal end or node shutdown retries it
        self._amcl_closed_by_us = False
        self._lidar_safety_disabled_by_us = False
        # last state reported by the providers, for feedback and logs
        self._amcl_state = 'unknown'          # on / off / unknown / unmanaged
        self._lidar_safety_state = 'unknown'  # enabled / disabled / unknown / unmanaged
        self._restore_lock = threading.Lock()
        if not self.manage_amcl_and_lidar_safety:
            self._amcl_state = 'unmanaged'
            self._lidar_safety_state = 'unmanaged'
            self.get_logger().warn(
                'manage_amcl_and_lidar_safety is false: AMCL and lidar safety '
                'will not be checked or changed.'
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
        self.get_logger().info(f'[publish] {self.cmd_pub.topic_name} stop vx=0 vy=0 wz=0')

    def _publish_control_command(self, vx: float, vy: float, vw: float) -> None:
        publish_twist(self.cmd_pub, vx, vy, vw)
        self.get_logger().info(
            f'[publish] {self.cmd_pub.topic_name} vx={vx:+.3f} vy={vy:+.3f} wz={vw:+.3f} '
            f'stage={self._alignment_stage}',
            throttle_duration_sec=LOG_THROTTLE_SEC,
        )

    def _safe_stop(self, reset_planner: bool = False) -> None:
        publish_twist(self.cmd_pub, 0.0, 0.0, 0.0)
        self.get_logger().info(f'[publish] {self.cmd_pub.topic_name} safe stop vx=0 vy=0 wz=0')
        if reset_planner:
            self.trajectory_planner.reset()
            self.lqr_tracker.reset()

    # ---- service calls (action execute thread / shutdown only) --------------

    def _call_service(self, client, request, timeout: float, what: str):
        """Call a service and wait for the response; return it, or None on failure."""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Cannot {what}: service {client.srv_name} not available.')
            return None

        self.get_logger().info(f'[request] {client.srv_name} ({what}): {msg_to_str(request)}')
        start = time.monotonic()
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout):
            client.remove_pending_request(future)
            self.get_logger().error(f'Cannot {what}: service {client.srv_name} timed out.')
            return None

        response = future.result()
        if response is None:
            self.get_logger().error(f'Cannot {what}: service {client.srv_name} call failed.')
            return None
        self.get_logger().info(
            f'[response] {client.srv_name} ({what}): {msg_to_str(response)} '
            f'in {time.monotonic() - start:.3f}s'
        )
        return response

    def _set_detection_enabled(self, enabled: bool) -> bool:
        """Call apriltag_detection's enable service; return True on success."""
        action = 'enable' if enabled else 'disable'
        response = self._call_service(
            self.detection_enable_client, SetBool.Request(data=enabled),
            self.detection_service_timeout, f'{action} detection')
        if response is None:
            return False
        if not response.success:
            self.get_logger().error(f'Cannot {action} detection: {response.message}')
            return False
        return True

    def _check_amcl(self) -> Optional[bool]:
        """Return True if AMCL reports it is running, False if not, None on failure."""
        response = self._call_service(
            self.amcl_check_client, Trigger.Request(),
            self.external_service_timeout, 'check AMCL')
        if response is None:
            self._amcl_state = 'unknown'
            return None
        self._amcl_state = 'on' if response.success else 'off'
        self.get_logger().info(f'AMCL status: {self._amcl_state} ({response.message})')
        return bool(response.success)

    def _set_amcl_enabled(self, enabled: bool) -> bool:
        """Check AMCL, switch it only if needed, then check again; return True if confirmed."""
        action = 'open' if enabled else 'close'
        is_on = self._check_amcl()
        if is_on is None:
            return False
        if is_on == enabled:
            if enabled:
                self._amcl_closed_by_us = False
            return True

        if not enabled:
            # mark before the call: if the call fails we cannot tell whether
            # AMCL was closed, so goal end must still try to open it
            self._amcl_closed_by_us = True
        client = self.amcl_open_client if enabled else self.amcl_close_client
        if self._call_service(client, Empty.Request(), self.external_service_timeout,
                              f'{action} AMCL') is None:
            return False

        is_on = self._check_amcl()
        if is_on != enabled:
            self.get_logger().error(
                f'Cannot {action} AMCL: status after {client.srv_name} is {self._amcl_state}.'
            )
            return False
        if enabled:
            self._amcl_closed_by_us = False
        return True

    def _set_lidar_safety_disabled(self, disabled: bool) -> bool:
        """Write the PLC lidar safety flag; return True if the provider confirms it."""
        action = 'disable' if disabled else 'enable'
        if disabled:
            # mark before the call, same reason as AMCL
            self._lidar_safety_disabled_by_us = True
        response = self._call_service(
            self.lidar_safety_client, SetBool.Request(data=disabled),
            self.external_service_timeout, f'{action} lidar safety')
        if response is None:
            self._lidar_safety_state = 'unknown'
            return False
        if not response.success:
            self._lidar_safety_state = 'unknown'
            self.get_logger().error(f'Cannot {action} lidar safety: {response.message}')
            return False

        self._lidar_safety_state = 'disabled' if disabled else 'enabled'
        self.get_logger().info(f'Lidar safety {self._lidar_safety_state} ({response.message})')
        if not disabled:
            self._lidar_safety_disabled_by_us = False
        return True

    def _restore_external_state(self) -> bool:
        """Undo what this node changed: lidar safety first, then AMCL."""
        with self._restore_lock:
            ok = True
            if self._lidar_safety_disabled_by_us and not self._set_lidar_safety_disabled(False):
                self.get_logger().error('Restore failed: lidar safety may still be disabled.')
                ok = False
            if self._amcl_closed_by_us and not self._set_amcl_enabled(True):
                self.get_logger().error('Restore failed: AMCL may still be closed.')
                ok = False
            return ok

    # ---- tracking lifecycle ---------------------------------------------------

    def _start_control(self, reason: str) -> None:
        with self._control_lock:
            self._control_enabled = True
            self._error_zero_since = None
            self._alignment_stage = 1
            self._stage2_pending = False
            self.desired_distance = self.stage1_distance
            self._last_stamp = None
            self._tracking_start_time = time.monotonic()
            self._last_target_time = None
            self._latest_best_target = {}
            self.trajectory_planner.reset()
            self.lqr_tracker.reset()
        self.get_logger().info(reason)

    def _start_tracking_goal_callback(self, goal_request: StartTracking.Goal):
        self.get_logger().info(f'[action] goal received: start={goal_request.start}')
        if not goal_request.start:
            return GoalResponse.ACCEPT
        with self._control_lock:
            if self._goal_active:
                self.get_logger().warn('Rejected start_tracking goal: a goal is already running.')
                return GoalResponse.REJECT
            self._goal_active = True
            self._goal_idle.clear()
        self.get_logger().info('[action] goal accepted: start=True')
        return GoalResponse.ACCEPT

    def _start_tracking_cancel_callback(self, goal_handle):
        _ = goal_handle
        self.get_logger().info('[action] cancel requested')
        return CancelResponse.ACCEPT

    def _log_result(self, status: str, result: StartTracking.Result) -> None:
        log = self.get_logger().info if result.success else self.get_logger().warn
        log(f'[action] result: {status} success={result.success} message={result.message!r}')

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
            # 喚醒卡在 while loop 的 start=True thread，避免 thread pool 耗盡;
            # that thread disables detection and restores AMCL / lidar safety
            if not self._tracking_done.is_set():
                with self._control_lock:
                    self._tracking_result = {'success': False, 'message': 'Tracking stopped.'}
                self._tracking_done.set()
            self._goal_idle.wait(timeout=self._goal_end_timeout())
            result.success = True
            result.message = 'Tracking stopped.'
            goal_handle.succeed()
            self._log_result('SUCCEEDED (start=False)', result)
            return result

        outcome, message = 'abort', 'Tracking ended unexpectedly.'
        try:
            outcome, message = self._run_tracking(goal_handle)
        finally:
            if not self._end_tracking():
                message += ' Restore failed, see apriltag_control log.'
            with self._control_lock:
                self._goal_active = False
            self._goal_idle.set()

        result.success = outcome == 'succeed'
        result.message = message
        if outcome == 'succeed':
            goal_handle.succeed()
        elif outcome == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self._log_result({'succeed': 'SUCCEEDED', 'canceled': 'CANCELED'}.get(outcome, 'ABORTED'), result)
        return result

    def _run_tracking(self, goal_handle):
        """Run one start=True goal; return (outcome, message).

        outcome is 'succeed', 'abort' or 'canceled'. Cleanup is done by the caller.
        """
        self._tracking_done.clear()
        self._tracking_result = {}

        if self.manage_amcl_and_lidar_safety and not self._set_amcl_enabled(False):
            return 'abort', 'Failed to close AMCL.'
        if not self._set_detection_enabled(True):
            return 'abort', 'Failed to enable apriltag_detection.'
        self._start_control('Control started from action server. Tracking target...')

        feedback = StartTracking.Feedback()
        while not self._tracking_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                self._pause_control('Tracking cancelled by client.')
                return 'canceled', 'Tracking cancelled.'

            if self._is_stage2_pending():
                self._begin_stage2()

            t = self._latest_best_target
            feedback.status = (
                f"tracking: x_err={t.get('x_error', float('nan')):.3f} "
                f"y_err={t.get('y_error', float('nan')):.3f} "
                f"yaw_err={t.get('yaw_error', float('nan')):.3f} "
                f"stage={self._alignment_stage} "
                f"amcl={self._amcl_state} lidar_safety={self._lidar_safety_state}"
            )
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'[action] feedback: {feedback.status}',
                                   throttle_duration_sec=LOG_THROTTLE_SEC)

        with self._control_lock:
            res = self._tracking_result.copy()
        return ('succeed' if res.get('success', False) else 'abort'), res.get('message', '')

    def _is_stage2_pending(self) -> bool:
        with self._control_lock:
            return self._control_enabled and self._stage2_pending

    def _begin_stage2(self) -> None:
        """Disable lidar safety while stopped, then switch to Stage 2 or fail the goal."""
        if self.manage_amcl_and_lidar_safety and not self._set_lidar_safety_disabled(True):
            if not self._is_control_enabled():
                return  # goal already ended while waiting for the service
            msg = 'Failed to disable lidar safety. Tracking stopped before Stage 2.'
            self._pause_control(msg)
            self._signal_tracking_complete(False, msg)
            return

        with self._control_lock:
            # target lost / cancelled while waiting for the service
            if not (self._control_enabled and self._stage2_pending):
                return
            self._stage2_pending = False
            self._alignment_stage = 2
            self.desired_distance = self.stage2_distance
            self._error_zero_since = None
            self._last_stamp = None
            self.trajectory_planner.reset()
            self.lqr_tracker.reset()
        self.get_logger().info(f'Advancing to Stage 2 ({self.stage2_distance:.2f}m).')

    def _end_tracking(self) -> bool:
        """Stop, disable detection and restore external state; return False if anything failed."""
        with self._control_lock:
            self._control_enabled = False
            self._stage2_pending = False
        self._publish_stop_command()
        ok = self._set_detection_enabled(False)
        return self._restore_external_state() and ok

    def _goal_end_timeout(self) -> float:
        # detection off + lidar safety + AMCL check/open/check, each wait + call
        return 2.0 * (self.detection_service_timeout + 4 * self.external_service_timeout)

    def shutdown(self) -> None:
        """End a running goal and restore external state before the node is destroyed."""
        with self._control_lock:
            active = self._goal_active
        if active:
            msg = 'Node shutting down.'
            self._pause_control(msg)
            self._signal_tracking_complete(False, msg)
            if not self._goal_idle.wait(timeout=self._goal_end_timeout()):
                self.get_logger().error('Running goal did not end in time.')
        self._restore_external_state()

    def _is_control_enabled(self) -> bool:
        with self._control_lock:
            return self._control_enabled

    def _pause_control(self, reason: str):
        with self._control_lock:
            self._control_enabled = False
            self._stage2_pending = False
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

        # stopped between the stages while lidar safety is being disabled;
        # keep the target watchdog fed but publish no motion
        with self._control_lock:
            if self._stage2_pending:
                return

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
                publish_twist_fn=self._publish_control_command,
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
            self._publish_stop_command()
            with self._control_lock:
                self._stage2_pending = True
                self._error_zero_since = None
            self.get_logger().info(
                f'Stage 1 aligned at {self.stage1_distance:.2f}m. '
                f'Stopped; starting Stage 2'
                + (' after disabling lidar safety.' if self.manage_amcl_and_lidar_safety else '.')
            )
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
    # rclpy's default SIGINT handler shuts the context down, after which the
    # restore service calls in node.shutdown() could not be made. Handle
    # SIGINT / SIGTERM here and shut rclpy down only after restoring.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AprilTagControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.wait(timeout=0.5) and spin_thread.is_alive():
            pass
        node.shutdown()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
