import signal
import threading
import time
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from apriltag_interfaces.action import StartTracking
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool, Trigger

from apriltag.control import LQRTracker, TrajectoryPlanner
from apriltag.domain.math_utils import optical_to_control_error, quaternion_to_rotation_matrix
from apriltag.ros.ros_io import publish_twist
from apriltag.runtime.control_flow import publish_control
from apriltag.runtime.safety_guard import handle_target_lost

# high-rate messages (cmd_vel, action feedback) are logged at most once per this period
LOG_THROTTLE_SEC = 1.0


class TrackingState(Enum):
    IDLE = 'IDLE'                # nothing running, AMCL / lidar safety restored
    STARTING = 'STARTING'        # start_tracking accepted: closing AMCL, enabling detection
    STAGE1 = 'STAGE1'            # tracking to stage1_distance on /cmd_vel
    STAGE2 = 'STAGE2'            # tracking to stage2_distance on /pre_cmd_vel
    IN_POSITION = 'IN_POSITION'  # Stage 2 aligned and stopped; AMCL off, lidar safety off
    LEAVING = 'LEAVING'          # leave_cs: tracking back to leave_distance on /pre_cmd_vel
    FINISHING = 'FINISHING'      # procedure ended: stopping, disabling detection, restoring


S = TrackingState
ALLOWED_TRANSITIONS = {
    S.IDLE: {S.STARTING},
    S.STARTING: {S.STAGE1, S.FINISHING},
    S.STAGE1: {S.STAGE2, S.FINISHING},
    S.STAGE2: {S.IN_POSITION, S.FINISHING},
    S.IN_POSITION: {S.LEAVING, S.FINISHING},
    S.LEAVING: {S.FINISHING},
    S.FINISHING: {S.IDLE},
}
# pose_callback publishes velocity commands only in these states
CONTROL_ACTIVE_STATES = frozenset({S.STAGE1, S.STAGE2, S.LEAVING})
# states a stop / cancel / failure / target loss moves to FINISHING
STOPPABLE_STATES = frozenset({S.STARTING, S.STAGE1, S.STAGE2, S.LEAVING})


def is_transition_allowed(src: TrackingState, dst: TrackingState) -> bool:
    return dst in ALLOWED_TRANSITIONS.get(src, set())


# width of the state in the log prefix, so the messages line up
STATE_LOG_WIDTH = max(len(s.value) for s in TrackingState)


class _StageLogger:
    """Logger for the runtime helpers: adds the node's stage prefix.

    The helpers only log plain warnings (no throttle / once), so sharing one
    rclpy call site per severity here is fine.
    """

    def __init__(self, node: 'AprilTagControlNode'):
        self._node = node

    def info(self, message: str) -> None:
        self._node.get_logger().info(self._node._stage() + message)

    def warn(self, message: str) -> None:
        self._node.get_logger().warn(self._node._stage() + message)

    def error(self, message: str) -> None:
        self._node.get_logger().error(self._node._stage() + message)


def msg_to_str(msg) -> str:
    """Compact 'field=value, ...' text of a ROS message; '{}' if it has no fields."""
    fields = msg.get_fields_and_field_types()
    if not fields:
        return '{}'
    return ', '.join(f'{name}={getattr(msg, name)!r}' for name in fields)


class AprilTagControlNode(Node):
    """Orchestrate AprilTag tracking and drive the robot to the tag.

    The procedure is a state machine (TrackingState):

        IDLE -> STARTING -> STAGE1 -> STAGE2 -> IN_POSITION -> LEAVING -> FINISHING -> IDLE

    A start_tracking goal closes AMCL and enables apriltag_detection
    (STARTING), tracks /apriltag_pose with TrajectoryPlanner + LQR on /cmd_vel
    (STAGE1), stops, disables the PLC lidar safety field and continues on
    /pre_cmd_vel (G7+ precision mode, the motors wait until the steering is
    within 5 deg) (STAGE2). When Stage 2 is aligned the robot stops, the goal
    succeeds and the node holds in IN_POSITION with AMCL closed, lidar safety
    disabled and detection enabled. A leave_cs goal then tracks back to
    leave_distance on /pre_cmd_vel (LEAVING). When leaving ends, or when any
    stage fails, is cancelled or stopped, the node goes through FINISHING: it
    stops the robot, disables detection, enables lidar safety and restores
    AMCL if it closed it, then returns to IDLE. start_tracking start=False in
    IN_POSITION abandons the alignment the same way. With
    manage_amcl_and_lidar_safety set to false it skips all AMCL and lidar
    safety calls.

    AMCL is switched through external services whose responses are checked.
    Lidar safety follows the G7+ AutoCharging way: a Bool is published on the
    PLC bridge topic with no read-back, so it cannot be confirmed here. What
    the providers actually do with a request is outside this package.
    """

    def __init__(self):
        super().__init__('apriltag_control')

        self.pose_sub = self.create_subscription(PoseStamped,
                                                 'apriltag_pose',
                                                 self.pose_callback,
                                                 1)

        ## AprilTag distance parameters
        # two-stage desired forward distances to tag (m), ROS parameters
        self.stage1_distance = float(self.declare_parameter(
            'stage1_distance', 0.50).value)   # 第一階段：50 cm 對齊
        self.stage2_distance = float(self.declare_parameter(
            'stage2_distance', 0.28).value)   # 第二階段：28 cm 對齊
        # leave_cs: track back to this distance from IN_POSITION
        self.leave_distance = float(self.declare_parameter(
            'leave_distance', 0.40).value)    # 離開：退到 40 cm
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
        # Stage 1: normal cmd_vel; Stage 2 and leaving: G7+ precision cmd_vel,
        # same as the final approach of G7+ AutoCharging
        self.stage1_cmd_vel_topic = self.declare_parameter(
            'stage1_cmd_vel_topic', '/cmd_vel').value
        self.stage2_cmd_vel_topic = self.declare_parameter(
            'stage2_cmd_vel_topic', '/pre_cmd_vel').value
        self.stage1_cmd_pub = self.create_publisher(Twist, self.stage1_cmd_vel_topic, 10)
        self.stage2_cmd_pub = self.create_publisher(Twist, self.stage2_cmd_vel_topic, 10)
        # capture time (s) of the last pose used for control, for dt
        self._last_stamp = None
        self.max_vx = 0.1
        self.max_vy = 0.1
        self.max_vw = 0.05
        self.max_dt = 0.2
        self.latest_plan = None
        self.lost_target_timeout = 1.0
        self.lost_check_period = 0.05
        self._last_target_time = None
        # guards _state and the flags below
        self._control_lock = threading.Lock()
        self._state = TrackingState.IDLE
        self._stage_logger = _StageLogger(self)
        self._error_zero_since = None
        # Stage 1 converged and the robot is stopped (state stays STAGE1); the
        # action execute thread disables lidar safety before Stage 2 starts
        self._stage2_pending = False
        self.stop_hold_seconds = 1.0
        self.stop_x_error_tolerance = 0.05
        self.stop_y_error_tolerance = 0.05
        self.stop_yaw_error_tolerance = 0.05
        # set when the running start_tracking / leave_cs goal has a result
        self._tracking_done = threading.Event()
        self._tracking_result: dict = {}
        self._latest_best_target: dict = {}
        self._tracking_start_time: Optional[float] = None

        # True while a start_tracking / leave_cs start=True goal (or an
        # abandon) is still running its execute thread
        self._goal_active = False
        self._goal_idle = threading.Event()
        self._goal_idle.set()
        # set by shutdown(): cleanup then restores AMCL / lidar safety before
        # disabling detection
        self._shutting_down = False

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
        # Bool data=True disables the lidar safety field, False enables it;
        # fire-and-forget, same as G7+ AutoCharging
        self.lidar_safety_topic = self.declare_parameter(
            'lidar_safety_topic', '/g7_plc/disable_lidar_safety').value
        self.external_service_timeout = float(self.declare_parameter(
            'external_service_timeout', 5.0).value)

        self.amcl_check_client = self.create_client(
            Trigger, self.amcl_check_service, callback_group=self._client_cb_group)
        self.amcl_close_client = self.create_client(
            Empty, self.amcl_close_service, callback_group=self._client_cb_group)
        self.amcl_open_client = self.create_client(
            Empty, self.amcl_open_service, callback_group=self._client_cb_group)
        self.lidar_safety_pub = self.create_publisher(Bool, self.lidar_safety_topic, 1)

        # AMCL this node closed and must open again; kept across goals if a
        # restore fails, so the next goal end or node shutdown retries it
        self._amcl_closed_by_us = False
        # for feedback and logs: AMCL as reported by its provider, lidar safety
        # as last published (not confirmed by the PLC)
        self._amcl_state = 'unknown'          # on / off / unknown / unmanaged
        self._lidar_safety_state = 'unknown'  # enabled / disabled / unknown / unmanaged
        self._restore_lock = threading.Lock()
        if not self.manage_amcl_and_lidar_safety:
            self._amcl_state = 'unmanaged'
            self._lidar_safety_state = 'unmanaged'
            self.get_logger().warn(
                self._stage() + 'manage_amcl_and_lidar_safety is false: AMCL and lidar safety '
                'will not be checked or changed.'
            )

        self._action_cb_group = ReentrantCallbackGroup()
        self.start_tracking_action_server = ActionServer(
            self,
            StartTracking,
            'start_tracking',
            execute_callback=self._execute_start_tracking,
            goal_callback=self._start_tracking_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )
        self.leave_cs_action_server = ActionServer(
            self,
            StartTracking,
            'leave_cs',
            execute_callback=self._execute_leave_cs,
            goal_callback=self._leave_cs_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )
        self.get_logger().info(
            self._stage() + f'State {self._state.value}. Send the start_tracking action to start.'
        )

    # ---- state machine ----------------------------------------------------------

    def _stage(self) -> str:
        """Prefix of every log message: '[stage LEAVING    ] : '.

        The launch file sets this node's RCUTILS_CONSOLE_OUTPUT_FORMAT to
        '[{severity}] [{name}]{message}', so a line reads
        '[INFO] [up.apriltag_control][stage LEAVING    ] : ...'.
        """
        return f'[stage {self._state.value:<{STATE_LOG_WIDTH}}] : '

    def _transition_locked(self, new: TrackingState, reason: str) -> bool:
        """Change state if allowed; the caller must hold _control_lock."""
        old = self._state
        if not is_transition_allowed(old, new):
            self.get_logger().error(
                self._stage() + f'[state] invalid transition {old.value} -> {new.value} ({reason}), ignored.')
            return False
        self._state = new
        # the Stage 1 -> 2 pause only exists inside STAGE1
        if new is not S.STAGE1:
            self._stage2_pending = False
        self.get_logger().info(self._stage() + f'[state] {old.value} -> {new.value} ({reason})')
        return True

    def _reset_control_locked(self, desired_distance: float) -> None:
        """Reset the controller for a new tracking phase; the caller must hold _control_lock."""
        self.desired_distance = desired_distance
        self._error_zero_since = None
        self._last_stamp = None
        self._last_target_time = None
        self._latest_best_target = {}
        self.trajectory_planner.reset()
        self.lqr_tracker.reset()

    def _elapsed_message(self, message: str) -> str:
        elapsed = (
            time.monotonic() - self._tracking_start_time
            if self._tracking_start_time is not None
            else float('nan')
        )
        return f'{message} elapsed={elapsed:.2f}s'

    def _finish_motion(self, success: bool, message: str) -> bool:
        """Move a running procedure to FINISHING and report its result.

        Used for leave aligned, target lost, cancel, stop, service failure and
        shutdown. Returns False if the state was not stoppable (already ended).
        """
        timed_message = self._elapsed_message(message)
        with self._control_lock:
            if self._state not in STOPPABLE_STATES:
                return False
            self._transition_locked(S.FINISHING, message.split('. ')[0].rstrip('.'))
            self._error_zero_since = None
            self._last_target_time = None
            self._tracking_result = {'success': success, 'message': timed_message}
            self._tracking_done.set()
        self.trajectory_planner.reset()
        self.lqr_tracker.reset()
        self._publish_stop_command()
        # rclpy rejects one call site logging at different severities
        if success:
            self.get_logger().info(self._stage() + timed_message)
        else:
            self.get_logger().warn(self._stage() + timed_message)
        return True

    def _reach_in_position(self, message: str) -> None:
        """Stage 2 aligned: STAGE2 -> IN_POSITION, stop and hold (no restore)."""
        timed_message = self._elapsed_message(message)
        with self._control_lock:
            if self._state is not S.STAGE2:
                return
            self._transition_locked(S.IN_POSITION, 'Stage 2 aligned')
            self._error_zero_since = None
            self._last_target_time = None
            self._tracking_result = {'success': True, 'message': timed_message}
            self._tracking_done.set()
        self.trajectory_planner.reset()
        self.lqr_tracker.reset()
        self._publish_stop_command()
        self.get_logger().info(
            self._stage() + f'{timed_message} Holding in IN_POSITION (AMCL / lidar safety not restored); '
            f'send the leave_cs action to leave.'
        )

    def _release_goal(self) -> None:
        with self._control_lock:
            self._goal_active = False
        self._goal_idle.set()

    def _reserve_goal_locked(self) -> None:
        """Mark a long-running goal as started; the caller must hold _control_lock."""
        self._goal_active = True
        self._goal_idle.clear()
        self._tracking_done.clear()
        self._tracking_result = {}
        self._tracking_start_time = time.monotonic()

    # ---- velocity output ------------------------------------------------------

    def _publish_zero_on_all(self, label: str) -> None:
        # stops go to both topics, so the robot stops whichever stage is active
        for pub in (self.stage1_cmd_pub, self.stage2_cmd_pub):
            publish_twist(pub, 0.0, 0.0, 0.0)
            self.get_logger().info(self._stage() + f'[publish] {pub.topic_name} {label} vx=0 vy=0 wz=0')

    def _publish_stop_command(self) -> None:
        self._publish_zero_on_all('stop')

    def _publish_control_command(self, vx: float, vy: float, vw: float) -> None:
        # published under the lock, so no command goes out after a transition
        # out of the control states (whose stop is published after it)
        with self._control_lock:
            state = self._state
            if state not in CONTROL_ACTIVE_STATES or self._stage2_pending:
                return
            pub = self.stage1_cmd_pub if state is S.STAGE1 else self.stage2_cmd_pub
            publish_twist(pub, vx, vy, vw)
        self.get_logger().info(
            self._stage() + f'[publish] {pub.topic_name} vx={vx:+.3f} vy={vy:+.3f} wz={vw:+.3f}',
            throttle_duration_sec=LOG_THROTTLE_SEC,
        )

    def _safe_stop(self, reset_planner: bool = False) -> None:
        self._publish_zero_on_all('safe stop')
        if reset_planner:
            self.trajectory_planner.reset()
            self.lqr_tracker.reset()

    # ---- service calls (action execute thread / shutdown only) --------------

    def _call_service(self, client, request, timeout: float, what: str):
        """Call a service and wait for the response; return it, or None on failure."""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(self._stage() + f'Cannot {what}: service {client.srv_name} not available.')
            return None

        self.get_logger().info(self._stage() + f'[request] {client.srv_name} ({what}): {msg_to_str(request)}')
        start = time.monotonic()
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout):
            client.remove_pending_request(future)
            self.get_logger().error(self._stage() + f'Cannot {what}: service {client.srv_name} timed out.')
            return None

        response = future.result()
        if response is None:
            self.get_logger().error(self._stage() + f'Cannot {what}: service {client.srv_name} call failed.')
            return None
        self.get_logger().info(
            self._stage() + f'[response] {client.srv_name} ({what}): {msg_to_str(response)} '
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
            self.get_logger().error(self._stage() + f'Cannot {action} detection: {response.message}')
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
        self.get_logger().info(self._stage() + f'AMCL status: {self._amcl_state} ({response.message})')
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
                self._stage() + f'Cannot {action} AMCL: status after {client.srv_name} is {self._amcl_state}.'
            )
            return False
        if enabled:
            self._amcl_closed_by_us = False
        return True

    def _set_lidar_safety_disabled(self, disabled: bool) -> None:
        """Publish the PLC disable-lidar-safety flag once (G7+ way: no read-back)."""
        self.lidar_safety_pub.publish(Bool(data=disabled))
        self._lidar_safety_state = 'disabled' if disabled else 'enabled'
        self.get_logger().info(
            self._stage() + f'[publish] {self.lidar_safety_pub.topic_name} data={disabled} '
            f'({"disable" if disabled else "enable"} lidar safety, not confirmed)'
        )

    def _restore_external_state(self) -> bool:
        """Enable lidar safety, then open AMCL if this node closed it."""
        if not self.manage_amcl_and_lidar_safety:
            return True
        with self._restore_lock:
            ok = True
            # always published at procedure end, whether or not Stage 2 was reached
            self._set_lidar_safety_disabled(False)
            if self._amcl_closed_by_us and not self._set_amcl_enabled(True):
                self.get_logger().error(self._stage() + 'Restore failed: AMCL may still be closed.')
                ok = False
            return ok

    def _end_tracking(self, restore_first: bool = False) -> bool:
        """Stop, disable detection and restore external state; return False if anything failed.

        restore_first (used at shutdown): restore AMCL / lidar safety before
        disabling detection, since detection may already be gone.
        """
        self._publish_stop_command()
        if restore_first:
            ok = self._restore_external_state()
            return self._set_detection_enabled(False) and ok
        ok = self._set_detection_enabled(False)
        return self._restore_external_state() and ok

    def _end_goal_to_idle(self) -> bool:
        """Clean up after a goal that did not end in IN_POSITION: FINISHING -> IDLE."""
        with self._control_lock:
            if self._state not in (S.FINISHING, S.IDLE):
                self._transition_locked(S.FINISHING, 'goal ended')
            restore_first = self._shutting_down
        ok = self._end_tracking(restore_first=restore_first)
        with self._control_lock:
            if self._state is S.FINISHING:
                self._transition_locked(
                    S.IDLE, 'cleanup done' if ok else 'cleanup done, restore failed')
        self._release_goal()
        return ok

    def _abandon_in_position(self, reason: str, restore_first: bool) -> Optional[bool]:
        """IN_POSITION -> FINISHING -> IDLE with full cleanup; None if not in IN_POSITION."""
        with self._control_lock:
            if self._state is not S.IN_POSITION or self._goal_active:
                return None
            self._transition_locked(S.FINISHING, reason)
            # block other goals / shutdown until the cleanup is done
            self._goal_active = True
            self._goal_idle.clear()
        ok = self._end_tracking(restore_first=restore_first)
        with self._control_lock:
            self._reset_control_locked(self.stage1_distance)
            self._transition_locked(
                S.IDLE, 'cleanup done' if ok else 'cleanup done, restore failed')
        self._release_goal()
        return ok

    def _goal_end_timeout(self) -> float:
        # detection off + AMCL check/open/check, each wait + call
        return 2.0 * (self.detection_service_timeout + 3 * self.external_service_timeout)

    # ---- action servers ---------------------------------------------------------

    def _reject_goal(self, action: str, start: bool, state: TrackingState, rule: str):
        self.get_logger().warn(
            self._stage() + f'[action] {action} goal rejected: start={start} state={state.value} ({rule})')
        return GoalResponse.REJECT

    def _start_tracking_goal_callback(self, goal_request: StartTracking.Goal):
        self.get_logger().info(self._stage() + f'[action] start_tracking goal received: start={goal_request.start}')
        if not goal_request.start:
            return GoalResponse.ACCEPT
        with self._control_lock:
            state = self._state
            if state is not S.IDLE or self._goal_active:
                return self._reject_goal('start_tracking', True, state,
                                         'start=True is only accepted in IDLE')
            self._reserve_goal_locked()
            self._transition_locked(S.STARTING, 'start_tracking goal accepted')
        self.get_logger().info(self._stage() + '[action] start_tracking goal accepted: start=True')
        return GoalResponse.ACCEPT

    def _leave_cs_goal_callback(self, goal_request: StartTracking.Goal):
        self.get_logger().info(self._stage() + f'[action] leave_cs goal received: start={goal_request.start}')
        with self._control_lock:
            state = self._state
            if not goal_request.start:
                if state is not S.LEAVING:
                    return self._reject_goal('leave_cs', False, state,
                                             'leave_cs start=False is only accepted in LEAVING')
                self.get_logger().info(self._stage() + '[action] leave_cs goal accepted: start=False')
                return GoalResponse.ACCEPT
            if state is not S.IN_POSITION or self._goal_active:
                return self._reject_goal('leave_cs', True, state,
                                         'leave_cs start=True is only accepted in IN_POSITION')
            # detection is still enabled, so control starts right away
            self._reserve_goal_locked()
            self._reset_control_locked(self.leave_distance)
            self._transition_locked(S.LEAVING, f'leave_cs goal accepted, back to {self.leave_distance:.2f}m')
        self.get_logger().info(self._stage() + '[action] leave_cs goal accepted: start=True')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        _ = goal_handle
        self.get_logger().info(self._stage() + '[action] cancel requested')
        return CancelResponse.ACCEPT

    def _log_result(self, action: str, status: str, result: StartTracking.Result) -> None:
        text = f'[action] {action} result: {status} success={result.success} message={result.message!r}'
        # rclpy rejects one call site logging at different severities
        if result.success:
            self.get_logger().info(self._stage() + text)
        else:
            self.get_logger().warn(self._stage() + text)

    def _finish_goal_handle(self, goal_handle, action: str, outcome: str, message: str):
        result = StartTracking.Result()
        result.success = outcome == 'succeed'
        result.message = message
        if outcome == 'succeed':
            goal_handle.succeed()
        elif outcome == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self._log_result(action, {'succeed': 'SUCCEEDED', 'canceled': 'CANCELED'}.get(outcome, 'ABORTED'),
                         result)
        return result

    def _goal_outcome(self):
        with self._control_lock:
            res = self._tracking_result.copy()
        return ('succeed' if res.get('success', False) else 'abort'), res.get('message', '')

    def _execute_start_tracking(self, goal_handle):
        if not goal_handle.request.start:
            return self._handle_stop_goal(goal_handle)

        outcome, message = 'abort', 'Tracking ended unexpectedly.'
        try:
            outcome, message = self._run_tracking(goal_handle)
        finally:
            with self._control_lock:
                in_position = self._state is S.IN_POSITION
            if in_position and outcome == 'succeed':
                # hold: AMCL, lidar safety and detection stay as they are
                self._release_goal()
            elif not self._end_goal_to_idle():
                message += ' Restore failed, see apriltag_control log.'
        return self._finish_goal_handle(goal_handle, 'start_tracking', outcome, message)

    def _stopped_while_starting(self, goal_handle):
        """Return (outcome, message) if the goal was cancelled or stopped in STARTING, else None."""
        if goal_handle.is_cancel_requested:
            self._finish_motion(False, 'Tracking cancelled.')
            return 'canceled', 'Tracking cancelled.'
        if self._tracking_done.is_set():
            return self._goal_outcome()
        return None

    def _run_tracking(self, goal_handle):
        """Run one start=True goal; return (outcome, message).

        outcome is 'succeed', 'abort' or 'canceled'. Cleanup is done by the caller.
        """
        if self.manage_amcl_and_lidar_safety and not self._set_amcl_enabled(False):
            self._finish_motion(False, 'Failed to close AMCL.')
            return self._goal_outcome()
        stopped = self._stopped_while_starting(goal_handle)
        if stopped is not None:
            return stopped
        if not self._set_detection_enabled(True):
            self._finish_motion(False, 'Failed to enable apriltag_detection.')
            return self._goal_outcome()
        stopped = self._stopped_while_starting(goal_handle)
        if stopped is not None:
            return stopped
        if not self._start_stage1():
            return self._goal_outcome()
        return self._wait_motion_done(goal_handle, 'Tracking cancelled.')

    def _start_stage1(self) -> bool:
        with self._control_lock:
            if self._state is not S.STARTING or self._tracking_done.is_set():
                return False
            self._reset_control_locked(self.stage1_distance)
            self._tracking_start_time = time.monotonic()
            self._transition_locked(S.STAGE1, 'AMCL closed, detection enabled')
        self.get_logger().info(
            self._stage() + f'Stage 1 ({self.stage1_distance:.2f}m) started. Tracking target...')
        return True

    def _wait_motion_done(self, goal_handle, cancel_message: str):
        """Wait until the running tracking / leaving has a result; return (outcome, message)."""
        feedback = StartTracking.Feedback()
        while not self._tracking_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                if self._finish_motion(False, cancel_message):
                    return 'canceled', cancel_message
                break  # ended meanwhile, report that result

            with self._control_lock:
                stage2_pending = self._state is S.STAGE1 and self._stage2_pending
            if stage2_pending:
                self._begin_stage2()

            t = self._latest_best_target
            feedback.status = (
                f"tracking: x_err={t.get('x_error', float('nan')):.3f} "
                f"y_err={t.get('y_error', float('nan')):.3f} "
                f"yaw_err={t.get('yaw_error', float('nan')):.3f} "
                f"state={self._state.value} "
                f"amcl={self._amcl_state} lidar_safety={self._lidar_safety_state}"
            )
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(self._stage() + f'[action] feedback: {feedback.status}',
                                   throttle_duration_sec=LOG_THROTTLE_SEC)
        return self._goal_outcome()

    def _begin_stage2(self) -> None:
        """Disable lidar safety while stopped, then switch to Stage 2."""
        if self.manage_amcl_and_lidar_safety:
            self._set_lidar_safety_disabled(True)

        with self._control_lock:
            # target lost / cancelled in the meantime
            if not (self._state is S.STAGE1 and self._stage2_pending):
                return
            self._reset_control_locked(self.stage2_distance)
            self._transition_locked(S.STAGE2, 'Stage 1 aligned, lidar safety disabled')
        self.get_logger().info(self._stage() + f'Advancing to Stage 2 ({self.stage2_distance:.2f}m).')

    def _handle_stop_goal(self, goal_handle):
        """start_tracking start=False: stop whatever runs; abandon IN_POSITION."""
        with self._control_lock:
            state = self._state
        message = 'Tracking stopped.'
        if state in STOPPABLE_STATES:
            message = 'Leave stopped.' if state is S.LEAVING else 'Tracking stopped.'
            self._finish_motion(False, message)

        # wait for the running goal's cleanup (also covers FINISHING and the
        # moment between Stage 2 aligned and the start_tracking result)
        with self._control_lock:
            active = self._goal_active
        if active and not self._goal_idle.wait(timeout=self._goal_end_timeout()):
            self.get_logger().error(self._stage() + 'Running goal did not end in time.')

        with self._control_lock:
            state = self._state
        if state is S.IN_POSITION:
            ok = self._abandon_in_position('abandoned by start_tracking start=False',
                                           restore_first=False)
            if ok is not None:
                message = ('Abandoned IN_POSITION: detection disabled, '
                           'lidar safety / AMCL restored.')
                if not ok:
                    message += ' Restore failed, see apriltag_control log.'
        elif state is S.IDLE:
            self._publish_stop_command()

        result = StartTracking.Result()
        result.success = True
        result.message = message
        goal_handle.succeed()
        self._log_result('start_tracking', 'SUCCEEDED (start=False)', result)
        return result

    def _execute_leave_cs(self, goal_handle):
        if not goal_handle.request.start:
            with self._control_lock:
                leaving = self._state is S.LEAVING
            if leaving:
                self._finish_motion(False, 'Leave stopped.')
            with self._control_lock:
                active = self._goal_active
            if active and not self._goal_idle.wait(timeout=self._goal_end_timeout()):
                self.get_logger().error(self._stage() + 'Running goal did not end in time.')
            result = StartTracking.Result()
            result.success = True
            result.message = 'Leave stopped.'
            goal_handle.succeed()
            self._log_result('leave_cs', 'SUCCEEDED (start=False)', result)
            return result

        outcome, message = 'abort', 'Leave ended unexpectedly.'
        try:
            outcome, message = self._wait_motion_done(goal_handle, 'Leave cancelled.')
        finally:
            if not self._end_goal_to_idle():
                message += ' Restore failed, see apriltag_control log.'
        return self._finish_goal_handle(goal_handle, 'leave_cs', outcome, message)

    def shutdown(self) -> None:
        """End a running procedure and restore external state before the node is destroyed."""
        with self._control_lock:
            self._shutting_down = True
            state = self._state
        msg = 'Node shutting down.'
        if state in STOPPABLE_STATES:
            self._finish_motion(False, msg)
        with self._control_lock:
            active = self._goal_active
        if active and not self._goal_idle.wait(timeout=self._goal_end_timeout()):
            self.get_logger().error(self._stage() + 'Running goal did not end in time.')
        with self._control_lock:
            state = self._state
        if state is S.IN_POSITION:
            self._abandon_in_position(msg, restore_first=True)
        else:
            self._restore_external_state()
        # give the lidar safety message time to go out before the node is destroyed
        time.sleep(0.2)

    # ---- control ------------------------------------------------------------------

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
        with self._control_lock:
            if self._state not in CONTROL_ACTIVE_STATES:
                return

        self._last_target_time, lost = handle_target_lost(
            now=time.monotonic(),
            last_target_time=self._last_target_time,
            lost_target_timeout=self.lost_target_timeout,
            stopped_on_target_loss=False,
            safe_stop_fn=self._safe_stop,
            logger=self._stage_logger,
        )
        if lost:
            self._finish_motion(
                False, f'AprilTag lost for {self.lost_target_timeout:.2f}s. Tracking stopped.')

    # callback to convert tag pose into control errors and publish cmd_vel
    def pose_callback(self, msg: PoseStamped):
        with self._control_lock:
            tracking_state = self._state
        if tracking_state not in CONTROL_ACTIVE_STATES:
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
                logger=self._stage_logger,
            )
        except Exception as exc:
            self.get_logger().warn(self._stage() + f"Control publish failed: {exc}")
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

        converged = (
            f'Error converged within tol: '
            f'x_error={state.x_error:.6f} (<= {self.stop_x_error_tolerance:.3f}), '
            f'y_error={state.y_error:.6f} (<= {self.stop_y_error_tolerance:.3f}), '
            f'yaw_error={state.yaw_error:.6f} (<= {self.stop_yaw_error_tolerance:.3f}), '
            f'held_for={hold_time:.2f}s (>= {self.stop_hold_seconds:.2f}s).'
        )
        if tracking_state is S.STAGE1:
            self._publish_stop_command()
            with self._control_lock:
                if self._state is not S.STAGE1:
                    return
                self._stage2_pending = True
                self._error_zero_since = None
            self.get_logger().info(
                self._stage() + f'Stage 1 aligned at {self.stage1_distance:.2f}m. '
                f'Stopped; starting Stage 2'
                + (' after disabling lidar safety.' if self.manage_amcl_and_lidar_safety else '.')
            )
        elif tracking_state is S.STAGE2:
            self._reach_in_position(f'Stage 2 aligned at {self.stage2_distance:.2f}m. {converged}')
        else:
            self._finish_motion(True, f'Leave aligned at {self.leave_distance:.2f}m. {converged}')


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
