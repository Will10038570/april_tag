import signal
import threading
import time
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from apriltag_interfaces.action import StartTracking
from apriltag_interfaces.msg import TagPoseArray
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger

from apriltag.control import LQRTracker, TrajectoryPlanner
from apriltag.domain.math_utils import optical_to_control_error, quaternion_to_rotation_matrix
from apriltag.ros.ros_io import publish_twist
from apriltag.runtime.control_flow import leave_step, publish_control
from apriltag.runtime.safety_guard import handle_target_lost
from apriltag.runtime.target_flow import ANY_TAG_ID, choose_best_target

# high-rate messages (cmd_vel, action feedback) are logged at most once per this period
LOG_THROTTLE_SEC = 1.0


class TrackingState(Enum):
    IDLE = 'IDLE'        # nothing running; AMCL / lidar safety as the last procedure left them
    STAGE1 = 'STAGE1'    # enable lidar safety, close AMCL; track stage1_distance on /cmd_vel
    STAGE2 = 'STAGE2'    # disable lidar safety; track stage2_distance on /pre_cmd_vel
    LEAVING = 'LEAVING'  # disable lidar safety, close AMCL; back straight to
                         # leave_distance; restore AMCL / lidar safety


class Step(Enum):
    """Step inside a non-IDLE state; not a state of its own."""
    PREPARING = 'preparing'  # switching external state, no motion, no target-loss check
    RUNNING = 'running'      # control loop active
    ENDING = 'ending'        # stopped; the execute thread cleans up, then IDLE


S = TrackingState
ALLOWED_TRANSITIONS = {
    S.IDLE: {S.STAGE1, S.LEAVING},
    S.STAGE1: {S.STAGE2, S.IDLE},
    S.STAGE2: {S.IDLE},
    S.LEAVING: {S.IDLE},
}
# states with a running goal; velocity is published only in their RUNNING step
ACTIVE_STATES = frozenset({S.STAGE1, S.STAGE2, S.LEAVING})
# (action, start) -> states in which that goal is accepted
GOAL_RULES = {
    ('start_tracking', True): frozenset({S.IDLE}),
    ('start_tracking', False): frozenset({S.STAGE1, S.STAGE2}),
    ('leave_cs', True): frozenset({S.IDLE}),
    ('leave_cs', False): frozenset({S.LEAVING}),
}


def is_transition_allowed(src: TrackingState, dst: TrackingState) -> bool:
    return dst in ALLOWED_TRANSITIONS.get(src, set())


def is_goal_accepted(action: str, start: bool, state: TrackingState) -> bool:
    return state in GOAL_RULES.get((action, bool(start)), frozenset())


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

        IDLE -> STAGE1 -> STAGE2 -> IDLE        (start_tracking)
        IDLE -> LEAVING -> IDLE                 (leave_cs)

    Each non-IDLE state runs the steps PREPARING -> RUNNING -> ENDING (Step)
    and switches the external state it needs itself. A start_tracking goal
    enters STAGE1: it enables lidar safety and closes AMCL, then tracks the
    selected tag from /apriltag_poses with TrajectoryPlanner + LQR on
    /cmd_vel. When Stage 1 converges the robot stops and STAGE2
    disables the PLC lidar safety field, then continues on /pre_cmd_vel (G7+
    precision mode, the motors wait until the steering is within 5 deg). When
    Stage 2 is aligned the robot stops and the node returns to IDLE with AMCL closed and lidar safety disabled; the goal then
    succeeds. A leave_cs goal (also accepted without a previous alignment)
    enters LEAVING: it disables lidar safety, closes AMCL (both are usually
    already so after Stage 2), then backs straight up at max_vx on
    /pre_cmd_vel, without the planner / LQR, until the tag is at least
    leave_distance ahead of the camera; then it stops, enables lidar safety and makes sure AMCL is open before IDLE. A failure,
    cancel, stop or target loss in any state ends the same way as LEAVING:
    stop, restore AMCL / lidar safety, IDLE. Node shutdown
    restores them too. With manage_amcl_and_lidar_safety set to false it skips
    all AMCL and lidar safety calls.

    apriltag_detection runs on its own and publishes every detected tag on
    /apriltag_poses (TagPoseArray). This node tracks the closest tag (smallest
    positive forward z) of family tag_family with id tag_id ("-1": any id);
    other tags are ignored.

    AMCL is switched through external services whose responses are checked.
    Lidar safety follows the G7+ AutoCharging way: a Bool is published on the
    PLC bridge topic with no read-back, so it cannot be confirmed here. What
    the providers actually do with a request is outside this package.
    """

    def __init__(self):
        super().__init__('apriltag_control')

        self.tags_sub = self.create_subscription(TagPoseArray,
                                                 'apriltag_poses',
                                                 self.tags_callback,
                                                 1)

        ## tracked tag: detection publishes all tags, this node picks one
        self.tag_family = str(self.declare_parameter('tag_family', 'tag36h11').value)
        # decimal string as in TagPose.id; '-1' tracks any id of tag_family
        self.tag_id = str(self.declare_parameter('tag_id', '0').value)

        ## AprilTag distance parameters
        # two-stage desired forward distances to tag (m), ROS parameters
        self.stage1_distance = float(self.declare_parameter(
            'stage1_distance', 0.50).value)   # 第一階段：50 cm 對齊
        self.stage2_distance = float(self.declare_parameter(
            'stage2_distance', 0.28).value)   # 第二階段：28 cm 對齊
        # leave_cs: back straight until the camera-to-tag forward distance
        # reaches this value
        self.leave_distance = float(self.declare_parameter(
            'leave_distance', 1.0).value)     # 離開：退到 100 cm
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
        # guards _state, _step and the flags below
        self._control_lock = threading.Lock()
        self._state = TrackingState.IDLE
        self._step: Optional[Step] = None  # None in IDLE
        # set while in IDLE: stop goals and shutdown wait on it
        self._idle = threading.Event()
        self._idle.set()
        self._stage_logger = _StageLogger(self)
        self._error_zero_since = None
        self.stop_hold_seconds = 1.0
        self.stop_x_error_tolerance = 0.05
        self.stop_y_error_tolerance = 0.05
        self.stop_yaw_error_tolerance = 0.05
        # set when the running goal has a result (its state is in ENDING)
        self._tracking_done = threading.Event()
        self._tracking_result: dict = {}
        self._latest_best_target: dict = {}
        self._tracking_start_time: Optional[float] = None

        # tags_callback and the lost-target timer share the default
        # (mutually exclusive) group, so they never run concurrently.
        self.lost_check_timer = self.create_timer(self.lost_check_period, self._check_target_lost)

        # service clients get their own group so responses can be handled
        # while the action execute thread is waiting on them
        self._client_cb_group = MutuallyExclusiveCallbackGroup()

        ## external services (AMCL, PLC lidar safety)
        # False: never call the AMCL / lidar safety services, so tracking is
        # not blocked by them; True: switch AMCL and lidar safety as below
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
            goal_callback=lambda goal: self._goal_callback('start_tracking', goal),
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )
        self.leave_cs_action_server = ActionServer(
            self,
            StartTracking,
            'leave_cs',
            execute_callback=self._execute_leave_cs,
            goal_callback=lambda goal: self._goal_callback('leave_cs', goal),
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )
        self.get_logger().info(
            self._stage() + 'Tracking tag: family=' + self.tag_family + ', id='
            + ('any' if self.tag_id == ANY_TAG_ID else self.tag_id)
        )
        self.get_logger().info(
            self._stage() + f'State {self._state.value}. Send the start_tracking or leave_cs action to start.'
        )

    # ---- state machine ----------------------------------------------------------

    def _stage(self) -> str:
        """Prefix of every log message: '[stage LEAVING] : '.

        The launch file sets this node's RCUTILS_CONSOLE_OUTPUT_FORMAT to
        '[{severity}] [{name}]{message}', so a line reads
        '[INFO] [up.apriltag_control][stage LEAVING] : ...'.
        """
        return f'[stage {self._state.value}] : '

    def _transition_locked(self, new: TrackingState, step: Optional[Step], reason: str) -> bool:
        """Change state (and its step) if allowed; the caller must hold _control_lock."""
        old = self._state
        if not is_transition_allowed(old, new):
            self.get_logger().error(
                self._stage() + f'[state] invalid transition {old.value} -> {new.value} ({reason}), ignored.')
            return False
        self._state = new
        self._step = None if new is S.IDLE else step
        if new is S.IDLE:
            self._idle.set()
        else:
            self._idle.clear()
        self.get_logger().info(self._stage() + f'[state] {old.value} -> {new.value} ({reason})')
        return True

    def _set_step_locked(self, step: Step, reason: str) -> None:
        """Change the step inside the current state; the caller must hold _control_lock."""
        old = self._step
        self._step = step
        self.get_logger().info(
            self._stage() + f'[step] {self._state.value} {old.value if old else "-"} -> {step.value} ({reason})')

    def _reset_control_locked(self, desired_distance: float) -> None:
        """Reset the controller for a new tracking phase; the caller must hold _control_lock."""
        self.desired_distance = desired_distance
        self._error_zero_since = None
        self._last_stamp = None
        self._last_target_time = None
        self._latest_best_target = {}
        self.trajectory_planner.reset()
        self.lqr_tracker.reset()

    def _begin_running_locked(self, desired_distance: float, reason: str) -> None:
        """PREPARING -> RUNNING with a fresh controller; the caller must hold _control_lock."""
        # _last_target_time is reset too, so the watchdog starts counting now
        self._reset_control_locked(desired_distance)
        self._set_step_locked(Step.RUNNING, reason)

    def _is_running_locked(self) -> bool:
        return self._state in ACTIVE_STATES and self._step is Step.RUNNING

    def _elapsed_message(self, message: str) -> str:
        elapsed = (
            time.monotonic() - self._tracking_start_time
            if self._tracking_start_time is not None
            else float('nan')
        )
        return f'{message} elapsed={elapsed:.2f}s'

    def _request_end(self, success: bool, message: str, canceled: bool = False,
                     only_in: frozenset = ACTIVE_STATES) -> bool:
        """Move the running procedure to its ENDING step, stop and record its result.

        Used for Stage 2 aligned, leave reached, target lost, cancel, stop,
        service failure and shutdown; the goal's execute thread then cleans
        up. Returns False if the state is not in only_in or already ending.
        """
        timed_message = self._elapsed_message(message)
        with self._control_lock:
            if self._state not in only_in or self._step is Step.ENDING:
                return False
            self._set_step_locked(Step.ENDING, message.split('. ')[0].rstrip('.'))
            self._error_zero_since = None
            self._last_target_time = None
            self._tracking_result = {'success': success, 'canceled': canceled, 'message': timed_message}
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

    # ---- velocity output ------------------------------------------------------

    def _publish_zero_on_all(self, label: str) -> None:
        # stops go to both topics, so the robot stops whichever stage is active
        for pub in (self.stage1_cmd_pub, self.stage2_cmd_pub):
            publish_twist(pub, 0.0, 0.0, 0.0)
            self.get_logger().info(self._stage() + f'[publish] {pub.topic_name} {label} vx=0 vy=0 wz=0')

    def _publish_stop_command(self) -> None:
        self._publish_zero_on_all('stop')

    def _publish_control_command(self, vx: float, vy: float, vw: float) -> None:
        # published under the lock, so no command goes out after the step
        # leaves RUNNING (whose stop is published after it)
        with self._control_lock:
            if not self._is_running_locked():
                return
            pub = self.stage1_cmd_pub if self._state is S.STAGE1 else self.stage2_cmd_pub
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
            return True

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
        """Enable lidar safety, then make sure AMCL is open (whoever closed it)."""
        if not self.manage_amcl_and_lidar_safety:
            return True
        with self._restore_lock:
            self._set_lidar_safety_disabled(False)
            if not self._set_amcl_enabled(True):
                self.get_logger().error(self._stage() + 'Restore failed: AMCL may still be closed.')
                return False
            return True

    def _end_tracking(self) -> bool:
        """Stop and restore external state; return False if the restore failed."""
        self._publish_stop_command()
        return self._restore_external_state()

    def _end_goal(self, restore: bool) -> bool:
        """ENDING step of the goal's state: clean up, then IDLE; return False if anything failed.

        restore=False (Stage 2 aligned) only stops, so AMCL stays closed and
        lidar safety disabled.
        """
        if restore:
            ok = self._end_tracking()
        else:
            self._publish_stop_command()
            ok = True
        with self._control_lock:
            self._reset_control_locked(self.stage1_distance)
            self._transition_locked(S.IDLE, None, 'cleanup done' if ok else 'cleanup done, with failures')
        return ok

    def _goal_end_timeout(self) -> float:
        # a goal may still be preparing (AMCL check/close/check) and then ends
        # (AMCL check/open/check); each call waits for the service and then
        # for the response
        return 2.0 * 6 * self.external_service_timeout

    # ---- action servers ---------------------------------------------------------

    def _reject_goal(self, action: str, start: bool, state: TrackingState):
        allowed = ', '.join(sorted(s.value for s in GOAL_RULES[(action, start)]))
        self.get_logger().warn(
            self._stage() + f'[action] {action} goal rejected: start={start} state={state.value} '
            f'(start={start} is only accepted in {allowed})')
        return GoalResponse.REJECT

    def _goal_callback(self, action: str, goal_request: StartTracking.Goal):
        start = bool(goal_request.start)
        self.get_logger().info(self._stage() + f'[action] {action} goal received: start={start}')
        with self._control_lock:
            state = self._state
            # decided under the lock, so of two start=True goals only one leaves IDLE
            if not is_goal_accepted(action, start, state):
                return self._reject_goal(action, start, state)
            if start:
                self._tracking_done.clear()
                self._tracking_result = {}
                self._tracking_start_time = time.monotonic()
                if action == 'start_tracking':
                    self._transition_locked(S.STAGE1, Step.PREPARING, 'start_tracking goal accepted')
                else:
                    self._transition_locked(
                        S.LEAVING, Step.PREPARING,
                        f'leave_cs goal accepted, back to {self.leave_distance:.2f}m')
        self.get_logger().info(self._stage() + f'[action] {action} goal accepted: start={start}')
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
        if res.get('canceled', False):
            return 'canceled', res.get('message', '')
        return ('succeed' if res.get('success', False) else 'abort'), res.get('message', '')

    def _run_goal(self, goal_handle, action: str, prepare, cancel_message: str, ended_message: str,
                  restore_on_success: bool):
        """Run one start=True goal: prepare, track, then clean up to IDLE; return its result."""
        try:
            if prepare(goal_handle):
                self._wait_motion_done(goal_handle, cancel_message)
        finally:
            # no-op if already ending; covers an exception above
            self._request_end(False, ended_message)
            outcome, message = self._goal_outcome()
            restore = restore_on_success or outcome != 'succeed'
            if not self._end_goal(restore=restore):
                message += ' Restore failed, see apriltag_control log.'
        # the result goes out in IDLE, so the next goal can be sent right away
        return self._finish_goal_handle(goal_handle, action, outcome, message)

    def _execute_start_tracking(self, goal_handle):
        if not goal_handle.request.start:
            return self._handle_stop_goal(goal_handle, 'start_tracking', 'Tracking stopped.')
        return self._run_goal(goal_handle, 'start_tracking', self._prepare_stage1,
                              'Tracking cancelled.', 'Tracking ended unexpectedly.',
                              restore_on_success=False)

    def _execute_leave_cs(self, goal_handle):
        if not goal_handle.request.start:
            return self._handle_stop_goal(goal_handle, 'leave_cs', 'Leave stopped.')
        return self._run_goal(goal_handle, 'leave_cs', self._prepare_leaving,
                              'Leave cancelled.', 'Leave ended unexpectedly.',
                              restore_on_success=True)

    def _stop_requested(self, goal_handle, cancel_message: str) -> bool:
        """True if the goal was cancelled (ended here) or already ended (stop / shutdown)."""
        if goal_handle.is_cancel_requested:
            self._request_end(False, cancel_message, canceled=True)
            return True
        return self._tracking_done.is_set()

    def _prepare_stage1(self, goal_handle) -> bool:
        """STAGE1 PREPARING: enable lidar safety, close AMCL; False if ended."""
        if self.manage_amcl_and_lidar_safety:
            # Stage 1 always runs with lidar safety on, also right after a Stage 2
            self._set_lidar_safety_disabled(False)
            if not self._set_amcl_enabled(False):
                self._request_end(False, 'Failed to close AMCL.')
                return False
        if self._stop_requested(goal_handle, 'Tracking cancelled.'):
            return False
        with self._control_lock:
            if not (self._state is S.STAGE1 and self._step is Step.PREPARING):
                return False
            self._tracking_start_time = time.monotonic()
            self._begin_running_locked(
                self.stage1_distance,
                'lidar safety enabled, AMCL closed'
                if self.manage_amcl_and_lidar_safety else 'AMCL / lidar safety unmanaged')
        self.get_logger().info(
            self._stage() + f'Stage 1 ({self.stage1_distance:.2f}m) started. Tracking target...')
        return True

    def _prepare_stage2(self) -> None:
        """STAGE2 PREPARING: disable lidar safety while stopped, then track Stage 2."""
        if self.manage_amcl_and_lidar_safety:
            self._set_lidar_safety_disabled(True)
        with self._control_lock:
            # target lost / cancelled in the meantime
            if not (self._state is S.STAGE2 and self._step is Step.PREPARING):
                return
            self._begin_running_locked(
                self.stage2_distance,
                'lidar safety disabled' if self.manage_amcl_and_lidar_safety else 'lidar safety unmanaged')
        self.get_logger().info(self._stage() + f'Advancing to Stage 2 ({self.stage2_distance:.2f}m).')

    def _prepare_leaving(self, goal_handle) -> bool:
        """LEAVING PREPARING: disable lidar safety, close AMCL; False if ended."""
        if self.manage_amcl_and_lidar_safety:
            # leaving starts next to the station, also without a previous
            # alignment: lidar safety off and AMCL closed, as in Stage 2
            self._set_lidar_safety_disabled(True)
            if not self._set_amcl_enabled(False):
                self._request_end(False, 'Failed to close AMCL.')
                return False
        if self._stop_requested(goal_handle, 'Leave cancelled.'):
            return False
        with self._control_lock:
            if not (self._state is S.LEAVING and self._step is Step.PREPARING):
                return False
            self._tracking_start_time = time.monotonic()
            self._begin_running_locked(
                self.leave_distance,
                'lidar safety disabled, AMCL closed'
                if self.manage_amcl_and_lidar_safety else 'AMCL / lidar safety unmanaged')
        self.get_logger().info(
            self._stage() + f'Leaving started: backing up to {self.leave_distance:.2f}m.')
        return True

    def _wait_motion_done(self, goal_handle, cancel_message: str) -> None:
        """Wait until the running tracking / leaving has a result (ENDING)."""
        feedback = StartTracking.Feedback()
        while not self._tracking_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                self._request_end(False, cancel_message, canceled=True)
                break  # if it ended meanwhile, that result is reported

            with self._control_lock:
                stage2_preparing = self._state is S.STAGE2 and self._step is Step.PREPARING
                state, step = self._state, self._step
            if stage2_preparing:
                self._prepare_stage2()

            t = self._latest_best_target
            feedback.status = (
                f"tracking: x_err={t.get('x_error', float('nan')):.3f} "
                f"y_err={t.get('y_error', float('nan')):.3f} "
                f"yaw_err={t.get('yaw_error', float('nan')):.3f} "
                f"state={state.value} step={step.value if step else '-'} "
                f"amcl={self._amcl_state} lidar_safety={self._lidar_safety_state}"
            )
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(self._stage() + f'[action] feedback: {feedback.status}',
                                   throttle_duration_sec=LOG_THROTTLE_SEC)

    def _handle_stop_goal(self, goal_handle, action: str, message: str):
        """start=False: stop that action's procedure and wait until it is back in IDLE."""
        only_in = GOAL_RULES[(action, False)]
        self._request_end(False, message, only_in=only_in)
        # wait for the cleanup (also if it was already ending); not for a
        # procedure of the other action started in the meantime
        with self._control_lock:
            waiting = self._state in only_in
        if waiting and not self._idle.wait(timeout=self._goal_end_timeout()):
            self.get_logger().error(self._stage() + 'Running goal did not end in time.')

        result = StartTracking.Result()
        result.success = True
        result.message = message
        goal_handle.succeed()
        self._log_result(action, 'SUCCEEDED (start=False)', result)
        return result

    def shutdown(self) -> None:
        """End a running procedure and restore external state before the node is destroyed."""
        self._request_end(False, 'Node shutting down.')
        idle = self._idle.wait(timeout=self._goal_end_timeout())
        if not idle:
            self.get_logger().error(self._stage() + 'Running goal did not end in time.')
        # always, since IDLE after Stage 2 keeps AMCL closed and lidar safety disabled
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

    # watchdog: tags_callback ignores arrays without the tracked tag (and
    # detection may stop publishing), so target loss is checked on a timer
    def _check_target_lost(self):
        with self._control_lock:
            if not self._is_running_locked():
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
            self._request_end(
                False, f'AprilTag lost for {self.lost_target_timeout:.2f}s. Tracking stopped.')

    # callback to pick the tracked tag, convert its pose into control errors
    # and publish cmd_vel
    def tags_callback(self, msg: TagPoseArray):
        with self._control_lock:
            if not self._is_running_locked():
                return
            tracking_state = self._state

        tag = choose_best_target(msg.tags, self.tag_family, self.tag_id)
        if tag is None:
            return

        now = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        p = tag.pose.position
        q = tag.pose.orientation
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

        if tracking_state is S.LEAVING:
            self._leave_step(state.x_error)
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
                if not (self._state is S.STAGE1 and self._step is Step.RUNNING):
                    return
                self._error_zero_since = None
                # the execute thread disables lidar safety, then Stage 2 runs
                self._transition_locked(S.STAGE2, Step.PREPARING, 'Stage 1 aligned')
            self.get_logger().info(
                self._stage() + f'Stage 1 aligned at {self.stage1_distance:.2f}m. '
                f'Stopped; starting Stage 2'
                + (' after disabling lidar safety.' if self.manage_amcl_and_lidar_safety else '.')
            )
        elif tracking_state is S.STAGE2:
            # back to IDLE without restoring: AMCL stays closed, lidar safety disabled
            self._request_end(
                True, f'Stage 2 aligned at {self.stage2_distance:.2f}m. {converged} '
                + ('AMCL off, lidar safety disabled; send leave_cs to restore.'
                   if self.manage_amcl_and_lidar_safety else 'AMCL / lidar safety unmanaged.'),
                only_in=frozenset({S.STAGE2}))

    def _leave_step(self, x_error: float) -> None:
        """LEAVING: back straight at max_vx until the tag is leave_distance away."""
        try:
            reached, vx, vy, vw = leave_step(x_error, self.max_vx)
        except ValueError as exc:
            self.get_logger().warn(self._stage() + f"Leave step failed: {exc}")
            self._safe_stop()
            return
        if reached:
            forward = x_error + self.leave_distance
            self._request_end(
                True, f'Leave reached {self.leave_distance:.2f}m (forward={forward:.3f}m).')
            return
        self._publish_control_command(vx, vy, vw)


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
