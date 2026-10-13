#!/usr/bin/env python3
"""Fake client for apriltag_control's /up/undock_wheelchair action.

    ros2 run apriltag wheelchair_undocking --goal     send an undock goal, print feedback and the result
    ros2 run apriltag wheelchair_undocking --cancel   cancel every undock goal on the server
                                                      (works from another terminal)

(or python3 tools/WheelChairUndocking.py --goal / --cancel without building)

Goal parameters are the constants below. Ctrl+C during --goal also cancels
that goal (CANCELED, error_code=5; AMCL / lidar safety restored) and waits
for the result.

Run it inside the april_tag_serive container after sourcing ROS and the
workspace (bash -ic does that). It uses the shell's ROS_DOMAIN_ID /
CYCLONEDDS_URI, e.g. ROS_DOMAIN_ID=99 for the virtual test.
"""

import argparse
import sys
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from apriltag_interfaces.action import UndockWheelchair
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# ---- parameters -------------------------------------------------------------
ACTION_NAME = '/up/undock_wheelchair'
TAG_ID = 0                    # -1 = any id; must match the tag apriltag_detection tracks
TAG_FAMILY = 'tag36h11'       # must match apriltag_detection's tracking_tag_family, else TAG_LOST
DISTANCE_M = 1.0              # m, back up until the tag is this far ahead of the camera; > 0
SERVER_WAIT_S = 5.0           # wait this long for the action server
FEEDBACK_PRINT_EVERY = 10     # print 1 of every N feedback messages (10 Hz -> 1 Hz)
# -----------------------------------------------------------------------------

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}
STAGE_NAMES = {
    UndockWheelchair.Feedback.STAGE_PREPARING: 'PREPARING',
    UndockWheelchair.Feedback.STAGE_LEAVING: 'LEAVING',
    UndockWheelchair.Feedback.STAGE_FINISHING: 'FINISHING',
}
CANCEL_RETURN_CODES = {
    CancelGoal.Response.ERROR_NONE: 'ERROR_NONE',
    CancelGoal.Response.ERROR_REJECTED: 'ERROR_REJECTED',
    CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID: 'ERROR_UNKNOWN_GOAL_ID',
    CancelGoal.Response.ERROR_GOAL_TERMINATED: 'ERROR_GOAL_TERMINATED',
}


class WheelChairUndocking(Node):
    def __init__(self):
        super().__init__('wheelchair_undocking_client')
        self.action_client = ActionClient(self, UndockWheelchair, ACTION_NAME)
        # zero goal id + zero stamp = cancel all goals, so --cancel does not
        # need the goal id of the process that sent the goal
        self.cancel_client = self.create_client(
            CancelGoal, ACTION_NAME + '/_action/cancel_goal')

    def send_goal(self) -> int:
        if not self.action_client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            print(f'action server {ACTION_NAME} not available')
            return 1

        goal = UndockWheelchair.Goal(tag_id=TAG_ID, tag_family=TAG_FAMILY, distance_m=DISTANCE_M)
        print(f'{ACTION_NAME} goal: tag_id={goal.tag_id} tag_family={goal.tag_family} '
              f'distance_m={goal.distance_m:.2f}')
        future = self.action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('goal rejected (see apriltag_control log for the reason)')
            return 1
        print('goal accepted (Ctrl+C to cancel)')

        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            print('\nCtrl+C: canceling goal ...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()
        result = response.result
        print(f'result: {STATUS_NAMES.get(response.status, response.status)} '
              f'success={result.success} error_code={result.error_code} '
              f'message={result.message!r}')
        return 0 if result.success else 1

    def cancel_all(self) -> int:
        if not self.cancel_client.wait_for_service(timeout_sec=SERVER_WAIT_S):
            print(f'action server {ACTION_NAME} not available')
            return 1
        future = self.cancel_client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        code = CANCEL_RETURN_CODES.get(response.return_code, response.return_code)
        print(f'cancel {ACTION_NAME}: {code}, {len(response.goals_canceling)} goal(s) canceling')
        return 0 if response.goals_canceling else 1

    def _on_feedback(self, msg):
        fb = msg.feedback
        if fb.seq % FEEDBACK_PRINT_EVERY:
            return
        print(f'feedback: seq={fb.seq} stamp={_clock_str(fb.stamp)} stage={STAGE_NAMES.get(fb.stage, fb.stage)} '
              f'x_error={fb.x_error:+.3f} y_shift={fb.y_shift:+.3f} yaw_shift={fb.yaw_shift:+.3f} '
              f'last_frame_age={fb.last_frame_age:.3f}s')


def _clock_str(stamp) -> str:
    """builtin_interfaces/Time -> local wall clock 'HH:MM:SS.mmm'."""
    t = datetime.fromtimestamp(stamp.sec + stamp.nanosec * 1e-9)
    return t.strftime('%H:%M:%S.') + f'{t.microsecond // 1000:03d}'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--goal', action='store_true', help='send an undock goal and wait for the result')
    mode.add_argument('--cancel', action='store_true', help='cancel every undock goal on the server')
    args = parser.parse_args()

    # keep the context alive on Ctrl+C so the goal can still be canceled
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WheelChairUndocking()
    try:
        return node.send_goal() if args.goal else node.cancel_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
