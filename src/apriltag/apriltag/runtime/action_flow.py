"""Goal checks, feedback stage and result outcome of the dock / undock actions.

Pure helpers (no rclpy), shared by apriltag_control and the tests.
"""

import math
from typing import Optional, Tuple

from apriltag_interfaces.action import DockWheelchair, UndockWheelchair

from apriltag.runtime.target_flow import ANY_TAG_ID

DOCK = 'dock'
UNDOCK = 'undock'

# both actions define the same error codes; Dock's are used for both
Codes = DockWheelchair.Result


def validate_goal(tag_id: int, tag_family: str,
                  distance_m: Optional[float] = None) -> Optional[str]:
    """Return None if the goal is valid, else the reason it is rejected.

    Only the format is checked: a family / id that apriltag_detection does not
    track is accepted and ends with TAG_LOST. distance_m None (dock) is not
    checked; given (undock) it must be finite and > 0.
    """
    if int(tag_id) < -1:
        return f'tag_id {int(tag_id)} must be >= 0, or -1 for any id'
    if not str(tag_family).strip():
        return 'tag_family is empty'
    if distance_m is not None and not (math.isfinite(distance_m) and distance_m > 0.0):
        return f'distance_m {distance_m} must be > 0'
    return None


def tag_id_filter(tag_id: int) -> str:
    """tag_id as the decimal string TagPose.id uses ('-1': any id)."""
    return ANY_TAG_ID if int(tag_id) == -1 else str(int(tag_id))


def feedback_stage(action: str, state: str, step: Optional[str]) -> int:
    """Feedback stage of the running goal from TrackingState / Step values."""
    if action == DOCK:
        return {
            'STAGE1': DockWheelchair.Feedback.STAGE_1,
            'STAGE2': DockWheelchair.Feedback.STAGE_2,
        }.get(state, DockWheelchair.Feedback.STAGE_IDLE)
    return {
        'preparing': UndockWheelchair.Feedback.STAGE_PREPARING,
        'running': UndockWheelchair.Feedback.STAGE_LEAVING,
    }.get(step, UndockWheelchair.Feedback.STAGE_FINISHING)


def final_outcome(outcome: str, error_code: int, restore_ok: bool) -> Tuple[str, int]:
    """Outcome ('succeed' / 'abort' / 'canceled') and error code after the cleanup.

    A failed restore always reports AMCL_SERVICE_FAIL; a succeeded goal then
    aborts, a cancelled one stays cancelled.
    """
    if restore_ok:
        return outcome, error_code
    if outcome == 'succeed':
        return 'abort', Codes.AMCL_SERVICE_FAIL
    return outcome, Codes.AMCL_SERVICE_FAIL
