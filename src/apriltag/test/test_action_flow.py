"""Goal checks, feedback stage and outcome of the dock / undock actions."""

import pytest
from apriltag_interfaces.action import DockWheelchair, UndockWheelchair

from apriltag.runtime.action_flow import (
    DOCK,
    UNDOCK,
    feedback_stage,
    final_outcome,
    tag_id_filter,
    validate_goal,
)

CODE_NAMES = ['OK', 'TAG_LOST', 'AMCL_SERVICE_FAIL', 'LIDAR_SERVICE_FAIL', 'TIMEOUT',
              'ACTION_CANCELED', 'OTHER', 'NODE_SHUTDOWN']
D = DockWheelchair.Result
DF = DockWheelchair.Feedback
UF = UndockWheelchair.Feedback


# ---- goal checks ------------------------------------------------------------------

@pytest.mark.parametrize('tag_id', [0, 3, -1])
def test_valid_dock_goal(tag_id):
    assert validate_goal(tag_id, 'tag36h11') is None


def test_valid_undock_goal():
    assert validate_goal(0, 'tag36h11', 1.0) is None


@pytest.mark.parametrize('tag_id, family, distance', [
    (-2, 'tag36h11', None),
    (0, '', None),
    (0, '   ', None),
    (0, 'tag36h11', 0.0),
    (0, 'tag36h11', -1.0),
    (0, 'tag36h11', float('nan')),
    (0, 'tag36h11', float('inf')),
])
def test_invalid_goal(tag_id, family, distance):
    assert validate_goal(tag_id, family, distance)


def test_family_not_checked_against_detection():
    # a goal for a family / id detection does not track is accepted; it ends
    # with TAG_LOST instead
    assert validate_goal(0, 'tag25h9') is None


def test_dock_does_not_check_distance():
    assert validate_goal(0, 'tag36h11', None) is None


def test_tag_id_filter():
    assert tag_id_filter(-1) == '-1'
    assert tag_id_filter(0) == '0'
    assert tag_id_filter(12) == '12'


# ---- feedback stage ---------------------------------------------------------------

@pytest.mark.parametrize('state, step, expected', [
    ('STAGE1', 'preparing', DF.STAGE_1),
    ('STAGE1', 'running', DF.STAGE_1),
    ('STAGE2', 'running', DF.STAGE_2),
    ('STAGE2', 'ending', DF.STAGE_2),
    ('IDLE', None, DF.STAGE_IDLE),
])
def test_dock_stage(state, step, expected):
    assert feedback_stage(DOCK, state, step) == expected


@pytest.mark.parametrize('state, step, expected', [
    ('LEAVING', 'preparing', UF.STAGE_PREPARING),
    ('LEAVING', 'running', UF.STAGE_LEAVING),
    ('LEAVING', 'ending', UF.STAGE_FINISHING),
    ('IDLE', None, UF.STAGE_FINISHING),
])
def test_undock_stage(state, step, expected):
    assert feedback_stage(UNDOCK, state, step) == expected


# ---- outcome ----------------------------------------------------------------------

@pytest.mark.parametrize('outcome, code', [
    ('succeed', D.OK), ('abort', D.TAG_LOST), ('canceled', D.ACTION_CANCELED)])
def test_restore_ok_keeps_outcome(outcome, code):
    assert final_outcome(outcome, code, True) == (outcome, code)


@pytest.mark.parametrize('outcome, code, expected', [
    ('succeed', D.OK, ('abort', D.AMCL_SERVICE_FAIL)),
    ('abort', D.TAG_LOST, ('abort', D.AMCL_SERVICE_FAIL)),
    ('canceled', D.ACTION_CANCELED, ('canceled', D.AMCL_SERVICE_FAIL)),
])
def test_restore_failed_reports_amcl(outcome, code, expected):
    assert final_outcome(outcome, code, False) == expected


def test_error_codes_match_between_actions():
    for value, name in enumerate(CODE_NAMES):
        assert getattr(DockWheelchair.Result, name) == value
        assert getattr(UndockWheelchair.Result, name) == value
