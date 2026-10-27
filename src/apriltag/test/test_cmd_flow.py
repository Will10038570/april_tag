"""Status JSON, parameter checks and outcome of the wheelchair interface."""

import json
import math

import pytest

from apriltag.runtime.cmd_flow import (
    SUCCESS_MSG,
    Codes as D,
    final_outcome,
    status_json,
    tag_id_filter,
    validate_goal,
)

CODE_NAMES = ['OK', 'TAG_LOST', 'AMCL_SERVICE_FAIL', 'LIDAR_SERVICE_FAIL', 'TIMEOUT',
              'OTHER', 'NODE_SHUTDOWN']
STATUS_KEYS = ['state', 'cmd_id', 'attached', 'error_code', 'error_msg', 'stamp',
               'x_error', 'y_error', 'yaw_error', 'last_frame_age']


# ---- status JSON ------------------------------------------------------------------

def test_status_json():
    data = json.loads(status_json('STAGE1', 17, D.OK, '', 1791355707.1, 0.1, -0.02, 0.03, 0.05))
    assert list(data) == STATUS_KEYS
    assert data['state'] == 'STAGE1' and data['cmd_id'] == 17 and data['attached'] is False
    assert data['x_error'] == pytest.approx(0.1)


def test_status_json_unknown_is_null():
    text = status_json('IDLE', 0, D.OK, '', 1.0, math.nan, math.inf, math.nan, math.nan)
    assert 'NaN' not in text and 'Infinity' not in text
    data = json.loads(text)
    assert data['x_error'] is None and data['y_error'] is None and data['last_frame_age'] is None


# ---- tag / leave parameter checks ---------------------------------------------

@pytest.mark.parametrize('tag_id', [0, 3, -1])
def test_valid_tag_params(tag_id):
    assert validate_goal(tag_id, 'tag36h11') is None


def test_valid_leave_params():
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
def test_invalid_params(tag_id, family, distance):
    assert validate_goal(tag_id, family, distance)


def test_family_not_checked_against_detection():
    # a family / id detection does not track is accepted; the cmd ends
    # with TAG_LOST instead
    assert validate_goal(0, 'tag25h9') is None


def test_distance_not_given_is_not_checked():
    assert validate_goal(0, 'tag36h11', None) is None


def test_tag_id_filter():
    assert tag_id_filter(-1) == '-1'
    assert tag_id_filter(0) == '0'
    assert tag_id_filter(12) == '12'


# ---- outcome ----------------------------------------------------------------------

@pytest.mark.parametrize('outcome, code', [
    ('succeed', D.OK), ('abort', D.TAG_LOST), ('canceled', D.OK)])
def test_restore_ok_keeps_outcome(outcome, code):
    assert final_outcome(outcome, code, True) == (outcome, code)


@pytest.mark.parametrize('outcome, code, expected', [
    ('succeed', D.OK, ('abort', D.AMCL_SERVICE_FAIL)),
    ('abort', D.TAG_LOST, ('abort', D.AMCL_SERVICE_FAIL)),
    ('canceled', D.OK, ('canceled', D.AMCL_SERVICE_FAIL)),
])
def test_restore_failed_reports_amcl(outcome, code, expected):
    assert final_outcome(outcome, code, False) == expected


def test_success_msg_values():
    assert SUCCESS_MSG == {'attach': 'Attach succeeded', 'detach': 'Detach succeeded'}


def test_error_code_values():
    for value, name in enumerate(CODE_NAMES):
        assert getattr(D, name) == value
