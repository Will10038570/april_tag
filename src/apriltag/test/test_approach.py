"""Stage 1 straight approach: approach-line lateral error and the vx scale."""

import math

import numpy as np
import pytest

from apriltag.domain.calibration import CalibrationPoint
from apriltag.domain.config_loader import Stage1Approach
from apriltag.domain.math_utils import (
    approach_lateral_error,
    optical_to_control_error,
    rotation_matrix_to_yaw_error,
)
from apriltag.runtime.control_flow import approach_vx_scale, ramp_scale
from tools.virtual_tracking_sim import camera_world_pose, goal_pose, tag_in_optical

CAMERA_OFFSET = np.array([0.0, 0.026])
CFG = Stage1Approach(enabled=True, lateral_full=0.03, lateral_zero=0.08,
                     yaw_full=0.035, yaw_zero=0.105)


def _measure(robot_pose):
    """(forward, lateral, yaw) of the tag as apriltag_control sees it."""
    t, r_mat = tag_in_optical(np.asarray(robot_pose, dtype=float), CAMERA_OFFSET)
    return float(t[2]), -float(t[0]), rotation_matrix_to_yaw_error(r_mat)


def _world_lateral(robot_pose, point):
    """Camera distance from the point's approach line in world, y_error sign."""
    goal = goal_pose(CAMERA_OFFSET, point)
    gx, gy, gth = camera_world_pose(np.array(goal), CAMERA_OFFSET)
    cx, cy, _ = camera_world_pose(np.asarray(robot_pose, dtype=float), CAMERA_OFFSET)
    left = np.array([-math.sin(gth), math.cos(gth)])
    # camera left of the line -> tag to its right -> negative y_error
    return -float(np.dot([cx - gx, cy - gy], left))


# ---- approach_lateral_error ------------------------------------------------------

@pytest.mark.parametrize('forward,lateral,x_ref', [(0.8, 0.1, 0.026), (0.5, -0.2, 0.0), (1.0, 0.0, -0.3)])
def test_equals_y_error_when_yaw_is_yaw_ref(forward, lateral, x_ref):
    t = np.array([-lateral, 0.0, forward])
    y_error = optical_to_control_error(t, 0.5, None, x_ref).y_error
    assert approach_lateral_error(forward, lateral, 0.2, x_ref, 0.2) == pytest.approx(y_error)


@pytest.mark.parametrize('x_ref', [0.0, 0.026, 0.3, -0.3])
@pytest.mark.parametrize('yaw_ref', [0.0, 0.2, -0.2])
def test_matches_sim_geometry(x_ref, yaw_ref):
    point = CalibrationPoint(z_ref=0.5, x_ref=x_ref, yaw_ref=yaw_ref)
    rng = np.random.default_rng(0)
    for _ in range(50):
        pose = (rng.uniform(-1.2, -0.4), rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3))
        forward, lateral, yaw = _measure(pose)
        assert approach_lateral_error(forward, lateral, yaw, x_ref, yaw_ref) == pytest.approx(
            _world_lateral(pose, point), abs=1e-9)


@pytest.mark.parametrize('yaw_ref', [0.0, 0.2])
def test_zero_on_the_approach_line(yaw_ref):
    point = CalibrationPoint(z_ref=0.5, x_ref=0.026, yaw_ref=yaw_ref)
    gx, gy, gth = goal_pose(CAMERA_OFFSET, point)
    for back in (0.0, 0.2, 0.5):
        pose = (gx - back * math.cos(gth), gy - back * math.sin(gth), gth)
        forward, lateral, yaw = _measure(pose)
        assert approach_lateral_error(forward, lateral, yaw, 0.026, yaw_ref) == pytest.approx(0.0, abs=1e-9)


def test_yaw_moves_the_line_offset():
    # tag centred in the image (y_error 0) but turned: the camera is off the line
    assert approach_lateral_error(0.7, 0.0, 0.1, 0.0, 0.0) == pytest.approx(-0.7 * math.sin(0.1))


def test_non_finite_raises():
    with pytest.raises(ValueError):
        approach_lateral_error(math.nan, 0.0, 0.0, 0.0, 0.0)


# ---- ramp_scale / approach_vx_scale ----------------------------------------------

@pytest.mark.parametrize('value,expected', [
    (0.0, 1.0), (0.03, 1.0), (-0.03, 1.0), (0.055, 0.5), (-0.055, 0.5), (0.08, 0.0), (0.5, 0.0),
])
def test_ramp_scale(value, expected):
    assert ramp_scale(value, 0.03, 0.08) == pytest.approx(expected)


def test_ramp_scale_non_finite_is_zero():
    assert ramp_scale(math.nan, 0.03, 0.08) == 0.0


def test_disabled_is_one():
    cfg = Stage1Approach(False, 0.03, 0.08, 0.035, 0.105)
    assert approach_vx_scale(1.0, 1.0, True, cfg) == 1.0


def test_yaw_ignored_until_valid():
    assert approach_vx_scale(0.0, 0.5, False, CFG) == 1.0
    assert approach_vx_scale(0.0, 0.5, True, CFG) == 0.0


def test_lateral_and_yaw_multiply():
    assert approach_vx_scale(0.055, 0.07, True, CFG) == pytest.approx(0.25)
