"""virtual_tracking_sim geometry: the goal pose of a calibration point has zero ground-truth error."""

import math

import numpy as np
import pytest

from apriltag.domain.calibration import CalibrationPoint
from tools.virtual_tracking_sim import ground_truth, goal_pose


def _point(z_ref, x_ref, yaw_ref):
    return CalibrationPoint(z_ref=z_ref, x_ref=x_ref, yaw_ref=yaw_ref)


def test_factory_goal_on_centre_line():
    # x_ref == camera_y_offset, yaw_ref 0: goal straight in front of the tag
    x, y, yaw = goal_pose(np.array([0.0, 0.026]), _point(0.5, 0.026, 0.0))
    assert (x, y, yaw) == pytest.approx((-0.5, 0.0, 0.0))


def test_detach_goal_without_offset():
    # detach has only z_ref (x_ref = yaw_ref = 0): the camera, not the robot
    # centre, is on the tag's centre line
    x, y, yaw = goal_pose(np.array([0.0, 0.026]), _point(1.0, 0.0, 0.0))
    assert (x, y, yaw) == pytest.approx((-1.0, -0.026, 0.0))


def test_camera_x_offset_moves_goal_back():
    x, y, _ = goal_pose(np.array([0.1, 0.026]), _point(0.28, 0.026, 0.0))
    assert (x, y) == pytest.approx((-0.38, 0.0))


@pytest.mark.parametrize('seed', range(20))
def test_goal_pose_has_zero_error(seed):
    rng = np.random.default_rng(seed)
    offset = rng.uniform([-0.2, -0.1], [0.2, 0.1])
    point = _point(rng.uniform(0.2, 1.5), rng.uniform(-0.1, 0.1), rng.uniform(-0.3, 0.3))
    pose = np.array(goal_pose(offset, point))
    err = ground_truth(pose, offset, point)
    assert abs(err.x_error) < 1e-9
    assert abs(err.y_error) < 1e-9
    assert abs(err.yaw_error) < 1e-9


def test_error_signs_follow_control_convention():
    point = _point(0.5, 0.026, 0.0)
    offset = np.array([0.0, 0.026])
    gx, gy, _ = goal_pose(offset, point)
    # 0.1 m farther from the tag: positive x error
    assert ground_truth(np.array([gx - 0.1, gy, 0.0]), offset, point).x_error == pytest.approx(0.1)
    # small rotation changes yaw error by minus that angle (measured yaw = -heading)
    err = ground_truth(np.array([gx, gy, math.radians(2.0)]), offset, point)
    assert err.yaw_error == pytest.approx(-math.radians(2.0), abs=1e-9)
