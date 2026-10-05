"""leave_cs straight-back decision of apriltag_control."""

import pytest

from apriltag.runtime.control_flow import leave_step


def test_backs_up_at_max_vx_before_leave_distance():
    assert leave_step(-0.72, 0.1) == (False, -0.1, 0.0, 0.0)


def test_reached_exactly_at_leave_distance():
    assert leave_step(0.0, 0.1) == (True, 0.0, 0.0, 0.0)


def test_reached_beyond_leave_distance():
    assert leave_step(0.01, 0.1) == (True, 0.0, 0.0, 0.0)


def test_non_finite_error_raises():
    with pytest.raises(ValueError):
        leave_step(float('nan'), 0.1)
