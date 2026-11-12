"""Per-axis max / min speed limits applied before publishing cmd_vel."""

import pytest

from apriltag.runtime.control_flow import clamp_control, validate_speed_limits

MAX = (0.1, 0.1, 0.05)
MIN = (0.01, 0.01, 0.01)


@pytest.mark.parametrize('v,expected', [
    (0.005, 0.01),     # below min -> min
    (-0.005, -0.01),   # sign kept
    (0.0, 0.0),        # zero stays zero
    (0.05, 0.05),      # in range unchanged
    (0.5, 0.1),        # above max -> max
    (-0.5, -0.1),
])
def test_linear_axes(v, expected):
    vx, vy, _ = clamp_control(v, v, 0.0, *MAX, *MIN)
    assert vx == pytest.approx(expected)
    assert vy == pytest.approx(expected)


@pytest.mark.parametrize('v,expected', [
    (0.003, 0.01), (-0.003, -0.01), (0.0, 0.0), (0.03, 0.03), (0.2, 0.05), (-0.2, -0.05),
])
def test_angular_axis(v, expected):
    assert clamp_control(0.0, 0.0, v, *MAX, *MIN)[2] == pytest.approx(expected)


def test_axes_are_independent():
    assert clamp_control(0.005, 0.0, -0.2, *MAX, *MIN) == pytest.approx((0.01, 0.0, -0.05))


def test_without_min_is_max_only():
    assert clamp_control(0.005, -0.5, 0.001, *MAX) == pytest.approx((0.005, -0.1, 0.001))


def test_valid_limits():
    assert validate_speed_limits(*MAX, *MIN) is None
    assert validate_speed_limits(*MAX, 0.0, 0.0, 0.0) is None
    assert validate_speed_limits(*MAX, *MAX) is None          # min == max allowed


@pytest.mark.parametrize('limits,key', [
    ((0.0, 0.1, 0.05, 0.01, 0.01, 0.01), 'max_vx'),
    ((0.1, -0.1, 0.05, 0.01, 0.01, 0.01), 'max_vy'),
    ((0.1, 0.1, 0.05, 0.01, -0.01, 0.01), 'min_vy'),
    ((0.1, 0.1, 0.05, 0.01, 0.01, 0.06), 'min_vw'),          # min > max
    ((0.1, 0.1, 0.05, 0.2, 0.01, 0.01), 'min_vx'),
    ((float('nan'), 0.1, 0.05, 0.01, 0.01, 0.01), 'max_vx'),
])
def test_invalid_limits(limits, key):
    reason = validate_speed_limits(*limits)
    assert reason is not None and key in reason
