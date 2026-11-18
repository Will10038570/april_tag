"""Stage 1 / 2 per-axis hold: axes_in_tolerance and publish_control(hold_axes=...)."""

import pytest

from apriltag.domain.config_loader import StageTolerance
from apriltag.runtime.control_flow import axes_in_tolerance, in_tolerance, publish_control

TOL = StageTolerance(x=0.05, y=0.03, yaw=0.04, hold_seconds=1.0)
MAX = dict(max_vx=0.1, max_vy=0.1, max_vw=0.05)
MIN = dict(min_vx=0.01, min_vy=0.01, min_vw=0.01)


def _target(x, y, yaw):
    return {'x_error': x, 'y_error': y, 'yaw_error': yaw}


@pytest.mark.parametrize('target,expected', [
    (_target(0.0, 0.0, 0.0), (True, True, True)),
    (_target(0.06, 0.0, 0.0), (False, True, True)),
    (_target(0.0, -0.031, 0.0), (True, False, True)),
    (_target(0.0, 0.0, -0.05), (True, True, False)),
    (_target(-0.05, 0.03, 0.04), (True, True, True)),    # boundary counts as within
    (_target(0.2, -0.2, 0.2), (False, False, False)),
    (None, (False, False, False)),
])
def test_axes_in_tolerance(target, expected):
    assert axes_in_tolerance(target, TOL) == expected
    assert in_tolerance(target, TOL) is all(expected)


class _Planner:
    def plan(self, x, y, yaw, dt):
        return 0.0, 0.0, 0.0


class _Lqr:
    """Returns fixed (vx, vy, wz) regardless of the state."""

    def __init__(self, out):
        self.out = out

    def update(self, *args):
        return self.out


class _Logger:
    def warn(self, msg):
        pass


def _run(lqr_out, hold_axes=None, wz_override=None):
    sent = []
    publish_control(
        best_target=_target(0.1, 0.1, 0.1), now=1.0, last_time=0.9, max_dt=0.2,
        trajectory_planner=_Planner(), lqr_tracker=_Lqr(lqr_out),
        publish_twist_fn=lambda vx, vy, vw: sent.append((vx, vy, vw)),
        logger=_Logger(), wz_override=wz_override, hold_axes=hold_axes, **MAX, **MIN)
    assert len(sent) == 1
    return sent[0]


def test_no_hold_is_old_behaviour():
    out = (0.05, -0.005, 0.2)
    assert _run(out) == pytest.approx((0.05, -0.01, 0.05))
    assert _run(out, hold_axes=(False, False, False)) == pytest.approx(_run(out))


@pytest.mark.parametrize('hold,expected', [
    ((True, False, False), (0.0, -0.01, 0.05)),
    ((False, True, False), (0.05, 0.0, 0.05)),
    ((False, False, True), (0.05, -0.01, 0.0)),
    ((True, True, True), (0.0, 0.0, 0.0)),
])
def test_held_axis_is_zero_not_raised_to_min(hold, expected):
    assert _run((0.05, -0.005, 0.2), hold_axes=hold) == pytest.approx(expected)


def test_held_yaw_overrides_probe_turn():
    assert _run((0.05, 0.05, 0.0), hold_axes=(False, False, True), wz_override=0.03)[2] == 0.0
    assert _run((0.05, 0.05, 0.0), hold_axes=(False, False, False), wz_override=0.03)[2] == \
        pytest.approx(0.03)
