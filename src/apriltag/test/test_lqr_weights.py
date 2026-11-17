"""control.lqr q_weights with R fixed to 1 (control_node.LQR_R_WEIGHTS)."""

import math

import pytest

from apriltag.control import LQRTracker

R_ONE = (1.0, 1.0, 1.0)


@pytest.mark.parametrize('dt', [0.1, 0.033])
def test_default_q_matches_old_gains(dt):
    # the shipped q_weights equal the old hard-coded Q (1, 1, 0.5) / R (3, 1, 3)
    old = LQRTracker(q_weights=(1.0, 1.0, 0.5), r_weights=(3.0, 1.0, 3.0))
    new = LQRTracker(q_weights=(0.3333, 1.0, 0.1667), r_weights=R_ONE)
    err = (0.2, -0.1, 0.15)
    u_old = old.update(*err, 0.0, 0.0, 0.0, dt)
    u_new = new.update(*err, 0.0, 0.0, 0.0, dt)
    for a, b in zip(u_old, u_new):
        assert b == pytest.approx(a, rel=1e-3)


@pytest.mark.parametrize('q', [0.1667, 0.3333, 1.0, 2.0])
def test_gain_about_sqrt_q(q):
    # with r = 1 each axis' gain is about sqrt(q) (documented rule of thumb)
    tracker = LQRTracker(q_weights=(q, q, q), r_weights=R_ONE)
    vx, vy, wz = tracker.update(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.1)
    for k in (vx, vy, wz):
        assert k == pytest.approx(math.sqrt(q), rel=0.1)
