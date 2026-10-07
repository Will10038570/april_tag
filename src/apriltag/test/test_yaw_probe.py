"""S3 stuck protection, closed loop: yaw_error decreases by wz * dt (LQR model)."""

import math
from collections import deque

import numpy as np

from apriltag.runtime.yaw_probe import YawProbe, YawProbeConfig

DT = 1 / 30


def run(true_yaw_deg, enabled=True, seconds=20.0, initial_direction=1.0, noise_deg=0.3, seed=0):
    rng = np.random.default_rng(seed)
    probe = YawProbe(YawProbeConfig(initial_direction=initial_direction))
    yaw = math.radians(true_yaw_deg)
    window = deque(maxlen=9)
    wz_log = []
    for k in range(int(seconds / DT)):
        window.append(abs(yaw + math.radians(rng.normal(0.0, noise_deg))))
        wz = probe.step(now=k * DT, enabled=enabled, c_sign=0.0, yaw_abs_f=float(np.median(window)))
        wz = 0.0 if wz is None else wz  # LQR wz is ~0: yaw_f ~ 0 when c_sign is 0
        wz_log.append(wz)
        yaw -= wz * DT
    return yaw, wz_log, probe


def first_turn(wz_log):
    return next(i for i, w in enumerate(wz_log) if w != 0.0)


def test_stuck_at_5deg_probes_after_10_frames_and_converges():
    yaw, wz_log, probe = run(5.0)
    assert first_turn(wz_log) == 9  # 10th frame
    assert not probe.active
    assert abs(math.degrees(yaw)) < 1.5 + 1.0  # below 1.5 deg, or the minimum one turn (~1.7 deg) wide


def test_wrong_first_direction_reverses_after_1s():
    yaw, wz_log, probe = run(-5.0, initial_direction=1.0)
    start = first_turn(wz_log)
    assert wz_log[start] > 0.0
    reverse = next(i for i in range(start, len(wz_log)) if wz_log[i] < 0.0)
    # 1 s turn + 9 still frames to read the new |yaw| median
    assert (reverse - start) * DT <= 1.0 + 10 * DT
    assert abs(math.degrees(yaw)) < 2.5
    assert not probe.active


def test_aligned_never_probes():
    _, wz_log, _ = run(1.0)
    assert not any(wz_log)


def test_not_enabled_never_probes():
    # far away (x / y not in tolerance) or Stage 2: the node passes enabled=False
    _, wz_log, _ = run(5.0, enabled=False)
    assert not any(wz_log)


def test_known_direction_hands_back_to_lqr():
    probe = YawProbe()
    for k in range(10):
        wz = probe.step(k * DT, True, 0.0, math.radians(5.0))
    assert wz is not None and probe.active
    assert probe.step(11 * DT, True, 1.0, math.radians(5.0)) is None
    assert not probe.active
