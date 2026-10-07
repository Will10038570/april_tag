"""YawEstimator stages (pure functions / classes, no ROS)."""

import math

import numpy as np
import pytest

from apriltag.domain.math_utils import rotation_matrix_to_yaw_error
from apriltag.yaw_estimator import (
    YawEstimator,
    YawEstimatorConfig,
    YawFilter,
    edge_height_diff,
    equivalent_distance,
    quality_score,
    sigma_size,
    sign_confidence,
    soft_sign_yaw,
)

FX = FY = 615.0
CX, CY = 320.0, 240.0


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def synthetic_tag(yaw_deg: float, d: float = 0.5, tag_size: float = 0.0635, x: float = 0.0):
    """Corner pixels (pupil order lb-rb-rt-lt) and R of a tag turned about the camera's vertical axis."""
    r_mat = rot_y(math.radians(yaw_deg))
    h = tag_size / 2.0
    tag_pts = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    cam = tag_pts @ r_mat.T + np.array([x, 0.0, d])
    px = cam[:, :2] / cam[:, 2:3] * np.array([FX, FY]) + np.array([CX, CY])
    return px, r_mat


# ---- S2 ---------------------------------------------------------------------

@pytest.mark.parametrize('yaw_deg', [20.0, -20.0])
def test_dh_sign_matches_yaw_sign(yaw_deg):
    corners, r_mat = synthetic_tag(yaw_deg)
    yaw = rotation_matrix_to_yaw_error(r_mat)
    dh = edge_height_diff(corners)
    assert abs(dh) > 1.0
    assert math.copysign(1.0, dh) == math.copysign(1.0, yaw)


def test_dh_ignores_corner_order_and_lateral_offset():
    corners, _ = synthetic_tag(0.0, x=0.1)
    assert abs(edge_height_diff(corners)) < 1e-9
    corners, _ = synthetic_tag(15.0)
    assert edge_height_diff(corners[::-1]) == pytest.approx(edge_height_diff(corners))


def test_c_sign_ramp():
    assert sign_confidence(0.39 * 0.5, 0.39) == pytest.approx((0.5, 0.0))
    assert sign_confidence(0.39 * 1.5, 0.39) == pytest.approx((1.5, 0.5))
    assert sign_confidence(-0.39 * 3.0, 0.39) == pytest.approx((3.0, 1.0))


def test_low_snr_yaw_used_near_zero():
    yaw_meas = math.radians(3.0)
    dh = 0.39 * 0.8  # SNR 0.8
    _, c_sign = sign_confidence(dh, 0.39)
    assert soft_sign_yaw(yaw_meas, c_sign, dh) == pytest.approx(0.0)


@pytest.mark.parametrize('yaw_deg', [8.0, -8.0, 20.0])
def test_high_snr_yaw_used_equals_yaw_meas(yaw_deg):
    corners, r_mat = synthetic_tag(yaw_deg)
    yaw_meas = rotation_matrix_to_yaw_error(r_mat)
    dh = edge_height_diff(corners)
    snr, c_sign = sign_confidence(dh, 0.39)
    assert snr >= 2.0
    assert soft_sign_yaw(yaw_meas, c_sign, dh) == pytest.approx(yaw_meas)


def test_high_snr_flipped_pose_takes_sign_of_dh():
    corners, r_mat = synthetic_tag(10.0)
    yaw_meas = rotation_matrix_to_yaw_error(r_mat)
    dh = edge_height_diff(corners)
    _, c_sign = sign_confidence(dh, 0.39)
    assert soft_sign_yaw(-yaw_meas, c_sign, dh) == pytest.approx(yaw_meas)


# ---- S1 / S4: yaw weight only, x / y always out ----------------------------------

def estimate(est, yaw_deg, d, x=0.0, tag_size=0.0635, decision_margin=100.0, hamming=0,
             corners=None, wz=0.0, dt=1 / 30):
    if corners is None:
        corners, _ = synthetic_tag(yaw_deg, d=d, tag_size=tag_size, x=x)
    return est.update(x=d, y=-x, yaw_meas=math.radians(yaw_deg), d=d, corners=corners,
                      image_width=640, image_height=480, hamming=hamming,
                      decision_margin=decision_margin, tag_size=tag_size, wz_cmd=wz, dt=dt)


def test_far_tag_keeps_xy_but_no_yaw():
    est = YawEstimator()
    for _ in range(5):
        e = estimate(est, 20.0, d=1.0)  # d_eq = 1.0 * 0.0475 / 0.0635 = 0.75 > 0.7
    assert math.isinf(e.sigma_size) and e.accepted is None
    assert e.x_f == pytest.approx(1.0) and e.y_f == pytest.approx(0.0)
    assert e.yaw_f == 0.0


def test_corner_near_border_keeps_xy_but_no_yaw():
    est = YawEstimator()
    corners, _ = synthetic_tag(20.0, d=0.4)
    corners = corners - corners.min(axis=0) + 5.0  # top-left corner 5 px from the edge
    e = estimate(est, 20.0, d=0.4, corners=corners)
    assert e.c_quality == 0.0 and math.isinf(e.sigma_size) and e.accepted is None
    assert e.x_f == pytest.approx(0.4)


def test_bit_errors_disable_yaw_only():
    corners, _ = synthetic_tag(10.0, d=0.4)
    assert quality_score(corners, 640, 480, hamming=1, decision_margin=100.0) == 0.0
    assert quality_score(corners, 640, 480, hamming=0, decision_margin=100.0) == 1.0
    assert quality_score(corners, 640, 480, hamming=0, decision_margin=40.0) == pytest.approx(0.5)


def test_s1_never_drops_a_frame():
    est = YawEstimator()
    for d, yaw, ham in [(1.5, 5.0, 0), (0.5, 5.0, 2), (0.4, 1.0, 0)]:
        e = estimate(est, yaw, d=d, hamming=ham)
        assert math.isfinite(e.x_f) and math.isfinite(e.y_f)


def test_s4_bigger_tag_same_as_closer_reference_tag():
    assert equivalent_distance(0.6, 0.095, 0.0475) == pytest.approx(0.3)
    assert math.degrees(sigma_size(0.6, 0.095, 0.0475)) == pytest.approx(1.0)


def test_s4_reference_tag_at_0_6m():
    assert 2.0 < math.degrees(sigma_size(0.6, 0.0475, 0.0475)) < 3.0


def test_s4_table_ends_and_quality():
    assert math.degrees(sigma_size(0.1, 0.0475, 0.0475)) == pytest.approx(1.0)
    assert math.isinf(sigma_size(0.71, 0.0475, 0.0475))
    assert math.degrees(sigma_size(0.5, 0.0475, 0.0475, c_quality=0.5)) == pytest.approx(4.0)
    assert math.isinf(sigma_size(0.5, 0.0475, 0.0475, c_quality=0.0))


def test_s4_follows_tag_size():
    assert sigma_size(0.6, 0.095, 0.0475) < sigma_size(0.6, 0.0635, 0.0475) < sigma_size(0.6, 0.0475, 0.0475)


# ---- S5 -------------------------------------------------------------------------

def run_filter(values_deg, sigma_deg=1.0):
    filt = YawFilter(YawEstimatorConfig())
    out = []
    for v in values_deg:
        out.append(filt.update(math.radians(v), math.radians(sigma_deg), math.radians(v), 0.0, 1 / 30))
    return filt, out


@pytest.mark.parametrize('jump', [15.0, -15.0])
def test_single_frame_jump_is_rejected(jump):
    filt, out = run_filter([0.0] * 30 + [jump] + [0.0] * 5)
    assert out[30] is False
    assert abs(math.degrees(filt.yaw_f)) < 0.5


def test_five_consistent_frames_are_accepted():
    filt, out = run_filter([0.0] * 30 + [15.0] * 5)
    assert out[30:34] == [False] * 4 and out[34] is True
    assert math.degrees(filt.yaw_f) == pytest.approx(15.0)


def test_inconsistent_rejections_are_not_accepted():
    filt, out = run_filter([0.0] * 30 + [15.0, -15.0, 15.0, -15.0, 15.0, -15.0])
    assert not any(out[30:])
    assert abs(math.degrees(filt.yaw_f)) < 0.5


def test_predict_follows_published_wz():
    filt = YawFilter(YawEstimatorConfig())
    filt.update(0.1, math.inf, 0.1, 0.0, 0.0)  # not used: stays uninitialised
    assert not filt.initialized
    filt.update(0.1, math.radians(1.0), 0.1, 0.0, 0.0)
    filt.update(0.0, math.inf, 0.0, 0.05, 1.0)  # turn 0.05 rad/s for 1 s, no measurement
    assert filt.yaw_f == pytest.approx(0.05)


# ---- in-position check: yaw_abs_f, never yaw_f ------------------------------------

def error_is_zero(target, tol=0.05):
    # same check as AprilTagControlNode._error_is_zero (x / y / yaw tolerances 0.05)
    from types import SimpleNamespace
    from apriltag.control_node import AprilTagControlNode
    node = SimpleNamespace(stop_x_error_tolerance=tol, stop_y_error_tolerance=tol,
                           stop_yaw_error_tolerance=tol)
    return AprilTagControlNode._error_is_zero(node, target)


def test_aligned_noisy_yaw_holds_in_position_for_1s():
    rng = np.random.default_rng(1)
    est = YawEstimator()
    held = best = 0
    for _ in range(90):
        meas = rng.uniform(-2.0, 2.0)
        if rng.random() < 0.1:
            meas = -meas * 1.3  # occasional flip
        corners, _ = synthetic_tag(0.0, d=0.5)
        e = est.update(x=0.5, y=0.0, yaw_meas=math.radians(meas), d=0.5, corners=corners,
                       image_width=640, image_height=480, hamming=0, decision_margin=100.0,
                       tag_size=0.0475, wz_cmd=0.0, dt=1 / 30)
        held = held + 1 if error_is_zero({'x_error': 0.0, 'y_error': 0.0, 'yaw_error': e.yaw_abs_f}) else 0
        best = max(best, held)
    assert best >= 30  # 1 s at 30 fps
    assert e.yaw_abs_f < math.radians(2.9)


def test_misaligned_with_unknown_sign_is_not_in_position():
    rng = np.random.default_rng(2)
    est = YawEstimator()
    corners, _ = synthetic_tag(0.0, d=0.5)  # Δh 0: direction unknown, c_sign 0
    for _ in range(60):
        meas = 5.0 * rng.choice([-1.0, 1.0]) + rng.normal(0.0, 0.3)
        e = est.update(x=0.5, y=0.0, yaw_meas=math.radians(meas), d=0.5, corners=corners,
                       image_width=640, image_height=480, hamming=0, decision_margin=100.0,
                       tag_size=0.0475, wz_cmd=0.0, dt=1 / 30)
        assert not error_is_zero({'x_error': 0.0, 'y_error': 0.0, 'yaw_error': e.yaw_abs_f})
    assert e.c_sign == 0.0 and abs(e.yaw_f) < 0.01
    # yaw_f would have passed: that is why the check must use yaw_abs_f
    assert error_is_zero({'x_error': 0.0, 'y_error': 0.0, 'yaw_error': e.yaw_f})
