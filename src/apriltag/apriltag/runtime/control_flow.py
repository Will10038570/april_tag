"""Control pipeline helpers extracted from AprilTag ROS node."""

import math
from typing import Dict, Optional, Tuple

import numpy as np


def is_finite_triplet(a: float, b: float, c: float) -> bool:
    return bool(np.isfinite([a, b, c]).all())


def extract_control_state(best_target: Dict[str, float]) -> Dict[str, float]:
    state = {
        "x_error": float(best_target.get("x_error", 0.0)),
        "y_error": float(best_target.get("y_error", 0.0)),
        "yaw_error": float(best_target.get("yaw_error", 0.0)),
    }
    if not is_finite_triplet(state["x_error"], state["y_error"], state["yaw_error"]):
        raise ValueError("Non-finite control state detected")
    return state


def plan_reference(trajectory_planner, state: Dict[str, float], dt: float) -> Dict[str, float]:
    x_ref, y_ref, yaw_ref = trajectory_planner.plan(
        state["x_error"],
        state["y_error"],
        state["yaw_error"],
        dt,
    )
    if not is_finite_triplet(x_ref, y_ref, yaw_ref):
        raise ValueError("Non-finite planner reference detected")
    return {
        "x_ref": x_ref,
        "y_ref": y_ref,
        "yaw_ref": yaw_ref,
    }


def compute_backend_control(lqr_tracker, state: Dict[str, float], plan: Dict[str, float], dt: float, logger) -> Tuple[float, float, float]:
    try:
        return lqr_tracker.update(
            state["x_error"],
            state["y_error"],
            state["yaw_error"],
            plan["x_ref"],
            plan["y_ref"],
            plan["yaw_ref"],
            dt,
        )
    except Exception as exc:
        logger.warn(f"LQR update failed: {exc}")
        return 0.0, 0.0, 0.0


def sanitize_control(vx: float, vy: float, vw: float, logger) -> Tuple[float, float, float]:
    if not is_finite_triplet(vx, vy, vw):
        logger.warn("Non-finite control command detected, publishing zero command.")
        return 0.0, 0.0, 0.0
    return vx, vy, vw


def _apply_min(v: float, min_v: float) -> float:
    """Raise a non-zero |v| below min_v to sign(v) * min_v; 0 stays 0."""
    if v != 0.0 and abs(v) < min_v:
        return math.copysign(min_v, v)
    return v


def clamp_control(
    vx: float, vy: float, vw: float,
    max_vx: float, max_vy: float, max_vw: float,
    min_vx: float = 0.0, min_vy: float = 0.0, min_vw: float = 0.0,
    vx_scale: float = 1.0,
) -> Tuple[float, float, float]:
    """Per axis: clip to [-max, max], then raise non-zero |v| < min to min.

    With 0 <= min <= max (validate_speed_limits) the result is 0 or
    |v| in [min, max]. min = 0 keeps the old max-only behaviour.
    vx_scale (clipped to [0, 1], non-finite -> 0) multiplies a forward vx
    (> 0) after the max clip and before the min: the Stage 1 straight
    approach. A backward vx is not scaled.
    """
    scale = min(max(float(vx_scale), 0.0), 1.0) if math.isfinite(vx_scale) else 0.0
    vx = max(min(vx, max_vx), -max_vx)
    if vx > 0.0:
        vx *= scale
    return (
        _apply_min(vx, min_vx),
        _apply_min(max(min(vy, max_vy), -max_vy), min_vy),
        _apply_min(max(min(vw, max_vw), -max_vw), min_vw),
    )


def ramp_scale(value: float, full: float, zero: float) -> float:
    """1 at |value| <= full, 0 at |value| >= zero, linear between; non-finite -> 0."""
    v = abs(float(value))
    if not math.isfinite(v) or v >= zero:
        return 0.0
    if v <= full:
        return 1.0
    return (zero - v) / (zero - full)


def approach_vx_scale(lateral: float, yaw: float, yaw_valid: bool, cfg) -> float:
    """Stage 1 forward-speed scale in [0, 1] from how far the robot is off its approach.

    cfg is a config_loader.Stage1Approach. lateral (m): approach-line
    lateral error, or the camera y_error while the yaw is not valid yet;
    yaw (rad, minus yaw_ref) counts only when yaw_valid. Disabled -> 1.
    """
    if not cfg.enabled:
        return 1.0
    scale = ramp_scale(lateral, cfg.lateral_full, cfg.lateral_zero)
    if yaw_valid:
        scale *= ramp_scale(yaw, cfg.yaw_full, cfg.yaw_zero)
    return scale


def validate_speed_limits(
    max_vx: float, max_vy: float, max_vw: float,
    min_vx: float, min_vy: float, min_vw: float,
) -> Optional[str]:
    """Return why the speed limits are invalid, or None if they are valid.

    Each max must be > 0, each min >= 0, and min <= max on every axis.
    """
    for axis, max_v, min_v in (('vx', max_vx, min_vx), ('vy', max_vy, min_vy), ('vw', max_vw, min_vw)):
        if not (math.isfinite(max_v) and max_v > 0.0):
            return f'max_{axis} ({max_v}) must be positive'
        if not (math.isfinite(min_v) and min_v >= 0.0):
            return f'min_{axis} ({min_v}) must be >= 0'
        if min_v > max_v:
            return f'min_{axis} ({min_v}) must be <= max_{axis} ({max_v})'
    return None


def compute_dt(now: float, last_time: float, max_dt: float) -> Tuple[float, float]:
    dt = now - last_time if last_time is not None else 0.1
    if dt <= 0.0:
        dt = 1e-3
    if dt > max_dt:
        dt = max_dt
    return dt, now


def publish_control(
    best_target: Dict[str, float],
    now: float,
    last_time: float,
    max_dt: float,
    trajectory_planner,
    lqr_tracker,
    max_vx: float,
    max_vy: float,
    max_vw: float,
    publish_twist_fn,
    logger,
    wz_override: Optional[float] = None,
    min_vx: float = 0.0,
    min_vy: float = 0.0,
    min_vw: float = 0.0,
    vx_scale: float = 1.0,
):
    """Run one control step and publish twist; return state updates.

    wz_override (S3 probe turn) replaces the LQR's wz; vx / vy stay the LQR's.
    vx_scale: see clamp_control (Stage 1 straight approach).
    """
    dt, new_last_time = compute_dt(now, last_time, max_dt)
    state = extract_control_state(best_target)
    plan = plan_reference(trajectory_planner, state, dt)
    vx, vy, vw = compute_backend_control(lqr_tracker, state, plan, dt, logger)
    if wz_override is not None:
        vw = float(wz_override)
    vx, vy, vw = sanitize_control(vx, vy, vw, logger)
    vx, vy, vw = clamp_control(vx, vy, vw, max_vx, max_vy, max_vw, min_vx, min_vy, min_vw,
                               vx_scale)
    publish_twist_fn(vx, vy, vw)

    return new_last_time, plan


def leave_step(x_error: float, max_vx: float) -> Tuple[bool, float, float, float]:
    """leave_cs: back straight until the tag is at least calibration.detach.z_ref away.

    x_error = forward - detach z_ref (m). Returns (reached, vx, vy, wz):
    reached -> (True, 0, 0, 0); otherwise (False, -max_vx, 0, 0).
    """
    if not np.isfinite(x_error):
        raise ValueError("Non-finite leave x_error detected")
    if x_error >= 0.0:
        return True, 0.0, 0.0, 0.0
    return False, -max_vx, 0.0, 0.0


def in_tolerance(target: Optional[Dict[str, float]], tol) -> bool:
    """In-position check: |x|, |y|, |yaw| of target within tol (a StageTolerance)."""
    if target is None:
        return False
    return (
        abs(float(target.get('x_error', 0.0))) <= tol.x
        and abs(float(target.get('y_error', 0.0))) <= tol.y
        and abs(float(target.get('yaw_error', 0.0))) <= tol.yaw
    )
