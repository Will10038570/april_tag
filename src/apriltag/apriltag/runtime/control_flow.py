"""Control pipeline helpers extracted from AprilTag ROS node."""

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


def clamp_control(vx: float, vy: float, vw: float, max_vx: float, max_vy: float, max_vw: float) -> Tuple[float, float, float]:
    return (
        max(min(vx, max_vx), -max_vx),
        max(min(vy, max_vy), -max_vy),
        max(min(vw, max_vw), -max_vw),
    )


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
):
    """Run one control step and publish twist; return state updates.

    wz_override (S3 probe turn) replaces the LQR's wz; vx / vy stay the LQR's.
    """
    dt, new_last_time = compute_dt(now, last_time, max_dt)
    state = extract_control_state(best_target)
    plan = plan_reference(trajectory_planner, state, dt)
    vx, vy, vw = compute_backend_control(lqr_tracker, state, plan, dt, logger)
    if wz_override is not None:
        vw = float(wz_override)
    vx, vy, vw = sanitize_control(vx, vy, vw, logger)
    vx, vy, vw = clamp_control(vx, vy, vw, max_vx, max_vy, max_vw)
    publish_twist_fn(vx, vy, vw)

    return new_last_time, plan


def leave_step(x_error: float, max_vx: float) -> Tuple[bool, float, float, float]:
    """leave_cs: back straight until the tag is at least leave_distance away.

    x_error = forward - leave_distance (m). Returns (reached, vx, vy, wz):
    reached -> (True, 0, 0, 0); otherwise (False, -max_vx, 0, 0).
    """
    if not np.isfinite(x_error):
        raise ValueError("Non-finite leave x_error detected")
    if x_error >= 0.0:
        return True, 0.0, 0.0, 0.0
    return False, -max_vx, 0.0, 0.0
