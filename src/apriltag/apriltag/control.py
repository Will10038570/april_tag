#!/usr/bin/env python3
"""Reference planners and controllers for tag tracking.

Provides:
- `TrajectoryPlanner` : segmented reference generator
- `LQRTracker` : discrete-time LQR state feedback tracker
"""

import numpy as np
from typing import Optional, Tuple
from scipy.linalg import solve_discrete_are  # pyright: ignore[reportMissingImports]


class TrajectoryPlanner:
    """Reference trajectory generator with first-order smoothing.

    All three axes (x, y, yaw) slide toward zero simultaneously.
    """

    def __init__(self,
                 position_threshold: float = 0.04,
                 smooth_tau: float = 0.25) -> None:
        self.position_threshold = position_threshold
        self.smooth_tau = smooth_tau
        self._ref_x = 0.0
        self._ref_y = 0.0
        self._ref_yaw = 0.0
        self._initialized = False

    def reset(self) -> None:
        self._ref_x = 0.0
        self._ref_y = 0.0
        self._ref_yaw = 0.0
        self._initialized = False

    def _smooth(self, current: float, target: float, dt: float) -> float:
        if dt <= 0.0:
            return current
        alpha = min(1.0, dt / max(self.smooth_tau, 1e-6))
        return current + alpha * (target - current)

    def plan(self, x_error: float, y_error: float, yaw_error: float, dt: float) -> Tuple[float, float, float]:
        if not self._initialized:
            self._ref_x = x_error
            self._ref_y = y_error
            self._ref_yaw = yaw_error
            self._initialized = True

        self._ref_x = self._smooth(self._ref_x, 0.0, dt)
        self._ref_y = self._smooth(self._ref_y, 0.0, dt)
        self._ref_yaw = self._smooth(self._ref_yaw, 0.0, dt)

        return self._ref_x, self._ref_y, self._ref_yaw


class LQRTracker:
    """Discrete-time LQR tracker for planar error dynamics.

    State is `[x_error, y_error, yaw_error]` and control is `[vx, vy, vw]`.
    The simplified error model is `e[k+1] = e[k] - dt * u[k]`.
    """

    def __init__(self,
                 q_weights: Tuple[float, float, float] = (1.0, 1.0, 0.5),
                 r_weights: Tuple[float, float, float] = (1.0, 1.0, 3.0),
                 nominal_dt: float = 0.1) -> None:
        self.Q = np.diag(q_weights)
        self.R = np.diag(r_weights)
        self.nominal_dt = nominal_dt
        self._cached_dt: Optional[float] = None
        self._cached_gain: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._cached_dt = None
        self._cached_gain = None

    _DT_CACHE_TOLERANCE = 0.20  # dt 在 cached 值 ±20% 以內重用 gain

    def _compute_gain(self, dt: float) -> Optional[np.ndarray]:
        dt = max(float(dt), 1e-3)
        if (self._cached_gain is not None and self._cached_dt is not None
                and abs(dt - self._cached_dt) / self._cached_dt < self._DT_CACHE_TOLERANCE):
            return self._cached_gain

        if solve_discrete_are is None:
            return None

        a_matrix = np.eye(3)
        b_matrix = -dt * np.eye(3)

        p_matrix = solve_discrete_are(a_matrix, b_matrix, self.Q, self.R)
        gain = np.linalg.solve(self.R + b_matrix.T @ p_matrix @ b_matrix, b_matrix.T @ p_matrix @ a_matrix)

        self._cached_dt = dt
        self._cached_gain = gain
        return gain

    def update(self,
               x_error: float,
               y_error: float,
               yaw_error: float,
               x_ref: float,
               y_ref: float,
               yaw_ref: float,
               dt: Optional[float] = None) -> Tuple[float, float, float]:
        gain = self._compute_gain(self.nominal_dt if dt is None else dt)
        if gain is None:
            return 0.0, 0.0, 0.0

        state = np.array([x_error, y_error, yaw_error], dtype=float)
        reference = np.array([x_ref, y_ref, yaw_ref], dtype=float)
        control = -gain @ (state - reference)
        return float(control[0]), float(control[1]), float(control[2])


class TwoStageLQRTracker:
    """Distance-based LQR gain scheduler.

    Stage 1 (|x_error| > x_threshold): approach gains — used when robot is far (~0.50 m).
    Stage 2 (|x_error| <= x_threshold): fine-dock gains — kicks in at ~0.30 m.

    Drop-in replacement for LQRTracker; same reset() / update() interface.
    """

    def __init__(self,
                 x_threshold: float = 0.30,
                 stage1_q: Tuple[float, float, float] = (1.0, 1.0, 0.5),
                 stage1_r: Tuple[float, float, float] = (1.0, 1.0, 3.0),
                 stage2_q: Tuple[float, float, float] = (1.0, 1.0, 0.5),
                 stage2_r: Tuple[float, float, float] = (3.0, 1.0, 3.0),
                 nominal_dt: float = 0.1) -> None:
        self.x_threshold = x_threshold
        self._stage1 = LQRTracker(q_weights=stage1_q, r_weights=stage1_r, nominal_dt=nominal_dt)
        self._stage2 = LQRTracker(q_weights=stage2_q, r_weights=stage2_r, nominal_dt=nominal_dt)

    def reset(self) -> None:
        self._stage1.reset()
        self._stage2.reset()

    def update(self,
               x_error: float,
               y_error: float,
               yaw_error: float,
               x_ref: float,
               y_ref: float,
               yaw_ref: float,
               dt: Optional[float] = None) -> Tuple[float, float, float]:
        tracker = self._stage1 if abs(x_error) > self.x_threshold else self._stage2
        return tracker.update(x_error, y_error, yaw_error, x_ref, y_ref, yaw_ref, dt)


def _run_lqr_demo() -> None:
    tracker = LQRTracker()
    planner = TrajectoryPlanner(position_threshold=0.04, smooth_tau=0.2)

    dt = 0.1
    # x=0.20 simulates robot stopping 0.5m from tag with desired_distance=0.3m
    state = np.array([0.20, -0.18, 0.5], dtype=float)

    print("LQR demo (single-phase: all axes corrected simultaneously)")
    print(f"initial error: x={state[0]:.3f}, y={state[1]:.3f}, yaw={state[2]:.3f}")
    for i in range(50):
        x_ref, y_ref, yaw_ref = planner.plan(state[0], state[1], state[2], dt)
        vx, vy, vw = tracker.update(state[0], state[1], state[2], x_ref, y_ref, yaw_ref, dt)
        control = np.array([vx, vy, vw], dtype=float)

        state = state - control * dt

        print(
            f"step {i:02d} "
            f"u=({vx:+.3f},{vy:+.3f},{vw:+.3f}) "
            f"ref=({x_ref:+.3f},{y_ref:+.3f},{yaw_ref:+.3f}) "
            f"err=({state[0]:+.3f},{state[1]:+.3f},{state[2]:+.3f})"
        )


if __name__ == "__main__":
    _run_lqr_demo()
    

