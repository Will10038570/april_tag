"""純數學與座標轉換工具函式。"""

import math
from typing import Tuple

import numpy as np

from apriltag.domain.app_types import ControlState


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋轉矩陣轉四元數 (x, y, z, w)。"""
    m = R
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    return (x, y, z, w)


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元數 (x, y, z, w) 轉 3x3 旋轉矩陣；為 rotation_matrix_to_quaternion 的反向轉換。"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ])


def rotation_matrix_to_yaw_error(R: np.ndarray) -> float:
    """從 AprilTag pose rotation matrix 提取控制座標偏航誤差。

    `pupil_apriltags` 的 `pose_R` 可視為 tag frame 到 camera optical frame 的旋轉。
    控制上的 yaw 是繞 base_link 的 z 軸，而這對應到 optical frame 的 `-y` 軸。
    在 tag 正對相機時 yaw 應為 0，因此可由 tag z 軸在 optical x-z 平面的投影求得。
    """
    r_mat = np.asarray(R, dtype=float)
    if r_mat.shape != (3, 3):
        raise ValueError(f"Expected a 3x3 rotation matrix, got shape {r_mat.shape}")

    return math.atan2(-float(r_mat[0, 2]), float(r_mat[2, 2]))


def optical_to_control_error(
    t_optical: np.ndarray,
    desired_distance: float,
    r_optical: np.ndarray | None = None,
    camera_y_offset: float = 0.0,
) -> ControlState:
    """光學座標誤差轉控制座標誤差。

    optical: x->right, y->down, z->forward
    control: x->forward, y->left

    camera_y_offset: 相機相對 AMR 控制原點的 y 偏移量 (m)。
    正值表示相機位於車體中心線左側，負值表示位於右側。
    """
    forward = float(t_optical[2])
    lateral = -float(t_optical[0])
    x_error = forward - float(desired_distance)
    y_error = lateral + float(camera_y_offset)

    if r_optical is not None:
        yaw_error = rotation_matrix_to_yaw_error(r_optical)
    else:
        yaw_error = math.atan2(lateral, forward) if forward != 0.0 else 0.0

    return ControlState(x_error=x_error, y_error=y_error, yaw_error=yaw_error)


