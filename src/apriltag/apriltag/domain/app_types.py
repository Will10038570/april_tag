"""资讯型別定義層。
跨模組通訊的共用資料結構，取代字典傳遞。
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class CameraIntrinsics:
    """相機內參。
    
    Attributes:
        fx, fy: 焦距 (pixels)
        cx, cy: 主點 (pixels)
        width, height: 影像解析度
    """
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class ControlState:
    """控制狀態誤差。
    
    座標系：x→前進, y→左側, z→上升 (ROS base_link 約定)
    
    Attributes:
        x_error: 前進方向誤差 (m, 正值表示距離太遠)
        y_error: 左側方向誤差 (m, 正值表示需向左移)
        yaw_error: 偏航角誤差 (rad, 正值表示逆時針偏差)
    """
    x_error: float
    y_error: float
    yaw_error: float


@dataclass
class AprilTagPose:
    """單個 AprilTag 的位姿 (相機座標系)。
    
    Attributes:
        t: 平移向量 (3,) - 光學座標系 (x→right, y→down, z→forward)
        R: 旋轉矩陣 (3,3) 或 None
    """
    t: np.ndarray
    R: Optional[np.ndarray] = None
