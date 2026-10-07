"""AprilTag 感知層：偵測資料提取、位姿估計。"""

from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np
from pupil_apriltags import Detector

from apriltag.domain.app_types import (
    AprilTagPose,
    CameraIntrinsics,
)


def extract_detection_data(raw_detection) -> Tuple[Optional[int], np.ndarray, Optional[Tuple[float, float]]]:
    """從 apriltag 偵測物件提取 tag_id、corners、center。"""
    tag_id = None
    corners = np.empty((0, 2), dtype=float)
    center = None

    if isinstance(raw_detection, dict):
        tag_id = raw_detection.get("id")
        if "lb-rb-rt-lt" in raw_detection:
            corners = np.array(raw_detection["lb-rb-rt-lt"], dtype=float)
        if "center" in raw_detection:
            c = raw_detection["center"]
            center = (float(c[0]), float(c[1]))
    else:
        tag_id = getattr(raw_detection, "tag_id", None)
        c = getattr(raw_detection, "corners", None)
        if c is not None:
            corners = np.array(c, dtype=float)
        ctr = getattr(raw_detection, "center", None)
        if ctr is not None:
            center = (float(ctr[0]), float(ctr[1]))

    return tag_id, corners, center


def detection_family(raw_detection) -> str:
    """Return the detection's tag family name; pupil_apriltags gives it as bytes."""
    if isinstance(raw_detection, dict):
        family = raw_detection.get("tag_family")
    else:
        family = getattr(raw_detection, "tag_family", None)
    if isinstance(family, bytes):
        return family.decode()
    return str(family) if family is not None else ""


def estimate_tag_pose(raw_detection, intrinsics: CameraIntrinsics, tag_size: float, detector) -> Optional[AprilTagPose]:
    """從 detection 物件提取位姿；需在 detect 時啟用 estimate_tag_pose。"""
    _ = intrinsics
    _ = tag_size
    _ = detector

    t_vec = getattr(raw_detection, "pose_t", None)
    if t_vec is None:
        return None

    t = np.asarray(t_vec, dtype=float).reshape(3)
    r_raw = getattr(raw_detection, "pose_R", None)
    r_mat = np.asarray(r_raw, dtype=float) if r_raw is not None else None
    return AprilTagPose(t=t, R=r_mat)


def build_detector(tag_family: str = "tag36h11"):
    """建立 pupil_apriltags 偵測器（封裝建立點，方便替換或 mock）。"""
    return Detector(
        families=tag_family,
        nthreads=2,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )


def draw_detections_and_collect_targets(
    img: np.ndarray,
    detections: List,
    intrinsics: CameraIntrinsics,
    detector,
    tag_size: float,
    logger,
    size_of: Optional[Callable[[int], float]] = None,
    is_tracked: Optional[Callable[[str, int], bool]] = None,
):
    """Draw detections and return target dictionaries for publishing.

    Each target has family, id (decimal string, comparable with TagPose.id),
    t, R, corners (4x2 px), decision_margin and hamming. detect() was run
    with `tag_size`; since the pose translation is proportional to the tag
    size, t is scaled by size_of(id) / tag_size so every id gets its own real
    size (size_of None: all tags are tag_size). Tags for which is_tracked
    (family, id) is true are outlined in yellow instead of green.
    """
    targets = []

    for det in detections:
        tag_id, corners, center = extract_detection_data(det)
        tracked = (
            tag_id is not None and is_tracked is not None
            and is_tracked(detection_family(det), int(tag_id))
        )
        outline = (0, 255, 255) if tracked else (0, 255, 0)

        if corners is not None and corners.size > 0:
            pts = corners.astype(int)
            for i in range(4):
                pt1 = tuple(pts[i])
                pt2 = tuple(pts[(i + 1) % 4])
                cv2.line(img, pt1, pt2, outline, 3 if tracked else 2)

        if center:
            center_i = (int(center[0]), int(center[1]))
            cv2.circle(img, center_i, 4, (0, 0, 255), -1)
        else:
            center_i = None

        if tag_id is None:
            continue

        try:
            pose = estimate_tag_pose(det, intrinsics, tag_size, detector)
            if pose is None:
                continue

            real_size = size_of(int(tag_id)) if size_of is not None else tag_size
            t_vec = pose.t * (float(real_size) / float(tag_size))
            distance = float(np.linalg.norm(t_vec))
            family = detection_family(det)
            text = f"{family}:{tag_id} {distance:.2f} m"
            if center_i:
                cv2.putText(
                    img,
                    text,
                    (center_i[0] + 10, center_i[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2,
                )

            # logger.info(text)
            targets.append(
                {
                    "family": family,
                    "id": str(int(tag_id)),
                    "t": t_vec,
                    "R": pose.R,
                    "corners": corners,
                    "decision_margin": float(getattr(det, "decision_margin", 0.0) or 0.0),
                    "hamming": int(getattr(det, "hamming", 0) or 0),
                }
            )
        except Exception as exc:
            logger.warn(f"Pose estimation failed: {exc}")

    return img, targets
