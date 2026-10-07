"""Target selection helpers (S0 of the yaw estimator handoff)."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# tag_id value that accepts any id of the tracked family
ANY_TAG_ID = "-1"


def resolve_target_id(goal_target_id: int, param_tag_id: str) -> str:
    """Tag id to track as a decimal string: the goal's if given, else the parameter's.

    goal_target_id -1 means "not given"; a parameter of "-1" means any id.
    """
    if int(goal_target_id) >= 0:
        return str(int(goal_target_id))
    return str(param_tag_id).strip()


def _candidates(tags, family: str, tag_id: str):
    return [
        tag for tag in tags
        if tag.family == family
        and (tag_id == ANY_TAG_ID or tag.id == tag_id)
        and tag.pose.position.z > 0
    ]


def choose_best_target(tags, family: str, tag_id: str):
    """Choose the closest tag in front of the camera.

    `tags` are TagPose-like objects (.family, .id, .pose.position.z). Only tags
    of `family` with id `tag_id` (any id if tag_id is "-1") and positive forward
    z are considered; the one with the smallest z is returned, None if there is
    none.
    """
    candidates = _candidates(tags, family, tag_id)
    if not candidates:
        return None
    return min(candidates, key=lambda tag: tag.pose.position.z)


def tag_center_and_width(tag) -> Tuple[Optional[np.ndarray], float]:
    """Pixel centre and width (max x - min x) of a TagPose's corners; (None, 0) if it has none."""
    corners = np.asarray(getattr(tag, "corners", ()), dtype=float)
    if corners.size != 8 or not np.any(corners):
        return None, 0.0
    pts = corners.reshape(4, 2)
    return pts.mean(axis=0), float(pts[:, 0].max() - pts[:, 0].min())


@dataclass
class SelectedTarget:
    tag: object
    # (family, id, instance); a new instance number means a different
    # physical tag, so the estimator must start over
    key: Tuple[str, str, int]
    switched: bool


class TargetSelector:
    """Pick the one tag to track from all detections of an image, with hysteresis.

    Only tags of the tracked family / id are candidates. The first frame
    locks the closest one (smallest z). Afterwards the locked instance is
    followed by its pixel centre (the same id whose centre is nearest to the
    previous one, at most max_jump_ratio x its pixel width away). It changes
    only when another candidate is at most switch_ratio x as far for
    switch_frames frames in a row, or the locked one is missing for
    lost_frames frames in a row; frames in between return None (not seen).
    Changing family / tag_id restarts the selection.
    """

    def __init__(self, switch_ratio: float = 0.85, switch_frames: int = 5,
                 lost_frames: int = 5, max_jump_ratio: float = 0.5):
        self.switch_ratio = float(switch_ratio)
        self.switch_frames = int(switch_frames)
        self.lost_frames = int(lost_frames)
        self.max_jump_ratio = float(max_jump_ratio)
        self._instance = 0
        self._filter: Optional[Tuple[str, str]] = None
        self.reset()

    def reset(self) -> None:
        self._locked = None          # TagPose of the last frame the lock was seen
        self._key = None
        self._missed = 0
        self._challenger_frames = 0

    def _lock(self, tag) -> SelectedTarget:
        self._instance += 1
        self._locked = tag
        self._key = (tag.family, tag.id, self._instance)
        self._missed = 0
        self._challenger_frames = 0
        return SelectedTarget(tag=tag, key=self._key, switched=True)

    def _follow(self, candidates):
        """Candidate that is the locked instance in this frame, or None."""
        same_id = [t for t in candidates if (t.family, t.id) == self._key[:2]]
        if not same_id:
            return None
        center, width = tag_center_and_width(self._locked)
        if center is None:
            return min(same_id, key=lambda t: t.pose.position.z)
        best, best_dist = None, None
        for tag in same_id:
            c, _ = tag_center_and_width(tag)
            if c is None:
                continue
            dist = float(np.linalg.norm(c - center))
            if best_dist is None or dist < best_dist:
                best, best_dist = tag, dist
        if best is None or best_dist > self.max_jump_ratio * max(width, 1.0):
            return None
        return best

    def select(self, tags, family: str, tag_id: str) -> Optional[SelectedTarget]:
        if self._filter != (family, tag_id):
            self._filter = (family, tag_id)
            self.reset()

        candidates = _candidates(tags, family, tag_id)
        if self._locked is None:
            if not candidates:
                return None
            return self._lock(min(candidates, key=lambda t: t.pose.position.z))

        current = self._follow(candidates)
        if current is None:
            self._missed += 1
            self._challenger_frames = 0
            if self._missed < self.lost_frames:
                return None
            self.reset()
            if not candidates:
                return None
            return self._lock(min(candidates, key=lambda t: t.pose.position.z))

        self._missed = 0
        self._locked = current
        others = [t for t in candidates if t is not current]
        closest = min(others, key=lambda t: t.pose.position.z) if others else None
        if closest is not None and closest.pose.position.z <= self.switch_ratio * current.pose.position.z:
            self._challenger_frames += 1
            if self._challenger_frames >= self.switch_frames:
                return self._lock(closest)
        else:
            self._challenger_frames = 0
        return SelectedTarget(tag=current, key=self._key, switched=False)
