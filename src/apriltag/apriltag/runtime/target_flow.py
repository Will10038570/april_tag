"""Target selection helpers.

apriltag_detection picks the tracked tag of an image with choose_best_target;
apriltag_control only checks it against the goal with tag_matches.
"""

# tag_id value that accepts any id of the tracked family
ANY_TAG_ID = "-1"


def tag_matches(tag, family: str, tag_id: str) -> bool:
    """True if a TagPose-like tag is of `family` with id `tag_id` (any id if "-1")."""
    return tag.family == family and (tag_id == ANY_TAG_ID or tag.id == tag_id)


def _candidates(tags, family: str, tag_id: str):
    return [
        tag for tag in tags
        if tag_matches(tag, family, tag_id) and tag.pose.position.z > 0
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
