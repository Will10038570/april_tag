"""Target selection helpers."""

# tag_id value that accepts any id of the tracked family
ANY_TAG_ID = "-1"


def choose_best_target(tags, family: str, tag_id: str):
    """Choose the tracked tag: the closest one in front of the camera.

    `tags` are TagPose-like objects (.family, .id, .pose.position.z). Only tags
    of `family` with id `tag_id` (any id if tag_id is "-1") and positive forward
    z are considered; the one with the smallest z is returned, None if there is
    none.
    """
    candidates = [
        tag for tag in tags
        if tag.family == family
        and (tag_id == ANY_TAG_ID or tag.id == tag_id)
        and tag.pose.position.z > 0
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda tag: tag.pose.position.z)
