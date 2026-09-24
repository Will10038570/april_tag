"""Target selection helpers."""


def choose_best_target(targets):
    """Choose target with smallest positive forward z; fallback to first target."""
    if not targets:
        return None

    forward_targets = [tr for tr in targets if tr["t"][2] > 0]
    if forward_targets:
        return min(forward_targets, key=lambda item: item["t"][2])
    return targets[0]
