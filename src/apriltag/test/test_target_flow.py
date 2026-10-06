from types import SimpleNamespace

from apriltag.runtime.target_flow import choose_best_target


def tag(family, tag_id, z):
    return SimpleNamespace(family=family, id=tag_id,
                           pose=SimpleNamespace(position=SimpleNamespace(z=z)))


def test_empty_returns_none():
    assert choose_best_target([], 'tag36h11', '0') is None


def test_family_mismatch_returns_none():
    assert choose_best_target([tag('tag25h9', '0', 0.5)], 'tag36h11', '0') is None


def test_id_mismatch_returns_none():
    assert choose_best_target([tag('tag36h11', '1', 0.5)], 'tag36h11', '0') is None


def test_same_id_picks_closest():
    near = tag('tag36h11', '0', 0.4)
    tags = [tag('tag36h11', '0', 0.9), near, tag('tag36h11', '1', 0.2)]
    assert choose_best_target(tags, 'tag36h11', '0') is near


def test_any_id_picks_closest_of_family():
    near = tag('tag36h11', '7', 0.3)
    tags = [tag('tag36h11', '0', 0.9), near, tag('tag25h9', '0', 0.1)]
    assert choose_best_target(tags, 'tag36h11', '-1') is near


def test_non_positive_z_ignored():
    tags = [tag('tag36h11', '0', 0.0), tag('tag36h11', '0', -0.5)]
    assert choose_best_target(tags, 'tag36h11', '0') is None
