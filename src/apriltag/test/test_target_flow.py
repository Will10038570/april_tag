from types import SimpleNamespace

from apriltag.runtime.target_flow import TargetSelector, choose_best_target, resolve_target_id


def tag(family, tag_id, z, cx=320.0, width=40.0):
    h = width / 2.0
    corners = [cx - h, 260.0, cx + h, 260.0, cx + h, 220.0, cx - h, 220.0]
    return SimpleNamespace(family=family, id=tag_id, corners=corners,
                           pose=SimpleNamespace(position=SimpleNamespace(z=z)))


# ---- choose_best_target -------------------------------------------------------

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


# ---- target_id priority -----------------------------------------------------------

def test_goal_target_id_wins():
    assert resolve_target_id(3, '5') == '3'
    assert resolve_target_id(0, '5') == '0'


def test_goal_unset_uses_parameter():
    assert resolve_target_id(-1, '5') == '5'


def test_both_unset_is_any_id():
    assert resolve_target_id(-1, '-1') == '-1'


# ---- TargetSelector -----------------------------------------------------------------

def test_only_target_id_is_used():
    sel = TargetSelector()
    target = tag('tag36h11', '3', 0.6, cx=400.0)
    picked = sel.select([tag('tag36h11', '1', 0.3, cx=200.0), target], 'tag36h11', '3')
    assert picked.tag is target


def test_only_other_ids_is_not_seen():
    sel = TargetSelector()
    assert sel.select([tag('tag36h11', '1', 0.3)], 'tag36h11', '3') is None
    sel.select([tag('tag36h11', '3', 0.5)], 'tag36h11', '3')
    for _ in range(10):
        assert sel.select([tag('tag36h11', '1', 0.3)], 'tag36h11', '3') is None


def test_changing_target_id_gives_new_key():
    sel = TargetSelector()
    tags = [tag('tag36h11', '3', 0.5, cx=200.0), tag('tag36h11', '4', 0.5, cx=450.0)]
    first = sel.select(tags, 'tag36h11', '3')
    assert sel.select(tags, 'tag36h11', '3').key == first.key
    second = sel.select(tags, 'tag36h11', '4')
    assert second.switched and second.key != first.key and second.tag.id == '4'


def test_duplicate_id_picks_closest():
    sel = TargetSelector()
    near = tag('tag36h11', '0', 0.5, cx=450.0)
    picked = sel.select([tag('tag36h11', '0', 0.8, cx=150.0), near], 'tag36h11', '0')
    assert picked.tag is near


def test_duplicate_switches_after_five_closer_frames():
    sel = TargetSelector()
    first = sel.select([tag('tag36h11', '0', 0.6, cx=150.0)], 'tag36h11', '0')
    for i in range(4):
        picked = sel.select([tag('tag36h11', '0', 0.6, cx=150.0), tag('tag36h11', '0', 0.5, cx=450.0)],
                            'tag36h11', '0')
        assert picked.key == first.key and picked.tag.pose.position.z == 0.6, i
    picked = sel.select([tag('tag36h11', '0', 0.6, cx=150.0), tag('tag36h11', '0', 0.5, cx=450.0)],
                        'tag36h11', '0')
    assert picked.switched and picked.key != first.key and picked.tag.pose.position.z == 0.5


def test_duplicate_not_closer_enough_never_switches():
    sel = TargetSelector()
    first = sel.select([tag('tag36h11', '0', 0.6, cx=150.0)], 'tag36h11', '0')
    for _ in range(20):
        # 0.52 > 0.85 * 0.6 = 0.51
        picked = sel.select([tag('tag36h11', '0', 0.6, cx=150.0), tag('tag36h11', '0', 0.52, cx=450.0)],
                            'tag36h11', '0')
        assert picked.key == first.key and picked.tag.pose.position.z == 0.6


def test_closer_streak_must_be_consecutive():
    sel = TargetSelector()
    first = sel.select([tag('tag36h11', '0', 0.6, cx=150.0)], 'tag36h11', '0')
    both = [tag('tag36h11', '0', 0.6, cx=150.0), tag('tag36h11', '0', 0.5, cx=450.0)]
    for _ in range(4):
        sel.select(both, 'tag36h11', '0')
    sel.select([tag('tag36h11', '0', 0.6, cx=150.0)], 'tag36h11', '0')
    for _ in range(4):
        assert sel.select(both, 'tag36h11', '0').key == first.key


def test_locked_instance_followed_by_pixel_position():
    sel = TargetSelector()
    first = sel.select([tag('tag36h11', '0', 0.5, cx=300.0)], 'tag36h11', '0')
    # the locked tag drifts a little and gets slightly farther than the other one
    picked = sel.select([tag('tag36h11', '0', 0.55, cx=310.0), tag('tag36h11', '0', 0.54, cx=500.0)],
                        'tag36h11', '0')
    assert picked.key == first.key and picked.tag.pose.position.z == 0.55


def test_missing_lock_switches_after_five_frames():
    sel = TargetSelector()
    first = sel.select([tag('tag36h11', '0', 0.5, cx=150.0)], 'tag36h11', '0')
    other = [tag('tag36h11', '0', 0.7, cx=500.0)]
    for _ in range(4):
        assert sel.select(other, 'tag36h11', '0') is None
    picked = sel.select(other, 'tag36h11', '0')
    assert picked.switched and picked.key != first.key
