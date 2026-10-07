"""Tag size comes from the tag_size / tag_sizes parameters only."""

import math
import pathlib
import re
from types import SimpleNamespace

import numpy as np
import pytest

from apriltag.domain.app_types import CameraIntrinsics
from apriltag.domain.math_utils import parse_tag_sizes, tag_size_for
from apriltag.perception.tag_perception import draw_detections_and_collect_targets
from apriltag.yaw_estimator import sigma_size

PKG = pathlib.Path(__file__).resolve().parents[1] / 'apriltag'


class _Logger:
    def warn(self, msg):
        raise AssertionError(msg)


def detection(tag_id, z):
    corners = np.array([[300.0, 260.0], [340.0, 260.0], [340.0, 220.0], [300.0, 220.0]])
    return SimpleNamespace(tag_id=tag_id, tag_family=b'tag36h11', corners=corners, center=(320.0, 240.0),
                           pose_t=np.array([[0.0], [0.0], [z]]), pose_R=np.eye(3),
                           decision_margin=80.0, hamming=0)


def collect(dets, tag_size, sizes):
    intr = CameraIntrinsics(615.0, 615.0, 320.0, 240.0, 640, 480)
    _, targets = draw_detections_and_collect_targets(
        np.zeros((480, 640, 3), np.uint8), dets, intr, None, tag_size, _Logger(),
        size_of=lambda i: tag_size_for(i, sizes, tag_size))
    return targets


def test_pose_distance_follows_tag_size():
    # detect() was run with tag_size, so pose_t is already in that scale
    small = collect([detection(0, 0.5)], 0.0475, {})
    assert small[0]['t'][2] == pytest.approx(0.5)
    # tag 0 really is 0.095: twice as far
    big = collect([detection(0, 0.5)], 0.0475, parse_tag_sizes('0:0.095'))
    assert big[0]['t'][2] == pytest.approx(1.0)


def test_tag_sizes_per_id():
    sizes = parse_tag_sizes('3:0.095')
    targets = collect([detection(3, 0.5), detection(1, 0.5)], 0.0475, sizes)
    assert [t['t'][2] for t in targets] == pytest.approx([1.0, 0.5])
    assert targets[0]['decision_margin'] == 80.0 and targets[0]['corners'].shape == (4, 2)


def test_s4_follows_tag_size_parameter():
    assert math.degrees(sigma_size(0.6, 0.095, 0.0475)) == pytest.approx(1.0)
    assert sigma_size(0.6, 0.0475, 0.0475) > sigma_size(0.6, 0.095, 0.0475)


def test_parse_tag_sizes():
    assert parse_tag_sizes('') == {}
    assert parse_tag_sizes('3:0.095 5:0.0475') == {3: 0.095, 5: 0.0475}
    with pytest.raises(ValueError):
        parse_tag_sizes('3=0.095')
    with pytest.raises(ValueError):
        parse_tag_sizes('3:0')


def test_no_hardcoded_tag_size_in_code():
    # 0.0475 may only be ref_tag_size's default (the S4 table base);
    # the tag size itself only appears as parameter defaults
    for path in PKG.rglob('*.py'):
        for n, line in enumerate(path.read_text().splitlines(), 1):
            if re.search(r'0\.0475\b', line):
                assert 'ref_tag_size' in line, f'{path.name}:{n}: {line.strip()}'
            if re.search(r'0\.0635\b', line):
                assert "'tag_size'" in line, f'{path.name}:{n}: {line.strip()}'
