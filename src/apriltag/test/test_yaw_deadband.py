"""Stage 1 / 2 yaw deadband: control.yaw_deadband parsing and yaw_deadband_hold hysteresis."""

import copy
import math
import pathlib

import pytest

from apriltag.domain.config_loader import YawDeadband, load_config, parse_yaw_deadband
from apriltag.runtime.control_flow import yaw_deadband_hold

REPO_CONFIG = pathlib.Path(__file__).resolve().parents[3] / 'april_tag_config.json'

BASE = {'control': {'yaw_deadband': {'in': 0.03, 'out': 0.05}}}


def _db(value):
    data = copy.deepcopy(BASE)
    data['control']['yaw_deadband'] = value
    return data


def test_parse_valid():
    assert parse_yaw_deadband(BASE) == YawDeadband(enter=0.03, exit=0.05)


def test_parse_int_accepted():
    assert parse_yaw_deadband(_db({'in': 1, 'out': 2})) == YawDeadband(enter=1.0, exit=2.0)


@pytest.mark.parametrize('data', [{}, {'control': {}}, _db(None), _db([0.03, 0.05]), _db(0.03)])
def test_parse_missing_section(data):
    with pytest.raises(ValueError, match='Missing or invalid control.yaw_deadband'):
        parse_yaw_deadband(data)


@pytest.mark.parametrize('key', ['in', 'out'])
@pytest.mark.parametrize('bad', [None, '0.03', True, float('nan'), float('inf')])
def test_parse_bad_number(key, bad):
    value = {'in': 0.03, 'out': 0.05}
    value[key] = bad
    with pytest.raises(ValueError, match=f'control.yaw_deadband.{key} must be a number'):
        parse_yaw_deadband(_db(value))


@pytest.mark.parametrize('key', ['in', 'out'])
def test_parse_missing_key(key):
    value = {'in': 0.03, 'out': 0.05}
    del value[key]
    with pytest.raises(ValueError, match=f'control.yaw_deadband.{key} must be a number'):
        parse_yaw_deadband(_db(value))


@pytest.mark.parametrize('enter', [0.0, -0.01])
def test_parse_in_not_positive(enter):
    with pytest.raises(ValueError, match='control.yaw_deadband.in .* must be positive'):
        parse_yaw_deadband(_db({'in': enter, 'out': 0.05}))


@pytest.mark.parametrize('out', [0.03, 0.02])
def test_parse_out_not_above_in(out):
    with pytest.raises(ValueError, match=r'control.yaw_deadband.out .* must be > in'):
        parse_yaw_deadband(_db({'in': 0.03, 'out': out}))


@pytest.mark.skipif(not REPO_CONFIG.exists(), reason='workspace april_tag_config.json not found')
def test_repo_config_has_yaw_deadband():
    db = parse_yaw_deadband(load_config(str(REPO_CONFIG)))
    assert 0.0 < db.enter < db.exit


@pytest.mark.parametrize('prev,yaw,expected', [
    # held: stays held up to out, leaves above it
    (True, 0.0, True),
    (True, 0.04, True),
    (True, 0.05, True),
    (True, 0.051, False),
    (True, -0.051, False),
    # not held: keeps correcting down to in, held below it
    (False, 0.04, False),
    (False, 0.03, False),
    (False, 0.029, True),
    (False, -0.029, True),
    (False, 0.2, False),
])
def test_hysteresis(prev, yaw, expected):
    assert yaw_deadband_hold(prev, yaw, 0.03, 0.05) is expected


def test_hysteresis_sequence():
    """A yaw coming down, noise inside the band, then leaving it."""
    hold, out = False, []
    for yaw in (0.1, 0.04, 0.02, 0.045, 0.035, 0.06, 0.04, 0.02):
        hold = yaw_deadband_hold(hold, yaw, 0.03, 0.05)
        out.append(hold)
    assert out == [False, False, True, True, True, False, False, True]


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite(bad):
    with pytest.raises(ValueError):
        yaw_deadband_hold(True, bad, 0.03, 0.05)
