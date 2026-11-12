"""april_tag_config.json: load_config validation and calibration read / write."""

import copy
import json
import pathlib

import pytest

from apriltag.domain.calibration import (
    CalibrationPoint,
    load_calibration_from_config,
    read_config_dict,
    save_calibration_point,
)
from apriltag.domain.config_loader import load_config

# the shipped config at the workspace root (src/apriltag/test -> ../../..)
REPO_CONFIG = pathlib.Path(__file__).resolve().parents[3] / 'april_tag_config.json'

VALID = {
    'detection': {'detect_tag_families': 'tag36h11',
                  'tracking_tag_family': 'tag36h11', 'tracking_tag_id': 0},
    'tag_size': {'tag_size': 0.0635, 'tag_sizes': ''},
    'control': {'manage_amcl_and_lidar_safety': False,
                'stage1_distance': 0.5, 'stage2_distance': 0.28, 'leave_distance': 1.0,
                'dock_timeout': 300.0, 'undock_timeout': 300.0, 'lost_target_timeout': 1.0},
    'calibration': {'use_calibration': True, 'stage1': None, 'stage2': None, 'detach': None},
    'yaw_estimator': {'s4_distances': [0.3, 0.5, 0.7], 's4_sigmas_deg': [1.0, 2.0, 3.0]},
    'ros_interfaces': {'amcl_check_service': '/check_mcl_if_trigger',
                       'amcl_close_service': '/close_amcl',
                       'amcl_open_service': '/open_amcl',
                       'lidar_safety_topic': '/g7_plc/disable_lidar_safety',
                       'stage1_cmd_vel_topic': '/cmd_vel',
                       'stage2_cmd_vel_topic': '/pre_cmd_vel'},
}


def _write(tmp_path, data):
    p = tmp_path / 'april_tag_config.json'
    p.write_text(json.dumps(data))
    return str(p)


def _with(section, key, value):
    data = copy.deepcopy(VALID)
    data[section][key] = value
    return data


def test_valid_config_loads(tmp_path):
    assert load_config(_write(tmp_path, VALID)) == VALID


@pytest.mark.skipif(not REPO_CONFIG.exists(), reason='workspace april_tag_config.json not found')
def test_shipped_config_is_valid():
    load_config(str(REPO_CONFIG))


def test_missing_file(tmp_path):
    with pytest.raises(ValueError, match='not found'):
        load_config(str(tmp_path / 'nope.json'))


def test_bad_json(tmp_path):
    p = tmp_path / 'bad.json'
    p.write_text('{"detection": ')
    with pytest.raises(ValueError, match='Cannot parse'):
        load_config(str(p))


@pytest.mark.parametrize('section', list(VALID))
def test_missing_section(tmp_path, section):
    data = copy.deepcopy(VALID)
    del data[section]
    with pytest.raises(ValueError, match=section):
        load_config(_write(tmp_path, data))


@pytest.mark.parametrize('section,key,value', [
    ('control', 'stage1_distance', True),             # bool is not a number
    ('control', 'stage1_distance', '0.5'),
    ('control', 'stage2_distance', 0.0),              # must be positive
    ('control', 'dock_timeout', -1.0),
    ('control', 'manage_amcl_and_lidar_safety', 1),   # int is not a bool
    ('detection', 'tracking_tag_id', False),          # bool is not an int
    ('detection', 'tracking_tag_id', 0.0),
    ('detection', 'tracking_tag_id', -2),
    ('detection', 'tracking_tag_family', None),
    ('tag_size', 'tag_size', 0.0),
    ('tag_size', 'tag_sizes', 3),
    ('calibration', 'use_calibration', 'true'),
    ('yaw_estimator', 's4_distances', []),
    ('yaw_estimator', 's4_distances', [0.3, 0.5]),    # length mismatch
    ('yaw_estimator', 's4_sigmas_deg', [1.0, True, 3.0]),
    ('ros_interfaces', 'stage1_cmd_vel_topic', None),
])
def test_invalid_value(tmp_path, section, key, value):
    with pytest.raises(ValueError, match=key):
        load_config(_write(tmp_path, _with(section, key, value)))


def test_tracking_tag_id_any_is_valid(tmp_path):
    load_config(_write(tmp_path, _with('detection', 'tracking_tag_id', -1)))


def test_calibration_from_config():
    data = copy.deepcopy(VALID)
    data['calibration']['stage1'] = {'z_ref': 0.5, 'x_ref': 0.01, 'yaw_ref': 0.02}
    data['calibration']['stage2'] = {'x_ref': 0.0}    # incomplete -> None
    cal = load_calibration_from_config(data)
    assert cal['stage1'].z_ref == pytest.approx(0.5)
    assert cal['stage2'] is None
    assert cal['detach'] is None


def test_save_calibration_keeps_other_sections(tmp_path):
    path = _write(tmp_path, VALID)
    save_calibration_point(path, 'stage2', CalibrationPoint(z_ref=0.28, x_ref=0.0, yaw_ref=0.0))
    data = load_config(path)   # still a valid config
    for section in VALID:
        if section != 'calibration':
            assert data[section] == VALID[section]
    assert data['calibration']['use_calibration'] is True
    assert data['calibration']['stage1'] is None
    assert load_calibration_from_config(data)['stage2'].z_ref == pytest.approx(0.28)


def test_save_calibration_unknown_stage(tmp_path):
    with pytest.raises(ValueError):
        save_calibration_point(_write(tmp_path, VALID), 'stage3',
                               CalibrationPoint(z_ref=0.3, x_ref=0.0, yaw_ref=0.0))


def test_read_config_dict_missing(tmp_path):
    with pytest.raises(ValueError, match='not found'):
        read_config_dict(str(tmp_path / 'none.json'))


def test_read_config_dict_bad_json(tmp_path):
    p = tmp_path / 'bad.json'
    p.write_text('{bad')
    with pytest.raises(ValueError, match='Cannot parse'):
        read_config_dict(str(p))


def test_read_config_dict_not_object(tmp_path):
    p = tmp_path / 'list.json'
    p.write_text('[]')
    with pytest.raises(ValueError, match='JSON object'):
        read_config_dict(str(p))


def test_save_calibration_does_not_touch_bad_files(tmp_path):
    point = CalibrationPoint(z_ref=0.28, x_ref=0.0, yaw_ref=0.0)
    # corrupt file: error, content unchanged (not overwritten)
    bad = tmp_path / 'bad.json'
    bad.write_bytes(b'{bad')
    with pytest.raises(ValueError):
        save_calibration_point(str(bad), 'stage2', point)
    assert bad.read_bytes() == b'{bad'
    # missing file: error, not created
    missing = tmp_path / 'sub' / 'none.json'
    with pytest.raises(ValueError):
        save_calibration_point(str(missing), 'stage2', point)
    assert not missing.exists()
    assert not missing.parent.exists()
