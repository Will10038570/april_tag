"""april_tag_config.json: load_config / validate_control_config and calibration read / write."""

import copy
import json
import pathlib

import pytest

from apriltag.domain.calibration import (
    CalibrationPoint,
    read_config_dict,
    require_calibration,
    save_calibration_point,
)
from apriltag.domain.config_loader import StageTolerance, load_config, validate_control_config

# the shipped config at the workspace root (src/apriltag/test -> ../../..)
REPO_CONFIG = pathlib.Path(__file__).resolve().parents[3] / 'april_tag_config.json'

VALID = {
    'detection': {'detect_tag_families': 'tag36h11',
                  'tracking_tag_family': 'tag36h11', 'tracking_tag_id': 0},
    'tag_size': {'tag_size': 0.0635, 'tag_sizes': ''},
    'control': {'manage_amcl_and_lidar_safety': False,
                'stage1_tolerance': {'x': 0.05, 'y': 0.05, 'yaw': 0.05, 'hold_seconds': 1.0},
                'stage2_tolerance': {'x': 0.02, 'y': 0.03, 'yaw': 0.04, 'hold_seconds': 0.5},
                'dock_timeout': 300.0, 'undock_timeout': 300.0, 'lost_target_timeout': 1.0,
                'max_vx': 0.1, 'max_vy': 0.1, 'max_vw': 0.05,
                'min_vx': 0.01, 'min_vy': 0.01, 'min_vw': 0.01},
    'calibration': {'stage1': {'z_ref': 0.5, 'x_ref': 0.026, 'yaw_ref': 0.0},
                    'stage2': {'z_ref': 0.28, 'x_ref': 0.01, 'yaw_ref': 0.02},
                    'detach': {'z_ref': 1.0, 'x_ref': 0.026, 'yaw_ref': 0.0}},
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
    validate_control_config(load_config(str(REPO_CONFIG)))


VIRTUAL_CONFIG = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'virtual_tracking_config.json'


def test_virtual_config_is_valid():
    tol, cal = validate_control_config(load_config(str(VIRTUAL_CONFIG)))
    # the sim's ground truth assumes the previous defaults
    assert [cal[k].z_ref for k in ('stage1', 'stage2', 'detach')] == [0.5, 0.28, 1.0]
    assert all(cal[k].x_ref == pytest.approx(0.026) and cal[k].yaw_ref == 0.0 for k in cal)


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
    ('control', 'dock_timeout', True),                # bool is not a number
    ('control', 'dock_timeout', -1.0),
    ('control', 'manage_amcl_and_lidar_safety', 1),   # int is not a bool
    ('control', 'max_vx', 0.0),                       # must be positive
    ('control', 'max_vy', True),                      # bool is not a number
    ('control', 'min_vx', -0.01),                     # must be >= 0
    ('control', 'min_vw', 0.06),                      # min > max_vw
    ('detection', 'tracking_tag_id', False),          # bool is not an int
    ('detection', 'tracking_tag_id', 0.0),
    ('detection', 'tracking_tag_id', -2),
    ('detection', 'tracking_tag_family', None),
    ('tag_size', 'tag_size', 0.0),
    ('tag_size', 'tag_sizes', 3),
    ('yaw_estimator', 's4_distances', []),
    ('yaw_estimator', 's4_distances', [0.3, 0.5]),    # length mismatch
    ('yaw_estimator', 's4_sigmas_deg', [1.0, True, 3.0]),
    ('ros_interfaces', 'stage1_cmd_vel_topic', None),
])
def test_invalid_value(tmp_path, section, key, value):
    with pytest.raises(ValueError, match=key):
        load_config(_write(tmp_path, _with(section, key, value)))


@pytest.mark.parametrize('key', ['max_vx', 'max_vy', 'max_vw', 'min_vx', 'min_vy', 'min_vw'])
def test_missing_speed_limit(tmp_path, key):
    data = copy.deepcopy(VALID)
    del data['control'][key]
    with pytest.raises(ValueError, match=key):
        load_config(_write(tmp_path, data))


def test_tracking_tag_id_any_is_valid(tmp_path):
    load_config(_write(tmp_path, _with('detection', 'tracking_tag_id', -1)))


def test_load_config_does_not_need_calibration_points(tmp_path):
    # apriltag_detection / tools/calibrate.py work before calibrating
    data = copy.deepcopy(VALID)
    data['calibration'] = {'stage1': None, 'stage2': None, 'detach': None}
    load_config(_write(tmp_path, data))
    data['calibration'] = {}
    load_config(_write(tmp_path, data))


def test_control_config():
    tol, cal = validate_control_config(copy.deepcopy(VALID))
    assert tol['stage1'] == StageTolerance(0.05, 0.05, 0.05, 1.0)
    assert tol['stage2'] == StageTolerance(0.02, 0.03, 0.04, 0.5)
    assert cal['stage2'].z_ref == pytest.approx(0.28)
    assert cal['stage2'].x_ref == pytest.approx(0.01)
    assert cal['stage2'].yaw_ref == pytest.approx(0.02)
    assert cal['detach'].z_ref == pytest.approx(1.0)


def test_hold_seconds_zero_is_valid():
    data = copy.deepcopy(VALID)
    data['control']['stage1_tolerance']['hold_seconds'] = 0
    assert validate_control_config(data)[0]['stage1'].hold_seconds == 0.0


@pytest.mark.parametrize('section,key,hint', [
    ('control', 'stage1_distance', 'calibration.stage1.z_ref'),
    ('control', 'stage2_distance', 'calibration.stage2.z_ref'),
    ('control', 'leave_distance', 'calibration.detach.z_ref'),
    ('calibration', 'use_calibration', 'always used'),
])
def test_removed_key_rejected(section, key, hint):
    data = copy.deepcopy(VALID)
    data[section][key] = 0.5
    with pytest.raises(ValueError, match=f'{section}.{key} was removed.*{hint}'):
        validate_control_config(data)


@pytest.mark.parametrize('stage', ['stage1', 'stage2'])
@pytest.mark.parametrize('field,value', [
    (None, None),                 # tolerance object missing
    (None, 0.05),                 # not an object
    ('x', None),                  # field missing
    ('y', True),                  # bool is not a number
    ('yaw', '0.05'),
    ('x', 0.0),                   # must be positive
    ('yaw', -0.01),
    ('hold_seconds', -1.0),       # must be >= 0
    ('hold_seconds', float('nan')),
])
def test_invalid_tolerance(stage, field, value):
    data = copy.deepcopy(VALID)
    key = f'{stage}_tolerance'
    if field is None:
        if value is None:
            del data['control'][key]
        else:
            data['control'][key] = value
    elif value is None:
        del data['control'][key][field]
    else:
        data['control'][key][field] = value
    with pytest.raises(ValueError, match=f'control.{key}'):
        validate_control_config(data)


@pytest.mark.parametrize('stage', ['stage1', 'stage2', 'detach'])
@pytest.mark.parametrize('entry', [
    None,
    {'x_ref': 0.0, 'yaw_ref': 0.0},                    # z_ref missing
    {'z_ref': 0.0, 'x_ref': 0.0, 'yaw_ref': 0.0},      # z_ref must be > 0
    {'z_ref': 0.5, 'x_ref': True, 'yaw_ref': 0.0},     # bool is not a number
    {'z_ref': 0.5, 'x_ref': 0.0, 'yaw_ref': '0'},
    {'z_ref': float('inf'), 'x_ref': 0.0, 'yaw_ref': 0.0},
])
def test_invalid_calibration_point(stage, entry):
    data = copy.deepcopy(VALID)
    data['calibration'][stage] = entry
    with pytest.raises(ValueError, match=f'calibration.{stage}'):
        validate_control_config(data)
    with pytest.raises(ValueError, match=f'calibration.{stage}'):
        require_calibration(data)


def test_save_calibration_keeps_other_sections(tmp_path):
    path = _write(tmp_path, VALID)
    save_calibration_point(path, 'stage2', CalibrationPoint(z_ref=0.3, x_ref=0.0, yaw_ref=0.0))
    data = load_config(path)   # still a valid config
    for section in VALID:
        if section != 'calibration':
            assert data[section] == VALID[section]
    assert data['calibration']['stage1'] == VALID['calibration']['stage1']
    assert 'use_calibration' not in data['calibration']
    assert require_calibration(data)['stage2'].z_ref == pytest.approx(0.3)


def test_save_calibration_creates_section(tmp_path):
    data = copy.deepcopy(VALID)
    del data['calibration']
    path = _write(tmp_path, data)
    save_calibration_point(path, 'detach', CalibrationPoint(z_ref=1.0, x_ref=0.0, yaw_ref=0.0))
    saved = json.loads(pathlib.Path(path).read_text())
    assert list(saved['calibration']) == ['detach']


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
