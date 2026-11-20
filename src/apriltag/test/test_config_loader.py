"""april_tag_config.json: load_config / validate_control_config and calibration read / write."""

import copy
import dataclasses
import json
import pathlib

import pytest

from apriltag.domain.calibration import (
    CalibrationPoint,
    read_config_dict,
    require_calibration,
    save_calibration_point,
)
from apriltag.domain.config_loader import (
    LqrConfig,
    Stage1Approach,
    StageTolerance,
    load_config,
    parse_lqr,
    parse_per_axis_hold,
    parse_stage1_approach,
    parse_yaw_estimator,
    parse_yaw_probe,
    validate_control_config,
)
from apriltag.runtime.yaw_probe import YawProbeConfig
from apriltag.yaw_estimator import YawEstimatorConfig

# dataclass defaults (no tag-size literals here, see test_tag_size.py)
EST_DEFAULTS = {k: v for k, v in dataclasses.asdict(YawEstimatorConfig()).items()
                if not k.startswith('s4_')}
PROBE_DEFAULTS = dataclasses.asdict(YawProbeConfig())
PROBE_DEFAULTS['initial_direction'] = 1

# the shipped config at the workspace root (src/apriltag/test -> ../../..)
REPO_CONFIG = pathlib.Path(__file__).resolve().parents[3] / 'april_tag_config.json'

VALID = {
    'detection': {'detect_tag_families': 'tag36h11',
                  'tracking_tag_family': 'tag36h11', 'tracking_tag_id': 0},
    'tag_size': {'tag_size': 0.0635, 'tag_sizes': ''},
    'control': {'manage_amcl_and_lidar_safety': False, 'per_axis_hold': True,
                'stage1_tolerance': {'x': 0.05, 'y': 0.05, 'yaw': 0.05, 'hold_seconds': 1.0},
                'stage2_tolerance': {'x': 0.02, 'y': 0.03, 'yaw': 0.04, 'hold_seconds': 0.5},
                'stage1_approach': {'enabled': True, 'lateral_full': 0.03, 'lateral_zero': 0.08,
                                    'yaw_full': 0.035, 'yaw_zero': 0.105},
                'lqr': {'q_weights': [0.3333, 1.0, 0.1667], 'smooth_tau': 0.5},
                'dock_timeout': 300.0, 'undock_timeout': 300.0, 'lost_target_timeout': 1.0,
                'max_vx': 0.1, 'max_vy': 0.1, 'max_vw': 0.05,
                'min_vx': 0.01, 'min_vy': 0.01, 'min_vw': 0.01,
                'min_vw_move': 0.01},
    'calibration': {'stage1': {'z_ref': 0.5, 'x_ref': 0.026, 'yaw_ref': 0.0},
                    'stage2': {'z_ref': 0.28, 'x_ref': 0.01, 'yaw_ref': 0.02},
                    'detach': {'z_ref': 1.0}},
    'yaw_estimator': {'s4_distances': [0.3, 0.5, 0.7], 's4_sigmas_deg': [1.0, 2.0, 3.0],
                      **EST_DEFAULTS},
    'yaw_probe': dict(PROBE_DEFAULTS),
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
    data = load_config(str(REPO_CONFIG))
    validate_control_config(data)
    parse_stage1_approach(data)
    assert parse_per_axis_hold(data) is True
    parse_lqr(data)
    parse_yaw_estimator(data)
    parse_yaw_probe(data)


def test_shipped_configs_clean():
    cal = json.loads(REPO_CONFIG.read_text())['calibration']
    assert set(cal['stage1']) == set(cal['stage2']) == {'z_ref', 'x_ref', 'yaw_ref'}
    assert list(cal['detach']) == ['z_ref']


def test_missing_file(tmp_path):
    with pytest.raises(ValueError, match='not found'):
        load_config(str(tmp_path / 'nope.json'))


def test_bad_json(tmp_path):
    p = tmp_path / 'bad.json'
    p.write_text('{"detection": ')
    with pytest.raises(ValueError, match='Cannot parse'):
        load_config(str(p))


# yaw_probe is checked by apriltag_control only (parse_yaw_probe, see
# test_yaw_sections_not_required_by_load_config)
@pytest.mark.parametrize('section', [s for s in VALID if s != 'yaw_probe'])
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
    ('control', 'min_vw_move', -0.01),                # must be >= 0
    ('control', 'min_vw_move', 0.06),                 # > max_vw
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


@pytest.mark.parametrize('key', ['max_vx', 'max_vy', 'max_vw', 'min_vx', 'min_vy', 'min_vw',
                                 'min_vw_move'])
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


_BAD_Z = [
    None,
    {'x_ref': 0.0, 'yaw_ref': 0.0},                    # z_ref missing
    {'z_ref': 0.0, 'x_ref': 0.0, 'yaw_ref': 0.0},      # z_ref must be > 0
    {'z_ref': True},                                   # bool is not a number
    {'z_ref': float('inf'), 'x_ref': 0.0, 'yaw_ref': 0.0},
]
_BAD_X_YAW = [
    {'z_ref': 0.5, 'x_ref': True, 'yaw_ref': 0.0},     # bool is not a number
    {'z_ref': 0.5, 'x_ref': 0.0, 'yaw_ref': '0'},
    {'z_ref': 0.5, 'yaw_ref': 0.0},                    # x_ref missing
]


@pytest.mark.parametrize('stage, entry',
                         [(s, e) for s in ('stage1', 'stage2', 'detach') for e in _BAD_Z]
                         + [(s, e) for s in ('stage1', 'stage2') for e in _BAD_X_YAW])
def test_invalid_calibration_point(stage, entry):
    data = copy.deepcopy(VALID)
    data['calibration'][stage] = entry
    with pytest.raises(ValueError, match=f'calibration.{stage}'):
        validate_control_config(data)
    with pytest.raises(ValueError, match=f'calibration.{stage}'):
        require_calibration(data)


@pytest.mark.parametrize('entry', [
    {'z_ref': 1.2},
    # old keys are ignored: x_ref / yaw_ref are not used by detach
    {'z_ref': 1.2, 'x_ref': 0.5, 'yaw_ref': 0.3, 'stamp': 123.0, 'image_path': '/x.png'},
    {'z_ref': 1.2, 'x_ref': True, 'yaw_ref': '0'},
])
def test_detach_only_z_ref(entry):
    data = copy.deepcopy(VALID)
    data['calibration']['detach'] = entry
    point = validate_control_config(data)[1]['detach']
    assert (point.z_ref, point.x_ref, point.yaw_ref) == (pytest.approx(1.2), 0.0, 0.0)


def test_extra_calibration_keys_ignored():
    data = copy.deepcopy(VALID)
    data['calibration']['stage1'].update(stamp=1700000000.0, image_path='/mnt/work_space/a.png')
    point = validate_control_config(data)[1]['stage1']
    assert (point.z_ref, point.x_ref, point.yaw_ref) == (0.5, pytest.approx(0.026), 0.0)


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


def test_save_stage_writes_three_keys(tmp_path):
    path = _write(tmp_path, VALID)
    save_calibration_point(path, 'stage2', CalibrationPoint(z_ref=0.3, x_ref=0.01, yaw_ref=-0.02))
    saved = json.loads(pathlib.Path(path).read_text())['calibration']['stage2']
    assert saved == {'z_ref': 0.3, 'x_ref': 0.01, 'yaw_ref': -0.02}


def test_save_detach_writes_only_z_ref(tmp_path):
    data = copy.deepcopy(VALID)
    # an old detach entry is replaced as a whole
    data['calibration']['detach'] = {'z_ref': 1.0, 'x_ref': 0.026, 'yaw_ref': 0.0,
                                     'stamp': None, 'image_path': None}
    path = _write(tmp_path, data)
    save_calibration_point(path, 'detach', CalibrationPoint(z_ref=0.9, x_ref=0.05, yaw_ref=0.1))
    saved = json.loads(pathlib.Path(path).read_text())['calibration']
    assert saved['detach'] == {'z_ref': 0.9}
    assert saved['stage1'] == VALID['calibration']['stage1']


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


# ---- control.stage1_approach -------------------------------------------------------

def test_stage1_approach_parsed():
    assert parse_stage1_approach(VALID) == Stage1Approach(True, 0.03, 0.08, 0.035, 0.105)


def _approach(**changes):
    data = copy.deepcopy(VALID)
    data['control']['stage1_approach'].update(changes)
    return data


def test_stage1_approach_missing():
    data = copy.deepcopy(VALID)
    del data['control']['stage1_approach']
    with pytest.raises(ValueError, match='Missing or invalid control.stage1_approach'):
        parse_stage1_approach(data)


@pytest.mark.parametrize('changes,match', [
    ({'enabled': 1}, 'enabled must be a bool'),
    ({'enabled': None}, 'enabled must be a bool'),
    ({'lateral_full': 'a'}, 'lateral_full must be a number'),
    ({'yaw_zero': True}, 'yaw_zero must be a number'),
    ({'yaw_zero': float('inf')}, 'yaw_zero must be a number'),
    ({'lateral_full': -0.01}, 'lateral_full .* must be >= 0'),
    ({'lateral_zero': 0.03}, 'lateral_zero .* must be > lateral_full'),
    ({'yaw_zero': 0.01}, 'yaw_zero .* must be > yaw_full'),
])
def test_stage1_approach_invalid(changes, match):
    with pytest.raises(ValueError, match=match):
        parse_stage1_approach(_approach(**changes))


def test_stage1_approach_missing_field():
    data = copy.deepcopy(VALID)
    del data['control']['stage1_approach']['yaw_full']
    with pytest.raises(ValueError, match='yaw_full must be a number'):
        parse_stage1_approach(data)



# ---- control.lqr ------------------------------------------------------------------

def test_lqr_parsed():
    assert parse_lqr(VALID) == LqrConfig((0.3333, 1.0, 0.1667), 0.5)


def test_lqr_ints_accepted():
    data = copy.deepcopy(VALID)
    data['control']['lqr'] = {'q_weights': [1, 2, 3], 'smooth_tau': 1}
    assert parse_lqr(data) == LqrConfig((1.0, 2.0, 3.0), 1.0)


@pytest.mark.parametrize('value', [True, False])
def test_per_axis_hold(value):
    assert parse_per_axis_hold(_with('control', 'per_axis_hold', value)) is value


@pytest.mark.parametrize('value', [1, 0, 'true', None, 1.0])
def test_per_axis_hold_not_bool(value):
    with pytest.raises(ValueError, match='control.per_axis_hold must be a bool'):
        parse_per_axis_hold(_with('control', 'per_axis_hold', value))


def test_per_axis_hold_missing():
    data = copy.deepcopy(VALID)
    del data['control']['per_axis_hold']
    with pytest.raises(ValueError, match='control.per_axis_hold must be a bool'):
        parse_per_axis_hold(data)


def test_per_axis_hold_not_required_by_load_config(tmp_path):
    # only apriltag_control requires it; detection and the sim use load_config alone
    data = copy.deepcopy(VALID)
    del data['control']['per_axis_hold']
    assert load_config(_write(tmp_path, data)) == data


def test_lqr_missing():
    data = copy.deepcopy(VALID)
    del data['control']['lqr']
    with pytest.raises(ValueError, match='Missing or invalid control.lqr'):
        parse_lqr(data)


def test_lqr_not_required_by_load_config(tmp_path):
    # only apriltag_control requires it; detection and the sim use load_config alone
    data = copy.deepcopy(VALID)
    del data['control']['lqr']
    assert load_config(_write(tmp_path, data)) == data


def _lqr(**changes):
    data = copy.deepcopy(VALID)
    data['control']['lqr'].update(changes)
    return data


@pytest.mark.parametrize('changes, match', [
    ({'q_weights': [1.0, 1.0]}, r'q_weights must be a list of 3'),
    ({'q_weights': [1.0, 1.0, 1.0, 1.0]}, r'q_weights must be a list of 3'),
    ({'q_weights': 1.0}, r'q_weights must be a list of 3'),
    ({'q_weights': [1.0, True, 1.0]}, r'q_weights\[1\] must be a number'),
    ({'q_weights': [1.0, 1.0, 'a']}, r'q_weights\[2\] must be a number'),
    ({'q_weights': [float('nan'), 1.0, 1.0]}, r'q_weights\[0\] must be a number'),
    ({'q_weights': [1.0, 0, 1.0]}, r'q_weights\[1\] \(0\) must be positive'),
    ({'q_weights': [1.0, 1.0, -1.0]}, r'q_weights\[2\] \(-1.0\) must be positive'),
    ({'smooth_tau': 0}, r'smooth_tau \(0\) must be positive'),
    ({'smooth_tau': -0.1}, r'smooth_tau \(-0.1\) must be positive'),
    ({'smooth_tau': True}, r'smooth_tau must be a number'),
    ({'smooth_tau': None}, r'smooth_tau must be a number'),
])
def test_lqr_invalid(changes, match):
    with pytest.raises(ValueError, match=match):
        parse_lqr(_lqr(**changes))


# ---- yaw_estimator / yaw_probe -----------------------------------------------------

def test_yaw_estimator_parsed():
    cfg = parse_yaw_estimator(VALID)
    assert cfg == YawEstimatorConfig(s4_distances=[0.3, 0.5, 0.7], s4_sigmas_deg=[1.0, 2.0, 3.0])


def test_yaw_probe_parsed():
    assert parse_yaw_probe(VALID) == YawProbeConfig()


def test_yaw_probe_negative_direction():
    data = copy.deepcopy(VALID)
    data['yaw_probe']['initial_direction'] = -1
    assert parse_yaw_probe(data).initial_direction == -1.0


@pytest.mark.parametrize('section', ['yaw_estimator', 'yaw_probe'])
def test_shipped_doc_keys(section):
    # every value has a description and every description a value
    sec = json.loads(REPO_CONFIG.read_text())[section]
    assert set(sec['_doc']) == set(sec) - {'_doc'}
    assert all(isinstance(v, str) and v for v in sec['_doc'].values())


def test_yaw_probe_missing():
    data = copy.deepcopy(VALID)
    del data['yaw_probe']
    with pytest.raises(ValueError, match='Missing or invalid yaw_probe'):
        parse_yaw_probe(data)


def test_yaw_estimator_field_missing():
    data = copy.deepcopy(VALID)
    del data['yaw_estimator']['sigma_dh']
    with pytest.raises(ValueError, match=r'yaw_estimator\.sigma_dh must be a number'):
        parse_yaw_estimator(data)


def test_yaw_sections_not_required_by_load_config(tmp_path):
    # only apriltag_control needs them; detection and the sim use load_config alone
    data = copy.deepcopy(VALID)
    del data['yaw_probe']
    data['yaw_estimator'] = {k: data['yaw_estimator'][k] for k in ('s4_distances', 's4_sigmas_deg')}
    assert load_config(_write(tmp_path, data)) == data


def test_yaw_doc_ignored():
    data = copy.deepcopy(VALID)
    data['yaw_estimator']['_doc'] = {'sigma_dh': 'text'}
    data['yaw_probe']['_doc'] = {}
    assert parse_yaw_estimator(data) == parse_yaw_estimator(VALID)
    assert parse_yaw_probe(data) == parse_yaw_probe(VALID)


def _yaw(section, **changes):
    data = copy.deepcopy(VALID)
    data[section].update(changes)
    return data


@pytest.mark.parametrize('changes, match', [
    ({'abs_window': 2.5}, r'abs_window must be an integer'),
    ({'accept_frames': 0}, r'accept_frames \(0\) must be >= 1'),
    ({'accept_frames': True}, r'accept_frames must be an integer'),
    ({'sigma_dh': 0}, r'sigma_dh \(0\) must be positive'),
    ({'ref_tag_size': -1.0}, r'ref_tag_size \(-1.0\) must be positive'),
    ({'border_px': -1}, r'border_px \(-1\) must be >= 0'),
    ({'q_proc': float('nan')}, r'q_proc must be a number'),
    ({'xy_tau': None}, r'xy_tau must be a number'),
    ({'margin_lo': 60.0}, r'margin_lo \(60.0\) must be < margin_hi'),
    ({'snr_lo': 2.0, 'snr_hi': 1.0}, r'snr_lo \(2.0\) must be < snr_hi'),
    ({'_doc': 'text'}, r'yaw_estimator\._doc must be an object'),
])
def test_yaw_estimator_invalid(changes, match):
    with pytest.raises(ValueError, match=match):
        parse_yaw_estimator(_yaw('yaw_estimator', **changes))


@pytest.mark.parametrize('changes, match', [
    ({'trigger_frames': 0}, r'trigger_frames \(0\) must be >= 1'),
    ({'max_turns': 1.0}, r'max_turns must be an integer'),
    ({'c_sign_low': 0}, r'c_sign_low \(0\) must be in \(0, 1\]'),
    ({'end_c_sign': 1.5}, r'end_c_sign \(1.5\) must be in \(0, 1\]'),
    ({'wz': 0}, r'wz \(0\) must be positive'),
    ({'turn_time': -1.0}, r'turn_time \(-1.0\) must be positive'),
    ({'change_deg': -0.5}, r'change_deg \(-0.5\) must be >= 0'),
    ({'trigger_yaw': float('inf')}, r'trigger_yaw must be a number'),
    ({'initial_direction': 0}, r'initial_direction must be 1 or -1'),
    ({'initial_direction': 2}, r'initial_direction must be 1 or -1'),
    ({'initial_direction': True}, r'initial_direction must be 1 or -1'),
    ({'_doc': ['x']}, r'yaw_probe\._doc must be an object'),
])
def test_yaw_probe_invalid(changes, match):
    with pytest.raises(ValueError, match=match):
        parse_yaw_probe(_yaw('yaw_probe', **changes))
