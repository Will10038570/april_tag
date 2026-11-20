"""Load and validate april_tag_config.json."""

import json
import math
import os
from dataclasses import dataclass
from typing import Dict, Tuple

from apriltag.domain.calibration import CalibrationPoint, require_calibration
from apriltag.runtime.control_flow import validate_speed_limits
from apriltag.runtime.yaw_probe import YawProbeConfig
from apriltag.yaw_estimator import YawEstimatorConfig

# keys that no longer exist, and what replaces them; only apriltag_control
# checks them (validate_control_config), so apriltag_detection still starts
REMOVED_KEYS = {
    ('control', 'stage1_distance'): 'set calibration.stage1.z_ref',
    ('control', 'stage2_distance'): 'set calibration.stage2.z_ref',
    ('control', 'leave_distance'): 'set calibration.detach.z_ref',
    ('calibration', 'use_calibration'): 'calibration is always used',
}

TOLERANCE_STAGES = ('stage1', 'stage2')


@dataclass(frozen=True)
class StageTolerance:
    """In-position tolerance of one stage: |x|, |y| (m), |yaw| (rad), held hold_seconds (s)."""
    x: float
    y: float
    yaw: float
    hold_seconds: float


@dataclass(frozen=True)
class Stage1Approach:
    """Stage 1 straight approach: forward vx scaled by ramps of |lateral| (m) and |yaw| (rad).

    Scale 1 at <= *_full, 0 at >= *_zero (see control_flow.approach_vx_scale).
    """
    enabled: bool
    lateral_full: float
    lateral_zero: float
    yaw_full: float
    yaw_zero: float


@dataclass(frozen=True)
class LqrConfig:
    """LQR weights: q_weights [x, y, yaw] with R fixed to 1, so each axis' gain
    K ~= sqrt(q) (1/s; the axes are decoupled), and the reference smoothing
    time constant smooth_tau (s, shared by all axes)."""
    q_weights: Tuple[float, float, float]
    smooth_tau: float


def load_config(path: str) -> dict:
    """Load april_tag_config.json.

    Raises ValueError if the file does not exist, cannot be parsed as JSON,
    or fails structural / value validation.
    """
    if not os.path.exists(path):
        raise ValueError(f'Config file not found: {path!r}')
    with open(path, 'r') as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as exc:
            raise ValueError(f'Cannot parse {path!r}: {exc}') from exc
    _validate_config(data)
    return data


def _require_section(data: dict, section: str) -> dict:
    sec = data.get(section)
    if not isinstance(sec, dict):
        raise ValueError(f'Missing or invalid section {section!r} in config')
    return sec


def _require_bool(sec: dict, section: str, key: str) -> bool:
    val = sec.get(key)
    if not isinstance(val, bool):
        raise ValueError(f'{section}.{key} must be a boolean, got {type(val).__name__!r}')
    return val


def _require_num(sec: dict, section: str, key: str) -> float:
    val = sec.get(key)
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        raise ValueError(f'{section}.{key} must be a number, got {type(val).__name__!r}')
    return float(val)


def _require_int(sec: dict, section: str, key: str) -> int:
    val = sec.get(key)
    if isinstance(val, bool) or not isinstance(val, int):
        raise ValueError(f'{section}.{key} must be an integer, got {type(val).__name__!r}')
    return int(val)


def _require_str(sec: dict, section: str, key: str) -> str:
    val = sec.get(key)
    if not isinstance(val, str):
        raise ValueError(f'{section}.{key} must be a string, got {type(val).__name__!r}')
    return val


def _require_num_list(sec: dict, section: str, key: str) -> list:
    val = sec.get(key)
    if not isinstance(val, list):
        raise ValueError(f'{section}.{key} must be a list, got {type(val).__name__!r}')
    for i, v in enumerate(val):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(
                f'{section}.{key}[{i}] must be a number, got {type(v).__name__!r}')
    return [float(v) for v in val]


def _validate_config(data: dict) -> None:
    det = _require_section(data, 'detection')
    _require_str(det, 'detection', 'detect_tag_families')
    _require_str(det, 'detection', 'tracking_tag_family')
    tid = _require_int(det, 'detection', 'tracking_tag_id')
    if tid < -1:
        raise ValueError('detection.tracking_tag_id must be >= -1')

    ts = _require_section(data, 'tag_size')
    tag_size = _require_num(ts, 'tag_size', 'tag_size')
    if tag_size <= 0.0:
        raise ValueError('tag_size.tag_size must be positive')
    _require_str(ts, 'tag_size', 'tag_sizes')

    ctrl = _require_section(data, 'control')
    _require_bool(ctrl, 'control', 'manage_amcl_and_lidar_safety')
    for key in ('dock_timeout', 'undock_timeout', 'lost_target_timeout'):
        v = _require_num(ctrl, 'control', key)
        if v <= 0.0:
            raise ValueError(f'control.{key} must be positive')
    speed = {key: _require_num(ctrl, 'control', key)
             for key in ('max_vx', 'max_vy', 'max_vw', 'min_vx', 'min_vy', 'min_vw',
                         'min_vw_move')}
    reason = validate_speed_limits(**speed)
    if reason is not None:
        raise ValueError(f'control.{reason}')

    # only its presence: the points are checked by validate_control_config, so
    # apriltag_detection / tools/calibrate.py work before calibrating
    _require_section(data, 'calibration')

    ye = _require_section(data, 'yaw_estimator')
    dists = _require_num_list(ye, 'yaw_estimator', 's4_distances')
    sigs = _require_num_list(ye, 'yaw_estimator', 's4_sigmas_deg')
    if not dists:
        raise ValueError('yaw_estimator.s4_distances must not be empty')
    if len(dists) != len(sigs):
        raise ValueError(
            'yaw_estimator.s4_distances and s4_sigmas_deg must have equal length')

    ri = _require_section(data, 'ros_interfaces')
    for key in ('amcl_check_service', 'amcl_close_service', 'amcl_open_service',
                'lidar_safety_topic', 'stage1_cmd_vel_topic', 'stage2_cmd_vel_topic'):
        _require_str(ri, 'ros_interfaces', key)


def _parse_tolerance(ctrl: dict, key: str) -> StageTolerance:
    name = f'control.{key}'
    sec = ctrl.get(key)
    if not isinstance(sec, dict):
        raise ValueError(f'Missing or invalid {name} (an object with x, y, yaw, hold_seconds)')
    values = {}
    for field in ('x', 'y', 'yaw', 'hold_seconds'):
        v = sec.get(field)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f'{name}.{field} must be a number, got {type(v).__name__!r}')
        if field == 'hold_seconds':
            if v < 0.0:
                raise ValueError(f'{name}.{field} must be >= 0')
        elif v <= 0.0:
            raise ValueError(f'{name}.{field} must be positive')
        values[field] = float(v)
    return StageTolerance(**values)


def validate_control_config(
        data: dict) -> Tuple[Dict[str, StageTolerance], Dict[str, CalibrationPoint]]:
    """apriltag_control's own checks on a config from load_config().

    Rejects removed keys, and returns the in-position tolerances
    {'stage1', 'stage2'} and the calibration points {'stage1', 'stage2',
    'detach'}, all required. Raises ValueError naming the bad field.
    """
    for (section, key), hint in REMOVED_KEYS.items():
        if key in data.get(section, {}):
            raise ValueError(f'{section}.{key} was removed; {hint}')
    ctrl = data['control']
    tolerances = {s: _parse_tolerance(ctrl, f'{s}_tolerance') for s in TOLERANCE_STAGES}
    return tolerances, require_calibration(data)


def parse_stage1_approach(data: dict) -> Stage1Approach:
    """control.stage1_approach (required) of a config from load_config().

    enabled: bool; lateral_full / lateral_zero (m), yaw_full / yaw_zero
    (rad): numbers with 0 <= full < zero. Raises ValueError naming the bad field.
    """
    name = 'control.stage1_approach'
    sec = data.get('control', {}).get('stage1_approach')
    if not isinstance(sec, dict):
        raise ValueError(f'Missing or invalid {name} (an object with enabled, lateral_full, '
                         'lateral_zero, yaw_full, yaw_zero)')
    enabled = sec.get('enabled')
    if not isinstance(enabled, bool):
        raise ValueError(f'{name}.enabled must be a bool, got {type(enabled).__name__!r}')
    values = {}
    for field in ('lateral_full', 'lateral_zero', 'yaw_full', 'yaw_zero'):
        v = sec.get(field)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f'{name}.{field} must be a number, got {type(v).__name__!r}')
        values[field] = float(v)
    for axis in ('lateral', 'yaw'):
        full, zero = values[f'{axis}_full'], values[f'{axis}_zero']
        if full < 0.0:
            raise ValueError(f'{name}.{axis}_full ({full}) must be >= 0')
        if zero <= full:
            raise ValueError(f'{name}.{axis}_zero ({zero}) must be > {axis}_full ({full})')
    return Stage1Approach(enabled=enabled, **values)


def parse_per_axis_hold(data: dict) -> bool:
    """control.per_axis_hold (required bool) of a config from load_config().

    True: in Stage 1 / 2 each axis within its stage tolerance gets 0 (not
    corrected); with all three within, one zero command is sent and nothing
    more until an axis leaves. Raises ValueError if missing or not a bool.
    """
    v = data.get('control', {}).get('per_axis_hold')
    if not isinstance(v, bool):
        raise ValueError(f'control.per_axis_hold must be a bool, got {type(v).__name__!r}')
    return v


def parse_lqr(data: dict) -> LqrConfig:
    """control.lqr (required) of a config from load_config().

    q_weights: list of 3 finite numbers [x, y, yaw], each > 0; smooth_tau:
    finite number > 0 (s). Raises ValueError naming the bad field.
    """
    name = 'control.lqr'
    sec = data.get('control', {}).get('lqr')
    if not isinstance(sec, dict):
        raise ValueError(f'Missing or invalid {name} (an object with q_weights, smooth_tau)')
    q = sec.get('q_weights')
    if not isinstance(q, list) or len(q) != 3:
        raise ValueError(f'{name}.q_weights must be a list of 3 numbers [x, y, yaw]')
    weights = []
    for i, v in enumerate(q):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f'{name}.q_weights[{i}] must be a number, got {type(v).__name__!r}')
        if v <= 0.0:
            raise ValueError(f'{name}.q_weights[{i}] ({v}) must be positive')
        weights.append(float(v))
    tau = sec.get('smooth_tau')
    if isinstance(tau, bool) or not isinstance(tau, (int, float)) or not math.isfinite(tau):
        raise ValueError(f'{name}.smooth_tau must be a number, got {type(tau).__name__!r}')
    if tau <= 0.0:
        raise ValueError(f'{name}.smooth_tau ({tau}) must be positive')
    return LqrConfig(q_weights=tuple(weights), smooth_tau=float(tau))


# ---- yaw_estimator / yaw_probe (apriltag_control only) ---------------------------

def _field_num(sec: dict, name: str, key: str, kind: str, rule: str) -> float:
    """One numeric field of section `name`.

    kind: 'int' (no bool / float) or 'float' (int accepted, finite);
    rule: '>0', '>=0', '>=1' or '(0,1]'. Raises ValueError naming the field.
    """
    v = sec.get(key)
    if kind == 'int':
        if isinstance(v, bool) or not isinstance(v, int):
            raise ValueError(f'{name}.{key} must be an integer, got {type(v).__name__!r}')
    elif isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f'{name}.{key} must be a number, got {v!r}')
    ok = {'>0': v > 0, '>=0': v >= 0, '>=1': v >= 1, '(0,1]': 0 < v <= 1}[rule]
    if not ok:
        raise ValueError(f'{name}.{key} ({v}) must be ' + {
            '>0': 'positive', '>=0': '>= 0', '>=1': '>= 1', '(0,1]': 'in (0, 1]'}[rule])
    return int(v) if kind == 'int' else float(v)


def _section_with_doc(data: dict, name: str) -> dict:
    sec = data.get(name)
    if not isinstance(sec, dict):
        raise ValueError(f'Missing or invalid {name} in config')
    if '_doc' in sec and not isinstance(sec['_doc'], dict):
        raise ValueError(f'{name}._doc must be an object (key -> description)')
    return sec


# key -> (kind, rule); keys are the YawEstimatorConfig fields (s4 lists apart)
YAW_ESTIMATOR_FIELDS = {
    'border_px': ('float', '>=0'),
    'margin_lo': ('float', '>=0'),
    'margin_hi': ('float', '>0'),
    'sigma_dh': ('float', '>0'),
    'snr_lo': ('float', '>=0'),
    'snr_hi': ('float', '>0'),
    'ref_tag_size': ('float', '>0'),
    'q_proc': ('float', '>=0'),
    'gate_sigma': ('float', '>0'),
    'gate_min_deg': ('float', '>=0'),
    'accept_frames': ('int', '>=1'),
    'consistency_deg': ('float', '>0'),
    'abs_window': ('int', '>=1'),
    'xy_tau': ('float', '>=0'),
}

# key -> (kind, rule); keys are the YawProbeConfig fields (initial_direction apart)
YAW_PROBE_FIELDS = {
    'trigger_frames': ('int', '>=1'),
    'c_sign_low': ('float', '(0,1]'),
    'trigger_yaw': ('float', '>0'),
    'wz': ('float', '>0'),
    'turn_time': ('float', '>0'),
    'settle_frames': ('int', '>=1'),
    'end_yaw_deg': ('float', '>0'),
    'change_deg': ('float', '>=0'),
    'end_c_sign': ('float', '(0,1]'),
    'max_turns': ('int', '>=1'),
}


def parse_yaw_estimator(data: dict) -> YawEstimatorConfig:
    """yaw_estimator (all keys required) of a config from load_config().

    s4_distances / s4_sigmas_deg: non-empty lists of equal length; the other
    keys are the YawEstimatorConfig fields (see YAW_ESTIMATOR_FIELDS), with
    margin_lo < margin_hi and snr_lo < snr_hi. `_doc` (optional) is ignored.
    Raises ValueError naming the bad field.
    """
    name = 'yaw_estimator'
    sec = _section_with_doc(data, name)
    dists = _require_num_list(sec, name, 's4_distances')
    sigs = _require_num_list(sec, name, 's4_sigmas_deg')
    if not dists:
        raise ValueError('yaw_estimator.s4_distances must not be empty')
    if len(dists) != len(sigs):
        raise ValueError('yaw_estimator.s4_distances and s4_sigmas_deg must have equal length')
    values = {k: _field_num(sec, name, k, kind, rule)
              for k, (kind, rule) in YAW_ESTIMATOR_FIELDS.items()}
    for lo, hi in (('margin_lo', 'margin_hi'), ('snr_lo', 'snr_hi')):
        if not values[lo] < values[hi]:
            raise ValueError(f'{name}.{lo} ({values[lo]}) must be < {hi} ({values[hi]})')
    return YawEstimatorConfig(s4_distances=dists, s4_sigmas_deg=sigs, **values)


def parse_yaw_probe(data: dict) -> YawProbeConfig:
    """yaw_probe (required, all keys) of a config from load_config().

    Keys are the YawProbeConfig fields (see YAW_PROBE_FIELDS);
    initial_direction must be 1 or -1. `_doc` (optional) is ignored.
    Raises ValueError naming the bad field.
    """
    name = 'yaw_probe'
    sec = _section_with_doc(data, name)
    values = {k: _field_num(sec, name, k, kind, rule)
              for k, (kind, rule) in YAW_PROBE_FIELDS.items()}
    d = sec.get('initial_direction')
    if isinstance(d, bool) or not isinstance(d, (int, float)) or d not in (1, -1):
        raise ValueError(f'{name}.initial_direction must be 1 or -1, got {d!r}')
    return YawProbeConfig(initial_direction=float(d), **values)
