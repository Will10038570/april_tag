"""Calibration reference points for apriltag_control.

Each stage (stage1, stage2, detach) has a CalibrationPoint that records the
tag pose at the desired docking / leaving position. apriltag_control requires
all three: z_ref is the stage's target forward distance (detach: the leave
distance), and x_ref / yaw_ref offset the control error so the robot
converges to the recorded pose.

Calibration data is stored in the 'calibration' section of april_tag_config.json
and written there by tools/calibrate.py.
"""

import json
import math
import os
from dataclasses import asdict, dataclass
from typing import Dict, Optional

_STAGES = ('stage1', 'stage2', 'detach')


@dataclass
class CalibrationPoint:
    z_ref: float            # optical frame forward distance (m) at calibrated position, > 0
    x_ref: float            # optical frame lateral (x) at calibrated position (m)
    yaw_ref: float          # yaw at calibrated position (rad)
    stamp: Optional[float] = None        # epoch seconds — metadata only, not used in control
    image_path: Optional[str] = None     # saved PNG path — metadata only


def require_calibration(config: dict) -> Dict[str, CalibrationPoint]:
    """Return the calibration points {'stage1', 'stage2', 'detach'} of a config dict.

    Every point must be an object with numeric (not bool), finite z_ref,
    x_ref, yaw_ref and z_ref > 0; otherwise ValueError names the point.
    """
    cal = config.get('calibration')
    if not isinstance(cal, dict):
        raise ValueError("Missing or invalid section 'calibration' in config")
    result: Dict[str, CalibrationPoint] = {}
    for stage in _STAGES:
        entry = cal.get(stage)
        bad = (f'calibration.{stage} is missing or incomplete '
               '(need numeric z_ref, x_ref, yaw_ref; z_ref > 0)')
        if not isinstance(entry, dict):
            raise ValueError(bad)
        values = []
        for key in ('z_ref', 'x_ref', 'yaw_ref'):
            v = entry.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(bad)
            values.append(float(v))
        if values[0] <= 0.0:
            raise ValueError(bad)
        result[stage] = CalibrationPoint(
            z_ref=values[0], x_ref=values[1], yaw_ref=values[2],
            stamp=entry.get('stamp'), image_path=entry.get('image_path'))
    return result


def read_config_dict(path: str) -> dict:
    """Read april_tag_config.json as a dict, without validating its sections.

    Raises ValueError if the file does not exist, is not valid JSON, or its
    top level is not a JSON object. Used by tools/calibrate.py before waiting
    for a tag, and by save_calibration_point before writing.
    """
    abspath = os.path.abspath(path)
    if not os.path.exists(path):
        raise ValueError(f'Config file not found: {abspath!r}')
    with open(path, 'r') as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as exc:
            raise ValueError(f'Cannot parse {abspath!r}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(
            f'{abspath!r} must contain a JSON object, got {type(data).__name__!r}')
    return data


def save_calibration_point(path: str, stage: str, point: CalibrationPoint) -> None:
    """Update one stage in april_tag_config.json (read-modify-write).

    Reads the full config JSON, updates only config['calibration'][stage],
    and writes the whole file back.
    Raises ValueError for an unknown stage name, or (via read_config_dict) when
    the file does not exist, is not valid JSON or is not a JSON object; the
    file is not written (nor created) in those cases.
    """
    if stage not in _STAGES:
        raise ValueError(f'Unknown calibration stage {stage!r}; must be one of {_STAGES}')
    data = read_config_dict(path)
    if not isinstance(data.get('calibration'), dict):
        data['calibration'] = {}
    data['calibration'][stage] = asdict(point)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
