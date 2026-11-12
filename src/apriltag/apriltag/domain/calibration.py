"""Calibration reference points for apriltag_control.

Each stage (stage1, stage2, detach) can have a CalibrationPoint that records
the tag pose at the desired docking/leaving position. When use_calibration is
true, apriltag_control uses z_ref in place of the default stage distance and
applies x_ref / yaw_ref offsets to the control error so the robot converges to
the recorded pose rather than a fixed distance + zero lateral / yaw.

Calibration data is stored in the 'calibration' section of april_tag_config.json
and written there by tools/calibrate.py.
"""

import json
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


def load_calibration_from_config(config: dict) -> Dict[str, Optional[CalibrationPoint]]:
    """Extract calibration points from a loaded april_tag_config dict.

    Returns a dict keyed by stage name ('stage1', 'stage2', 'detach'), each
    value either a CalibrationPoint or None (not found / invalid).
    Per-stage entries with missing / invalid fields are silently set to None.
    """
    result: Dict[str, Optional[CalibrationPoint]] = {s: None for s in _STAGES}
    cal = config.get('calibration', {})
    for stage in _STAGES:
        entry = cal.get(stage)
        if not isinstance(entry, dict):
            continue
        try:
            z_ref = float(entry['z_ref'])
            x_ref = float(entry['x_ref'])
            yaw_ref = float(entry['yaw_ref'])
        except (KeyError, TypeError, ValueError):
            continue
        if z_ref <= 0.0:
            continue
        result[stage] = CalibrationPoint(
            z_ref=z_ref,
            x_ref=x_ref,
            yaw_ref=yaw_ref,
            stamp=entry.get('stamp'),
            image_path=entry.get('image_path'),
        )
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
        data['calibration'] = {'use_calibration': True}
    data['calibration'][stage] = asdict(point)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
