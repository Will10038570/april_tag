"""Calibration reference points for apriltag_control.

Each stage (stage1, stage2, detach) can have a CalibrationPoint that records
the tag pose at the desired docking/leaving position. When use_calibration is
true, apriltag_control uses z_ref in place of the default stage distance and
applies x_ref / yaw_ref offsets to the control error so the robot converges to
the recorded pose rather than a fixed distance + zero lateral / yaw.
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


def load_calibration(path: str) -> Dict[str, Optional[CalibrationPoint]]:
    """Load calibration points from a JSON file.

    Returns a dict keyed by stage name ('stage1', 'stage2', 'detach'), each
    value either a CalibrationPoint or None (not found / invalid).

    Returns all-None silently when the file does not exist (not an error:
    the user has not calibrated yet).
    Raises ValueError when the file exists but cannot be parsed as JSON.
    Per-stage entries with missing / invalid fields are silently set to None.
    """
    result: Dict[str, Optional[CalibrationPoint]] = {s: None for s in _STAGES}
    if not os.path.exists(path):
        return result
    with open(path, 'r') as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as exc:
            raise ValueError(f'Cannot parse calibration file {path!r}: {exc}') from exc
    for stage in _STAGES:
        entry = data.get(stage)
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


def save_calibration_point(path: str, stage: str, point: CalibrationPoint) -> None:
    """Update one stage in a calibration JSON file (read-modify-write).

    Creates the file and its parent directories if they do not exist.
    Raises ValueError for an unknown stage name.
    """
    if stage not in _STAGES:
        raise ValueError(f'Unknown calibration stage {stage!r}; must be one of {_STAGES}')
    data: dict = {}
    if os.path.exists(path):
        with open(path, 'r') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                data = {}   # corrupt file: overwrite
    data[stage] = asdict(point)
    dirpath = os.path.dirname(path)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
