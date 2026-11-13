#!/usr/bin/env python3
"""Save an AprilTag calibration reference point.

Usage:
    python3 calibrate.py --stage1 [--file PATH]
    python3 calibrate.py --stage2 [--file PATH]
    python3 calibrate.py --detach [--file PATH]

Requires apriltag_detection (and camera) to be running.
apriltag_control does not need to be running.

Saves the tracked tag's z_ref / x_ref / yaw_ref (--detach: z_ref only) from
the current pose to the calibration section of april_tag_config.json
(read-modify-write, only the specified stage is updated).
--file must be an existing april_tag_config.json containing a JSON object;
otherwise it exits 1 at once, without waiting for a tag or writing anything.

Exit code: 0 success, 1 no valid tag / save failed, 2 argument error.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from apriltag_interfaces.msg import TagTracing

from apriltag.domain.calibration import CalibrationPoint, read_config_dict, save_calibration_point
from apriltag.domain.math_utils import quaternion_to_rotation_matrix, rotation_matrix_to_yaw_error

WAIT_TIMEOUT = 10.0
TAG_TRACING_TOPIC = '/wheelchair/tag_tracing'


def _parse_args():
    p = argparse.ArgumentParser(
        description='Save an AprilTag calibration reference point',
        epilog='Only one of --stage1 / --stage2 / --detach is accepted at a time.')
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--stage1', action='store_true', help='Save Stage 1 (approach) reference')
    group.add_argument('--stage2', action='store_true', help='Save Stage 2 (final dock) reference')
    group.add_argument('--detach', action='store_true', help='Save Detach (leave) reference, z_ref only')
    p.add_argument('--file', default='/mnt/work_space/april_tag_config.json',
                   help='path to april_tag_config.json (default: /mnt/work_space/april_tag_config.json)')
    args, _ = p.parse_known_args()
    stage = 'stage1' if args.stage1 else ('stage2' if args.stage2 else 'detach')
    return stage, args.file


def main() -> int:
    stage, cal_file = _parse_args()

    # fail fast: check the config file before waiting for a tag / saving a PNG
    try:
        read_config_dict(cal_file)
    except ValueError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1

    rclpy.init()
    node = Node('apriltag_calibrator')

    latest_tracing = None

    def tracing_cb(msg: TagTracing) -> None:
        nonlocal latest_tracing
        if msg.valid:
            latest_tracing = msg

    node.create_subscription(TagTracing, TAG_TRACING_TOPIC, tracing_cb, 1)

    print(f'Waiting up to {WAIT_TIMEOUT:.0f}s for a valid tag on {TAG_TRACING_TOPIC} ...')
    deadline = time.monotonic() + WAIT_TIMEOUT
    while time.monotonic() < deadline and latest_tracing is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if latest_tracing is None:
        print(
            f'ERROR: No valid tag received within {WAIT_TIMEOUT:.0f}s. '
            'Is apriltag_detection running and can it see the tag?',
            file=sys.stderr)
        return 1

    # extract pose from the tracked tag
    pos = latest_tracing.tag.pose.position
    ori = latest_tracing.tag.pose.orientation
    z_ref = float(pos.z)
    x_ref = float(pos.x)
    r_mat = quaternion_to_rotation_matrix(ori.x, ori.y, ori.z, ori.w)
    yaw_ref = rotation_matrix_to_yaw_error(r_mat)

    # save calibration point (read-modify-write; detach stores z_ref only)
    point = CalibrationPoint(z_ref=z_ref, x_ref=x_ref, yaw_ref=yaw_ref)
    try:
        save_calibration_point(cal_file, stage, point)
    except Exception as exc:
        print(f'ERROR: Failed to save calibration: {exc}', file=sys.stderr)
        return 1

    values = f'z_ref={z_ref:.4f}m'
    if stage != 'detach':
        values += f'  x_ref={x_ref:.4f}m  yaw_ref={yaw_ref:.4f}rad'
    print(f'Saved {stage}:  {values}  →  {cal_file}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
