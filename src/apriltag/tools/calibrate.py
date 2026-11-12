#!/usr/bin/env python3
"""Save an AprilTag calibration reference point.

Usage:
    python3 calibrate.py --stage1 [--file PATH]
    python3 calibrate.py --stage2 [--file PATH]
    python3 calibrate.py --detach [--file PATH]

Requires apriltag_detection (and camera) to be running.
apriltag_control does not need to be running.

Saves the tracked tag's z_ref / x_ref / yaw_ref from the current pose to the
calibration section of april_tag_config.json (read-modify-write, only the
specified stage is updated) and writes an annotated PNG alongside the JSON.

Exit code: 0 success, 1 no valid tag / save failed, 2 argument error.
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node

from apriltag_interfaces.msg import TagTracing
from sensor_msgs.msg import Image

from apriltag.domain.calibration import CalibrationPoint, save_calibration_point
from apriltag.domain.math_utils import quaternion_to_rotation_matrix, rotation_matrix_to_yaw_error

WAIT_TIMEOUT = 10.0
TAG_TRACING_TOPIC = '/wheelchair/tag_tracing'
MARKED_IMAGE_TOPIC = '/wheelchair/apriltag/marked_image'


def _parse_args():
    p = argparse.ArgumentParser(
        description='Save an AprilTag calibration reference point',
        epilog='Only one of --stage1 / --stage2 / --detach is accepted at a time.')
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--stage1', action='store_true', help='Save Stage 1 (approach) reference')
    group.add_argument('--stage2', action='store_true', help='Save Stage 2 (final dock) reference')
    group.add_argument('--detach', action='store_true', help='Save Detach (leave) reference')
    p.add_argument('--file', default='/mnt/work_space/april_tag_config.json',
                   help='path to april_tag_config.json (default: /mnt/work_space/april_tag_config.json)')
    args, _ = p.parse_known_args()
    stage = 'stage1' if args.stage1 else ('stage2' if args.stage2 else 'detach')
    return stage, args.file


def main() -> int:
    stage, cal_file = _parse_args()

    rclpy.init()
    node = Node('apriltag_calibrator')

    latest_tracing = None
    latest_image = None

    def tracing_cb(msg: TagTracing) -> None:
        nonlocal latest_tracing
        if msg.valid:
            latest_tracing = msg

    def image_cb(msg: Image) -> None:
        nonlocal latest_image
        latest_image = msg

    node.create_subscription(TagTracing, TAG_TRACING_TOPIC, tracing_cb, 1)
    node.create_subscription(Image, MARKED_IMAGE_TOPIC, image_cb, 1)

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

    # save annotated PNG
    png_path = None
    cal_dir = os.path.dirname(os.path.abspath(cal_file))
    if latest_image is not None:
        png_path = os.path.join(cal_dir, f'calibration_{stage}.png')
        try:
            import cv2
            from cv_bridge import CvBridge
            img = CvBridge().imgmsg_to_cv2(latest_image, desired_encoding='bgr8')
            cv2.imwrite(png_path, img)
            print(f'Image saved: {png_path}')
        except Exception as exc:
            print(f'WARNING: Could not save image ({exc}); continuing.', file=sys.stderr)
            png_path = None
    else:
        print('WARNING: No marked_image received; skipping PNG.', file=sys.stderr)

    # save calibration point (read-modify-write)
    point = CalibrationPoint(
        z_ref=z_ref, x_ref=x_ref, yaw_ref=yaw_ref,
        stamp=time.time(), image_path=png_path)
    try:
        save_calibration_point(cal_file, stage, point)
    except Exception as exc:
        print(f'ERROR: Failed to save calibration: {exc}', file=sys.stderr)
        return 1

    print(
        f'Saved {stage}:  z_ref={z_ref:.4f}m  '
        f'x_ref={x_ref:.4f}m  yaw_ref={yaw_ref:.4f}rad  →  {cal_file}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
