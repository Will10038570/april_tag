#!/usr/bin/env python3
"""Debug tool: print the TagPoseArray published by apriltag_detection.

Subscribes to apriltag_poses (/up/apriltag_poses with namespace up) and prints
at most PRINT_RATE_HZ messages per second (the others are skipped): the array
header, then every TagPose (family, id, position in m and orientation
quaternion, camera optical frame).

Usage (inside the container, after sourcing the workspace):
    ros2 run apriltag detection_viewer --ros-args -r __ns:=/up
"""

import time

import rclpy
from apriltag_interfaces.msg import TagPose, TagPoseArray
from rclpy.node import Node

PRINT_RATE_HZ = 5.0


def format_tag_pose(tag: TagPose) -> str:
    p = tag.pose.position
    q = tag.pose.orientation
    return (f"family={tag.family} id={tag.id} "
            f"position=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) "
            f"orientation=({q.x:+.3f}, {q.y:+.3f}, {q.z:+.3f}, {q.w:+.3f})")


def format_tag_pose_array(msg: TagPoseArray) -> str:
    stamp = msg.header.stamp
    lines = [f"stamp={stamp.sec}.{stamp.nanosec:09d} frame_id={msg.header.frame_id} "
             f"tags={len(msg.tags)}"]
    lines += [f"  [{i}] {format_tag_pose(tag)}" for i, tag in enumerate(msg.tags)]
    return "\n".join(lines)


class DetectionViewer(Node):
    def __init__(self):
        super().__init__('detection_viewer')
        self._last_print = None
        self.create_subscription(TagPoseArray, 'apriltag_poses', self._on_tags, 10)
        self.get_logger().info('Waiting for apriltag_poses...')

    def _on_tags(self, msg: TagPoseArray) -> None:
        now = time.monotonic()
        if self._last_print is not None and now - self._last_print < 1.0 / PRINT_RATE_HZ:
            return
        self._last_print = now
        self.get_logger().info(format_tag_pose_array(msg))


def main(args=None):
    rclpy.init(args=args)
    node = DetectionViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
