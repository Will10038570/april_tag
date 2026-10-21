#!/usr/bin/env python3
"""Debug tool: print what apriltag_detection publishes.

Parameter `topic` picks the topic (relative, /wheelchair/... with namespace wheelchair):
- `tag_tracing` (default): the TagTracing with the tracked tag; the header,
  valid, image size, then the tag if valid.
- `apriltag_poses`: the TagPoseArray with every detected tag; the header,
  then every TagPose.
A tag is printed as family, id, position in m and orientation quaternion
(camera optical frame). At most PRINT_RATE_HZ messages per second are
printed (the others are skipped).

Usage (inside the container, after sourcing the workspace):
    ros2 run apriltag detection_viewer --ros-args -r __ns:=/wheelchair
    ros2 run apriltag detection_viewer --ros-args -r __ns:=/wheelchair -p topic:=apriltag_poses
"""

import time

import rclpy
from apriltag_interfaces.msg import TagPose, TagPoseArray, TagTracing
from rclpy.node import Node

PRINT_RATE_HZ = 5.0
TOPICS = ('tag_tracing', 'apriltag_poses')


def format_tag_pose(tag: TagPose) -> str:
    p = tag.pose.position
    q = tag.pose.orientation
    return (f"family={tag.family} id={tag.id} "
            f"position=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) "
            f"orientation=({q.x:+.3f}, {q.y:+.3f}, {q.z:+.3f}, {q.w:+.3f})")


def _format_header(header) -> str:
    return f"stamp={header.stamp.sec}.{header.stamp.nanosec:09d} frame_id={header.frame_id}"


def format_tag_pose_array(msg: TagPoseArray) -> str:
    lines = [f"{_format_header(msg.header)} tags={len(msg.tags)}"]
    lines += [f"  [{i}] {format_tag_pose(tag)}" for i, tag in enumerate(msg.tags)]
    return "\n".join(lines)


def format_tag_tracing(msg: TagTracing) -> str:
    line = (f"{_format_header(msg.header)} valid={msg.valid} "
            f"image={msg.image_width}x{msg.image_height}")
    if msg.valid:
        line += f"\n  {format_tag_pose(msg.tag)}"
    return line


class DetectionViewer(Node):
    def __init__(self):
        super().__init__('detection_viewer')
        self.topic = str(self.declare_parameter('topic', 'tag_tracing').value).strip()
        if self.topic not in TOPICS:
            raise ValueError(f"Parameter topic '{self.topic}' must be one of: {', '.join(TOPICS)}.")
        self._last_print = None
        if self.topic == 'tag_tracing':
            self.create_subscription(TagTracing, 'tag_tracing',
                                     lambda msg: self._print(format_tag_tracing, msg), 10)
        else:
            self.create_subscription(TagPoseArray, 'apriltag_poses',
                                     lambda msg: self._print(format_tag_pose_array, msg), 10)
        self.get_logger().info(f'Waiting for {self.topic}...')

    def _print(self, formatter, msg) -> None:
        now = time.monotonic()
        if self._last_print is not None and now - self._last_print < 1.0 / PRINT_RATE_HZ:
            return
        self._last_print = now
        self.get_logger().info(formatter(msg))


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
