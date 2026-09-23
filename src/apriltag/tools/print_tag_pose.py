#!/usr/bin/env python3
"""Debug tool: print distance and yaw angle from camera to the tracked AprilTag.

Subscribes to /apriltag_pose (only published while tracking is active, i.e.
after sending the start_tracking action goal) and prints a line per frame:

    distance: 0.52 m | yaw: 3.2 deg

distance = straight-line distance from the camera to the tag (norm of the
position vector in the camera optical frame).
yaw = same convention as the node's own control loop
(apriltag.domain.math_utils.rotation_matrix_to_yaw_error): angle between the
tag's face normal and the camera's forward axis, recovered here from the
published quaternion instead of the raw rotation matrix.

Usage (inside the ros2_humble container, after sourcing the workspace):
    python3 src/apriltag/tools/print_tag_pose.py
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

PRINT_INTERVAL_S = 0.5


class TagPosePrinter(Node):
    def __init__(self):
        super().__init__('tag_pose_printer')
        self._last_print = 0.0
        self.create_subscription(PoseStamped, '/apriltag_pose', self._on_pose, 10)
        self.get_logger().info(
            "Waiting for /apriltag_pose... "
            "(send the start_tracking action goal to start publishing)"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        if now - self._last_print < PRINT_INTERVAL_S:
            return
        self._last_print = now

        p = msg.pose.position
        distance = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)

        q = msg.pose.orientation
        r_mat = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        yaw_rad = math.atan2(-r_mat[0, 2], r_mat[2, 2])
        yaw_deg = math.degrees(yaw_rad)

        self.get_logger().info(f"distance: {distance:.3f} m | yaw: {yaw_deg:+.1f} deg")


def main(args=None):
    rclpy.init(args=args)
    node = TagPosePrinter()
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
