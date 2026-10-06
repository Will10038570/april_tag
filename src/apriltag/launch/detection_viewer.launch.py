"""Print the TagPoseArray published by apriltag_detection on /up/apriltag_poses (at most 5 Hz)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detection_viewer = Node(
        package='apriltag',
        executable='detection_viewer',
        name='detection_viewer',
        namespace='up',
        output='screen',
    )
    return LaunchDescription([detection_viewer])
