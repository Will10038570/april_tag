"""Print what apriltag_detection publishes under /wheelchair (at most 5 Hz).

topic:=tag_tracing (default) prints the tracked tag (TagTracing),
topic:=apriltag_poses every detected tag (TagPoseArray).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='tag_tracing',
        description='topic to print: tag_tracing (tracked tag) or apriltag_poses (all tags)')
    detection_viewer = Node(
        package='apriltag',
        executable='detection_viewer',
        name='detection_viewer',
        namespace='wheelchair',
        output='screen',
        parameters=[{'topic': LaunchConfiguration('topic')}],
    )
    return LaunchDescription([topic_arg, detection_viewer])
