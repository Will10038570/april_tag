from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense',
            executable='rgb_publisher',
            name='realsense_rgb_publisher',
            output='screen',
        ),
    ])
