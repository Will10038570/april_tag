from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('realsense'), 'launch', 'camera.launch.py']
            )
        ),
    )

    apriltag_node = Node(
        package='apriltag',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
    )

    print_tag_pose = Node(
        package='apriltag',
        executable='print_tag_pose',
        name='tag_pose_printer',
        output='screen',
    )

    return LaunchDescription([camera, apriltag_node, print_tag_pose])
