import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# log lines without time; ros2 launch's '[<process>-N] ' prefix is dropped
LOG_FORMAT = '[{severity}] [{name}]: {message}'
OUTPUT_FORMAT = '{line}'
# apriltag_control prefixes every message with '[stage <STATE>] : ', giving
# '[INFO] [wheelchair.apriltag_control][stage LEAVING] : ...'
CONTROL_LOG_ENV = {'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]{message}'}

# Default config path: install/apriltag/share/apriltag → ../../../../ = workspace root
_pkg_share = get_package_share_directory('apriltag')
_DEFAULT_CONFIG = os.path.normpath(
    os.path.join(_pkg_share, '..', '..', '..', '..', 'april_tag_config.json'))


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=_DEFAULT_CONFIG,
        description='path to april_tag_config.json')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('realsense'), 'launch', 'camera.launch.py']
            )
        ),
    )

    apriltag_detection = Node(
        package='apriltag',
        executable='apriltag_detection',
        name='apriltag_detection',
        namespace='wheelchair',
        output='screen',
        output_format=OUTPUT_FORMAT,
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
    )

    apriltag_control = Node(
        package='apriltag',
        executable='apriltag_control',
        name='apriltag_control',
        namespace='wheelchair',
        output='screen',
        output_format=OUTPUT_FORMAT,
        additional_env=CONTROL_LOG_ENV,
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
    )

    return LaunchDescription([
        config_file_arg,
        # '[INFO] [wheelchair.apriltag_detection]: ...' (apriltag_control: see CONTROL_LOG_ENV)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', LOG_FORMAT),
        camera,
        apriltag_detection,
        apriltag_control,
    ])
