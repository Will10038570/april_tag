"""End-to-end virtual test: real apriltag_detection + apriltag_control, with the
camera, the AMR and the G7+ AMCL services / lidar safety topic replaced by
tools/virtual_tracking_sim. A run is an attach cmd (Stage 1 + 2, back to IDLE
with AMCL closed and lidar safety disabled) followed by a detach cmd, both
called by the sim on the /wheelchair/attach / detach services; headless + auto_start runs both and checks AMCL
/ lidar safety after the attach (held) and at the end (restored).

Runs in ROS domain 65, the same as the robot: apriltag_control publishes the
absolute /cmd_vel, /pre_cmd_vel and /g7_plc/disable_lidar_safety and calls the AMCL
services, so unplug the robot's network cable before running it. CLI debugging:
    ROS_DOMAIN_ID=65 ros2 topic echo /cmd_vel

All node settings come from config_file (default: the installed
config/virtual_tracking_config.json). Variant tests: copy it, change e.g.
tracking_tag_id to 5 (TAG_LOST) or dock_timeout to 3.0 (TIMEOUT), and pass
config_file:=/path/to/copy.json.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# log lines without time; ros2 launch's '[<process>-N] ' prefix is dropped
LOG_FORMAT = '[{severity}] [{name}]: {message}'
OUTPUT_FORMAT = '{line}'
# apriltag_control prefixes every message with '[stage <STATE>] : ', giving
# '[INFO] [wheelchair.apriltag_control][stage LEAVING] : ...'
CONTROL_LOG_ENV = {'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]{message}'}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('domain_id', default_value='65',
                              description='ROS_DOMAIN_ID for the whole virtual test'),
        DeclareLaunchArgument('init_x', default_value='-1.0'),
        DeclareLaunchArgument('init_y', default_value='0.15'),
        DeclareLaunchArgument('init_yaw_deg', default_value='10.0'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='no UI; run one attach + detach and save CSV + PNG'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='send attach as soon as apriltag_control publishes its status'),
        DeclareLaunchArgument('log_dir', default_value='virtual_tracking_logs'),
        # every apriltag_detection / apriltag_control / sim setting (tag, tag size,
        # timeouts, calibration points = stage / leave distances, tolerances);
        # for variant tests copy it and pass config_file:=/path/to/copy.json
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution(
            [FindPackageShare('apriltag'), 'config', 'virtual_tracking_config.json'])),
    ]

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

    virtual_tracking_sim = Node(
        package='apriltag',
        executable='virtual_tracking_sim',
        name='virtual_tracking_sim',
        namespace='wheelchair',
        output='screen',
        output_format=OUTPUT_FORMAT,
        parameters=[{
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_yaw_deg': LaunchConfiguration('init_yaw_deg'),
            'headless': LaunchConfiguration('headless'),
            'auto_start': LaunchConfiguration('auto_start'),
            'log_dir': LaunchConfiguration('log_dir'),
            'config_file': LaunchConfiguration('config_file'),
        }],
        # quitting the dashboard ends the whole test, so no detection/control
        # is left running
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        *args,
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        # '[INFO] [wheelchair.apriltag_detection]: ...' (apriltag_control: see CONTROL_LOG_ENV)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', LOG_FORMAT),
        apriltag_detection,
        apriltag_control,
        virtual_tracking_sim,
    ])
