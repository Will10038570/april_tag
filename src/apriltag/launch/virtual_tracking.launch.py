"""End-to-end virtual test: real apriltag_detection + apriltag_control, with the
camera, the AMR and the G7+ AMCL services / lidar safety topic replaced by
tools/virtual_tracking_sim. A run is an attach cmd (Stage 1 + 2, back to IDLE
with AMCL closed and lidar safety disabled) followed by a detach cmd, both
called by the sim on the /wheelchair/attach / detach services; headless + auto_start runs both and checks AMCL
/ lidar safety after the attach (held) and at the end (restored).

Runs in ROS domain domain_id (default 65). apriltag_control publishes the
absolute /cmd_vel, /pre_cmd_vel and /g7_plc/disable_lidar_safety and calls the AMCL
services; the robot's domain is set in docker/.ros2_setup, but do not rely on it
being different: unplug the robot's network cable, or use domain_id:=99 with
CycloneDDS on loopback only. CLI debugging:
    ROS_DOMAIN_ID=99 ros2 topic echo /cmd_vel

All node settings come from config_file (default: the workspace root's
april_tag_config.json, the same file the robot launch uses, read directly, so
edits apply without a colcon build; apriltag_detection and the sim read it only at start,
so relaunch after editing it). Variant tests: copy it, change e.g.
control.dock_timeout to 3.0 (TIMEOUT, error_code 4), and pass
config_file:=/path/to/copy.json. TAG_LOST (error_code 1): hide_tag_after:=5.0
(the sim stops drawing the tag 5 s after the run starts).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# log lines without time; ros2 launch's '[<process>-N] ' prefix is dropped
LOG_FORMAT = '[{severity}] [{name}]: {message}'
OUTPUT_FORMAT = '{line}'
# apriltag_control prefixes every message with '[stage <STATE>] : ', giving
# '[INFO] [wheelchair.apriltag_control][stage LEAVING] : ...'
CONTROL_LOG_ENV = {'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]{message}'}

# Default config: install/apriltag/share/apriltag → ../../../../ = workspace root,
# so edits to <workspace>/april_tag_config.json apply without a build
_pkg_share = get_package_share_directory('apriltag')
_DEFAULT_CONFIG = os.path.normpath(os.path.join(
    _pkg_share, '..', '..', '..', '..', 'april_tag_config.json'))


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
        DeclareLaunchArgument('hide_tag_after', default_value='-1.0',
                              description='sim stops drawing the tag this many seconds after a run '
                                          'starts (TAG_LOST test); < 0 = never'),
        # every apriltag_detection / apriltag_control / sim setting (tag, tag size,
        # timeouts, calibration points = stage / leave distances, tolerances); the
        # default is the workspace april_tag_config.json (no build needed, relaunch
        # after editing);
        # for variant tests copy it and pass config_file:=/path/to/copy.json
        DeclareLaunchArgument('config_file', default_value=_DEFAULT_CONFIG),
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
            'hide_tag_after': LaunchConfiguration('hide_tag_after'),
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
