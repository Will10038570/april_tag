"""End-to-end virtual test: real apriltag_detection + apriltag_control, with the
camera, the AMR and the G7+ AMCL services / lidar safety topic replaced by
tools/virtual_tracking_sim. A run is start_tracking (Stage 1 + 2, back to IDLE
with AMCL closed and lidar safety disabled) followed by leave_cs; headless +
auto_start runs both and checks AMCL / lidar safety after start_tracking
(held) and at the end (restored).

Runs in ROS domain 65, the same as the robot: apriltag_control publishes the
absolute /cmd_vel, /pre_cmd_vel and /g7_plc/disable_lidar_safety and calls the AMCL
services, so unplug the robot's network cable before running it. CLI debugging:
    ROS_DOMAIN_ID=65 ros2 topic echo /cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# log lines without time; ros2 launch's '[<process>-N] ' prefix is dropped
LOG_FORMAT = '[{severity}] [{name}]: {message}'
OUTPUT_FORMAT = '{line}'
# apriltag_control prefixes every message with '[stage <STATE>] : ', giving
# '[INFO] [up.apriltag_control][stage LEAVING] : ...'
CONTROL_LOG_ENV = {'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]{message}'}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('domain_id', default_value='65',
                              description='ROS_DOMAIN_ID for the whole virtual test'),
        DeclareLaunchArgument('init_x', default_value='-1.0'),
        DeclareLaunchArgument('init_y', default_value='0.15'),
        DeclareLaunchArgument('init_yaw_deg', default_value='10.0'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='no UI; run one goal and save CSV + PNG'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='send start_tracking as soon as the action server is up'),
        DeclareLaunchArgument('log_dir', default_value='virtual_tracking_logs'),
        DeclareLaunchArgument('manage_amcl_and_lidar_safety', default_value='true',
                              description='false: apriltag_control skips AMCL / lidar safety'),
        # simulation's own stage / leave distances, independent of launch/april_tag.launch.py;
        # the sim reads them back from apriltag_control
        # detection families and the tag apriltag_control tracks; the sim draws tag36h11 id 0
        DeclareLaunchArgument('detect_tag_families', default_value='tag36h11'),
        DeclareLaunchArgument('tag_family', default_value='tag36h11'),
        DeclareLaunchArgument('tag_id', default_value='0', description='-1 = any id'),
        DeclareLaunchArgument('stage1_distance', default_value='0.50'),
        DeclareLaunchArgument('stage2_distance', default_value='0.28'),
        DeclareLaunchArgument('leave_distance', default_value='1.0'),
    ]

    apriltag_detection = Node(
        package='apriltag',
        executable='apriltag_detection',
        name='apriltag_detection',
        namespace='up',
        output='screen',
        output_format=OUTPUT_FORMAT,
        parameters=[{
            'tag_families': ParameterValue(LaunchConfiguration('detect_tag_families'), value_type=str),
        }],
    )

    apriltag_control = Node(
        package='apriltag',
        executable='apriltag_control',
        name='apriltag_control',
        namespace='up',
        output='screen',
        output_format=OUTPUT_FORMAT,
        additional_env=CONTROL_LOG_ENV,
        parameters=[{
            'manage_amcl_and_lidar_safety': ParameterValue(
                LaunchConfiguration('manage_amcl_and_lidar_safety'), value_type=bool),
            'stage1_distance': ParameterValue(LaunchConfiguration('stage1_distance'), value_type=float),
            'stage2_distance': ParameterValue(LaunchConfiguration('stage2_distance'), value_type=float),
            'leave_distance': ParameterValue(LaunchConfiguration('leave_distance'), value_type=float),
            'tag_family': ParameterValue(LaunchConfiguration('tag_family'), value_type=str),
            'tag_id': ParameterValue(LaunchConfiguration('tag_id'), value_type=str),
        }],
    )

    virtual_tracking_sim = Node(
        package='apriltag',
        executable='virtual_tracking_sim',
        name='virtual_tracking_sim',
        namespace='up',
        output='screen',
        output_format=OUTPUT_FORMAT,
        parameters=[{
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_yaw_deg': LaunchConfiguration('init_yaw_deg'),
            'headless': LaunchConfiguration('headless'),
            'auto_start': LaunchConfiguration('auto_start'),
            'log_dir': LaunchConfiguration('log_dir'),
        }],
        # quitting the dashboard ends the whole test, so no detection/control
        # is left running
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        *args,
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        # '[INFO] [up.apriltag_detection]: ...' (apriltag_control: see CONTROL_LOG_ENV)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', LOG_FORMAT),
        apriltag_detection,
        apriltag_control,
        virtual_tracking_sim,
    ])
