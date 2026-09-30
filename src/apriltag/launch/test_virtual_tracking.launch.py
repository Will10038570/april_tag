"""End-to-end virtual test: real apriltag_detection + apriltag_control, with the
camera, the AMR and the G7+ AMCL / lidar safety services replaced by
tools/virtual_tracking_sim.

Runs in ROS domain 65, the same as the robot: apriltag_control publishes the
absolute /cmd_vel_nav and calls the AMCL / lidar safety services, so unplug the
robot's network cable before running it. CLI debugging:
    ROS_DOMAIN_ID=65 ros2 topic echo /cmd_vel_nav
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    ]

    apriltag_detection = Node(
        package='apriltag',
        executable='apriltag_detection',
        name='apriltag_detection',
        namespace='up',
        output='screen',
    )

    apriltag_control = Node(
        package='apriltag',
        executable='apriltag_control',
        name='apriltag_control',
        namespace='up',
        output='screen',
        parameters=[{
            'manage_amcl_and_lidar_safety': ParameterValue(
                LaunchConfiguration('manage_amcl_and_lidar_safety'), value_type=bool),
        }],
    )

    virtual_tracking_sim = Node(
        package='apriltag',
        executable='virtual_tracking_sim',
        name='virtual_tracking_sim',
        namespace='up',
        output='screen',
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
        apriltag_detection,
        apriltag_control,
        virtual_tracking_sim,
    ])
