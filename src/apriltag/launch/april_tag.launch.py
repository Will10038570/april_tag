from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    manage_arg = DeclareLaunchArgument(
        'manage_amcl_and_lidar_safety', default_value='false',
        description='false: apriltag_control skips AMCL / lidar safety')

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
            # two-stage target distances from camera to tag (m)
            'stage1_distance': 0.50,
            'stage2_distance': 0.28,
            # G7+ AMCL services (absolute names, not affected by namespace 'up')
            'amcl_check_service': '/check_mcl_if_trigger',
            'amcl_close_service': '/close_amcl',
            'amcl_open_service': '/open_amcl',
            # G7+ PLC lidar safety topic (Bool: true disables the safety field,
            # false enables it; fire-and-forget like G7+ AutoCharging)
            'lidar_safety_topic': '/g7_plc/disable_lidar_safety',
            # velocity topics: Stage 1 normal, Stage 2 G7+ precision mode
            'stage1_cmd_vel_topic': '/cmd_vel',
            'stage2_cmd_vel_topic': '/pre_cmd_vel',
        }],
    )

    # print_tag_pose = Node(
    #     package='apriltag',
    #     executable='print_tag_pose',
    #     name='tag_pose_printer',
    #     namespace='up',
    #     output='screen',
    # )

    return LaunchDescription([
        manage_arg,
        # shorter log time: '2026-09-30 09:13:00.383' instead of epoch seconds
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                               '[{severity}] [{date_time_with_ms}] [{name}]: {message}'),
        camera,
        apriltag_detection,
        apriltag_control,
        # print_tag_pose,
    ])
