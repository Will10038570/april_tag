from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# log lines without time; ros2 launch's '[<process>-N] ' prefix is dropped
LOG_FORMAT = '[{severity}] [{name}]: {message}'
OUTPUT_FORMAT = '{line}'
# apriltag_control prefixes every message with '[stage <STATE>] : ', giving
# '[INFO] [up.apriltag_control][stage LEAVING] : ...'
CONTROL_LOG_ENV = {'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]{message}'}


def generate_launch_description():
    manage_arg = DeclareLaunchArgument(
        'manage_amcl_and_lidar_safety', default_value='false',
        description='false: apriltag_control skips AMCL / lidar safety')
    detect_tag_families_arg = DeclareLaunchArgument(
        'detect_tag_families', default_value='tag36h11',
        description='AprilTag families apriltag_detection detects, space separated '
                    '(pupil_apriltags names, e.g. "tag36h11 tag25h9"); each adds CPU load')
    tracking_tag_family_arg = DeclareLaunchArgument(
        'tracking_tag_family', default_value='tag36h11',
        description='family of the tag apriltag_detection publishes on tag_tracing; '
                    'must be one of detect_tag_families')
    tracking_tag_id_arg = DeclareLaunchArgument(
        'tracking_tag_id', default_value='0',
        description='id of the tag published on tag_tracing (closest one), -1 = any id')
    dock_timeout_arg = DeclareLaunchArgument(
        'dock_timeout', default_value='60.0',
        description='dock_wheelchair timeout (s), from goal accepted until it ends')
    undock_timeout_arg = DeclareLaunchArgument(
        'undock_timeout', default_value='60.0',
        description='undock_wheelchair timeout (s), from goal accepted until it ends')
    lost_target_timeout_arg = DeclareLaunchArgument(
        'lost_target_timeout', default_value='1.0',
        description='tracked tag not seen for this long (s) while running ends the goal (TAG_LOST)')
    tag_size_arg = DeclareLaunchArgument(
        'tag_size', default_value='0.0635',
        description='real black-border side of the tags (m)')
    tag_sizes_arg = DeclareLaunchArgument(
        'tag_sizes', default_value='',
        description="per-id tag sizes 'id:size ...' (m), e.g. '3:0.095'; other ids use tag_size")

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
        output_format=OUTPUT_FORMAT,
        parameters=[{
            'tag_families': ParameterValue(LaunchConfiguration('detect_tag_families'), value_type=str),
            'tracking_tag_family': ParameterValue(LaunchConfiguration('tracking_tag_family'), value_type=str),
            'tracking_tag_id': ParameterValue(LaunchConfiguration('tracking_tag_id'), value_type=int),
            'tag_size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float),
            'tag_sizes': ParameterValue(LaunchConfiguration('tag_sizes'), value_type=str),
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
            'dock_timeout': ParameterValue(LaunchConfiguration('dock_timeout'), value_type=float),
            'undock_timeout': ParameterValue(LaunchConfiguration('undock_timeout'), value_type=float),
            'lost_target_timeout': ParameterValue(LaunchConfiguration('lost_target_timeout'), value_type=float),
            'tag_size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float),
            'tag_sizes': ParameterValue(LaunchConfiguration('tag_sizes'), value_type=str),
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
            # velocity topics: Stage 1 normal, Stage 2 and leaving G7+ precision mode
            'stage1_cmd_vel_topic': '/cmd_vel',
            'stage2_cmd_vel_topic': '/pre_cmd_vel',
        }],
    )

    return LaunchDescription([
        manage_arg,
        detect_tag_families_arg,
        tracking_tag_family_arg,
        tracking_tag_id_arg,
        dock_timeout_arg,
        undock_timeout_arg,
        lost_target_timeout_arg,
        tag_size_arg,
        tag_sizes_arg,
        # '[INFO] [up.apriltag_detection]: ...' (apriltag_control: see CONTROL_LOG_ENV)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', LOG_FORMAT),
        camera,
        apriltag_detection,
        apriltag_control,
    ])
