from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stream_args = [
        DeclareLaunchArgument('enable_rgb', default_value='true'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('enable_ir1', default_value='false'),
        DeclareLaunchArgument('enable_ir2', default_value='false'),
    ]

    enable_rgb = LaunchConfiguration('enable_rgb')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_ir1 = LaunchConfiguration('enable_ir1')
    enable_ir2 = LaunchConfiguration('enable_ir2')

    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )
        ),
        launch_arguments={
            'enable_color': enable_rgb,
            'enable_depth': enable_depth,
            'enable_infra1': enable_ir1,
            'enable_infra2': enable_ir2,
        }.items(),
    )

    viewer_node = Node(
        package='realsense',
        executable='camera_viewer',
        name='realsense_camera_viewer',
        output='screen',
        parameters=[{
            'enable_rgb': ParameterValue(enable_rgb, value_type=bool),
            'enable_depth': ParameterValue(enable_depth, value_type=bool),
            'enable_ir1': ParameterValue(enable_ir1, value_type=bool),
            'enable_ir2': ParameterValue(enable_ir2, value_type=bool),
        }],
    )

    up_group = GroupAction([
        PushRosNamespace('up'),
        realsense_driver,
        viewer_node,
    ])

    return LaunchDescription([*stream_args, up_group])
