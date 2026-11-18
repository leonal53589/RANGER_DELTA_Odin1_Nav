from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def start_chassis(context):
    source = LaunchConfiguration('source').perform(context)
    enable_control = LaunchConfiguration('enable_control').perform(context) == 'true'
    interface = LaunchConfiguration('can_interface').perform(context)
    if enable_control and source != 'socketcan':
        raise ValueError('Velocity transmission requires source:=socketcan')

    package = FindPackageShare('ranger_delta_bringup')
    environment = {}
    if source == 'replay':
        environment = {'ROS_DOMAIN_ID': LaunchConfiguration('replay_domain_id'), 'ROS_LOCALHOST_ONLY': '1'}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([package, 'launch', 'feedback.launch.py'])),
            launch_arguments={
                'source': source,
                'can_interface': interface,
                'replay_file': LaunchConfiguration('replay_file'),
                'replay_domain_id': LaunchConfiguration('replay_domain_id'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
                'odom_topic': '/odom',
            }.items(),
        ),
        Node(
            package='ranger_delta_bringup', executable='ranger_delta_command', output='screen',
            parameters=[PathJoinSubstitution([package, 'config', 'command.yaml']), {
                'enable_control': enable_control, 'can_interface': interface,
            }], additional_env=environment,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='replay', choices=['replay', 'socketcan']),
        DeclareLaunchArgument('replay_file', default_value=''),
        DeclareLaunchArgument('replay_domain_id', default_value='117'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['false', 'true']),
        OpaqueFunction(function=start_chassis),
    ])
