from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def start_feedback(context):
    config = PathJoinSubstitution([FindPackageShare('ranger_delta_bringup'), 'config', 'feedback.yaml'])
    source = LaunchConfiguration('source').perform(context)
    parameters = {'source': source, 'replay_file': ParameterValue(LaunchConfiguration('replay_file'), value_type=str)}
    can_interface = LaunchConfiguration('can_interface').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    publish_tf = LaunchConfiguration('publish_odom_tf').perform(context)
    if can_interface:
        parameters['can_interface'] = can_interface
    if odom_topic:
        parameters['odom_topic'] = odom_topic
    if publish_tf:
        parameters['publish_odom_tf'] = publish_tf == 'true'

    environment = {}
    if source == 'replay':
        environment = {
            'ROS_DOMAIN_ID': LaunchConfiguration('replay_domain_id'),
            'ROS_LOCALHOST_ONLY': '1',
        }
    return [Node(
        package='ranger_delta_bringup', executable='ranger_delta_feedback', output='screen',
        parameters=[config, parameters], additional_env=environment,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='replay', choices=['replay', 'socketcan']),
        DeclareLaunchArgument('replay_file', default_value='', description='Path to a candump -L log'),
        DeclareLaunchArgument('replay_domain_id', default_value='117', description='Local ROS domain for replay'),
        DeclareLaunchArgument('can_interface', default_value='', description='Override the configured SocketCAN interface'),
        DeclareLaunchArgument('odom_topic', default_value='', description='Override the configured odometry topic'),
        DeclareLaunchArgument('publish_odom_tf', default_value='', choices=['', 'true', 'false']),
        OpaqueFunction(function=start_feedback),
    ])
