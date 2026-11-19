from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def start_teleop_test(context):
    source = LaunchConfiguration('source').perform(context)
    enable_control = LaunchConfiguration('enable_control').perform(context) == 'true'
    interface = LaunchConfiguration('can_interface').perform(context)
    safe_mode = LaunchConfiguration('safe_mode').perform(context) == 'true'
    launch_gui = LaunchConfiguration('launch_gui').perform(context) == 'true'

    if enable_control and source != 'socketcan':
        raise ValueError('Velocity transmission requires source:=socketcan')

    package = FindPackageShare('ranger_delta_bringup')
    config_name = 'safe_command.yaml' if safe_mode else 'command.yaml'

    environment = {}
    if source == 'replay':
        environment = {
            'ROS_DOMAIN_ID': LaunchConfiguration('replay_domain_id').perform(context),
            'ROS_LOCALHOST_ONLY': '1'
        }

    actions = [
        # 1. 底盘反馈节点 (解析CAN并发布里程计、系统状态)
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
        # 2. 运动控制驱动节点 (订阅 /cmd_vel，安全门控，下发 CAN 速度帧)
        Node(
            package='ranger_delta_bringup',
            executable='ranger_delta_command',
            output='screen',
            parameters=[
                PathJoinSubstitution([package, 'config', config_name]),
                {
                    'enable_control': enable_control,
                    'can_interface': interface,
                }
            ],
            additional_env=environment,
        ),
    ]

    # 3. 安全运控可视化 UI
    if launch_gui:
        actions.append(
            Node(
                package='ranger_delta_bringup',
                executable='ranger_delta_teleop_gui',
                output='screen',
                additional_env=environment,
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='socketcan', choices=['socketcan', 'replay']),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('enable_control', default_value='true', choices=['false', 'true']),
        DeclareLaunchArgument('safe_mode', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('publish_odom_tf', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('replay_file', default_value=''),
        DeclareLaunchArgument('replay_domain_id', default_value='117'),
        OpaqueFunction(function=start_teleop_test),
    ])
