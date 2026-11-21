from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('odin_nav_sim')
    nav2 = FindPackageShare('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('mode_switch_delay', default_value='0.35'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([package, 'maps', 'wall.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([package, 'config', 'nav2.yaml'])),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        Node(
            package='odin_nav_sim', executable='simulated_base', output='screen',
            parameters=[{
                'mode_switch_delay': ParameterValue(
                    LaunchConfiguration('mode_switch_delay'), value_type=float),
                'initial_x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
                'initial_y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
                'initial_yaw': ParameterValue(LaunchConfiguration('initial_yaw'), value_type=float),
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom', arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        ),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([nav2, 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_nav_sim',
            arguments=['-d', PathJoinSubstitution([package, 'rviz', 'navigation_sim.rviz'])],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
