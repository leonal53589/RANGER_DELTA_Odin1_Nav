from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def prepare_nav2_params(context):
    mode = LaunchConfiguration('goal_time_mode').perform(context)
    params = LaunchConfiguration('params_file').perform(context)
    if mode == 'stock':
        return [SetLaunchConfiguration('_odin_sim_nav2_params', params)]

    prefix = Path(get_package_prefix('nav2_controller'))
    if not (prefix / 'share/nav2_controller/odin_goal_time_test').is_file():
        raise RuntimeError('Build and source the isolated overlay with scripts/build_goal_time_test.sh')
    with open(params, encoding='utf-8') as source:
        configuration = yaml.safe_load(source)
    controller = configuration['controller_server']['ros__parameters']
    controller.update(align_goal_tf_time=(mode == 'aligned'), trace_goal_tf_time=True)
    controller['goal_checker'].update(
        plugin='nav2_controller::SimpleGoalChecker', xy_goal_tolerance=0.06,
        yaw_goal_tolerance=0.10, stateful=False,
    )
    controller['FollowPath'].update(
        plugin='odin_nav_sim::DeltaPursuitController', desired_linear_vel=0.04,
        minimum_turn_radius=0.50, min_approach_linear_velocity=0.03,
    )
    controller['FollowPath'].pop('tracking_controller', None)
    controller['FollowPath'].pop('tracking', None)
    navigator = configuration['bt_navigator']['ros__parameters']
    navigator['default_nav_to_pose_bt_xml'] = str(
        Path(get_package_share_directory('nav2_bt_navigator')) /
        'behavior_trees/navigate_w_replanning_only_if_path_becomes_invalid.xml'
    )
    for plugin in ('nav2_is_path_valid_condition_bt_node', 'nav2_globally_updated_goal_condition_bt_node'):
        if plugin not in navigator['plugin_lib_names']:
            navigator['plugin_lib_names'].append(plugin)
    with NamedTemporaryFile(mode='w', suffix='.yaml', encoding='utf-8', delete=False) as output:
        yaml.safe_dump(configuration, output)
    test_params = Path(output.name)

    def remove_test_params(_context):
        test_params.unlink(missing_ok=True)

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=remove_test_params)])),
        SetLaunchConfiguration('_odin_sim_nav2_params', str(test_params)),
    ]


def generate_launch_description():
    package = FindPackageShare('odin_nav_sim')
    test_mode = PythonExpression(["'", LaunchConfiguration('goal_time_mode'), "' != 'stock'"])
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('mode_switch_delay', default_value='0.35'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([package, 'maps', 'wall.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([package, 'config', 'nav2.yaml'])),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_time_mode', default_value='stock',
                              choices=['stock', 'original', 'aligned']),
        DeclareLaunchArgument('goal_time_shift_x', default_value='0.0'),
        OpaqueFunction(function=prepare_nav2_params),
        Node(
            package='odin_nav_sim', executable='simulated_base', output='screen',
            parameters=[{
                'mode_switch_delay': ParameterValue(
                    LaunchConfiguration('mode_switch_delay'), value_type=float),
                'initial_x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
                'initial_y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
                'initial_yaw': ParameterValue(LaunchConfiguration('initial_yaw'), value_type=float),
                'publish_test_map_odom': ParameterValue(test_mode, value_type=bool),
                'test_map_odom_shift_x': ParameterValue(LaunchConfiguration('goal_time_shift_x'), value_type=float),
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom', arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            condition=UnlessCondition(test_mode),
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
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py',
            ])),
            launch_arguments={'params_file': LaunchConfiguration('_odin_sim_nav2_params')}.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_nav_sim',
            arguments=['-d', PathJoinSubstitution([package, 'rviz', 'navigation_sim.rviz'])],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
