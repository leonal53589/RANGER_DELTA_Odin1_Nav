from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_setup(context):
    pkg_odin_driver = FindPackageShare('odin_ros_driver')
    pkg_chassis = FindPackageShare('ranger_delta_bringup')
    pkg_nav = FindPackageShare('odin_nav_sim')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    can_interface = LaunchConfiguration('can_interface').perform(context)
    enable_control = LaunchConfiguration('enable_control').perform(context) == 'true'
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context) == 'true'
    publish_map_odom_tf = LaunchConfiguration('publish_map_odom_tf').perform(context) == 'true'
    map_yaml_file = LaunchConfiguration('map').perform(context)
    odin_relocalization_config = LaunchConfiguration('odin_relocalization_config').perform(context)
    odin_reference_pcd = LaunchConfiguration('odin_reference_pcd').perform(context)
    odin_spin_guard = LaunchConfiguration('odin_spin_guard')
    goal_time_ab = LaunchConfiguration('goal_time_ab').perform(context) == 'true'
    goal_profile = LaunchConfiguration('goal_profile').perform(context)

    if goal_time_ab:
        marker = Path(get_package_prefix('nav2_controller')) / 'share/nav2_controller/odin_goal_time_test'
        if not marker.is_file() or 'per-goal checker selection' not in marker.read_text():
            raise ValueError('Build and source the current overlay with scripts/build_goal_time_test.sh')
        if goal_profile != 'precision' or not odin_relocalization_config:
            raise ValueError('goal_time_ab requires the precision profile and ODIN1 relocalization')
    if enable_control and publish_map_odom_tf:
        raise ValueError('Static map alignment is available only with enable_control:=false')
    if odin_relocalization_config and publish_map_odom_tf:
        raise ValueError('Odin relocalization and static alignment both publish map -> odom')
    if odin_reference_pcd and (not odin_relocalization_config or not Path(odin_reference_pcd).is_file()):
        raise ValueError('odin_reference_pcd requires a saved PCD and ODIN1 relocalization')
    if odin_spin_guard.perform(context) == 'true' and not odin_relocalization_config:
        raise ValueError('odin_spin_guard requires ODIN1 relocalization')
    if enable_control and not all(
        LaunchConfiguration(f'sensor_{axis}').perform(context) for axis in ('x', 'y', 'z')
    ):
        raise ValueError('Provide measured sensor_x, sensor_y and sensor_z before enabling control')
    if not Path(map_yaml_file).is_file():
        raise ValueError(f'Map YAML is missing: {map_yaml_file}')

    driver_config = PathJoinSubstitution([pkg_odin_driver, 'config', 'control_command_robot.yaml'])
    odin_tf_remappings = []
    if odin_relocalization_config:
        config_path = Path(odin_relocalization_config).expanduser().resolve()
        with config_path.open(encoding='utf-8') as config_file:
            keys = yaml.safe_load(config_file)['register_keys']
        for key, value in (('custom_map_mode', 2), ('sendodom', 1), ('send_odom_baselink_tf', 1)):
            if keys.get(key) != value:
                raise ValueError(f'{config_path}: set {key} to {value} for Odin relocalization')
        device_map = Path(keys.get('relocalization_map_abs_path') or '')
        if not device_map.is_absolute() or not device_map.is_file():
            raise ValueError(f'{config_path}: relocalization_map_abs_path must point to the saved Odin map')
        driver_config = str(config_path)
        odin_tf_remappings = [('/tf', '/odin1/tf'), ('/tf_static', '/odin1/tf_static')]
    nav2_params = PathJoinSubstitution([pkg_nav, 'config', 'nav2_real.yaml'])
    cleanup = []
    if goal_time_ab:
        config = Path(get_package_share_directory('odin_nav_sim')) / 'config/nav2_real.yaml'
        configuration = yaml.safe_load(config.read_text())
        controller = configuration['controller_server']['ros__parameters']
        controller.update(
            goal_checker_plugins=['goal_checker', 'goal_checker_aligned'],
            aligned_goal_checker_id='goal_checker_aligned',
            trace_goal_tf_time=True,
        )
        controller['goal_checker']['plugin'] = 'nav2_controller::SimpleGoalChecker'
        controller['FollowPath'].pop('tracking_controller', None)
        controller['FollowPath'].pop('tracking', None)
        controller['goal_checker_aligned'] = controller['goal_checker'].copy()
        controller['goal_checker_aligned'].update(xy_goal_tolerance=0.06, yaw_goal_tolerance=0.10)
        with NamedTemporaryFile(mode='w', suffix='.yaml', encoding='utf-8', delete=False) as output:
            yaml.safe_dump(configuration, output)
        test_params = Path(output.name)
        nav2_params = str(test_params)

        def remove_test_params(_context):
            test_params.unlink(missing_ok=True)

        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=remove_test_params,
        )])))
    if goal_profile == 'precision':
        nav2_params = RewrittenYaml(
            source_file=nav2_params,
            param_rewrites={
                'planner_server.ros__parameters.GridBased.tolerance': '0.035',
                'controller_server.ros__parameters.goal_checker.xy_goal_tolerance': '0.06',
                'controller_server.ros__parameters.goal_checker.yaw_goal_tolerance': '0.10',
            },
            convert_types=True,
        )
    rviz_config = PathJoinSubstitution([pkg_nav, 'rviz', 'real_navigation.rviz'])

    nodes = [
        # 1. 底盘反馈节点 (SocketCAN 接收，发布 /odom 与 odom -> base_link TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_chassis, 'launch', 'feedback.launch.py'])),
            launch_arguments={
                'source': 'socketcan',
                'can_interface': can_interface,
                'publish_odom_tf': 'true',
                'odom_topic': '/odom',
            }.items(),
        ),

        # 2. 底盘速度驱动节点 (订阅 Nav2 规划出的 /cmd_vel，下发 CAN 0x111 帧)
        Node(
            package='ranger_delta_bringup',
            executable='ranger_delta_command',
            name='ranger_delta_command',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_chassis, 'config', 'safe_command.yaml']),
                nav2_params,
                {
                    'enable_control': enable_control,
                    'can_interface': can_interface,
                }
            ],
        ),

        # 3. 留形 Odin1 发布传感器系原始点云，设备里程计使用独立帧名。
        Node(
            package='odin_ros_driver',
            executable='host_sdk_sample',
            name='host_sdk_sample',
            output='screen',
            parameters=[{'config_file': driver_config, 'odom_frame': 'odin_odom', 'map_frame': 'odin_map'}],
            remappings=odin_tf_remappings,
        ),

        # 6. 地图服务器 (加载 2D OccupancyGrid 地图)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),

        # 7. Nav2 核心导航栈 (Planner, Controller, Costmaps with PointCloud ObstacleLayer)
        GroupAction([
            # Humble's stock launch remaps the controller but leaves behavior commands on /cmd_vel.
            SetRemap(src='behavior_server:cmd_vel', dst='cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    pkg_nav2_bringup, 'launch', 'navigation_launch.py',
                ])),
                launch_arguments={
                    'params_file': nav2_params,
                    'use_sim_time': 'false',
                    'autostart': 'true',
                }.items(),
            ),
        ]),
    ]

    # Only one source may publish the global map -> odom transform.
    if odin_relocalization_config:
        nodes.append(Node(
            package='odin_nav_sim',
            executable='odin_localization',
            name='odin_localization',
            output='screen',
            parameters=[
                {
                    name: ParameterValue(LaunchConfiguration(name), value_type=float)
                    for name in ('odin_map_x', 'odin_map_y', 'odin_map_yaw')
                },
                {
                    'odin_reference_pcd': ParameterValue(odin_reference_pcd, value_type=str),
                    'odin_spin_guard': ParameterValue(odin_spin_guard, value_type=bool),
                },
            ],
        ))
    elif publish_map_odom_tf:
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_broadcaster',
                arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
                output='screen',
            )
        )

    # 可选: 启动 Nav2 交互式 RViz2 面板 (带 2D Goal Pose 工具)
    if launch_rviz:
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_navigation',
                output='screen',
                arguments=['-d', rviz_config],
            )
        )

    return cleanup + nodes


def generate_launch_description():
    description = PathJoinSubstitution([FindPackageShare('ranger_delta_bringup'), 'launch', 'description.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('map', description='场地栅格地图 YAML 绝对路径'),
        DeclareLaunchArgument('can_interface', default_value='can0', description='SocketCAN 接口名'),
        DeclareLaunchArgument('goal_profile', default_value='navigation',
                              choices=['navigation', 'precision'],
                              description='通行或精确停车的规划与到点容差'),
        DeclareLaunchArgument('goal_time_ab', default_value='false', choices=['true', 'false'],
                              description='精确停车实验中按目标选择原始或时间对齐判定'),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false'], description='是否允许向底盘下发控制指令'),
        DeclareLaunchArgument('publish_map_odom_tf', default_value='false', choices=['true', 'false'], description='只读联调时发布静态 map → odom'),
        DeclareLaunchArgument('odin_relocalization_config', default_value='', description='ODIN1 重定位配置 YAML 路径'),
        DeclareLaunchArgument('odin_reference_pcd', default_value='', description='用于复核地图校正的同次采图二进制 PCD'),
        DeclareLaunchArgument('odin_spin_guard', default_value='false', choices=['true', 'false'],
                              description='自旋时限制 ODIN1 定位修正，恢复平移时逐步融合'),
        DeclareLaunchArgument('odin_map_x', default_value='0.0', description='ODIN1 地图原点在导航 map 中的 X（米）'),
        DeclareLaunchArgument('odin_map_y', default_value='0.0', description='ODIN1 地图原点在导航 map 中的 Y（米）'),
        DeclareLaunchArgument('odin_map_yaw', default_value='0.0', description='ODIN1 地图相对导航 map 的偏航（弧度）'),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false'], description='是否启动 RViz2 导航面板'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description)),
        OpaqueFunction(function=launch_setup),
    ])
