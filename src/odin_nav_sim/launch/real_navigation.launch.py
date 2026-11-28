from pathlib import Path
from tempfile import NamedTemporaryFile

import numpy as np
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


def configure_station(configuration, target):
    controller = configuration['controller_server']['ros__parameters']
    controller['controller_plugins'].append('StationAlign')
    controller['StationAlign'] = {'plugin': 'odin_nav_sim::StationController', 'target': target}
    configuration['bt_navigator']['ros__parameters']['station_a_target'] = list(target)
    smoother = configuration['velocity_smoother']['ros__parameters']
    smoother['min_velocity'][1], smoother['max_velocity'][1] = -0.03, 0.03
    configuration['global_costmap']['global_costmap']['ros__parameters']['always_send_full_costmap'] = True


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
    localization_mode = LaunchConfiguration('localization_mode').perform(context)
    vgicp_reference_pcd = LaunchConfiguration('vgicp_reference_pcd').perform(context)
    vgicp_localization_yaml = LaunchConfiguration('vgicp_localization_yaml').perform(context)
    odin_relocalization_config = LaunchConfiguration('odin_relocalization_config').perform(context)
    tf_stamp_lead = 0.30 if (odin_relocalization_config or localization_mode == 'vgicp') else 0.0
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
    if (odin_relocalization_config or localization_mode == 'vgicp') and publish_map_odom_tf:
        raise ValueError('Active localization and static alignment both publish map -> odom')
    if localization_mode == 'vgicp' and not Path(vgicp_reference_pcd).is_file():
        raise ValueError(f'vgicp_reference_pcd is missing: {vgicp_reference_pcd}')
    if vgicp_localization_yaml and not Path(vgicp_localization_yaml).is_file():
        raise ValueError(f'vgicp_localization_yaml is missing: {vgicp_localization_yaml}')
    if odin_reference_pcd and (localization_mode != 'odin' or not odin_relocalization_config or not Path(odin_reference_pcd).is_file()):
        raise ValueError('odin_reference_pcd requires a saved PCD and ODIN1 odin-mode relocalization')
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
    station = LaunchConfiguration('station_reference').perform(context)
    if station and (goal_time_ab or not odin_relocalization_config):
        raise ValueError('station alignment requires normal Nav2 and ODIN relocalization')
    if goal_time_ab or station:
        config = Path(get_package_share_directory('odin_nav_sim')) / 'config/nav2_real.yaml'
        configuration = yaml.safe_load(config.read_text())
        controller = configuration['controller_server']['ros__parameters']
        if goal_time_ab:
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
        else:
            with np.load(station, allow_pickle=False) as ref:
                target = ref['map_pose'].tolist()
            configure_station(configuration, target)
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
        rewrites = {
            'planner_server.ros__parameters.GridBased.tolerance': '0.035',
            'controller_server.ros__parameters.goal_checker.xy_goal_tolerance':
                '0.022' if localization_mode == 'vgicp' else '0.06',
            'controller_server.ros__parameters.goal_checker.yaw_goal_tolerance':
                '0.05' if localization_mode == 'vgicp' else '0.10',
        }
        if localization_mode == 'vgicp':
            rewrites.update({
                'controller_server.ros__parameters.FollowPath.terminal_settle_hold_time': '0.55',
                'controller_server.ros__parameters.FollowPath.terminal_settle_timeout': '4.0',
            })
        nav2_params = RewrittenYaml(
            source_file=nav2_params,
            param_rewrites=rewrites,
            convert_types=True,
        )
    rviz_config = PathJoinSubstitution([pkg_nav, 'rviz', 'real_navigation.rviz'])
    parking_reference = LaunchConfiguration('parking_reference').perform(context)
    if parking_reference and not Path(parking_reference).is_file():
        raise ValueError(f'Parking reference file missing: {parking_reference}')

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

        Node(
            package='odin_nav_sim', executable='navigation_velocity_guard',
            name='navigation_velocity_guard', output='screen',
            parameters=[nav2_params, {
                'map_stamp_lead': tf_stamp_lead,
                'allow_static_map': publish_map_odom_tf and not enable_control,
            }],
        ),

        # 实车底盘仅消费经过近障与数据时效检查的速度。
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
                    'cmd_vel_topic': '/cmd_vel_guarded',
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

    if station:
        nodes.append(Node(package='odin_nav_sim', executable='station_observation', output='screen',
                          parameters=[{'reference': station}],
                          additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}))
    if parking_reference:
        nodes.append(Node(package='odin_nav_sim', executable='parking_check',
                          arguments=['watch', parking_reference], output='screen'))

    # Only one source may publish the global map -> odom transform.
    if localization_mode == 'vgicp':
        vgicp_params = {
            'reference_pcd': vgicp_reference_pcd,
            'tf_stamp_lead': tf_stamp_lead,
            'use_odin_seed': bool(odin_relocalization_config),
            'odin_map_x': ParameterValue(LaunchConfiguration('odin_map_x'), value_type=float),
            'odin_map_y': ParameterValue(LaunchConfiguration('odin_map_y'), value_type=float),
            'odin_map_yaw': ParameterValue(LaunchConfiguration('odin_map_yaw'), value_type=float),
        }
        if vgicp_localization_yaml:
            loc_cfg = yaml.safe_load(Path(vgicp_localization_yaml).read_text(encoding='utf-8')) or {}
            align = loc_cfg.get('old_map_to_glim_xy_yaw')
            if align and len(align) == 3:
                vgicp_params['odin_map_x'] = float(align[0])
                vgicp_params['odin_map_y'] = float(align[1])
                vgicp_params['odin_map_yaw'] = float(align[2])
            m_xyz = loc_cfg.get('mount_xyz')
            m_rpy = loc_cfg.get('mount_rpy')
            if m_xyz and len(m_xyz) == 3:
                vgicp_params['mount_x'] = float(m_xyz[0])
                vgicp_params['mount_y'] = float(m_xyz[1])
                vgicp_params['mount_z'] = float(m_xyz[2])
            if m_rpy and len(m_rpy) == 3:
                vgicp_params['mount_roll'] = float(m_rpy[0])
                vgicp_params['mount_pitch'] = float(m_rpy[1])
                vgicp_params['mount_yaw'] = float(m_rpy[2])
        nodes.append(Node(
            package='odin_nav_sim',
            executable='vgicp_localization',
            name='vgicp_localization',
            output='screen',
            parameters=[vgicp_params],
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '2'},
        ))
    elif odin_relocalization_config:
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
                    'tf_stamp_lead': tf_stamp_lead,
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
        DeclareLaunchArgument('station_reference', default_value='',
                              description='工位 A 墙面闭环使用的参考帧 .npz'),
        DeclareLaunchArgument('parking_reference', default_value='',
                              description='可选的停车参考帧 .npz，到点后只读复核'),
        DeclareLaunchArgument('goal_time_ab', default_value='false', choices=['true', 'false'],
                              description='精确停车实验中按目标选择原始或时间对齐判定'),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false'], description='是否允许向底盘下发控制指令'),
        DeclareLaunchArgument('publish_map_odom_tf', default_value='false', choices=['true', 'false'], description='只读联调时发布静态 map → odom'),
        DeclareLaunchArgument('localization_mode', default_value='odin', choices=['odin', 'vgicp'], description='全局定位方式：ODIN 重定位或 GLIM 常驻 4-DoF VGICP'),
        DeclareLaunchArgument('vgicp_reference_pcd', default_value='', description='VGICP 定位使用的参考 3D 点云 .pcd 路径'),
        DeclareLaunchArgument('vgicp_localization_yaml', default_value='', description='VGICP 地图对齐与安装外参 YAML 路径'),
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
