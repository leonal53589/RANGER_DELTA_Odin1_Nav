from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    if enable_control and publish_map_odom_tf:
        raise ValueError('Static map alignment is available only with enable_control:=false')
    if enable_control and not all(
        LaunchConfiguration(f'sensor_{axis}').perform(context) for axis in ('x', 'y', 'z')
    ):
        raise ValueError('Provide measured sensor_x, sensor_y and sensor_z before enabling control')
    if not Path(map_yaml_file).is_file():
        raise ValueError(f'Map YAML is missing: {map_yaml_file}')

    driver_config = PathJoinSubstitution([pkg_odin_driver, 'config', 'control_command_robot.yaml'])
    nav2_params = PathJoinSubstitution([pkg_nav, 'config', 'nav2_real.yaml'])
    rviz_config = PathJoinSubstitution([pkg_nav, 'rviz', 'navigation_sim.rviz'])

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
    ]

    # 可选: 全局定位 TF (在无独立重定位节点时提供 map -> odom 原点基准)
    if publish_map_odom_tf:
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

    return nodes


def generate_launch_description():
    description = PathJoinSubstitution([FindPackageShare('ranger_delta_bringup'), 'launch', 'description.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('map', description='场地栅格地图 YAML 绝对路径'),
        DeclareLaunchArgument('can_interface', default_value='can0', description='SocketCAN 接口名'),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false'], description='是否允许向底盘下发控制指令'),
        DeclareLaunchArgument('publish_map_odom_tf', default_value='false', choices=['true', 'false'], description='只读联调时发布静态 map → odom'),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false'], description='是否启动 RViz2 导航面板'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description)),
        OpaqueFunction(function=launch_setup),
    ])
