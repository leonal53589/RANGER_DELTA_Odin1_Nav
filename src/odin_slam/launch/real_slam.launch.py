from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    pkg_odin_driver = FindPackageShare('odin_ros_driver')
    pkg_odin_slam = FindPackageShare('odin_slam')
    pkg_chassis = FindPackageShare('ranger_delta_bringup')

    can_interface = LaunchConfiguration('can_interface').perform(context)
    enable_control = LaunchConfiguration('enable_control').perform(context) == 'true'
    launch_gui = LaunchConfiguration('launch_gui').perform(context) == 'true'
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context) == 'true'

    odin_odom_frame = 'odin_odom'
    slam_config = PathJoinSubstitution([pkg_odin_slam, 'config', 'odin_slam.yaml'])
    driver_config = PathJoinSubstitution([pkg_odin_driver, 'config', 'control_command_robot.yaml'])
    rviz_config = PathJoinSubstitution([pkg_odin_driver, 'config', 'odin_ros2.rviz'])

    nodes = [
        # 1. 底盘反馈节点 (发布 /odom 及 odom -> base_link TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_chassis, 'launch', 'feedback.launch.py'])),
            launch_arguments={
                'source': 'socketcan',
                'can_interface': can_interface,
                'publish_odom_tf': 'true',
                'odom_topic': '/odom',
            }.items(),
        ),

        # 2. 底盘速度控制节点，使用首测限速和门控。
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

        # 3. 建图点云保留在 Odin1 自己的里程计坐标系。
        Node(
            package='odin_ros_driver',
            executable='host_sdk_sample',
            name='host_sdk_sample',
            output='screen',
            parameters=[{'config_file': driver_config, 'odom_frame': odin_odom_frame, 'map_frame': 'odin_map'}],
        ),

        # Register the origin in TF for RViz without joining it to the chassis.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odin_odom_origin_tf',
            arguments=['--frame-id', odin_odom_frame, '--child-frame-id', 'odin_odom_origin'],
        ),

        # 6. SLAM 实时点云累加与建图节点 (带 Qt 控制界面与保存 PCD 功能)
        Node(
            package='odin_slam',
            executable='odin_slam_node',
            name='odin_slam_node',
            output='screen',
            parameters=[slam_config],
        ),
    ]

    # 可选: 启动安全运控 UI 面板
    if launch_gui:
        nodes.append(
            Node(
                package='ranger_delta_bringup',
                executable='ranger_delta_teleop_gui',
                name='ranger_delta_teleop_gui',
                output='screen',
            )
        )

    # 可选: 启动 RViz2
    if launch_rviz:
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_slam',
                output='screen',
                arguments=['-d', rviz_config, '-f', odin_odom_frame],
            )
        )

    return nodes


def generate_launch_description():
    description = PathJoinSubstitution([FindPackageShare('ranger_delta_bringup'), 'launch', 'description.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0', description='SocketCAN 接口名'),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false'], description='是否允许向底盘下发速度'),
        DeclareLaunchArgument('launch_gui', default_value='false', choices=['true', 'false'], description='是否启动图形化运控面板'),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false'], description='是否启动 RViz2 观察点云'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description)),
        OpaqueFunction(function=launch_setup),
    ])
