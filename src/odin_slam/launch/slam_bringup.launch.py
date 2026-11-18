import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_odin_driver = FindPackageShare('odin_ros_driver')
    pkg_odin_slam = FindPackageShare('odin_slam')

    # Arguments
    launch_driver_arg = DeclareLaunchArgument(
        'launch_driver',
        default_value='true',
        description='Whether to start odin_ros_driver'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    # Config files
    slam_config = PathJoinSubstitution([
        pkg_odin_slam,
        'config',
        'odin_slam.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_odin_driver,
        'config',
        'odin_ros2.rviz'
    ])

    # Driver launch
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_odin_driver,
                'launch',
                'odin1_ros2.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('launch_driver'))
    )

    # SLAM node
    slam_node = Node(
        package='odin_slam',
        executable='odin_slam_node',
        name='odin_slam_node',
        output='screen',
        parameters=[slam_config]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        launch_driver_arg,
        launch_rviz_arg,
        driver_launch,
        slam_node,
        rviz_node
    ])
