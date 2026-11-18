from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('odin_slam'),
        'config',
        'odin_slam.yaml'
    ])

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('odin_ros_driver'),
                'launch',
                'odin1_ros2.launch.py'
            ])
        )
    )

    return LaunchDescription([
        # driver_launch,
        Node(
            package='odin_slam',
            executable='odin_slam_node',
            name='odin_slam_node',
            output='screen',
            parameters=[config]
        )
    ])
