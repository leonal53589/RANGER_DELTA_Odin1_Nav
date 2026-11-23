import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    xyz = [LaunchConfiguration(f'sensor_{axis}').perform(context) for axis in ('x', 'y', 'z')]
    rpy = [LaunchConfiguration(f'sensor_{axis}').perform(context) for axis in ('roll', 'pitch', 'yaw')]
    if any(xyz) and not all(xyz):
        raise ValueError('Provide sensor_x, sensor_y and sensor_z together')
    if not all(math.isfinite(float(value)) for value in (*filter(None, xyz), *rpy)):
        raise ValueError('Odin1 mount coordinates and angles must be finite')

    pkg_share = FindPackageShare('ranger_delta_bringup')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'ranger_delta.urdf.xacro'])
    mount_arguments = []
    if all(xyz):
        mount_arguments = [
            ' sensor_mounted:=true',
            ' "sensor_xyz:=', ' '.join(xyz), '"',
            ' "sensor_rpy:=', ' '.join(rpy), '"',
        ]
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' "', xacro_file, '"', *mount_arguments]),
        value_type=str,
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Measured fixed ODIN1 mount. The base_link origin is at ground level.
        DeclareLaunchArgument('sensor_x', default_value='0.210', description='LiDAR origin forward from base_link (m)'),
        DeclareLaunchArgument('sensor_y', default_value='0.0', description='LiDAR origin left of base_link (m)'),
        DeclareLaunchArgument('sensor_z', default_value='1.125', description='LiDAR origin above base_link (m)'),
        DeclareLaunchArgument('sensor_roll', default_value='0.0', description='LiDAR mounting roll (rad)'),
        DeclareLaunchArgument('sensor_pitch', default_value='0.0', description='LiDAR mounting pitch (rad)'),
        DeclareLaunchArgument('sensor_yaw', default_value='0.0', description='LiDAR mounting yaw (rad)'),
        OpaqueFunction(function=launch_setup),
    ])
