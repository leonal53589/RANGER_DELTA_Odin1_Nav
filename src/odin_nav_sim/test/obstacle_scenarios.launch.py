"""Run with ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 launch_test <this file>."""
import os
import time
import unittest
from pathlib import Path

import launch_testing
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from std_srvs.srv import Trigger


def generate_test_description():
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('Use isolated localhost ROS domain 87 for this simulation')
    share = Path(get_package_share_directory('odin_nav_sim'))
    config = str(share / 'config/nav2_real.yaml')
    return LaunchDescription([
        GroupAction([
            SetRemap(src='simulated_base:/cmd_vel', dst='/cmd_vel_guarded'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / 'launch/navigation_sim.launch.py')),
                launch_arguments={'launch_rviz': 'false', 'params_file': config}.items()),
        ]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--x', '0.21', '--z', '1.125', '--frame-id', 'base_link', '--child-frame-id', 'lidar']),
        Node(package='odin_nav_sim', executable='navigation_velocity_guard',
             parameters=[config, {'allow_static_map': True}]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestNavigation(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('obstacle_fixture')
        self.addCleanup(rclpy.shutdown)
        self.addCleanup(self.node.destroy_node)
        self.endpoints = []
        self.node.create_subscription(PoseStamped, '/controller_server/tracking_endpoint',
                                      self.endpoints.append, 10)
        self.cloud = self.node.create_publisher(PointCloud2, '/odin1/cloud_raw', qos_profile_sensor_data)
        self.returns = 'clear'
        self.velocities = []
        self.node.create_subscription(Twist, '/cmd_vel_guarded', self.receive_velocity, 10)
        self.node.create_timer(.08, self.scan)

    def scan(self):
        if self.returns == 'off':
            return
        # Downward returns have 16 cm horizontal range but >50 cm slant range.
        x = .16 if self.returns == 'blocked' else 3.1
        header = Header(frame_id='lidar', stamp=self.node.get_clock().now().to_msg())
        points = [[x, x*i/300., -.52] for i in range(-24, 25)]
        self.cloud.publish(create_cloud_xyz32(header, points))

    def receive_velocity(self, msg):
        self.velocities.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def until(self, condition, timeout):
        end = time.monotonic() + timeout
        while not condition():
            self.assertLess(time.monotonic(), end, 'simulation timeout')
            rclpy.spin_once(self.node, timeout_sec=.025)

    def start(self, action, name, positions):
        client = ActionClient(self.node, action, name)
        self.addCleanup(client.destroy)
        self.assertTrue(client.wait_for_server(timeout_sec=25))
        poses = []
        for x in positions:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
            poses.append(pose)
        goal = action.Goal()
        if action is NavigateToPose:
            goal.pose = poses[0]
        else:
            goal.poses = poses
        self.velocities.clear()
        pending = client.send_goal_async(goal)
        self.until(pending.done, 5)
        self.assertTrue(pending.result().accepted)
        result = pending.result().get_result_async()
        self.until(lambda: self.velocities and self.velocities[-1][1] > .045, 12)
        return result

    def interrupt(self, result, mode, duration):
        self.returns = mode
        begin = time.monotonic()
        self.until(lambda: time.monotonic() >= begin + duration, duration + 1)
        zeros = [t for t, vx, wz in self.velocities if t >= begin and vx == wz == 0.]
        self.assertTrue(zeros)
        self.assertEqual(self.velocities[-1][1:], (0., 0.))
        self.assertFalse(result.done())
        print(f'{mode}: guarded zero after {zeros[0]-begin:.3f} s', flush=True)
        self.returns = 'clear'
        self.until(result.done, 45)
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)

    def test_single_then_waypoints(self):
        def isolated_chassis():
            readers = self.node.get_subscriptions_info_by_topic('/cmd_vel_guarded')
            return {reader.node_name for reader in readers} == {'simulated_base', self.node.get_name()}
        self.until(isolated_chassis, 25)
        active = self.node.create_client(Trigger, 'lifecycle_manager_navigation/is_active')
        self.assertTrue(active.wait_for_service(timeout_sec=25))
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            check = active.call_async(Trigger.Request())
            self.until(check.done, 3)
            if check.result().success:
                break
        else:
            self.fail('navigation lifecycle did not activate')
        single = self.start(NavigateToPose, 'navigate_to_pose', [.90])
        self.interrupt(single, 'off', .50)
        multi = self.start(NavigateThroughPoses, 'navigate_through_poses', [1.30, 1.80])
        self.interrupt(multi, 'blocked', .80)
        self.assertGreaterEqual(len(self.endpoints), 2)
        self.assertEqual(self.endpoints[-1].header.frame_id, 'map')
        self.assertLess(abs(self.endpoints[-1].pose.position.x - 1.80), .20)

        # A permanently unreachable goal must return an action result without driving.
        single_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.addCleanup(single_client.destroy)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = -10.
        goal.pose.pose.orientation.w = 1.
        self.velocities.clear()
        begin = time.monotonic()
        request = single_client.send_goal_async(goal)
        self.until(request.done, 5)
        self.assertTrue(request.result().accepted)
        result = request.result().get_result_async()
        self.until(result.done, 28)
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertGreaterEqual(time.monotonic() - begin, 19.)
        self.assertTrue(self.velocities)
        self.assertTrue(all(vx == wz == 0. for _, vx, wz in self.velocities))
