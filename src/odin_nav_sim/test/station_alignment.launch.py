"""Offline Nav2 test on a simulated base and reprojected measured station cloud."""
import math
import os
from pathlib import Path
import runpy
import tempfile
import time
import unittest

import launch_testing
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from std_srvs.srv import Trigger


def generate_test_description():
    if os.getenv('ROS_DOMAIN_ID') != '89' or os.getenv('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('Use isolated simulation ROS domain 89 on localhost')
    share = Path(get_package_share_directory('odin_nav_sim'))
    site = Path(__file__).resolve().parents[3] / 'maps/odin_20261124_192624_a0FBCc'
    reference = site / 'station_A.npz'
    with np.load(reference) as f:
        target = f['map_pose'].tolist()
    config = yaml.safe_load((share/'config/nav2_real.yaml').read_text())
    runpy.run_path(str(share/'launch/real_navigation.launch.py'))['configure_station'](config, target)
    left = os.getenv('STATION_SIM_APPROACH', 'right') == 'left'
    nav_offset = [-0.0183, 0.0785, -0.1770] if left else [0.0303, -0.0742, 0.1743]
    physical_offset = [0.0152, 0.0160, -0.1700] if left else [-0.0076, -0.1764, 0.1806]
    bias_yaw = physical_offset[2] - nav_offset[2]
    bias_xy = physical_offset[:2] - rot(bias_yaw) @ nav_offset[:2]
    initial_bias = np.r_[bias_xy, bias_yaw]
    start_xy = np.array(target[:2]) + rot(target[2]) @ nav_offset[:2]
    tmp = tempfile.TemporaryDirectory()
    params = Path(tmp.name)/'nav.yaml'
    params.write_text(yaml.safe_dump(config))
    return LaunchDescription([
        GroupAction([SetRemap(src='simulated_base:/cmd_vel', dst='/cmd_vel_guarded'),
                     IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/navigation_sim.launch.py')),
                         launch_arguments=dict(launch_rviz='false', map=str(site/'site_nav.yaml'),
                             params_file=str(params), initial_x=str(start_xy[0]),
                             initial_y=str(start_xy[1]), initial_yaw=str(target[2] + nav_offset[2])).items())]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.21', '--z', '1.125', '--frame-id', 'base_link', '--child-frame-id', 'lidar']),
        Node(package='odin_nav_sim', executable='station_observation', parameters=[{'reference': str(reference)}]),
        Node(package='odin_nav_sim', executable='navigation_velocity_guard',
             parameters=[str(params), {'allow_static_map': True}]),
        launch_testing.actions.ReadyToTest(),
    ]), dict(reference=reference, tmp=tmp, initial_bias=initial_bias)


def rot(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


class TestStation(unittest.TestCase):
    def test_station(self, reference, initial_bias):
        rclpy.init()
        node = rclpy.create_node('station_fixture')
        self.addCleanup(rclpy.shutdown)
        self.addCleanup(node.destroy_node)
        with np.load(reference) as data:
            a, scene = data['map_pose'], data['points']
        cmd = []
        node.create_subscription(Twist, '/cmd_vel_guarded', lambda m: cmd.append(m), 1)
        self.odom = None
        node.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 1)
        pub = node.create_publisher(PointCloud2, '/odin1/cloud_raw', qos_profile_sensor_data)
        bias = initial_bias.copy()
        self.blocked = False
        self.cloud_on = True
        def actual():
            p = self.odom.pose.pose
            dxy = rot(-a[2]) @ (np.array([p.position.x, p.position.y])-a[:2])
            yaw = 2*math.atan2(p.orientation.z, p.orientation.w)-a[2]
            return np.r_[bias[:2]+rot(bias[2]) @ dxy, yaw+bias[2]]
        def scan():
            if self.odom is None or not self.cloud_on:
                return
            robot = actual()
            pts = scene.copy()
            pts[:, :2] = (pts[:, :2] - robot[:2]) @ rot(robot[2])
            pts -= [0.21, 0., 1.125]
            if self.blocked:
                pts = np.vstack((pts, [[0.04, y, -.5] for y in (-.02, 0., .02, .04)]))
            header = Header(frame_id='lidar', stamp=node.get_clock().now().to_msg())
            pub.publish(create_cloud_xyz32(header, pts.astype(np.float32)))
        node.create_timer(0.09, scan)
        def until(fn, limit):
            end = time.monotonic()+limit
            while not fn():
                self.assertLess(time.monotonic(), end)
                rclpy.spin_once(node, timeout_sec=0.02)
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.addCleanup(client.destroy)
        until(lambda: client.server_is_ready() and self.odom is not None, 35)
        manager = node.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        until(manager.service_is_ready, 30)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            pending = manager.call_async(Trigger.Request())
            until(pending.done, 5)
            if pending.result().success:
                break
            finish = time.monotonic() + .2
            until(lambda: time.monotonic() >= finish, 1)
        else:
            self.fail('Nav2 lifecycle activation timed out')
        def send(tree):
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = 'map'
            g.pose.pose.position.x, g.pose.pose.position.y = a[:2]
            q = g.pose.pose.orientation
            q.z, q.w = math.sin(a[2]/2), math.cos(a[2]/2)
            g.behavior_tree = str(Path(get_package_share_directory('odin_nav_sim'))/'behavior_trees'/tree)
            pending = client.send_goal_async(g)
            until(pending.done, 5)
            handle = pending.result()
            self.assertTrue(handle.accepted)
            return handle, handle.get_result_async()
        _, result = send('return_to_station.xml')
        until(result.done, 75)
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)
        final = actual()
        print('Simulated physical end pose (m, rad)', final, flush=True)
        self.assertLess(np.linalg.norm(final[:2]), 0.03)
        self.assertLess(abs(final[2]), math.radians(5))
        self.assertTrue(any(abs(v.angular.z) > 0.01 for v in cmd))
        # Cancel during fine alignment, then verify loss of fresh scans stops and fails the next goal.
        bias[1] += .10
        cmd.clear()
        handle, result = send('align_at_station.xml')
        until(lambda: any(abs(v.linear.y) > .005 for v in cmd), 8)
        cancel = handle.cancel_goal_async()
        until(cancel.done, 5)
        until(result.done, 5)
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        until(lambda: cmd and abs(cmd[-1].linear.y) < .001, 2)
        cmd.clear()
        _, result = send('align_at_station.xml')
        until(lambda: cmd and abs(cmd[-1].linear.y) > .005, 8)
        self.cloud_on = False
        until(lambda: abs(cmd[-1].linear.y) < .001, 1.2)
        resume_at = time.monotonic() + .35
        until(lambda: time.monotonic() >= resume_at, 1.0)
        self.assertFalse(result.done())
        self.cloud_on = True
        until(result.done, 25)
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)

        bias[1] -= .10
        cmd.clear()
        _, result = send('align_at_station.xml')
        until(lambda: any(abs(v.linear.y) > .005 for v in cmd), 8)
        self.cloud_on = False
        until(result.done, 6)
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertAlmostEqual(cmd[-1].linear.y, 0.)

        self.cloud_on = True
        self.blocked = True
        # Re-establish fresh wall observations while a simulated obstacle sits in the footprint.
        end = time.monotonic() + 1.5
        until(lambda: time.monotonic() >= end, 3)
        cmd.clear()
        _, result = send('align_at_station.xml')
        until(result.done, 9)
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertTrue(cmd)
        self.assertFalse(any(abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z) > .001 for v in cmd))
