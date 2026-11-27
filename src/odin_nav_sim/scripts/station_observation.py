#!/usr/bin/env python3
"""Publish the measured A -> base SE(2) offset at the ODIN scan timestamp."""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_ros import Buffer, TransformException, TransformListener

from parking_geometry import map_fingerprint, pose_xy_yaw, stamp_ns
from parking_keyframe import base_cloud, relative_pose
from station_walls import StationWalls


class StationObservation(Node):
    def __init__(self):
        super().__init__('station_observation')
        path = self.declare_parameter('reference', '').value
        with np.load(path, allow_pickle=False) as ref:
            self.expected_map = str(ref['map_id'])
            self.walls = StationWalls(ref['points'], ref['map_pose'])
        self.current_map = None
        self.last = 0
        self.reason = ''
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scans = deque(maxlen=4)
        self.output = self.create_publisher(PoseStamped, '/station_a/offset', 1)
        self.create_subscription(PointCloud2, '/odin1/cloud_raw', self.scans.append, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_received,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.06, self.observe)

    def map_received(self, msg):
        self.current_map = map_fingerprint(msg)

    def observe(self):
        now = self.get_clock().now().nanoseconds
        for scan in reversed(self.scans):
            stamp = stamp_ns(scan.header.stamp)
            if stamp <= self.last or not 0 <= now - stamp < 300_000_000:
                continue
            at = Time.from_msg(scan.header.stamp)
            try:
                nav = self.tf.lookup_transform('map', 'base_link', at).transform
                mount = self.tf.lookup_transform('base_link', 'lidar', at).transform
            except TransformException:
                continue
            self.last = stamp
            msg = PoseStamped()
            msg.header.stamp = scan.header.stamp
            try:
                if self.current_map != self.expected_map or scan.header.frame_id != 'lidar':
                    raise ValueError('map or lidar frame mismatch')
                guess = pose_xy_yaw(nav.translation, nav.rotation)
                if np.linalg.norm(relative_pose(self.walls.map_pose, guess)[:2]) > 0.4:
                    raise ValueError('outside station acquisition region')
                fields = read_points(scan, field_names=('x', 'y', 'z'))
                xyz = np.column_stack([fields[k] for k in ('x', 'y', 'z')])
                q, p = mount.rotation, mount.translation
                points = base_cloud(xyz, Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                    [p.x, p.y, p.z])
                x, y, yaw = self.walls.offset(points, guess)
                msg.header.frame_id = 'station_a'
                msg.pose.position.x, msg.pose.position.y = float(x), float(y)
                msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                reason = 'station walls matched'
            except (ValueError, KeyError, TypeError, AssertionError, np.linalg.LinAlgError) as exc:
                reason = str(exc)  # Empty frame immediately invalidates the previous match.
            if reason != self.reason:
                self.get_logger().info(reason)
                self.reason = reason
            self.output.publish(msg)
            return


def main():
    rclpy.init()
    node = StationObservation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
