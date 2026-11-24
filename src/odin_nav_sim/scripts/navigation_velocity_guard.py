#!/usr/bin/env python3
"""Validate live navigation inputs and the near-field sweep before forwarding a Twist."""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cloud_xy(message, transform, low, high):
    if (message.header.frame_id != 'lidar' or not message.width or not message.height
            or message.row_step != message.width * message.point_step
            or len(message.data) != message.row_step * message.height):
        raise ValueError('incomplete lidar cloud')
    fields = read_points(message, field_names=('x', 'y', 'z'))
    xyz = np.column_stack([fields[key] for key in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz != 0).any(axis=1)]
    if not len(xyz):
        raise ValueError('no valid lidar returns')
    p, q = transform.translation, transform.rotation
    base = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(xyz) + [p.x, p.y, p.z]
    if not np.isfinite(base).all():
        raise ValueError('invalid mount transform')
    return base[(base[:, 2] >= low) & (base[:, 2] <= high), :2]


def occupied_sweep(points, vx, vy, wz, radius, margin, reaction, decel):
    speed = math.hypot(vx, vy)
    distance = margin + speed * reaction + speed ** 2 / (2 * decel)
    # Bound lateral travel during a turn by arc length times maximum heading change.
    radius += distance * min(1.0, abs(wz) * (reaction + speed / decel)) if speed else 0.0
    # Distance from each return to the segment swept by a circular footprint.
    if speed > 1e-5:
        dx, dy = vx / speed, vy / speed
        along = np.clip(points[:, 0] * dx + points[:, 1] * dy, 0, distance)
        x, y = points[:, 0] - along * dx, points[:, 1] - along * dy
    else:
        x, y = points[:, 0], points[:, 1]
    return np.count_nonzero(x*x + y*y <= radius*radius) >= 3


class NavigationVelocityGuard(Node):
    def __init__(self):
        super().__init__('navigation_velocity_guard')

        def number(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            return value

        self.cloud_timeout = number('cloud_timeout', 0.30)
        self.odom_timeout = number('odom_timeout', 0.20)
        self.map_timeout = number('map_timeout', 0.50)
        self.command_timeout = number('command_timeout', 0.20)
        self.allow_static_map = self.declare_parameter('allow_static_map', False).value
        self.map_lead = self.declare_parameter('map_stamp_lead', 0.0).value
        if not math.isfinite(self.map_lead) or self.map_lead < 0:
            raise ValueError('invalid map TF timestamp lead')
        self.radius = number('sweep_radius', 0.32)
        self.margin = number('distance_margin', 0.04)
        self.reaction = number('reaction_time', 0.35)
        self.decel = number('brake_deceleration', 0.5)
        self.low = number('min_height', 0.12)
        self.high = number('max_height', 1.35)
        if self.low >= self.high:
            raise ValueError('invalid cloud height interval')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.points = self.wheel = self.command = None
        self.cloud_stamp = self.wheel_stamp = 0.
        self.received_cloud = self.received_wheel = self.received_command = -math.inf
        self.rejected_command_time = -math.inf
        self.resume_time = None
        self.blocked_velocity = None
        self.reason = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/odin1/cloud_raw', self.scan, qos)
        self.create_subscription(Odometry, '/odom', self.odom, qos)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 1)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = self.create_publisher(String, '/navigation_velocity_guard/state', state_qos)
        self.output = self.create_publisher(Twist, '/cmd_vel_guarded', 1)
        self.create_timer(0.025, self.tick)

    def scan(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if stamp <= self.cloud_stamp:
            return  # duplicate stamps cannot refresh the watchdog
        now = self.get_clock().now().nanoseconds * 1e-9
        if not 0 <= now - stamp <= self.cloud_timeout:
            self.points = None
            return
        try:
            mount = self.tf.lookup_transform('base_link', 'lidar', Time()).transform
            self.points = cloud_xy(message, mount, self.low, self.high)
        except (TransformException, AssertionError, ValueError, TypeError, KeyError):
            self.points = None
            return
        self.cloud_stamp, self.received_cloud = stamp, time.monotonic()

    def odom(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if stamp <= self.wheel_stamp:
            return
        v = message.twist.twist
        if (message.header.frame_id != 'odom' or message.child_frame_id != 'base_link'
                or not all(map(math.isfinite, (v.linear.x, v.linear.y, v.angular.z)))
                or not 0 <= self.get_clock().now().nanoseconds * 1e-9 - stamp <= self.odom_timeout):
            self.wheel = None
            return
        self.wheel = (v.linear.x, v.linear.y, v.angular.z)
        self.wheel_stamp, self.received_wheel = stamp, time.monotonic()

    def velocity(self, message):
        self.command, self.received_command = message, time.monotonic()

    def inhibition(self, now, wall):
        if (self.points is None or wall - self.received_cloud > self.cloud_timeout
                or not 0 <= now - self.cloud_stamp <= self.cloud_timeout):
            return 'lidar missing or stale'
        if (self.wheel is None or wall - self.received_wheel > self.odom_timeout
                or not 0 <= now - self.wheel_stamp <= self.odom_timeout):
            return 'wheel odometry missing or stale'
        try:
            tf = self.tf.lookup_transform('map', 'odom', Time())
            age = now - (stamp_seconds(tf.header.stamp) - self.map_lead)
            static_preview = self.allow_static_map and stamp_seconds(tf.header.stamp) == 0.0
            if not static_preview and not -0.025 <= age <= self.map_timeout:
                return 'map alignment stale'
        except TransformException:
            return 'map alignment missing'
        try:
            wheel_tf = self.tf.lookup_transform('odom', 'base_link', Time())
            if not 0 <= now - stamp_seconds(wheel_tf.header.stamp) <= self.odom_timeout:
                return 'wheel transform stale'
        except TransformException:
            return 'wheel transform missing'
        if self.command is None or wall - self.received_command > self.command_timeout:
            return 'waiting for fresh velocity'
        v = self.command
        if (not all(map(math.isfinite, (v.linear.x, v.linear.y, v.angular.z)))
                or v.linear.z or v.angular.x or v.angular.y):
            return 'invalid velocity'

        def occupied(velocity):
            return occupied_sweep(self.points, *velocity, self.radius, self.margin, self.reaction, self.decel)

        # Keep the original braking envelope while a same-direction command slows.
        # Rotation or retreat can still use their own envelope once the wheels stop.
        if self.blocked_velocity:
            bx, by, _ = self.blocked_velocity
            if not occupied(self.blocked_velocity):
                self.blocked_velocity = None
            elif bx * v.linear.x + by * v.linear.y > 0:
                return 'near obstacle'
        for velocity in ((v.linear.x, v.linear.y, v.angular.z), self.wheel):
            if occupied(velocity):
                self.blocked_velocity = velocity
                return 'near obstacle'
        if self.received_command <= self.rejected_command_time:
            return 'waiting for fresh velocity'
        return None

    def tick(self):
        wall = time.monotonic()
        reason = self.inhibition(self.get_clock().now().nanoseconds * 1e-9, wall)
        out = Twist()
        if reason:
            self.rejected_command_time = self.received_command
            self.resume_time = None
        else:
            if self.resume_time is None:
                self.resume_time = wall
            # The upstream open-loop smoother may have reached full speed during a stop.
            # Ramp up after release, preserving curvature and allowing immediate braking.
            elapsed = wall - self.resume_time
            v = self.command
            scale = min(1., 0.5 * elapsed / max(1e-9, math.hypot(v.linear.x, v.linear.y)),
                        elapsed / max(1e-9, abs(v.angular.z)))
            out.linear.x, out.linear.y = v.linear.x * scale, v.linear.y * scale
            out.angular.z = v.angular.z * scale
        self.output.publish(out)
        if reason != self.reason:
            state = reason or 'velocity forwarding active'
            self.state.publish(String(data=state))
            self.get_logger().info(state)
            self.reason = reason


def main():
    rclpy.init()
    guard = NavigationVelocityGuard()
    try:
        rclpy.spin(guard)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        guard.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
