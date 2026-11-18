#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MotionMode(Enum):
    DRIVE = 'drive'
    CRAB = 'crab'
    SPIN = 'spin'


class SimulatedBase(Node):
    def __init__(self):
        super().__init__('simulated_base')
        self.switch_delay = self.declare_parameter('mode_switch_delay', 0.35).value
        self.command_timeout = self.declare_parameter('command_timeout', 0.35).value
        self.min_turn_radius = self.declare_parameter('minimum_turn_radius', 0.5).value
        if self.switch_delay < 0 or self.command_timeout <= 0 or self.min_turn_radius < 0:
            raise ValueError('Mode switch delay and turning radius must be nonnegative, and timeout must be positive.')

        self.x = self.declare_parameter('initial_x', 0.0).value
        self.y = self.declare_parameter('initial_y', 0.0).value
        self.yaw = self.declare_parameter('initial_yaw', 0.0).value
        self.mode = MotionMode.DRIVE
        self.pending_mode = None
        self.switch_ends_at = 0.0
        self.command = (0.0, 0.0, 0.0)
        self.last_update = time.monotonic()
        self.last_command = self.last_update

        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.receive_command, 1)
        self.create_timer(0.02, self.update)

    def receive_command(self, message):
        command = (message.linear.x, message.linear.y, message.angular.z)
        self.command = tuple(
            value if math.isfinite(value) and abs(value) > 1e-4 else 0.0
            for value in command
        )
        self.last_command = time.monotonic()

    def applied_velocity(self, now):
        stopped = (0.0, 0.0, 0.0)
        if now - self.last_command > self.command_timeout or self.command == stopped:
            self.pending_mode = None
            return stopped

        vx, vy, wz = self.command
        if vy:
            requested_mode = MotionMode.CRAB
            velocity = (vx, vy, 0.0)
        elif wz and (not vx or abs(vx / wz) < self.min_turn_radius):
            requested_mode = MotionMode.SPIN
            velocity = (0.0, 0.0, wz)
        else:
            requested_mode = MotionMode.DRIVE
            velocity = (vx, 0.0, wz)

        if requested_mode == self.mode:
            self.pending_mode = None
            return velocity
        if requested_mode != self.pending_mode:
            self.pending_mode = requested_mode
            self.switch_ends_at = now + self.switch_delay
            self.get_logger().info(f'Switching from {self.mode.value} to {requested_mode.value}')
        if now < self.switch_ends_at:
            return stopped

        self.mode = requested_mode
        self.pending_mode = None
        self.get_logger().info(f'Motion mode ready: {self.mode.value}')
        return velocity

    def update(self):
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now
        vx, vy, wz = self.applied_velocity(now)
        mid_yaw = self.yaw + wz * dt / 2.0
        self.x += (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt
        self.y += (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt
        self.yaw = math.remainder(self.yaw + wz * dt, math.tau)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)


def main():
    rclpy.init()
    node = SimulatedBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
