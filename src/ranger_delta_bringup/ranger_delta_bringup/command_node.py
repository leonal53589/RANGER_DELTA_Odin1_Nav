import math
import socket
import time

import rclpy
from agx_bringup.msg import MotionModeFeedback, SystemStatus
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .commands import CommandGate, velocity_command
from .protocol import CAN_FRAME


class RangerDeltaCommand(Node):
    def __init__(self):
        super().__init__('ranger_delta_command')
        self.can_socket = None
        self.command = None
        self.status = None
        self.mode = None
        startup_only = ParameterDescriptor(read_only=True)
        self.enabled = self.declare_parameter('enable_control', False, startup_only).value
        interface = self.declare_parameter('can_interface', 'can0', startup_only).value
        topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel', startup_only).value
        self.limits = {
            'wheelbase': self.declare_parameter('wheelbase', 0.378, startup_only).value,
            'track': self.declare_parameter('track', 0.338, startup_only).value,
            'max_linear': self.declare_parameter('max_linear_speed', 0.35, startup_only).value,
            'max_angular': self.declare_parameter('max_angular_speed', 0.6, startup_only).value,
            'min_turn_radius': self.declare_parameter('minimum_turn_radius', 0.5, startup_only).value,
        }
        command_timeout = self.declare_parameter('command_timeout', 0.30, startup_only).value
        feedback_timeout = self.declare_parameter('feedback_timeout', 0.20, startup_only).value
        numbers = (*self.limits.values(), command_timeout, feedback_timeout)
        if not all(math.isfinite(value) and value > 0.0 for value in numbers):
            raise ValueError('Chassis geometry, limits and timeouts must be finite and positive')
        if self.limits['min_turn_radius'] <= self.limits['track'] / 2:
            raise ValueError('minimum_turn_radius must exceed half the track')

        self.gate = CommandGate(command_timeout, feedback_timeout)
        self.create_subscription(Twist, topic, self.receive_command, 1)
        if not self.enabled:
            self.get_logger().info(f'Subscribed to {topic}. CAN transmission disabled.')
            return

        if not interface:
            raise ValueError('Control requires an explicit SocketCAN interface')
        try:
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b'')
            self.can_socket.setblocking(False)
            self.can_socket.bind((interface,))
        except OSError:
            self.destroy_node()
            raise

        self.create_subscription(SystemStatus, '/system_status', self.receive_status, 1)
        self.create_subscription(MotionModeFeedback, '/motion_mode_feedback', self.receive_mode, 1)
        self.timer = self.create_timer(0.02, self.send_update)
        self.get_logger().info(f'Control enabled on {interface}; waiting for fresh feedback and velocity.')

    def receive_command(self, message):
        if self.can_socket is None:
            return
        vx, vy, wz = message.linear.x, message.linear.y, message.angular.z
        unsupported = message.linear.z or message.angular.x or message.angular.y
        command = None if unsupported else velocity_command(vx, vy, wz, **self.limits)
        self.command = (time.monotonic(), command)

    def receive_status(self, message):
        self.status = (time.monotonic(), message.status, message.ctrl_mode, any(bit.value for bit in message.faults))

    def receive_mode(self, message):
        self.mode = (time.monotonic(), message.mode, message.switch_status)

    def send_frames(self, frames):
        for frame_id, payload in frames:
            self.can_socket.send(CAN_FRAME.pack(frame_id, len(payload), payload))

    def send_update(self):
        try:
            self.send_frames(self.gate.frames(time.monotonic(), self.command, self.status, self.mode))
        except OSError as error:
            self.timer.cancel()
            self.get_logger().error(f'CAN transmission stopped: {error}')
            self.close_can()

    def close_can(self):
        if self.can_socket is None:
            return
        try:
            self.send_frames(self.gate.stop())
        except OSError as error:
            self.get_logger().error(f'Could not send the final stop frame: {error}')
        finally:
            self.can_socket.close()
            self.can_socket = None


def main():
    rclpy.init()
    node = None
    try:
        node = RangerDeltaCommand()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close_can()
            node.destroy_node()
        rclpy.try_shutdown()
