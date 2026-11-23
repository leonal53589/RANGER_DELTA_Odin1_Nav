import math
import socket
import struct
import time
from pathlib import Path

import rclpy
from agx_bringup.msg import ChassisMotion, FaultBit, MotionModeFeedback, SteeringAngles, SystemStatus, WheelSpeeds
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .protocol import (
    CAN_FRAME,
    CHASSIS_MOTION,
    FRAME_FORMATS,
    MOTION_MODE,
    STEERING_ANGLES,
    SYSTEM_STATUS,
    WHEEL_SPEEDS,
    OdometryEstimate,
    body_velocity,
    decode_frame,
    replay_frames,
)


STATUS_NAMES = {0: 'Normal', 2: 'Error'}
CONTROL_NAMES = {0: 'Standby', 1: 'CAN', 2: 'Serial', 3: 'Remote control'}
MODE_NAMES = {0: 'Front-rear Ackermann', 1: 'Crab steering', 2: 'Spin turn', 3: 'Parking'}
SWITCH_NAMES = {0: 'Completed', 1: 'Switching'}
DRIVE_NAMES = {0: 'Current drive', 1: 'Voltage drive'}


class RangerDeltaFeedback(Node):
    def __init__(self):
        super().__init__('ranger_delta_feedback')
        source = self.declare_parameter('source', 'replay').value
        replay_file = self.declare_parameter('replay_file', '').value
        interface = self.declare_parameter('can_interface', 'can0').value
        self.wheelbase = self.declare_parameter('wheelbase', 0.378).value
        self.track = self.declare_parameter('track', 0.338).value
        self.feedback_timeout = self.declare_parameter('feedback_timeout', 0.2).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        odom_topic = self.declare_parameter('odom_topic', '/agx/wheel_odom_raw').value
        publish_tf = self.declare_parameter('publish_odom_tf', True).value
        if not all(math.isfinite(v) and v > 0 for v in (self.wheelbase, self.track, self.feedback_timeout)):
            raise ValueError('wheelbase, track and feedback_timeout must be finite and positive')
        if not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame:
            raise ValueError('odom_frame and base_frame must be distinct and nonempty')

        self.estimate = OdometryEstimate()
        self.mode_feedback = None
        self.mode_timestamp = None
        self.status_pub = self.create_publisher(SystemStatus, '/system_status', 10)
        self.motion_pub = self.create_publisher(ChassisMotion, '/chassis_motion_feedback', 10)
        self.mode_pub = self.create_publisher(MotionModeFeedback, '/motion_mode_feedback', 10)
        self.steering_pub = self.create_publisher(SteeringAngles, '/steering_angles', 10)
        self.wheel_speeds_pub = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf = TransformBroadcaster(self) if publish_tf else None

        self.can_socket = None
        self.capture = None
        try:
            if source == 'replay':
                if not replay_file:
                    raise ValueError('replay mode requires replay_file pointing to a candump -L log')
                self.capture = Path(replay_file).open(encoding='ascii')
                self.frames = replay_frames(self.capture)
                self.pending_frame = next(self.frames, None)
                if self.pending_frame is None:
                    raise ValueError(f'No Delta feedback frames in {replay_file}')
                self.capture_start = self.pending_frame[0]
                self.replay_start = time.monotonic()
                self.timer = self.create_timer(0.01, self.read_replay)
                self.get_logger().info(f'Replaying Delta feedback from {replay_file}')
            elif source == 'socketcan':
                self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                mask = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
                filters = b''.join(struct.pack('=II', frame_id, mask) for frame_id in FRAME_FORMATS)
                self.can_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
                self.can_socket.setblocking(False)
                self.can_socket.bind((interface,))
                self.timer = self.create_timer(0.01, self.read_socket)
                self.get_logger().info(f'Listening to Delta feedback on {interface}')
            else:
                raise ValueError(f'Unsupported feedback source: {source}')
        except (OSError, ValueError):
            self.destroy_node()
            raise

    def read_replay(self):
        playback_time = self.capture_start + (time.monotonic() - self.replay_start)
        while self.pending_frame is not None and self.pending_frame[0] <= playback_time:
            self.handle_frame(*self.pending_frame)
            self.pending_frame = next(self.frames, None)
        if self.pending_frame is None:
            self.capture.close()
            self.timer.cancel()
            self.get_logger().info('Delta feedback replay complete')

    def read_socket(self):
        for _ in range(128):
            try:
                frame = self.can_socket.recv(CAN_FRAME.size)
            except BlockingIOError:
                return
            except OSError as error:
                self.timer.cancel()
                self.get_logger().error(f'CAN reception stopped: {error}')
                return
            if len(frame) == CAN_FRAME.size:
                can_id, length, data = CAN_FRAME.unpack(frame)
                if length <= len(data):
                    self.handle_frame(time.monotonic(), can_id, data[:length])

    def handle_frame(self, timestamp, can_id, data):
        feedback = decode_frame(can_id, data)
        if feedback is None:
            return
        if can_id == SYSTEM_STATUS:
            status, control_mode, voltage, fault_bits = feedback
            # Keep the wire location rather than assigning names from another Ranger variant.
            faults = [FaultBit(item=f'0x211 byte {7 - bit // 8} bit {bit % 8}', value=1, text='Active')
                      for bit in range(32) if fault_bits & (1 << bit)]
            self.status_pub.publish(SystemStatus(
                status=status, status_text=STATUS_NAMES.get(status, 'Unknown'),
                ctrl_mode=control_mode, ctrl_mode_text=CONTROL_NAMES.get(control_mode, 'Unknown'),
                vol=voltage, faults=faults,
            ))
        elif can_id == MOTION_MODE:
            mode, switching, drive_mode = feedback
            if self.mode_feedback != (mode, switching):
                self.estimate.timestamp = None
            self.mode_feedback = (mode, switching)
            self.mode_timestamp = timestamp
            self.mode_pub.publish(MotionModeFeedback(
                mode=mode, mode_text=MODE_NAMES.get(mode, 'Unknown'),
                switch_status=switching, switch_text=SWITCH_NAMES.get(switching, 'Unknown'),
                drive_mode=drive_mode, drive_mode_text=DRIVE_NAMES.get(drive_mode, 'Unknown'),
            ))
        elif can_id == STEERING_ANGLES:
            self.steering_pub.publish(SteeringAngles(
                steering_01=feedback[0],
                steering_02=feedback[1],
                steering_03=feedback[2],
                steering_04=feedback[3],
            ))
        elif can_id == WHEEL_SPEEDS:
            self.wheel_speeds_pub.publish(WheelSpeeds(
                wheel_01=feedback[0],
                wheel_02=feedback[1],
                wheel_03=feedback[2],
                wheel_04=feedback[3],
            ))
        elif can_id == CHASSIS_MOTION:
            self.motion_pub.publish(ChassisMotion(
                linear_speed=feedback[0], rotate_angular=feedback[1], steering_angle=feedback[2],
            ))
            self.update_odometry(timestamp, feedback)

    def update_odometry(self, timestamp, motion):
        """
        self.mode_timestamp is None：尚未收到 0x291，拒绝更新。
        0 <= timestamp - self.mode_timestamp：防止运动反馈早于模式反馈。
        timestamp - self.mode_timestamp <= feedback_timeout：模式反馈必须在当前运动反馈前 0.2 秒内
        """
        if self.mode_timestamp is None or not 0 <= timestamp - self.mode_timestamp <= self.feedback_timeout:
            self.estimate.timestamp = None
            return
        mode, switching = self.mode_feedback
        body_twist = body_velocity(mode, motion, self.wheelbase, self.track)
        # switching == 1 (底盘正处于模态切换过程中)时会立即暂停里程计更新
        if switching or body_twist is None:
            self.estimate.timestamp = None
            return

        self.estimate.update(timestamp, body_twist, self.feedback_timeout)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.estimate.x
        odom.pose.pose.position.y = self.estimate.y
        odom.pose.pose.orientation.z = math.sin(self.estimate.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.estimate.yaw / 2.0)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = body_twist
        self.odom_pub.publish(odom)

        if self.tf is not None:
            transform = TransformStamped(header=odom.header, child_frame_id=odom.child_frame_id)
            transform.transform.translation.x = self.estimate.x
            transform.transform.translation.y = self.estimate.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.close()
        if self.can_socket is not None:
            self.can_socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = RangerDeltaFeedback()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
