#!/usr/bin/env python3
from collections import deque
import math

from agx_bringup.msg import MotionModeFeedback
import PyKDL
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from odin_reference_cloud import MapCorrectionCheck
from odin_spin_guard import ChassisMotion, SpinAlignmentGuard


def transform_frame(transform):
    q = transform.rotation
    p = transform.translation
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w), PyKDL.Vector(p.x, p.y, p.z))


def map_to_odom(map_from_odin_map, odin_map_from_lidar, base_from_lidar, odom_from_base):
    # Remove the 3D mount before projecting the ground-level base onto the 2D map.
    map_from_base = map_from_odin_map * odin_map_from_lidar * base_from_lidar.Inverse()
    planar_base = PyKDL.Frame(
        PyKDL.Rotation.RotZ(map_from_base.M.GetRPY()[2]),
        PyKDL.Vector(map_from_base.p.x(), map_from_base.p.y(), 0.0),
    )
    return planar_base * odom_from_base.Inverse()


class OdinLocalization(Node):
    def __init__(self):
        super().__init__('odin_localization')
        x = self.declare_parameter('odin_map_x', 0.0).value
        y = self.declare_parameter('odin_map_y', 0.0).value
        yaw = self.declare_parameter('odin_map_yaw', 0.0).value
        self.max_pose_age = self.declare_parameter('max_pose_age', 0.5).value
        if not all(math.isfinite(n) for n in (x, y, yaw, self.max_pose_age)) or self.max_pose_age <= 0:
            raise ValueError('Map alignment must be finite and max_pose_age must be positive')
        self.map_from_odin_map = PyKDL.Frame(PyKDL.Rotation.RotZ(yaw), PyKDL.Vector(x, y, 0.0))

        # The Odin driver and the URDF both use the name lidar, with different parents.
        self.odin_tf = Buffer()
        self.robot_tf = Buffer()
        self.robot_listener = TransformListener(self.robot_tf, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, '/odin1/tf', self.receive_odin_tf, 100)
        self.create_subscription(
            TFMessage, '/odin1/tf_static', self.receive_odin_tf_static,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        filename = self.declare_parameter('odin_reference_pcd', '').value
        self.map_check = MapCorrectionCheck(filename) if filename else None
        if self.map_check is not None:
            self.create_subscription(
                PointCloud2, '/odin1/cloud_raw', self.map_check.clouds.append, qos_profile_sensor_data,
            )
        spin_guard_enabled = self.declare_parameter(
            'odin_spin_guard', False, ParameterDescriptor(read_only=True),
        ).value
        self.spin_guard = SpinAlignmentGuard() if spin_guard_enabled else None
        if self.spin_guard is not None:
            self.wheel_velocities = deque(maxlen=64)
            self.motion_modes = deque(maxlen=64)
            self.create_subscription(Odometry, '/odom', self.receive_wheel_velocity, 30)
            self.create_subscription(MotionModeFeedback, '/motion_mode_feedback', self.receive_motion_mode, 30)
            self.get_logger().info('Odin spin alignment guard enabled')
        self.last_stamp = None
        self.create_timer(0.05, self.publish_alignment)

    def receive_odin_tf(self, message):
        for transform in message.transforms:
            self.odin_tf.set_transform(transform, 'odin_driver')

    def receive_odin_tf_static(self, message):
        for transform in message.transforms:
            self.odin_tf.set_transform_static(transform, 'odin_driver')

    def receive_wheel_velocity(self, message):
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            return
        velocity = message.twist.twist
        speed = math.hypot(velocity.linear.x, velocity.linear.y)
        yaw_rate = velocity.angular.z
        if math.isfinite(speed) and math.isfinite(yaw_rate):
            stamp = Time.from_msg(message.header.stamp).nanoseconds
            self.wheel_velocities.append((stamp, (speed, yaw_rate)))

    def receive_motion_mode(self, message):
        self.motion_modes.append((self.get_clock().now().nanoseconds, (message.mode, message.switch_status)))

    def chassis_motion_at(self, stamp):
        age_limit_ns = 200_000_000
        mode = next((value for time, value in reversed(self.motion_modes)
                     if 0 <= stamp - time <= age_limit_ns), None)
        velocity = next((value for time, value in reversed(self.wheel_velocities)
                         if 0 <= stamp - time <= age_limit_ns), None)
        return ChassisMotion(*mode, *velocity) if mode is not None and velocity is not None else None

    def checked_lidar_pose(self, stamp):
        local = transform_frame(self.odin_tf.lookup_transform('odin_odom', 'lidar', stamp).transform)
        candidate = transform_frame(self.odin_tf.lookup_transform('odin_map', 'odin_odom', stamp).transform)
        check = self.map_check
        if check.needs_scan(candidate, local, stamp):
            for cloud in reversed(check.clouds):
                cloud_stamp = Time.from_msg(cloud.header.stamp)
                if cloud.header.frame_id != 'lidar' or not 0 <= (stamp - cloud_stamp).nanoseconds <= 400_000_000:
                    continue
                if check.last_scan is not None and cloud_stamp <= check.last_scan:
                    break
                try:
                    cloud_pose = transform_frame(
                        self.odin_tf.lookup_transform('odin_odom', 'lidar', cloud_stamp).transform,
                    )
                    cloud_correction = transform_frame(
                        self.odin_tf.lookup_transform('odin_map', 'odin_odom', cloud_stamp).transform,
                    )
                except TransformException:
                    continue
                check.last_scan = cloud_stamp
                check.observe(cloud_correction, cloud_pose, cloud, cloud_stamp)
                break
        accepted = check.alignment(stamp)
        if accepted is None:
            self.get_logger().warning('No map correction supported by the reference point cloud', throttle_duration_sec=2)
            return None
        return accepted * local

    def publish_alignment(self):
        try:
            odin_pose = self.odin_tf.lookup_transform('odin_map', 'lidar', Time())
            wheel_pose = self.robot_tf.lookup_transform('odom', 'base_link', Time())
            stamp = min(Time.from_msg(odin_pose.header.stamp), Time.from_msg(wheel_pose.header.stamp))
            pose_age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if not 0 <= pose_age <= self.max_pose_age:
                self.get_logger().warning('Waiting for fresh Odin and wheel poses', throttle_duration_sec=3)
                return
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return

            if self.map_check is None:
                odin_pose = transform_frame(self.odin_tf.lookup_transform('odin_map', 'lidar', stamp).transform)
            else:
                odin_pose = self.checked_lidar_pose(stamp)
                if odin_pose is None:
                    return
            wheel_pose = self.robot_tf.lookup_transform('odom', 'base_link', stamp)
            mount = self.robot_tf.lookup_transform('base_link', 'lidar', stamp)
        except TransformException as error:
            self.get_logger().info(f'Waiting for localization transforms: {error}', throttle_duration_sec=3)
            return

        odom_from_base = transform_frame(wheel_pose.transform)
        alignment = map_to_odom(
            self.map_from_odin_map, odin_pose,
            transform_frame(mount.transform), odom_from_base,
        )
        if self.spin_guard is not None:
            previous_state = self.spin_guard.state
            alignment = self.spin_guard.update(
                stamp.nanoseconds, alignment, odom_from_base, self.chassis_motion_at(stamp.nanoseconds),
            )
            if self.spin_guard.state != previous_state:
                self.get_logger().info(f'Odin spin alignment guard: {self.spin_guard.state}')
        yaw = alignment.M.GetRPY()[2]
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        # Let Nav2 use newer wheel poses with this correction, until it expires.
        transform.header.stamp = (stamp + Duration(seconds=0.30)).to_msg()
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = alignment.p.x()
        transform.transform.translation.y = alignment.p.y()
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(transform)
        if self.last_stamp is None:
            self.get_logger().info('Publishing map -> odom from Odin1 relocalization and wheel odometry')
        self.last_stamp = stamp


def main():
    rclpy.init()
    node = OdinLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
