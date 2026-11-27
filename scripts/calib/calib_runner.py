#!/usr/bin/env python3
"""Guarded data collection runner for base_link spin center & 6-DoF LiDAR extrinsic calibration."""

import argparse
import json
import math
import os
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

from agx_bringup.msg import MotionModeFeedback, SteeringAngles, SystemStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import String

BAG_TOPICS = [
    '/odin1/cloud_raw',
    '/odin1/imu',
    '/odin1/tf',
    '/tf',
    '/tf_static',
    '/odom',
    '/steering_angles',
    '/motion_mode_feedback',
    '/system_status',
    '/cmd_vel_nav',
    '/cmd_vel',
    '/cmd_vel_guarded',
    '/navigation_velocity_guard/state',
]


class CalibCollector(Node):
    def __init__(self):
        super().__init__('calib_collector')
        self.tf_buf = Buffer()
        self.tf_lst = TransformListener(self.tf_buf, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.odom = None
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.mode_fb = None
        self.sys_status = None
        self.guard_state = None
        self.steering = None

        self.recording_clouds = False
        self.clouds = []
        self.recording_imus = False
        self.imus = []

        self.create_subscription(Odometry, '/odom', self._odom_cb, 30)
        self.create_subscription(MotionModeFeedback, '/motion_mode_feedback', self._mode_cb, 10)
        self.create_subscription(SystemStatus, '/system_status', self._status_cb, 10)
        self.create_subscription(SteeringAngles, '/steering_angles', self._steer_cb, 10)
        self.create_subscription(String, '/navigation_velocity_guard/state', self._guard_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointCloud2, '/odin1/cloud_raw', self._cloud_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/odin1/imu', self._imu_cb, qos_profile_sensor_data)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)
        v = msg.twist.twist
        self.linear_vel = math.hypot(v.linear.x, v.linear.y)
        self.angular_vel = v.angular.z

    def _mode_cb(self, msg: MotionModeFeedback):
        self.mode_fb = (msg.mode, msg.switch_status)

    def _status_cb(self, msg: SystemStatus):
        self.sys_status = (msg.status, msg.ctrl_mode, any(b.value for b in msg.faults))

    def _steer_cb(self, msg: SteeringAngles):
        self.steering = [msg.steering_01, msg.steering_02, msg.steering_03, msg.steering_04]

    def _guard_cb(self, msg: String):
        self.guard_state = msg.data

    def _cloud_cb(self, msg: PointCloud2):
        if self.recording_clouds:
            self.clouds.append(msg)

    def _imu_cb(self, msg: Imu):
        if self.recording_imus:
            self.imus.append(msg)

    def wait_streams(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None and self.sys_status is not None and self.mode_fb is not None:
                return True
        return False

    def wait_stopped(self, timeout=15.0, stable_duration=1.5):
        t0 = time.monotonic()
        still_since = None
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.linear_vel <= 0.008 and abs(self.angular_vel) <= 0.016:
                if still_since is None:
                    still_since = time.monotonic()
                elif time.monotonic() - still_since >= stable_duration:
                    return True
            else:
                still_since = None
        return False

    def capture_stop(self, out_path: Path, n_clouds=10):
        if not self.wait_stopped(timeout=10.0, stable_duration=1.5):
            raise RuntimeError("Vehicle is not stationary!")
        self.clouds.clear()
        self.imus.clear()
        self.recording_clouds = True
        self.recording_imus = True
        t0 = time.time()
        while time.time() - t0 < 5.0 and len(self.clouds) < n_clouds:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.recording_clouds = False
        self.recording_imus = False

        if len(self.clouds) < 3:
            raise RuntimeError(f"Only captured {len(self.clouds)} clouds")

        # Extract valid XYZ for each frame & merged
        frames_xyz = []
        stamps_ns = []
        for c in self.clouds[:n_clouds]:
            f = read_points(c, field_names=('x', 'y', 'z'))
            xyz = np.column_stack([f[k] for k in ('x', 'y', 'z')]).astype(np.float32)
            mask = np.isfinite(xyz).all(axis=1) & (xyz != 0).any(axis=1)
            frames_xyz.append(xyz[mask])
            stamps_ns.append(c.header.stamp.sec * 10**9 + c.header.stamp.nanosec)

        imu_arr = np.array([
            [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
             m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
            for m in self.imus
        ], dtype=np.float64) if self.imus else np.zeros((0, 6))

        # Mean stationary cloud by voxelizing merged points at 5 mm for compact high-SNR reference
        merged = np.vstack(frames_xyz)
        nav_pose = None
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            p, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            nav_pose = [p.x, p.y, yaw]
        except Exception:
            pass

        np.savez_compressed(
            out_path,
            merged_xyz=merged,
            frame0_xyz=frames_xyz[0],
            stamps_ns=np.array(stamps_ns, dtype=np.int64),
            imu=imu_arr,
            odom=np.array(self.odom, dtype=np.float64),
            steering=np.array(self.steering or [0, 0, 0, 0], dtype=np.float64),
            nav_map_pose=np.array(nav_pose if nav_pose else [math.nan]*3, dtype=np.float64),
        )
        print(f"Captured stop -> {out_path} (frames={len(frames_xyz)}, pts={len(merged)}, imus={len(imu_arr)}, odom={self.odom})")


def start_bag(bag_path: Path):
    if bag_path.exists():
        raise FileExistsError(f"Bag already exists: {bag_path}")
    bag_path.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update({'ROS_DOMAIN_ID': '44', 'ROS_LOCALHOST_ONLY': '1', 'PYTHONNOUSERSITE': '1'})
    proc = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(bag_path)] + BAG_TOPICS,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env, preexec_fn=os.setsid,
    )
    t0 = time.time()
    while time.time() - t0 < 5.0:
        if bag_path.exists() and any(bag_path.iterdir()):
            break
        time.sleep(0.2)
    return proc


def stop_bag(proc, bag_path: Path):
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=8.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    db_files = list(bag_path.glob("*.db3"))
    if db_files:
        conn = sqlite3.connect(str(db_files[0]))
        cur = conn.cursor()
        cur.execute("PRAGMA quick_check")
        qc = cur.fetchone()[0]
        cur.execute("SELECT count(*) FROM messages")
        cnt = cur.fetchone()[0]
        conn.close()
        print(f"Bag closed: quick_check={qc}, messages={cnt}")


def run_motion(node: CalibCollector, mode: str, target_val: float, speed: float, timeout: float, bag_dir: Path | None):
    if not node.wait_stopped(timeout=10.0, stable_duration=1.5):
        raise RuntimeError("Vehicle not stationary before motion!")

    bag_proc = start_bag(bag_dir) if bag_dir else None
    sx, sy, syaw = node.odom
    accum_yaw = 0.0
    prev_yaw = syaw
    stale_since = None
    abort_reason = None

    twist = Twist()
    if mode == 'spin':
        twist.angular.z = math.copysign(abs(speed), target_val)
    elif mode == 'drive':
        twist.linear.x = math.copysign(abs(speed), target_val)
    else:
        raise ValueError(mode)

    stop_twist = Twist()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout:
            node.cmd_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)

            if node.sys_status and (node.sys_status[0] != 0 or node.sys_status[2]):
                abort_reason = f"chassis_fault:{node.sys_status}"
                break

            gstate = (node.guard_state or '').lower()
            if 'block' in gstate or 'reject' in gstate or 'obstacle' in gstate:
                if time.monotonic() - t0 > 0.3:
                    abort_reason = f"guard_blocked:{node.guard_state}"
                    break
            if 'missing' in gstate or 'stale' in gstate:
                if stale_since is None:
                    stale_since = time.monotonic()
                elif time.monotonic() - stale_since > 1.0:
                    abort_reason = f"guard_stale:{node.guard_state}"
                    break
            else:
                stale_since = None

            cx, cy, cyaw = node.odom
            dyaw_step = math.remainder(cyaw - prev_yaw, math.tau)
            accum_yaw += dyaw_step
            prev_yaw = cyaw
            dist = math.hypot(cx - sx, cy - sy)

            if mode == 'spin' and abs(accum_yaw) >= abs(target_val):
                print(f"Target spin angle reached: {math.degrees(accum_yaw):+.2f}°")
                break
            if mode == 'drive' and dist >= abs(target_val):
                print(f"Target straight distance reached: {dist:.3f} m")
                break
        else:
            abort_reason = f"timeout_{timeout}s"
    finally:
        t_stop = time.monotonic()
        while time.monotonic() - t_stop < 0.6:
            node.cmd_pub.publish(stop_twist)
            rclpy.spin_once(node, timeout_sec=0.05)
        stopped = node.wait_stopped(timeout=10.0, stable_duration=2.0)
        if bag_proc:
            stop_bag(bag_proc, bag_dir)

    cx, cy, cyaw = node.odom
    print(f"Motion done: mode={mode}, abort={abort_reason}, stopped={stopped}, "
          f"odom_delta=({cx-sx:+.4f}m, {cy-sy:+.4f}m, dyaw={math.degrees(accum_yaw):+.2f}°)")
    if abort_reason:
        sys.exit(2)


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_cap = sub.add_parser('capture')
    p_cap.add_argument('--out', required=True, type=Path)
    p_cap.add_argument('--frames', type=int, default=10)

    p_spin = sub.add_parser('spin')
    p_spin.add_argument('--angle-deg', required=True, type=float)
    p_spin.add_argument('--speed', type=float, default=0.16)
    p_spin.add_argument('--timeout', type=float, default=35.0)
    p_spin.add_argument('--bag', type=Path, default=None)

    p_drv = sub.add_parser('drive')
    p_drv.add_argument('--dist', required=True, type=float)
    p_drv.add_argument('--speed', type=float, default=0.15)
    p_drv.add_argument('--timeout', type=float, default=25.0)
    p_drv.add_argument('--bag', type=Path, default=None)

    args = parser.parse_args()
    rclpy.init()
    node = CalibCollector()
    if not node.wait_streams(timeout=5.0):
        print("ERROR: Sensor streams (/odom, /system_status, /motion_mode_feedback) not ready!")
        sys.exit(1)

    if args.cmd == 'capture':
        args.out.parent.mkdir(parents=True, exist_ok=True)
        node.capture_stop(args.out, n_clouds=args.frames)
    elif args.cmd == 'spin':
        run_motion(node, 'spin', math.radians(args.angle_deg), args.speed, args.timeout, args.bag)
    elif args.cmd == 'drive':
        run_motion(node, 'drive', args.dist, args.speed, args.timeout, args.bag)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
