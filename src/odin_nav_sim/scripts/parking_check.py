#!/usr/bin/env python3
"""Read-only ODIN1 station reference capture and stopped arrival verification."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
from statistics import median
import tempfile
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, qos_profile_action_status_default,
                       qos_profile_sensor_data)
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_ros import Buffer, TransformException, TransformListener

from parking_geometry import map_fingerprint, pose_xy_yaw, stamp_ns
from parking_keyframe import ParkingKeyframe, base_cloud, relative_pose

DECISION_FRAME_COUNT = 3
DECISION_MAX_AGE_NS = 2_000_000_000


def decide_lateral_correction(records, now_ns, after_ns):
    """Evaluate the experimental left correction from a fresh stopped batch."""
    if any(not record['accepted'] for record in records):
        failed = next(record for record in records if not record['accepted'])
        return dict(decision='abort', reason=failed['reason'])
    stamps = [record['cloud_stamp_ns'] for record in records]
    if len(stamps) != DECISION_FRAME_COUNT or stamps != sorted(set(stamps)):
        return dict(decision='abort', reason='need_three_distinct_frames')
    ages = [now_ns - stamp for stamp in stamps]
    if stamps[0] < after_ns or any(not 0 <= age <= DECISION_MAX_AGE_NS for age in ages):
        return dict(decision='abort', reason='stale_decision_batch', frame_age_ns=ages)
    x, y, yaw = map(median, zip(*(record['station_offset_m_rad'] for record in records)))
    ys = [record['station_offset_m_rad'][1] for record in records]
    spread = max(ys) - min(ys)
    metrics = dict(station_offset_m_rad=[x, y, yaw], y_range_m=spread, frame_age_ns=ages)
    if spread >= 0.02:
        return dict(metrics, decision='abort', reason='unstable_lateral_offset')
    if y > -0.10:
        return dict(metrics, decision='skip', reason='right_offset_below_threshold')
    if math.cos(yaw) <= 0:
        return dict(metrics, decision='abort', reason='left_step_cannot_reduce_right_offset')
    # Project the requested change in station Y onto the current body-left axis.
    step = min(0.05, -y * math.cos(yaw))
    return dict(metrics, decision='propose', reason='right_offset_confirmed', body_left_step_m=step)


def save_reference(path, points, nav, map_id, stamp):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent) as temporary:
        np.savez_compressed(temporary, points=points, map_pose=nav, map_id=map_id, stamp_ns=stamp)
        temporary.flush()
        os.link(temporary.name, path)  # Atomically create; never replace a measured station.


class StationObserver(Node):
    def __init__(self, mode, path):
        super().__init__('parking_check')
        self.mode = mode
        self.path = path
        self.matcher = None
        if mode == 'capture':
            if path.exists():
                raise FileExistsError(f'Station reference already exists: {path}')
        else:
            with np.load(path, allow_pickle=False) as ref:
                self.ref_pose = ref['map_pose']
                self.ref_map = str(ref['map_id'])
                self.matcher = ParkingKeyframe(ref['points'])
        self.report_path = path.with_name(path.name + '.jsonl')
        self.map_id = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.clouds = deque(maxlen=5)
        self.still_since = None
        self.odom_ns = 0
        self.active = set()
        self.pending = None
        self.endpoint = None
        self.decision_frames = []
        self.decision_fits = deque()
        self.decision_worker = (
            ThreadPoolExecutor(max_workers=DECISION_FRAME_COUNT) if mode == 'decide' else None)
        self.done = False
        self.ok = False
        self.reason = 'waiting_for_map_cloud_tf_and_stationary_odom'
        self.create_subscription(PointCloud2, '/odin1/cloud_raw', self.clouds.append, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if mode in ('watch', 'decide'):
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                     self.status, qos_profile_action_status_default)
        self.create_subscription(PoseStamped, '/controller_server/tracking_endpoint', self.track, 5)
        if mode != 'watch':
            self.arm(None, 0)
        self.create_timer(0.10, self.tick)

    def arm(self, goal_id, goal_stamp):
        self.pending = dict(goal_id=goal_id, goal_stamp_ns=goal_stamp,
                            after_ns=self.get_clock().now().nanoseconds)
        self.deadline = time.monotonic() + 15
        self.reason = 'waiting_for_map_cloud_tf_and_stationary_odom'

    def map_callback(self, msg):
        self.map_id = map_fingerprint(msg)

    def odom(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        v = msg.twist.twist
        if stamp <= self.odom_ns or stamp - self.odom_ns > 200_000_000:
            self.still_since = None
        self.odom_ns = stamp
        if (msg.child_frame_id == 'base_link' and math.hypot(v.linear.x, v.linear.y) <= 0.008
                and abs(v.angular.z) <= 0.016):
            if self.still_since is None:
                self.still_since = stamp
        else:
            self.still_since = None

    def track(self, msg):
        if msg.header.frame_id == 'map':
            self.endpoint = (self.get_clock().now().nanoseconds,
                             pose_xy_yaw(msg.pose.position, msg.pose.orientation))

    def status(self, msg):
        active = set()
        succeeded = []
        for state in msg.status_list:
            goal = bytes(state.goal_info.goal_id.uuid).hex()
            if state.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                GoalStatus.STATUS_CANCELING):
                active.add(goal)
            elif state.status == GoalStatus.STATUS_SUCCEEDED and goal in self.active:
                succeeded.append((goal, stamp_ns(state.goal_info.stamp)))
        self.active = active
        if active and self.pending:
            self.finish(dict(accepted=False, reason='new_goal_before_snapshot'))
        if self.mode == 'watch' and not active and succeeded:
            self.arm(*max(succeeded, key=lambda pair: pair[1]))

    def finish(self, result):
        if self.mode == 'decide':
            self.decision_frames.append(result)
            if result['accepted'] and len(self.decision_frames) < DECISION_FRAME_COUNT:
                return
            decision = decide_lateral_correction(self.decision_frames, self.get_clock().now().nanoseconds,
                                                 max(self.pending['after_ns'], self.still_since or 0))
            row = dict(reference=str(self.path), **self.pending, **decision, frames=self.decision_frames)
            print(json.dumps(row, allow_nan=False), flush=True)
            self.pending = None
            self.done = True
            self.ok = decision['decision'] != 'abort'
            return
        row = dict(reference=str(self.path), **self.pending, **result)
        if 'station_offset_m_rad' in row:
            x, y, yaw = row['station_offset_m_rad']
            self.get_logger().info(f'工位偏差 x={x * 100:+.1f} cm, y={y * 100:+.1f} cm, '
                                   f'yaw={math.degrees(yaw):+.2f} deg')
        if self.mode == 'watch' and self.endpoint and self.endpoint[0] >= self.pending['goal_stamp_ns']:
            row['tracking_endpoint_map_m_rad'] = self.endpoint[1]
        line = json.dumps(row, allow_nan=False)
        if self.mode != 'capture':
            with self.report_path.open('a', encoding='utf-8') as out:
                out.write(line + '\n')
        self.get_logger().info(line)
        self.pending = None
        self.done = self.mode != 'watch'
        self.ok = result['accepted']

    def match_cloud(self, cloud, mount, record, guess):
        data = read_points(cloud, field_names=('x', 'y', 'z'))
        xyz = np.column_stack([data[k] for k in ('x', 'y', 'z')])
        q, p = mount.rotation, mount.translation
        points = base_cloud(xyz, Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                            [p.x, p.y, p.z])
        if self.matcher:
            return dict(record, **self.matcher.check(points, guess))
        return points

    def destroy_node(self):
        if self.decision_worker:
            self.decision_worker.shutdown(cancel_futures=True)
        return super().destroy_node()

    def tick(self):
        if self.pending is None:
            return
        if time.monotonic() > self.deadline:
            diagnostics = {}
            if self.mode == 'decide':
                now = self.get_clock().now().nanoseconds
                cloud_ns = stamp_ns(self.clouds[-1].header.stamp) if self.clouds else None
                diagnostics = dict(odom_age_ns=now - self.odom_ns if self.odom_ns else None,
                                   still_since_ns=self.still_since, last_cloud_stamp_ns=cloud_ns,
                                   last_cloud_age_ns=now - cloud_ns if cloud_ns is not None else None)
            self.finish(dict(accepted=False, reason=self.reason, **diagnostics))
            return
        if self.map_id is None:
            return
        if self.matcher and self.ref_map != self.map_id:
            self.finish(dict(accepted=False, reason='different_map'))
            return
        now = self.get_clock().now().nanoseconds
        if self.still_since is None or not 0 <= now - self.odom_ns <= 300_000_000:
            if self.mode == 'decide':
                self.reason = 'waiting_for_fresh_stationary_odom'
            return
        if self.decision_frames and self.decision_frames[0]['cloud_stamp_ns'] < self.still_since:
            self.decision_frames.clear()
        while self.decision_fits and self.decision_fits[0][1].done():
            stamp, fit = self.decision_fits.popleft()
            if stamp - self.still_since < 1_000_000_000:
                continue
            try:
                result = fit.result()
            except (AssertionError, ValueError) as exc:
                result = dict(accepted=False, reason=str(exc))
            self.finish(result)
            if self.done:
                return
        if len(self.decision_frames) + len(self.decision_fits) == DECISION_FRAME_COUNT:
            return
        last_stamp = self.decision_frames[-1]['cloud_stamp_ns'] if self.decision_frames else 0
        if self.decision_fits:
            last_stamp = self.decision_fits[-1][0]
        if self.mode == 'decide':
            self.reason = 'waiting_for_new_cloud_during_stationary_odom'
        for cloud in reversed(self.clouds):
            stamp = stamp_ns(cloud.header.stamp)
            if (cloud.header.frame_id != 'lidar' or not 0 <= now - stamp <= 450_000_000
                    or stamp < self.pending['after_ns'] or stamp <= last_stamp or stamp > self.odom_ns
                    or stamp - self.still_since < 1_000_000_000):
                continue
            at = Time.from_msg(cloud.header.stamp)
            try:
                base = self.tf.lookup_transform('map', 'base_link', at).transform
                mount = self.tf.lookup_transform('base_link', 'lidar', at).transform
            except TransformException:
                self.reason = 'no_tf_at_cloud_stamp'
                continue
            nav = pose_xy_yaw(base.translation, base.rotation)
            record = dict(cloud_stamp_ns=stamp, nav_map_m_rad=nav)
            guess = None
            if self.matcher:
                guess = relative_pose(self.ref_pose, nav)
                record.update(station_goal_map_m_rad=self.ref_pose.tolist(),
                              nav_offset_m_rad=guess.tolist())
                if np.linalg.norm(guess[:2]) > 0.50 or abs(guess[2]) > math.radians(30):
                    self.finish(dict(record, accepted=False, reason='outside_station_view'))
                    return
            if self.decision_worker:
                # Register the three frames in parallel while still consuming wheel feedback.
                fit = self.decision_worker.submit(self.match_cloud, cloud, mount, record, guess)
                self.decision_fits.append((stamp, fit))
                return
            try:
                result = self.match_cloud(cloud, mount, record, guess)
            except (AssertionError, ValueError) as exc:
                self.reason = str(exc)
                continue
            if self.matcher:
                self.finish(result)
            else:
                points = result
                quality = ParkingKeyframe(points).check(points, np.zeros(3))
                if not quality['accepted']:
                    self.finish(dict(record, **quality))
                    return
                save_reference(self.path, points, nav, self.map_id, stamp)
                self.finish(dict(record, accepted=True, reason='reference_saved'))
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['capture', 'check', 'watch', 'decide'])
    parser.add_argument('reference', type=Path, help='station .npz path')
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    observer = None
    try:
        observer = StationObserver(args.mode, args.reference.expanduser().resolve())
        while rclpy.ok() and not observer.done:
            rclpy.spin_once(observer, timeout_sec=0.5)
        return 0 if observer.ok or args.mode == 'watch' else 1
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'parking_check: {exc}\n')
    except KeyboardInterrupt:
        return 0
    finally:
        if observer is not None:
            observer.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
