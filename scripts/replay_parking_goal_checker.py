#!/usr/bin/env python3
"""Replay a parking bag using Nav2 Humble's goal transforms and a common-time check.

This reads a finalized SQLite bag and the parking trial's run artifacts.
It creates neither a ROS node nor any publishers.
"""

import argparse
from contextlib import closing
from copy import deepcopy
import csv
import json
import math
from pathlib import Path
import re
import sqlite3

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path as PathMsg
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_msgs.msg import TFMessage
from tf2_py import BufferCore, ExtrapolationException, TransformException


NS = 1_000_000_000
DYNAMIC_FRAMES = {('map', 'odom'), ('odom', 'base_link')}


def to_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def to_time(ns):
    return Time(nanoseconds=ns)


def planar(pose):
    q = pose.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    return pose.position.x, pose.position.y, yaw


def planar_error(robot, goal):
    x, y, yaw = planar(robot)
    gx, gy, gyaw = planar(goal)
    return math.hypot(gx-x, gy-y), abs(math.remainder(gyaw-yaw, 2*math.pi))


def check(buffer, endpoint, xy_tolerance, yaw_tolerance, transform_tolerance):
    base = buffer.lookup_transform_core('odom', 'base_link', Time())
    robot = PoseStamped()
    robot.header = base.header
    robot.pose.position.x = base.transform.translation.x
    robot.pose.position.y = base.transform.translation.y
    robot.pose.orientation = base.transform.rotation

    plan_lookup = 'plan_stamp'
    plan_lookup_error = ''
    try:
        old_tf = buffer.lookup_transform_core('odom', 'map', to_time(to_ns(endpoint.header.stamp)))
    except ExtrapolationException as error:
        old_tf = buffer.lookup_transform_core('odom', 'map', Time())
        lag = to_ns(endpoint.header.stamp) - to_ns(old_tf.header.stamp)
        if lag > transform_tolerance * NS:
            raise
        plan_lookup = 'latest_fallback'
        plan_lookup_error = str(error)

    used_goal = do_transform_pose_stamped(endpoint, old_tf)
    original_xy, original_yaw = planar_error(robot.pose, used_goal.pose)
    result = {
        'robot_stamp_ns': to_ns(robot.header.stamp),
        'plan_pose_stamp_ns': to_ns(endpoint.header.stamp),
        'original_tf_stamp_ns': to_ns(old_tf.header.stamp),
        'original_lookup': plan_lookup,
        'original_lookup_error': plan_lookup_error,
        'robot_odom': planar(robot.pose),
        'original_goal_odom': planar(used_goal.pose),
        'original_xy_cm': original_xy * 100,
        'original_yaw_deg': math.degrees(original_yaw),
        'original_reached': original_xy <= xy_tolerance and original_yaw < yaw_tolerance,
    }
    try:
        same_tf = buffer.lookup_transform_core('odom', 'map', to_time(to_ns(robot.header.stamp)))
    except TransformException as error:
        # An unavailable transform is a separate result, not a pose at a different time.
        return dict(result, aligned_reached=None, aligned_lookup_error=str(error))
    aligned_goal = do_transform_pose_stamped(endpoint, same_tf)
    aligned_xy, aligned_yaw = planar_error(robot.pose, aligned_goal.pose)
    return dict(
        result,
        aligned_tf_stamp_ns=to_ns(same_tf.header.stamp),
        aligned_goal_odom=planar(aligned_goal.pose),
        aligned_xy_cm=aligned_xy * 100,
        aligned_yaw_deg=math.degrees(aligned_yaw),
        aligned_reached=aligned_xy <= xy_tolerance and aligned_yaw < yaw_tolerance,
    )


def topic_messages(db, name, begin, end, message_type):
    sql = '''SELECT timestamp, data FROM messages
        WHERE topic_id = (SELECT id FROM topics WHERE name = ?)
          AND timestamp BETWEEN ? AND ? ORDER BY timestamp, id'''
    return [(ts, deserialize_message(data, message_type))
            for ts, data in db.execute(sql, (name, begin, end))]


def launch_goal_windows(launch_log):
    events = re.findall(
        r'\[(\d+)\.(\d{9})\] \[controller_server\]: (Received a goal|Reached the goal!)',
        launch_log,
    )
    opened = None
    for sec, nsec, event in events:
        stamp = int(sec)*NS + int(nsec)
        if event == 'Received a goal':
            opened = stamp
        elif opened is not None:
            yield opened, stamp
            opened = None


def replay_window(db, begin, end, tolerances):
    plans = topic_messages(db, '/plan', begin-NS, end, PathMsg)
    # This dataset has no path replacement during the control window. Do not infer a
    # FollowPath input for a dataset with multiple plans, since actions weren't recorded.
    if len(plans) != 1:
        raise ValueError(f'Expected one published plan near controller activation, found {len(plans)}')
    path = plans[0][1]
    endpoint = deepcopy(path.poses[-1])
    endpoint.header.frame_id = path.header.frame_id
    transitions = topic_messages(db, '/tf', begin-2*NS, end+100_000_000, TFMessage)
    # The recorder's arrival time brackets each observed controller tick. The
    # success log is a separate observation with no assumed subscriber ordering.
    sampled_ticks = [t for t, _ in topic_messages(db, '/cmd_vel_nav', begin, end, Twist)]
    query_times = sorted(set(sampled_ticks + [end-50_000_000, end, end+50_000_000]))
    buffer = BufferCore(Duration(seconds=10))
    cursor = 0
    for when in query_times:
        while cursor < len(transitions) and transitions[cursor][0] <= when:
            for transform in transitions[cursor][1].transforms:
                if (transform.header.frame_id, transform.child_frame_id) in DYNAMIC_FRAMES:
                    buffer.set_transform(transform, 'bag')
            cursor += 1
        try:
            result = check(buffer, endpoint, *tolerances)
        except TransformException as error:
            result = {'missing_original_tf': str(error)}
        yield dict(
            result, query_ns=when, from_reached_log_ms=(when-end)/1_000_000,
            is_reached_log=when == end,
        )


def synthetic():
    def edge(parent, child, seconds, x):
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.header.stamp = to_time(seconds*NS).to_msg()
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = x
        return transform

    odom_edges = [
        edge('odom', 'base_link', 11, 0.0),
        edge('odom', 'base_link', 12, 0.02),
    ]
    endpoint = PoseStamped()
    endpoint.header.frame_id = 'map'
    endpoint.header.stamp = to_time(11*NS).to_msg()
    endpoint.pose.position.x = 0.075
    endpoint.pose.orientation.w = 1.0

    def evaluate(drift, latest_time=12):
        buffer = BufferCore(Duration(seconds=10))
        for transform in odom_edges + [edge('map', 'odom', 11, 0.0),
                                       edge('map', 'odom', latest_time, drift)]:
            buffer.set_transform(transform, 'synthetic')
        return check(buffer, endpoint, 0.06, 0.10, 0.30)

    stable = evaluate(0.0)
    shifted = evaluate(-0.017)
    unavailable = evaluate(0.0, latest_time=11)
    assert stable['original_reached'] and stable['aligned_reached']
    assert shifted['original_reached'] and not shifted['aligned_reached']
    assert math.isclose(shifted['original_xy_cm'], 5.5, abs_tol=1e-9)
    assert math.isclose(shifted['aligned_xy_cm'], 7.2, abs_tol=1e-9)
    assert unavailable['original_reached'] and unavailable['aligned_reached'] is None
    print('Synthetic TF2 cases passed: stable, 1.7 cm map correction, and unavailable common time.')


def run_replay(root, output):
    params = json.loads((root/'controller_params.json').read_text())
    if params['goal_checker.stateful']:
        raise ValueError('This replay expects the recorded stateless SimpleGoalChecker')
    tolerances = (params['goal_checker.xy_goal_tolerance'], params['goal_checker.yaw_goal_tolerance'], 0.30)
    trials = sorted(
        ((json.loads(path.read_text())['stamp'], path.name.removesuffix('_stopped.json'))
         for path in root.glob('*_stopped.json')),
    )
    windows = list(launch_goal_windows((root/'active'/'launch.log').read_text()))
    if len(windows) != len(trials) or not windows:
        raise ValueError('Expected one successful controller window per recorded parking run')

    traces, summary = [], []
    with closing(sqlite3.connect((root/'active'/'bag'/'bag_0.db3').resolve().as_uri()+'?mode=ro', uri=True)) as db:
        integrity = db.execute('PRAGMA quick_check').fetchone()[0]
        if integrity != 'ok':
            raise ValueError(f'Bag integrity check failed: {integrity}')
        for (dispatch, name), (begin,end) in zip(trials, windows):
            if not 0 <= begin/NS - dispatch <= 1:
                raise ValueError(f'Trial {name} does not match the controller window')
            rows = [dict(row, trial=name) for row in replay_window(db, begin, end, tolerances)]
            observed = next(row for row in rows if row['is_reached_log'])
            observed['original_only_ticks'] = sum(
                row.get('original_reached') is True and row.get('aligned_reached') is False
                for row in rows if row['from_reached_log_ms'] <= 0
            )
            summary.append(observed)
            traces.extend(rows)

    output.mkdir(parents=True, exist_ok=True)
    columns = list(dict.fromkeys(key for row in traces for key in row))
    with (output/'checker_cycles.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(traces)
    (output/'reached_log_comparison.json').write_text(json.dumps(summary, indent=2)+'\n')
    for row in summary:
        print(row['trial'],
              row.get('original_lookup', 'unavailable'),
              f"original={row.get('original_xy_cm', math.nan):.2f} cm",
              f"aligned={row.get('aligned_xy_cm', math.nan):.2f} cm",
              f"result={row.get('original_reached')}/{row.get('aligned_reached')}",
              f"original-only ticks={row['original_only_ticks']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path, nargs='?')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--synthetic', action='store_true')
    args = parser.parse_args()
    if args.synthetic:
        synthetic()
    if args.run_dir:
        run_replay(args.run_dir, args.output_dir or args.run_dir/'goal_checker_replay')
    elif not args.synthetic:
        parser.error('provide a parking run directory or --synthetic')


if __name__ == '__main__':
    main()
