#!/usr/bin/env python3
"""Send the exact captured goal to Nav2, optionally selecting station A fine alignment."""
import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory


def navigator_available():
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node('station_navigator_check')
    try:
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        return client.wait_for_server(timeout_sec=5.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def check_path(x, y, yaw):
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.srv import IsPathValid
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node('station_path_check')
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            print('找不到全局规划器，请检查导航是否启动以及 ROS 域是否一致。')
            return 1
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(yaw / 2)
        goal.goal.pose.orientation.w = math.cos(yaw / 2)
        response = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, response, timeout_sec=5.0)
        if not response.done() or not response.result().accepted:
            print('全局规划器未接受此次路径检查。')
            return 1
        result = response.result().get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=8.0)
        if not result.done():
            print('全局规划超时。')
            return 1
        reply = result.result()
        if reply.status != GoalStatus.STATUS_SUCCEEDED or not reply.result.path.poses:
            print(f'从当前位置到 ({x:.3f}, {y:.3f}) 未找到有效路径，请核对全局代价地图。')
            return 1
        validator = node.create_client(IsPathValid, '/is_path_valid')
        if not validator.wait_for_service(timeout_sec=2.0):
            print('全局规划已返回路径，但找不到 /is_path_valid，无法确认路线和终点净空。')
            return 1
        target = IsPathValid.Request()
        target.path.header = goal.goal.header
        target.path.poses = [goal.goal]
        route = IsPathValid.Request()
        route.path = reply.result.path
        for request, label in ((target, '保存的终点姿态'), (route, '规划路径')):
            validation = validator.call_async(request)
            rclpy.spin_until_future_complete(node, validation, timeout_sec=2.0)
            if not validation.done() or validation.result() is None:
                print(f'{label}的净空检查超时。')
                return 1
            if not validation.result().is_valid:
                print(f'全局规划已返回路径，但{label}未通过净空检查，请核对全局代价地图。')
                return 1
        end = reply.result.path.poses[-1].pose.position
        print(f'路线和保存的终点姿态通过净空检查，共 {len(reply.result.path.poses)} 个路径点，'
              f'路径末点 ({end.x:.3f}, {end.y:.3f})。')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('reference', type=Path)
parser.add_argument('--mode', choices=['return', 'baseline', 'align'], default='return',
                    help='return: approach and align; baseline: approach only; align: fine alignment only')
parser.add_argument('--plan-only', action='store_true',
                    help='check the global path to the saved pose without sending a navigation goal')
args = parser.parse_args()
with np.load(args.reference, allow_pickle=False) as ref:
    x, y, yaw = ref['map_pose'].tolist()
if args.plan_only:
    raise SystemExit(check_path(x, y, yaw))
goal = dict(pose=dict(header=dict(frame_id='map'), pose=dict(position=dict(x=x, y=y),
                 orientation=dict(z=math.sin(yaw/2), w=math.cos(yaw/2)))))
if args.mode != 'baseline':
    tree = 'return_to_station' if args.mode == 'return' else 'align_at_station'
    goal['behavior_tree'] = str(Path(get_package_share_directory('odin_nav_sim')) /
                                'behavior_trees' / (tree + '.xml'))
if not navigator_available():
    parser.exit(
        1, '5 秒内未发现 /navigate_to_pose。请确认 Nav2 已就绪，并核对两个终端的 '
        f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "<未设置>")}、'
        f'ROS_LOCALHOST_ONLY={os.environ.get("ROS_LOCALHOST_ONLY", "<未设置>")}。\n')
os.execvp('ros2', ['ros2', 'action', 'send_goal', '/navigate_to_pose',
                   'nav2_msgs/action/NavigateToPose', json.dumps(goal)])
