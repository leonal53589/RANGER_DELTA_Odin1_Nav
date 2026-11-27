#!/usr/bin/env python3
"""Send the exact captured goal to Nav2, optionally selecting station A fine alignment."""
import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('reference', type=Path)
parser.add_argument('--mode', choices=['return', 'baseline', 'align'], default='return',
                    help='return: approach and align; baseline: approach only; align: fine alignment only')
args = parser.parse_args()
with np.load(args.reference, allow_pickle=False) as ref:
    x, y, yaw = ref['map_pose'].tolist()
goal = dict(pose=dict(header=dict(frame_id='map'), pose=dict(position=dict(x=x, y=y),
                 orientation=dict(z=math.sin(yaw/2), w=math.cos(yaw/2)))))
if args.mode != 'baseline':
    tree = 'return_to_station' if args.mode == 'return' else 'align_at_station'
    goal['behavior_tree'] = str(Path(get_package_share_directory('odin_nav_sim')) /
                                'behavior_trees' / (tree + '.xml'))
os.execvp('ros2', ['ros2', 'action', 'send_goal', '/navigate_to_pose',
                   'nav2_msgs/action/NavigateToPose', json.dumps(goal)])
