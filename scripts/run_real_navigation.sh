#!/usr/bin/env bash
set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

# 实车配置在这里修改。enable_control=false 可用于停车联调。
map_dir="$project_dir/maps/odin_20261124_192624_a0FBCc"
sensor_x=0.1754
sensor_y=0.0025
sensor_z=1.1241
sensor_roll=-0.00085
sensor_pitch=0.01325
sensor_yaw=-0.02631
can_interface=can0
enable_control=true
launch_rviz=true
goal_profile=navigation
goal_time_ab=false
odin_spin_guard=true
station_reference=""  # 用当前地图和外参重采工位 A 后，填入新参考帧的绝对路径。
parking_reference=""  # 可填用 parking_check capture 保存的工位 .npz 绝对路径
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1

if (( $# != 0 )); then
    printf '此脚本直接运行，配置请在脚本开头修改。\n' >&2
    exit 2
fi

if [[ ! -f "$project_dir/install/setup.bash" ]]; then
    printf '找不到 %s/install/setup.bash，请先构建工作空间。\n' "$project_dir" >&2
    exit 1
fi

cd -- "$project_dir"
source "$project_dir/install/setup.bash"

exec ros2 launch odin_nav_sim real_navigation.launch.py \
    map:="$map_dir/site_nav.yaml" \
    odin_relocalization_config:="$map_dir/odin_relocalization.yaml" \
    sensor_x:="$sensor_x" sensor_y:="$sensor_y" sensor_z:="$sensor_z" \
    sensor_roll:="$sensor_roll" sensor_pitch:="$sensor_pitch" sensor_yaw:="$sensor_yaw" \
    can_interface:="$can_interface" enable_control:="$enable_control" launch_rviz:="$launch_rviz" \
    goal_profile:="$goal_profile" goal_time_ab:="$goal_time_ab" odin_spin_guard:="$odin_spin_guard" \
    ${station_reference:+station_reference:="$station_reference"} \
    ${parking_reference:+parking_reference:="$parking_reference"}
