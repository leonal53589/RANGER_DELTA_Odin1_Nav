#!/usr/bin/env bash
set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

# 基于 GLIM 点云参考与常驻双体素 4-DoF VGICP 的实车高精度导航启动脚本
glim_map_dir="$project_dir/maps/glim_20261127"
odin_map_dir="$project_dir/maps/odin_20261124_192624_a0FBCc"
sensor_x=0.1754
sensor_y=0.0025
sensor_z=1.1241
sensor_roll=-0.00085
sensor_pitch=0.01325
sensor_yaw=-0.02631
can_interface="${CAN_INTERFACE:-can0}"
enable_control="${ENABLE_CONTROL:-true}"
launch_rviz="${LAUNCH_RVIZ:-true}"
goal_profile="${GOAL_PROFILE:-precision}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-49}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export PYTHONNOUSERSITE=1

if (( $# != 0 )); then
    printf '此脚本直接运行，常用开关可通过环境变量覆盖（如 ENABLE_CONTROL=false）。\n' >&2
    exit 2
fi

# Match the executable (after an optional Python interpreter), rather than topic
# names and paths mentioned by rosbag or other tools in their arguments.
executable_prefix='^([^[:space:]]*/)?(python([0-9]+(\.[0-9]+)?)?[[:space:]]+([^[:space:]]*/)?)?'
navigation_process='ros2[[:space:]]+launch[[:space:]]+([^[:space:]]+[[:space:]]+)?([^[:space:]]*/)?real_navigation\.launch\.py'
if existing=$(pgrep -af "${executable_prefix}(host_sdk_sample|odin_localization|vgicp_localization|$navigation_process)([[:space:]]|$)"); then
    printf '检测到已有导航/雷达进程正在运行，请先停止后再启动以免抢占 USB 与 CAN：\n%s\n' "$existing" >&2
    exit 1
fi

if [[ ! -f "$project_dir/install/setup.bash" ]]; then
    printf '找不到 %s/install/setup.bash，请先构建工作空间。\n' "$project_dir" >&2
    exit 1
fi

cd -- "$project_dir"
source "$project_dir/install/setup.bash"

exec ros2 launch odin_nav_sim real_navigation.launch.py \
    map:="$glim_map_dir/site_nav.yaml" \
    localization_mode:=vgicp \
    vgicp_reference_pcd:="$glim_map_dir/glim_reference.pcd" \
    vgicp_localization_yaml:="$glim_map_dir/localization.yaml" \
    odin_relocalization_config:="$odin_map_dir/odin_relocalization.yaml" \
    sensor_x:="$sensor_x" sensor_y:="$sensor_y" sensor_z:="$sensor_z" \
    sensor_roll:="$sensor_roll" sensor_pitch:="$sensor_pitch" sensor_yaw:="$sensor_yaw" \
    can_interface:="$can_interface" enable_control:="$enable_control" launch_rviz:="$launch_rviz" \
    goal_profile:="$goal_profile" goal_time_ab:=false odin_spin_guard:=false
