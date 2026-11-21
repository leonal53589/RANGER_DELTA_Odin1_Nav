#!/usr/bin/env bash
set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command_file=/tmp/odin_command.txt
launch_pid=

fail() {
    printf '采图失败：%s\n' "$*" >&2
    if [[ -n ${session_dir:-} ]]; then
        printf '采集目录和日志保留在 %s\n' "$session_dir" >&2
    fi
    exit 1
}

cleanup() {
    if [[ -z $launch_pid ]]; then
        return
    fi

    # ROS launch forwards SIGINT and gives the driver time to shut down.
    kill -INT "$launch_pid" 2>/dev/null || true
    for (( attempt = 0; attempt < 50; attempt++ )); do
        kill -0 "$launch_pid" 2>/dev/null || break
        sleep 0.2
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for (( attempt = 0; attempt < 10; attempt++ )); do
        kill -0 -- "-$launch_pid" 2>/dev/null || break
        sleep 0.2
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}

capture_process_exited() {
    ! kill -0 "$launch_pid" 2>/dev/null ||
        grep -Eq '\[(host_sdk_sample|odin_slam_node)-[0-9]+\]: process (has finished|has died)' "$launch_log"
}

wait_for_startup_log() {
    local message=$1
    local deadline=$((SECONDS + 45))
    until grep -Fq -- "$message" "$launch_log"; do
        if capture_process_exited; then
            fail '驱动或采图窗口在启动时退出。'
        fi
        (( SECONDS < deadline )) || fail "ODIN1 未就绪，日志中未找到 $message。"
        sleep 1
    done
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    printf '用法：%s\n启动 ODIN1、采图窗口和 RViz。停车后在采图窗口保存，脚本自动下载设备地图。\n' "$0"
    exit 0
fi
[[ $# -eq 0 ]] || fail '此脚本无需参数。使用 --help 查看用法。'
[[ -f $project_dir/install/setup.bash ]] || fail '找不到 install/setup.bash，请先构建工作空间。'

mkdir -p -- "$project_dir/maps"
exec 9< "$project_dir/maps"
flock -n 9 || fail '已有一个采图脚本在运行。'
if pgrep -x host_sdk_sample >/dev/null; then
    fail '已有 ODIN1 驱动在运行。请先在它的终端停止，再运行采图脚本。'
fi
[[ ! -e $command_file && ! -L $command_file ]] || fail "存在待处理的 ODIN1 指令 $command_file。"

source "$project_dir/install/setup.bash"
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

session_dir=$(mktemp -d "$project_dir/maps/odin_$(date +%Y%m%d_%H%M%S)_XXXXXX")
driver_config="$session_dir/odin_mapping.yaml"
launch_log="$session_dir/capture.log"
pcd_file="$session_dir/1.pcd"
bin_map="$session_dir/map.bin"

python3 - "$project_dir/src/odin_ros_driver/config/control_command_robot.yaml" "$driver_config" "$session_dir" <<'PY'
from pathlib import Path
import sys

import yaml

source, target, session_dir = sys.argv[1:]
config = yaml.safe_load(Path(source).read_text(encoding='utf-8'))
config['register_keys'].update(
    custom_map_mode=1,
    send_odom_baselink_tf=1,
    mapping_result_dest_dir=session_dir,
    mapping_result_file_name='map.bin',
)
Path(target).write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
PY

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf '采集目录 %s\n正在启动 ODIN1、采图窗口和 RViz。\n' "$session_dir"
setsid ros2 launch odin_slam slam_bringup.launch.py \
    config_file:="$driver_config" > "$launch_log" 2>&1 9>&- &
launch_pid=$!

wait_for_startup_log 'Command interface ready'
grep -Fq 'Custom map mode: 1' "$launch_log" || fail '驱动未进入建图模式 1。'

deadline=$((SECONDS + 15))
until param_result=$(timeout 4s ros2 param set /odin_slam_node save_pcd_path "$session_dir" 2>&1) &&
    [[ $param_result == *'Set parameter successful'* ]]; do
    if capture_process_exited || (( SECONDS >= deadline )); then
        fail "无法设置 PCD 输出目录。$param_result"
    fi
    sleep 1
done
cloud_header=$(timeout 15s ros2 topic echo /odin1/cloud_slam sensor_msgs/msg/PointCloud2 \
    --once --qos-profile sensor_data --field header) || fail '未收到 /odin1/cloud_slam 点云。'
grep -Fxq 'frame_id: odom' <<< "$cloud_header" || fail 'SLAM 点云的帧名不是 odom。'

printf '\n采图就绪。停车时点击 Qt 窗口的「运行 SLAM」，用原厂遥控器低速绕行。\n'
printf '回到起点停稳后，点击「暂停 SLAM」再点「保存结果」。\n'
printf 'RViz 的 Fixed Frame 用 odom。可按 Ctrl+C 结束采集。\n'

until grep -Fq -- "Successfully saved filtered point cloud to: $pcd_file" "$launch_log"; do
    if capture_process_exited; then
        fail '驱动或采图窗口已退出，尚未保存 PCD。'
    fi
    sleep 1
done
[[ -s $pcd_file ]] || fail 'PCD 文件为空。'

[[ ! -e $command_file && ! -L $command_file ]] || fail "存在待处理的 ODIN1 指令 $command_file。"
printf 'PCD 已保存。正在下载 ODIN1 重定位地图，请保持雷达供电。\n'
bash "$project_dir/src/odin_ros_driver/set_param.sh" save_map 1 >> "$launch_log"

deadline=$((SECONDS + 180))
until grep -Fq -- "Map transfer completed: $bin_map" "$launch_log"; do
    kill -0 "$launch_pid" 2>/dev/null || fail '驱动已退出，未完成设备地图下载。'
    if grep -Eq 'Map transfer (failed|skipped)|Device not connected!' "$launch_log"; then
        fail 'ODIN1 未能下载设备地图。PCD 已保存，请查看 capture.log。'
    fi
    (( SECONDS < deadline )) || fail '等待设备地图下载超时。PCD 已保存，请查看 capture.log。'
    sleep 1
done
[[ -s $bin_map ]] || fail '设备地图文件为空。'

printf '\n采图完成。\n点云地图 %s\n重定位地图 %s\n' "$pcd_file" "$bin_map"
