#!/usr/bin/env bash
set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
reference="$project_dir/local_experiments/stations/glim_home.npz"
usage='用法: go_to_glim_home.sh [--check]。直接运行发送返航目标，--check 只检查路线和终点净空。'

args=()
if (( $# )); then
    case "$1" in
        -h|--help) printf '%s\n' "$usage"; exit 0 ;;
        --check) args+=(--plan-only); shift ;;
    esac
    if (( $# )); then
        printf '%s\n' "$usage" >&2
        exit 2
    fi
fi

if [[ ! -r "$reference" ]]; then
    printf '找不到已保存的目标 %s\n' "$reference" >&2
    exit 1
fi

if [[ ! -r "$project_dir/install/setup.bash" ]]; then
    printf '请先构建工作区 %s\n' "$project_dir" >&2
    exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-49}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PYTHONNOUSERSITE=1
source "$project_dir/install/setup.bash"

exec python3 "$project_dir/src/odin_nav_sim/scripts/return_to_station.py" \
    "$reference" --mode baseline "${args[@]}"
