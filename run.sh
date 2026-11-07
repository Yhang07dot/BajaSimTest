#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ ! -f install/setup.bash ]]; then
  echo 'Run ./build.sh first.' >&2
  exit 1
fi
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"
export GZ_PARTITION="${GZ_PARTITION:-baja_perception}"
mkdir -p "$ROS_LOG_DIR"
launch_args=("run_dir:=$PWD/generated/current")
while (($#)); do
  case "$1" in
    --headless) launch_args+=(headless:=true rviz:=false);shift;;
    --no-rviz) launch_args+=(rviz:=false);shift;;
    --example) launch_args+=(example:=true);shift;;
    --seed|--obstacles|--config|--run-dir|--speed|--algorithm|--rviz-mode)
      key="${1#--}";key="${key//-/_}"
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2;exit 2; }
      launch_args+=("$key:=$2");shift 2;;
    --help|-h)
      echo './run.sh [--headless] [--no-rviz] [--rviz-mode debug|evaluation|both] [--algorithm PACKAGE | --example] [--speed M/S] [--seed N] [--obstacles N] [--config FILE] [--run-dir DIR]'
      exit 0;;
    *) echo "Unknown argument: $1" >&2;exit 2;;
  esac
done
echo "评分查看：另开终端执行 ./score.sh；自定义 --run-dir 时请同步传给评分查看器。"
exec ros2 launch baja_perception_sim simulation.launch.py "${launch_args[@]}"
