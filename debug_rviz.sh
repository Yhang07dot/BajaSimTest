#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"
exec ros2 run rviz2 rviz2 -d "$PWD/src/baja_perception_sim/config/debug.rviz" \
  --ros-args -r __node:=debug_rviz -p use_sim_time:=true
