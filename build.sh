#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"
mkdir -p "$ROS_LOG_DIR"
colcon build --symlink-install --base-paths src --event-handlers console_direct+
