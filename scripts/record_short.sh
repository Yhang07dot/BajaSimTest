#!/usr/bin/env bash
# Optional point-cloud-only recording, 2 seconds by default; no raw image stream.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source install/setup.bash
duration="${1:-2}"
[[ "$duration" =~ ^[1-5]$ ]] || { echo 'Duration must be 1..5 seconds.' >&2;exit 2; }
mkdir -p recordings
export ROS_LOG_DIR="$PWD/log/ros"
set +e
timeout --signal=INT --kill-after=10 "${duration}s" ros2 bag record \
  -o "recordings/lidar_$(date +%Y%m%d_%H%M%S)" \
  /sensors/lidar/points /tf /tf_static
status=$?
[[ $status -eq 0 || $status -eq 124 || $status -eq 130 ]]
