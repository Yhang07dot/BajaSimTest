#!/usr/bin/env bash
# Full-loop checks with unchanged terrain at 2m/s and 3m/s.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"
python3 scripts/validate_live.py --algorithm baja_algorithm_template --speed 2 \
  --sim-seconds 90 --wall-timeout 300 --output generated/validation_speed2
python3 scripts/validate_live.py --algorithm baja_algorithm_template --speed 3 \
  --sim-seconds 60 --wall-timeout 240 --output generated/validation_speed3
