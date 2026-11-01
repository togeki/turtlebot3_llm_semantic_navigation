#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
exec ros2 launch tb3_llm_controller demo_sim.launch.py "$@"
