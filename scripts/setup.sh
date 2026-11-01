#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
python -m pip install 'google-genai==2.34.0'
cd ros2_ws
python -m colcon build --symlink-install --packages-select tb3_llm_controller
