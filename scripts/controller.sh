#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source ros2_ws/install/setup.bash
if [[ -z "${GEMINI_API_KEY:-}" && -z "${GOOGLE_API_KEY:-}" ]]; then
  if [[ -s "$HOME/.config/pai/gemini_api_key" ]]; then
    GEMINI_API_KEY="$(cat "$HOME/.config/pai/gemini_api_key")"
  else
    read -rsp 'Gemini API key: ' GEMINI_API_KEY
    echo
  fi
  export GEMINI_API_KEY
fi
exec ros2 run tb3_llm_controller llm_controller --ros-args -p use_sim_time:=true "$@"
