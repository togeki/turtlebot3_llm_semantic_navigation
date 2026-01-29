#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

echo "[1/3] Start Gazebo"
gnome-terminal -- bash -lc "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py; exec bash"

echo "[2/3] Start Nav2"
gnome-terminal -- bash -lc "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true; exec bash"

echo "[3/3] Start full stack (collector + bridges + nav2_client)"
gnome-terminal -- bash -lc "cd ~/turtlebot3_llm_semantic_navigation/ros2_ws; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 launch tb3_llm_controller full_stack.launch.py; exec bash"

echo "Now run llm_controller in THIS terminal:"
echo "  cd ~/turtlebot3_llm_semantic_navigation/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run tb3_llm_controller llm_controller"
