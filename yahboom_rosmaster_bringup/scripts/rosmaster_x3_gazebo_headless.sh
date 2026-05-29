#!/bin/bash
# Launch ROSMASTER X3 in headless Gazebo Fortress (no GUI, for autonomous Claude debugging).
# Requires: xvfb-run (sudo apt-get install -y xvfb)
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/juan/Documents/rosmaster_ws/install/setup.bash

cleanup() {
    echo "Cleaning up..."
    pkill -TERM -f "ros2|ruby|gz|rviz2|robot_state_publisher|twist_to_stamped" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "ros2|ruby|gz|rviz2|robot_state_publisher|twist_to_stamped" 2>/dev/null || true
}

echo "Killing any stale simulation processes..."
pkill -KILL -f "ruby.*ign|ign gazebo|gz_ros2_control|controller_manager" 2>/dev/null || true
sleep 2

trap 'cleanup; exit' SIGINT SIGTERM

echo "Launching headless Gazebo Fortress (xvfb-run for Ogre2 sensor rendering)..."
xvfb-run -a -s "-screen 0 1280x720x24" \
    ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
    headless:=true rviz:=false use_sim_time:=true &

wait
