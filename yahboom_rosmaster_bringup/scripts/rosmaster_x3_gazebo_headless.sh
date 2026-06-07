#!/bin/bash
# Launch ROSMASTER X3 in headless Gazebo Fortress.
# Requires: xvfb-run (sudo apt-get install -y xvfb)
set -euo pipefail

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 was not found. Source your ROS 2 environment first." >&2
    exit 1
fi

if ! ros2 pkg prefix yahboom_rosmaster_gazebo >/dev/null 2>&1; then
    echo "yahboom_rosmaster_gazebo was not found." >&2
    echo "Build the workspace and source install/setup.bash before running this script." >&2
    exit 1
fi

if ! command -v xvfb-run >/dev/null 2>&1; then
    echo "xvfb-run was not found. Install it with: sudo apt-get install -y xvfb" >&2
    exit 1
fi

cleanup() {
    echo "Cleaning up..."
    pkill -TERM -f "ros2|ruby|gz|rviz2|robot_state_publisher" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "ros2|ruby|gz|rviz2|robot_state_publisher" 2>/dev/null || true
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
