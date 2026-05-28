#!/bin/bash
# Launch the ROSMASTER X3 in Gazebo Fortress with ROS 2 controllers.
set -euo pipefail

cleanup() {
    echo "Cleaning up..."
    pkill -TERM -f "ros2|ruby|gz|rviz2|robot_state_publisher|joint_state_publisher|twist_to_stamped" 2>/dev/null || true
    sleep 3
    pkill -KILL -f "ros2|ruby|gz|rviz2|robot_state_publisher|joint_state_publisher|twist_to_stamped" 2>/dev/null || true
}

# Kill any stale Gazebo/ROS processes from a previous session before starting.
# A lingering Gazebo server means the new launch connects to an already-populated
# world. Note: ghost DDS RSP nodes are harmless — the spawn now uses -string (not
# -topic), so it never subscribes to robot_description via DDS.
echo "Killing any stale simulation processes..."
pkill -KILL -f "ruby.*ign"          2>/dev/null || true   # Gazebo server/client
pkill -KILL -f "ign gazebo"         2>/dev/null || true   # alternate invocation
pkill -KILL -f "gz_ros2_control"    2>/dev/null || true   # hardware plugin
pkill -KILL -f "controller_manager" 2>/dev/null || true
sleep 2   # short wait for Gazebo process cleanup

trap 'cleanup; exit' SIGINT SIGTERM

echo "Launching Gazebo Fortress simulation..."
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
    use_sim_time:=true &

# Keep the script running until Ctrl+C
wait
