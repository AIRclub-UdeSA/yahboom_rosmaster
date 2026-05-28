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
# world; ghost DDS RSP nodes (alive for ~5 min after Ctrl+C) can deliver stale
# TRANSIENT_LOCAL robot_description to the create node → double robot spawn.
echo "Killing any stale simulation processes..."
pkill -KILL -f "ruby.*ign"          2>/dev/null || true   # Gazebo server/client
pkill -KILL -f "ign gazebo"         2>/dev/null || true   # alternate invocation
pkill -KILL -f "gz_ros2_control"    2>/dev/null || true   # hardware plugin
pkill -KILL -f "robot_state_publisher" 2>/dev/null || true # clears ghost DDS RSP
pkill -KILL -f "controller_manager" 2>/dev/null || true
sleep 3   # wait for DDS participant cleanup (fast enough to not block startup)

trap 'cleanup; exit' SIGINT SIGTERM

echo "Launching Gazebo Fortress simulation..."
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
    use_sim_time:=true &

# Keep the script running until Ctrl+C
wait
