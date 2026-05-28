#!/bin/bash
# Launch the ROSMASTER X3 in Gazebo Fortress with ROS 2 controllers.
set -euo pipefail

cleanup() {
    echo "Cleaning up..."
    # Give processes a chance to shut down cleanly before force-killing
    pkill -TERM -f "ros2|gz|rviz2|robot_state_publisher|joint_state_publisher" 2>/dev/null || true
    sleep 3
    pkill -KILL -f "ros2|gz|rviz2|robot_state_publisher|joint_state_publisher" 2>/dev/null || true
}

trap 'cleanup; exit' SIGINT SIGTERM

echo "Launching Gazebo Fortress simulation..."
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
    use_sim_time:=true &

# Keep the script running until Ctrl+C
wait
