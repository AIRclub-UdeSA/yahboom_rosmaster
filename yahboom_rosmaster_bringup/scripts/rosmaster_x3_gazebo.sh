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
# Without this, a lingering Gazebo instance can cause the new launch to find
# an already-populated world (the old robot), or DDS ghost nodes can make the
# create-entity node pick up a cached robot_description from the old RSP.
echo "Killing any stale simulation processes..."
pkill -KILL -f "ruby.*ign|gz sim|gz_ros2_control" 2>/dev/null || true
sleep 1

trap 'cleanup; exit' SIGINT SIGTERM

echo "Launching Gazebo Fortress simulation..."
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
    use_sim_time:=true &

# Keep the script running until Ctrl+C
wait
