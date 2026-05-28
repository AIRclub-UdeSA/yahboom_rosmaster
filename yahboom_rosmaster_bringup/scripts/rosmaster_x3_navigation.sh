#!/bin/bash
# Launch the ROSMASTER X3 with Gazebo Fortress, Nav2, and ROS 2 controllers.
set -euo pipefail

cleanup() {
    echo "Cleaning up..."
    pkill -TERM -f "ros2|gz|nav2|amcl|bt_navigator|rviz2|robot_state_publisher|joint_state_publisher" 2>/dev/null || true
    sleep 3
    pkill -KILL -f "ros2|gz|nav2|amcl|bt_navigator|rviz2|robot_state_publisher|joint_state_publisher" 2>/dev/null || true
}

trap 'cleanup; exit' SIGINT SIGTERM

# Pass "slam" as the first argument to enable SLAM mode
if [ "${1:-}" = "slam" ]; then
    SLAM_ARG="slam:=True"
else
    SLAM_ARG="slam:=False"
fi

echo "Launching Gazebo Fortress simulation with Nav2..."
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py \
    enable_odom_tf:=false \
    use_sim_time:=true \
    "$SLAM_ARG" &

echo "Waiting for simulation to initialize..."
sleep 25

echo "Adjusting Gazebo camera position..."
gz service -s /gui/move_to/pose \
    --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}" || true

wait
