# Sourced by pixi when activating the `classic` environment (Gazebo Classic on macOS).

# A Mac exposes ~20 multicast-capable interfaces (en0, awdl0, bridge0, utun0-5).
# DDS participants pick different ones and then never discover each other, so
# nodes see only part of the ROS graph and wait forever on services that are
# running. Gazebo Classic runs its own separate discovery with the same flaw.
# Pinning both to loopback keeps the graph whole.
# Export ROS_LOCALHOST_ONLY=0 after activation to reach a physical robot.
export ROS_LOCALHOST_ONLY=1
export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI="http://127.0.0.1:11345"

# FastRTPS, the Humble default, hits an allocator assertion on osx-arm64
# (RoboStack/ros-humble#32, wontfix) that takes down ros2 CLI tools and RViz.
# Nav2 bring-up is service and bond heavy, which is exactly the traffic that
# trips it.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Let Gazebo find the world models this repository vendors.
if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
  _models="${PIXI_PROJECT_ROOT}/install_classic/yahboom_rosmaster_gazebo/share/yahboom_rosmaster_gazebo/models"
  [ -d "$_models" ] && export GAZEBO_MODEL_PATH="${_models}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

  # Overlay the Classic colcon workspace once it has been built.
  if [ -f "${PIXI_PROJECT_ROOT}/install_classic/setup.sh" ]; then
    . "${PIXI_PROJECT_ROOT}/install_classic/setup.sh"
  fi
fi
