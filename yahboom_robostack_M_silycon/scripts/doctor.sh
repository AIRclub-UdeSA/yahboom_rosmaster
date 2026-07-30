#!/usr/bin/env bash
# Check that the macOS simulation environment is set up correctly.
# Run through ./run doctor
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "${HERE}/.." && pwd)"
fail=0

ok()   { printf "  \033[32mok\033[0m    %s\n" "$1"; }
bad()  { printf "  \033[31mFAIL\033[0m  %s\n" "$1"; fail=1; }
warn() { printf "  \033[33mwarn\033[0m  %s\n" "$1"; }

echo "Platform"
[ "$(uname -s)" = "Darwin" ] && ok "macOS $(sw_vers -productVersion 2>/dev/null)" \
  || warn "not macOS - this folder targets Apple Silicon; use the Fortress backend instead"
[ "$(uname -m)" = "arm64" ] && ok "Apple Silicon (arm64)" || warn "not arm64 ($(uname -m))"

echo "Tools"
command -v pixi >/dev/null 2>&1 && ok "pixi $(pixi --version 2>/dev/null | awk '{print $2}')" \
  || bad "pixi missing - install with: brew install pixi"

echo "Environment"
if [ -d "${REPO}/.pixi/envs/classic" ]; then
  ok "classic environment installed"
else
  bad "classic environment missing - run: ./run setup"
fi

echo "Workspace build"
if [ -f "${REPO}/install_classic/setup.sh" ]; then
  ok "workspace built (install_classic/)"
else
  bad "workspace not built - run: ./run build"
fi

echo "Assets"
for f in launch/simulation.launch.py launch/slam_nav2.launch.py \
         worlds/classic_obstacles.world rviz/classic.rviz; do
  [ -f "${HERE}/${f}" ] && ok "$f" || bad "$f missing"
done

# The drift model lives in the shared gazebo package, not this folder.
bias="${REPO}/yahboom_rosmaster_gazebo/config/motion_bias.yaml"
[ -f "$bias" ] && ok "motion_bias.yaml" || bad "motion_bias.yaml missing"

# colcon --symlink-install links these into the install tree instead of copying,
# so a source file without the executable bit makes ros2 launch report the node
# as "not found on the libexec directory".
for s in cmd_vel_watchdog.py wheel_state_odometry.py; do
  script="${REPO}/yahboom_rosmaster_gazebo/scripts/${s}"
  if [ ! -f "$script" ]; then
    bad "${s} missing"
  elif [ -x "$script" ]; then
    ok "${s} is executable"
  else
    bad "${s} is not executable - run: chmod +x ${script}"
  fi
done

echo "Runtime checks"
if [ -d "${REPO}/.pixi/envs/classic" ]; then
  cd "$REPO"
  # Gazebo Classic plugins are .dylib on macOS, not the .so every tutorial shows.
  for lib in planar_move ray_sensor factory init; do
    if ls .pixi/envs/classic/lib/libgazebo_ros_${lib}.dylib >/dev/null 2>&1; then
      ok "libgazebo_ros_${lib}.dylib"
    else
      bad "libgazebo_ros_${lib}.dylib missing"
    fi
  done
fi

echo "Stray processes"
n=$(pgrep -f "gzserver|gzclient|rviz2|robot_state_publisher" 2>/dev/null | wc -l | tr -d ' ')
if [ "$n" = "0" ]; then ok "none running"; else warn "$n running - './run stop' clears them"; fi

echo
if [ "$fail" = "0" ]; then
  echo "All good. Start with:  ./run sim"
else
  echo "Some checks failed - follow the hints above."
  exit 1
fi
