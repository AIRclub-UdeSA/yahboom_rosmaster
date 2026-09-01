#!/usr/bin/env bash
# Run the representative headless simulator contract gate.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# A normal ROS checkout lives at <workspace>/src/yahboom_rosmaster.  CI checks
# the repository out directly and uses the repository root as its workspace.
if [[ "$(basename "$(dirname "${REPO_DIR}")")" == "src" ]]; then
    WORKSPACE_DIR="$(cd "${REPO_DIR}/../.." && pwd)"
else
    WORKSPACE_DIR="${REPO_DIR}"
fi

CONTRACT_REGEX='^(robot_description_contract|motion_profile_contract|practice_world_probe_contract|launch_shutdown_contract|sensor_contract_empty|base_feedback|ground_truth_contract|motion_profile_(divergence_ideal|yaw_ideal)|wheel_odometry_resilience)$'

if [[ -r /opt/ros/humble/setup.bash ]]; then
    # Workflow steps start in a fresh shell even though ROS_DISTRO is exported.
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
elif [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "ROS 2 is not sourced and /opt/ros/humble/setup.bash is unavailable." >&2
    exit 2
fi

if [[ "${ROS_DISTRO}" != "humble" ]]; then
    echo "The simulator contract gate requires ROS 2 Humble (found '${ROS_DISTRO}')." >&2
    exit 2
fi

if [[ ! -r "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    echo "Missing ${WORKSPACE_DIR}/install/setup.bash; build the workspace first." >&2
    exit 2
fi

# shellcheck disable=SC1090
source "${WORKSPACE_DIR}/install/setup.bash"
cd "${WORKSPACE_DIR}"

echo "Running representative simulator contracts from ${WORKSPACE_DIR}"
echo "CTest selection: ${CONTRACT_REGEX}"

test_status=0
colcon test \
    --packages-select yahboom_rosmaster_description yahboom_rosmaster_gazebo \
    --executor sequential \
    --return-code-on-test-failure \
    --event-handlers console_direct+ \
    --ctest-args -R "${CONTRACT_REGEX}" --output-on-failure \
    || test_status=$?

result_status=0
colcon test-result --verbose --all || result_status=$?

if ((test_status != 0)); then
    exit "${test_status}"
fi
exit "${result_status}"
