#!/usr/bin/env bash
# Kill leftover Fortress simulator processes and clear Fast DDS shared memory.
#
# Run it before EVERY simulator launch on a Linux host. A stale
# robot_state_publisher or gazebo from an earlier run keeps publishing an old
# robot_description, and gz_ros2_control then reads it instead of the new one.
# macOS (pixi) uses scripts/stop_sim.sh instead.
#
#   bash scripts/clean_sim.sh [-n|--dry-run] [workspace_root ...]
#
# workspace_root  colcon workspace whose install/ processes to kill. Defaults to
#                 the workspace this script lives in. Pass every workspace a
#                 sim may have been launched from (e.g. a parallel worktree).
# -n, --dry-run   list what would be killed; change nothing.
#
# Exit status: 0 when nothing matching is left, 1 otherwise.
#
# Only anchored patterns are used: a bare `pkill -f ign` also matches Firefox
# (-signalPipe) and unattended-upgrades (--wait-for-signal). The gazebo server's
# command line is `ign gazebo -r -s ...` (older setups showed `/usr/bin/ign
# gazebo`), so the pattern accepts both and requires the word `gazebo`. The bracketed first
# character keeps a pattern from matching the command line that carries it. The
# script also never signals its own ancestors or any shell, so calling it from a
# shell whose command line mentions `ros2 launch` or `<ws>/install/` (or from a
# pipeline whose other half is a forked copy of that shell) cannot kill the caller.
#
# This kills every `ros2 launch` on the machine and removes the Fast DDS shared
# memory of every user-space DDS application, not just this simulator's.

set -u

dry_run=0
workspaces=()
for arg in "$@"; do
    case "${arg}" in
        -n|--dry-run) dry_run=1 ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown option: ${arg}" >&2; exit 2 ;;
        *) workspaces+=("${arg}") ;;
    esac
done

# <workspace>/src/yahboom_rosmaster/scripts/clean_sim.sh, or the repository root
# itself when it is checked out directly (CI).
if ((${#workspaces[@]} == 0)); then
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    repo_dir="$(cd "${script_dir}/.." && pwd)"
    if [[ "$(basename "$(dirname "${repo_dir}")")" == "src" ]]; then
        workspaces=("$(cd "${repo_dir}/../.." && pwd)")
    else
        workspaces=("${repo_dir}")
    fi
fi

patterns=(
    '(^|[ /])[i]gn gazebo( |$)'
    '[r]os2 launch'
    '[/]opt/ros/humble/lib/(robot_state_publisher|ros_gz_bridge|ros_gz_image|controller_manager|rviz2)/'
)
for ws in "${workspaces[@]}"; do
    escaped="$(printf '%s' "${ws%/}" | sed 's/[][\.*^$+?(){}|]/\\&/g')"
    patterns+=("[/]${escaped#/}/install/")
done

# This shell and everything above it, so a pattern in our own caller's command
# line can never make us kill the caller.
ancestors=" "
pid=$$
while [[ -n "${pid}" && "${pid}" != 0 && "${pid}" != 1 ]]; do
    ancestors+="${pid} "
    pid="$(ps -o ppid= -p "${pid}" 2>/dev/null | tr -d ' ')"
done

# A simulator process is never a shell, but a forked copy of the caller's shell
# carries the caller's whole command line until it execs (and for shell
# functions such as an aliased grep, it never does), so shells are skipped.
is_shell() {
    case "$(basename "$(readlink "/proc/$1/exe" 2>/dev/null)")" in
        bash|sh|dash|zsh|fish) return 0 ;;
        *) return 1 ;;
    esac
}

matching_pids() {
    local p
    for p in $(pgrep -f -- "$1" 2>/dev/null); do
        [[ "${ancestors}" == *" ${p} "* ]] && continue
        is_shell "${p}" && continue
        echo "${p}"
    done
}

all_matching_pids() {
    local pattern
    for pattern in "${patterns[@]}"; do matching_pids "${pattern}"; done | sort -un
}

clear_shm() { rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; }

targets="$(all_matching_pids)"

if ((dry_run)); then
    if [[ -z "${targets}" ]]; then
        echo "dry run: nothing to kill"
    else
        echo "dry run: would kill:"
        # shellcheck disable=SC2086
        ps -o pid=,args= -p ${targets} | cut -c1-160
    fi
    echo "dry run: would remove $(ls /dev/shm 2>/dev/null | grep -c fastrtps) Fast DDS shared-memory file(s)"
    exit 0
fi

if [[ -n "${targets}" ]]; then
    # shellcheck disable=SC2086
    kill -9 ${targets} 2>/dev/null
    sleep 3
fi

clear_shm
if [[ -r /opt/ros/humble/setup.bash ]]; then
    # ros2 needs ROS sourced, and the setup script is not nounset-clean.
    (set +u; source /opt/ros/humble/setup.bash; ros2 daemon stop >/dev/null 2>&1)
    sleep 2
    clear_shm
fi

# xvfb-run leaves its Xvfb behind when killed. It cannot be told apart from
# another user's X server by name, so it is reported and left to the caller.
orphans="$(pgrep -a Xvfb 2>/dev/null || true)"
[[ -n "${orphans}" ]] && echo "note: Xvfb still running (kill it by PID if it is yours):" && echo "${orphans}"

left="$(all_matching_pids)"
if [[ -z "${left}" ]]; then
    echo "clean"
    exit 0
fi
echo "still running after kill -9:"
# shellcheck disable=SC2086
ps -o pid=,args= -p ${left} | cut -c1-160
exit 1
