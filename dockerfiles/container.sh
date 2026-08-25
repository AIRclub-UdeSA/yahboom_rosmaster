#!/usr/bin/env bash
#
# Yahboom ROSMASTER Simulation Container Helper (Docker)
#
# Usage:
#    ./container.sh start         create + start the container
#    ./container.sh enter         open a new shell inside the container
#    ./container.sh deps          install repository dependencies with rosdep
#    ./container.sh build         recompile the workspace
#    ./container.sh sim           launch Gazebo + RViz simulation
#    ./container.sh sim-headless  launch headless simulation
#    ./container.sh teleop        launch keyboard teleoperation
#    ./container.sh stop          stop the container
#    ./container.sh clean         stop and remove the container
#    ./container.sh doctor        diagnose display / GPU setup
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# Workspace root is either the parent of src/ or the repo itself if cloned directly
if [[ "$(basename "$(dirname "$REPO_DIR")")" == "src" ]]; then
    HOST_WS="$(cd "${REPO_DIR}/../.." && pwd)"
else
    HOST_WS="$REPO_DIR"
fi

ENGINE="docker"
IMAGE="${IMAGE:-yahboom_rosmaster:$(id -un)}"
CONTAINER_NAME="${CONTAINER_NAME:-yahboom_rosmaster_sim}"
CONTAINER_USER="alumno"
CONT_WS="/home/${CONTAINER_USER}/rosmaster_ws"

X11_HOST_DIR="/tmp/rosmaster-x11-$(id -u)-${CONTAINER_NAME}"
X11_CONT_DIR="/tmp/.x11host"

die() {
    echo "Error: $*" >&2
    exit 1
}

info() {
    echo ">> $*"
}

container_state() {
    $ENGINE inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || echo "missing"
}

require_running() {
    [ "$(container_state)" = "running" ] || die "Container is not running. Run: $0 start"
}

build_image() {
    info "Building image '$IMAGE'..."
    info "Container user '${CONTAINER_USER}' will match host UID $(id -u), GID $(id -g)."
    $ENGINE build \
        --build-arg USER_UID="$(id -u)" \
        --build-arg USER_GID="$(id -g)" \
        -t "$IMAGE" \
        -f "${SCRIPT_DIR}/Dockerfile" \
        "${SCRIPT_DIR}"
    info "Image built successfully."
}

ensure_image() {
    $ENGINE image inspect "$IMAGE" >/dev/null 2>&1 && return 0
    info "Image '$IMAGE' not found locally."
    build_image
}

has_nvidia_gpu() {
    command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1
}

nvidia_runtime_available() {
    $ENGINE info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'
}

refresh_xauth() {
    local cookie="${X11_HOST_DIR}/Xauthority"
    mkdir -p "$X11_HOST_DIR"
    chmod 755 "$X11_HOST_DIR"

    [ -n "${DISPLAY:-}" ] || return 1

    if ! command -v xauth >/dev/null 2>&1; then
        return 1
    fi

    rm -f "${cookie}.tmp"
    : >"${cookie}.tmp"
    xauth nlist "$DISPLAY" 2>/dev/null |
        sed -e 's/^..../ffff/' |
        xauth -f "${cookie}.tmp" nmerge - 2>/dev/null || true

    mv -f "${cookie}.tmp" "$cookie"
    chmod 644 "$cookie"
    [ -s "$cookie" ]
}

get_display_args() {
    local args=()
    if [ -n "${DISPLAY:-}" ]; then
        args+=(-e "DISPLAY=$DISPLAY")
        if refresh_xauth; then
            args+=(-e "XAUTHORITY=${X11_CONT_DIR}/Xauthority")
        fi
    fi
    echo "${args[@]}"
}

install_dependencies() {
    require_running
    info "Installing workspace dependencies with rosdep..."
    exec_in_container bash -c \
        "source /opt/ros/humble/setup.bash && \
         if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
           sudo rosdep init; \
         fi && \
         rosdep update && \
         rosdep install --from-paths . --ignore-src -r -y --rosdistro humble"
}

start_container() {
    ensure_image
    local state
    state="$(container_state)"

    if [ "$state" = "running" ]; then
        info "Container '$CONTAINER_NAME' is already running."
        return 0
    elif [ "$state" = "exited" ]; then
        info "Restarting existing container '$CONTAINER_NAME'..."
        $ENGINE start "$CONTAINER_NAME" >/dev/null
        return 0
    fi

    info "Starting new container '$CONTAINER_NAME'..."
    mkdir -p "$X11_HOST_DIR"
    chmod 755 "$X11_HOST_DIR"
    local run_args=(
        -d
        --name "$CONTAINER_NAME"
        --net=host
        --ipc=host
        -v "${X11_HOST_DIR}:${X11_CONT_DIR}:ro"
        -v "${HOST_WS}:${CONT_WS}:rw"
    )

    if [ -d /tmp/.X11-unix ]; then
        run_args+=(-v "/tmp/.X11-unix:/tmp/.X11-unix:rw")
    fi

    if [ -d /dev/dri ]; then
        run_args+=(--device=/dev/dri)
        while IFS= read -r device_group; do
            run_args+=(--group-add "$device_group")
        done < <(find /dev/dri -maxdepth 1 -type c -printf '%g\n' | sort -u)
    fi

    if has_nvidia_gpu && nvidia_runtime_available; then
        run_args+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display)
    elif [ ! -d /dev/dri ]; then
        run_args+=(-e LIBGL_ALWAYS_SOFTWARE=1)
    fi

    local disp_args
    disp_args="$(get_display_args)"

    # shellcheck disable=SC2086
    $ENGINE run "${run_args[@]}" $disp_args "$IMAGE" sleep infinity >/dev/null
    info "Container started. Workspace mounted at: $CONT_WS"
}

exec_in_container() {
    require_running
    local tty_args=(-i)
    if [ -t 0 ] && [ -t 1 ]; then
        tty_args+=(-t)
    fi
    local disp_args
    disp_args="$(get_display_args)"
    # shellcheck disable=SC2086
    $ENGINE exec "${tty_args[@]}" -u "$CONTAINER_USER" -w "$CONT_WS" \
        $disp_args "$CONTAINER_NAME" "$@"
}

cmd="${1:-}"
if ! command -v "$ENGINE" >/dev/null 2>&1; then
    case "$cmd" in
        ""|-h|--help|help|doctor) ;;
        *) die "$ENGINE is not installed." ;;
    esac
fi

case "$cmd" in
    start)
        start_container
        ;;
    enter|shell)
        require_running
        exec_in_container bash
        ;;
    deps)
        install_dependencies
        ;;
    build)
        install_dependencies
        info "Building workspace in container..."
        exec_in_container bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
        ;;
    sim)
        require_running
        info "Launching simulation..."
        exec_in_container bash -c "source /opt/ros/humble/setup.bash && [ -f install/setup.bash ] && source install/setup.bash; ros2 launch yahboom_rosmaster_bringup rosmaster_x3_sim.launch.py"
        ;;
    sim-headless)
        require_running
        info "Launching headless simulation..."
        exec_in_container bash -c "source /opt/ros/humble/setup.bash && [ -f install/setup.bash ] && source install/setup.bash; ros2 launch yahboom_rosmaster_bringup rosmaster_x3_sim.launch.py gui:=false headless:=true rviz:=false"
        ;;
    teleop)
        require_running
        exec_in_container bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
        ;;
    stop)
        info "Stopping container '$CONTAINER_NAME'..."
        $ENGINE stop "$CONTAINER_NAME" 2>/dev/null || true
        info "Stopped."
        ;;
    clean)
        info "Cleaning container '$CONTAINER_NAME'..."
        $ENGINE rm -f "$CONTAINER_NAME" 2>/dev/null || true
        info "Cleaned."
        ;;
    doctor)
        info "=== Simulator Container Doctor ==="
        echo "Docker: $(command -v docker || echo 'not installed')"
        echo "Container State: $(container_state)"
        echo "DISPLAY: ${DISPLAY:-unset}"
        echo "Host xauth: $(command -v xauth >/dev/null 2>&1 && echo 'available' || echo 'missing (install xauth for GUI forwarding)')"
        echo "NVIDIA GPU: $(has_nvidia_gpu && echo 'detected' || echo 'none')"
        echo "NVIDIA Docker Runtime: $(nvidia_runtime_available && echo 'available' || echo 'unavailable')"
        if [ "$(container_state)" = "running" ] && [ -n "${DISPLAY:-}" ]; then
            disp_args="$(get_display_args)"
            # shellcheck disable=SC2086
            if $ENGINE exec -u "$CONTAINER_USER" $disp_args "$CONTAINER_NAME" \
                xdpyinfo >/dev/null 2>&1; then
                echo "Container X11: reachable"
            else
                echo "Container X11: unavailable"
            fi
        fi
        ;;
    ""|-h|--help|help)
        echo "Usage: $0 {start|enter|deps|build|sim|sim-headless|teleop|stop|clean|doctor}"
        ;;
    *)
        die "Unknown command: $cmd"
        ;;
esac
