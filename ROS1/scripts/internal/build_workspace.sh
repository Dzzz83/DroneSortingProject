#!/usr/bin/env bash
# Run in the selected ROS environment; never replace an existing source package.
set -eo pipefail

PROJECT_ROOT="$1"
WORKSPACE="$2"
ENDPOINT_SOURCE="$3"
RUN_DIR="$4"
export CAMERA_DEMO_RUN_DIR="$RUN_DIR"
BUILD_PID=''
cleanup() {
    trap - EXIT INT TERM
    if [[ -n "$BUILD_PID" ]]; then
        kill -TERM -- "-$BUILD_PID" 2>/dev/null || true
        wait "$BUILD_PID" 2>/dev/null || true
    fi
    rm -f "$RUN_DIR/prepare-supervisor.pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '%s\n' "$$" > "$RUN_DIR/prepare-supervisor.pid"

source /opt/ros/noetic/setup.bash
command -v catkin_make >/dev/null || { echo '[FAIL] catkin_make is unavailable.'; exit 1; }
[[ -f "$ENDPOINT_SOURCE/package.xml" && -f "$ENDPOINT_SOURCE/launch/endpoint.launch" ]] || {
    echo "[FAIL] ROS-TCP-Endpoint source is missing: $ENDPOINT_SOURCE"; exit 1;
}

link_package() {
    local source_path="$1" destination="$2"
    if [[ -e "$destination" || -L "$destination" ]]; then
        if [[ ! -L "$destination" || "$(readlink -f "$destination")" != "$(readlink -f "$source_path")" ]]; then
            echo "[FAIL] Workspace path is already owned by another package: $destination"
            exit 1
        fi
    else
        ln -s "$source_path" "$destination"
    fi
}

mkdir -p "$WORKSPACE/src"
# Migrate only the symlink created by the previous demo layout.
legacy_link="$WORKSPACE/src/drone_sorting"
if [[ -L "$legacy_link" && ( "$(readlink "$legacy_link")" == "$PROJECT_ROOT/ROS1/src/drone_sorting" || "$(readlink "$legacy_link")" == "$PROJECT_ROOT/ROS1" ) ]]; then
    rm "$legacy_link"
fi
link_package "$PROJECT_ROOT/ROS1/package" "$WORKSPACE/src/drone_sorting"
link_package "$ENDPOINT_SOURCE" "$WORKSPACE/src/ROS-TCP-Endpoint"
cd "$WORKSPACE"
# A separate build group lets cancellation stop make and compiler children too.
setsid catkin_make -j2 -l2 &
BUILD_PID=$!
wait "$BUILD_PID"
BUILD_PID=''
