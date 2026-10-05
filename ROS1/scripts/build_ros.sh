#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
ROS_WS="${ROS_WS:-$HOME/drone_sorting_ros1_ws}"


is_wsl() {
    grep -qiE "(microsoft|wsl)" /proc/version 2>/dev/null
}


if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    if is_wsl; then
        echo "[FAIL] ROS Noetic was not found inside WSL2."
        exit 1
    fi

    if ! command -v distrobox >/dev/null 2>&1; then
        echo "[FAIL] ROS Noetic is unavailable and distrobox was not found."
        exit 1
    fi

    echo "[INFO] Entering ros-noetic Distrobox..."

    exec distrobox enter --name ros-noetic -- bash -lc \
        "cd \"$PROJECT_ROOT\" && bash ROS1/scripts/build_ros.sh"
fi

source /opt/ros/noetic/setup.bash

PROJECT_ROOT="$PROJECT_ROOT" ROS_WS="$ROS_WS" \
    bash "$PROJECT_ROOT/ROS1/scripts/sync_ros_workspace.sh"

cd "$ROS_WS"
catkin_make

echo "ROS workspace built successfully."
