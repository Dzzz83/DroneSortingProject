#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
UNITY_PROJECT="$PROJECT_ROOT/Unity"
ROS_WS="${ROS_WS:-$HOME/drone_sorting_ros1_ws}"

ENABLE_AVOIDANCE="${1:-false}"

UNITY_EDITOR="${UNITY_EDITOR:-$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/unityhub-unity-editor-6000.6.0f1}"

UNITY_LOG="/tmp/drone-warehouse-unity.log"
ROS_LOG="/tmp/drone-warehouse-ros.log"
BUILD_LOG="/tmp/drone-warehouse-build.log"

MONITOR_SCRIPT="$PROJECT_ROOT/ROS1/src/drone_sorting/scripts/delivery_demo_monitor.py"


is_wsl() {
    grep -qiE "(microsoft|wsl)" /proc/version 2>/dev/null
}


validate_arguments() {
    if [[ "$ENABLE_AVOIDANCE" != "false" && "$ENABLE_AVOIDANCE" != "true" ]]; then
        echo "Usage: bash ROS1/scripts/run_warehouse_ros_demo.sh [false|true]"
        echo "  false = ROS navigation + pickup/drop"
        echo "  true  = ROS obstacle avoidance enabled"
        exit 1
    fi
}


enter_ros_environment_if_needed() {
    if [[ -f /opt/ros/noetic/setup.bash ]]; then
        return
    fi

    if is_wsl; then
        echo "[FAIL] ROS Noetic was not found inside WSL2."
        echo "       Install/use Ubuntu 20.04 with ROS Noetic, then run this script again."
        exit 1
    fi

    if ! command -v distrobox >/dev/null 2>&1; then
        echo "[FAIL] ROS Noetic is unavailable and distrobox was not found."
        exit 1
    fi

    echo "[INFO] Entering ros-noetic Distrobox..."

    exec distrobox enter --name ros-noetic -- bash -lc \
        "cd \"$PROJECT_ROOT\" && bash ROS1/scripts/run_warehouse_ros_demo.sh \"$ENABLE_AVOIDANCE\""
}


stop_ros_processes() {
    if [[ -n "${ROS_PID:-}" ]]; then
        kill "$ROS_PID" 2>/dev/null || true
    fi

    pkill -f "[r]oslaunch drone_sorting" 2>/dev/null || true
    pkill -f "[d]efault_server_endpoint.py" 2>/dev/null || true
    pkill -x rosmaster 2>/dev/null || true
    pkill -x roscore 2>/dev/null || true
}


cleanup() {
    echo
    echo "[CLEANUP] Stopping ROS..."
    stop_ros_processes
}


sync_and_build_ros() {
    echo
    echo "[1/5] Syncing and building ROS package..."

    source /opt/ros/noetic/setup.bash

    PROJECT_ROOT="$PROJECT_ROOT" ROS_WS="$ROS_WS" \
        bash "$PROJECT_ROOT/ROS1/scripts/sync_ros_workspace.sh"

    cd "$ROS_WS"

    if ! catkin_make >"$BUILD_LOG" 2>&1; then
        echo "[FAIL] ROS build failed."
        tail -80 "$BUILD_LOG"
        exit 1
    fi

    source devel/setup.bash

    echo "[OK] ROS package built."
}


prepare_ros() {
    echo
    echo "[2/5] Cleaning previous ROS processes..."

    stop_ros_processes
    sleep 1
}


show_wsl_unity_instructions() {
    local wsl_ip
    local windows_unity_path=""

    wsl_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"

    if command -v wslpath >/dev/null 2>&1; then
        windows_unity_path="$(wslpath -w "$UNITY_PROJECT" 2>/dev/null || true)"
    fi

    echo "[INFO] WSL2 detected."
    echo "[INFO] Unity must run on Windows, so it will not be launched from this script."
    echo
    echo "       In Windows Unity Hub:"

    if [[ -n "$windows_unity_path" ]]; then
        echo "       1. Open: $windows_unity_path"
    else
        echo "       1. Open the matching DroneSortingProject/Unity project."
    fi
    echo "       2. Open Assets/Scenes/MainWarehouse.unity."
    echo "       3. Set Active Input Handling to Both."
    echo "       4. In Robotics -> ROS Settings, use port 10000."
    echo "       5. Try ROS IP 127.0.0.1 first."

    if [[ -n "$wsl_ip" ]]; then
        echo "          If localhost does not connect, use WSL2 IP: $wsl_ip"
    fi

    echo "       6. Press Play after ROS starts."
}


open_unity() {
    echo
    echo "[3/5] Preparing MainWarehouse..."

    if is_wsl; then
        show_wsl_unity_instructions
        return
    fi

    if command -v distrobox-host-exec >/dev/null 2>&1; then
        distrobox-host-exec \
            pkill -f \
            "[u]nityhub-unity-editor-6000.6.0f1.*$UNITY_PROJECT" \
            2>/dev/null || true

        sleep 2
        rm -f "$UNITY_LOG"

        distrobox-host-exec \
            "$UNITY_EDITOR" \
            -projectPath "$UNITY_PROJECT" \
            -executeMethod DeliveryDemoSceneLauncher.OpenMainWarehouse \
            >"$UNITY_LOG" 2>&1 &

        echo "[OK] MainWarehouse requested through host Unity."
        return
    fi

    if [[ -x "$UNITY_EDITOR" ]]; then
        pkill -f \
            "[u]nityhub-unity-editor-6000.6.0f1.*$UNITY_PROJECT" \
            2>/dev/null || true

        sleep 2
        rm -f "$UNITY_LOG"

        "$UNITY_EDITOR" \
            -projectPath "$UNITY_PROJECT" \
            -executeMethod DeliveryDemoSceneLauncher.OpenMainWarehouse \
            >"$UNITY_LOG" 2>&1 &

        echo "[OK] MainWarehouse requested."
        return
    fi

    echo "[INFO] Unity Editor was not found at:"
    echo "       $UNITY_EDITOR"
    echo "[INFO] Open this project manually in Unity:"
    echo "       $UNITY_PROJECT"
    echo "[INFO] Then open Assets/Scenes/MainWarehouse.unity."
}


start_ros() {
    echo
    echo "[4/5] Starting ROS warehouse mission..."

    rm -f "$ROS_LOG"

    roslaunch \
        --screen \
        drone_sorting \
        warehouse_demo.launch \
        enable_obstacle_avoidance:="$ENABLE_AVOIDANCE" \
        >"$ROS_LOG" 2>&1 &

    ROS_PID=$!
}


wait_for_ros() {
    for _ in $(seq 1 30); do
        if rosparam get /rosversion >/dev/null 2>&1; then
            echo "[OK] ROS is running."
            return
        fi

        if ! kill -0 "$ROS_PID" 2>/dev/null; then
            echo "[FAIL] ROS stopped during startup."
            tail -80 "$ROS_LOG"
            exit 1
        fi

        sleep 1
    done

    echo "[FAIL] ROS master did not start."
    tail -80 "$ROS_LOG"
    exit 1
}


run_monitor() {
    echo
    echo "[5/5] Mission monitor"
    echo
    echo "========================================"
    echo " Scene: MainWarehouse"
    echo " Control: ROS"
    echo " Avoidance: $ENABLE_AVOIDANCE"
    echo
    echo " Press PLAY in Unity."
    echo "========================================"
    echo

    if [[ ! -f "$MONITOR_SCRIPT" ]]; then
        echo "[FAIL] Monitor script not found:"
        echo "       $MONITOR_SCRIPT"
        exit 1
    fi

    python3 -u "$MONITOR_SCRIPT"
}


validate_arguments
enter_ros_environment_if_needed

trap cleanup EXIT INT TERM

echo "========================================"
echo " ROS-Controlled Warehouse Delivery Demo"
echo "========================================"
echo "Project: $PROJECT_ROOT"
echo "Obstacle avoidance: $ENABLE_AVOIDANCE"

sync_and_build_ros
prepare_ros
open_unity
start_ros
wait_for_ros
run_monitor
