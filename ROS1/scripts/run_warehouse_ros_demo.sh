#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$HOME/DroneSortingProject"
UNITY_PROJECT="$PROJECT_ROOT/Unity"
ROS_WS="$HOME/drone_sorting_ros1_ws"

ENABLE_AVOIDANCE="${1:-false}"

UNITY_EDITOR="$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/unityhub-unity-editor-6000.6.0f1"

UNITY_LOG="/tmp/drone-warehouse-unity.log"
ROS_LOG="/tmp/drone-warehouse-ros.log"
BUILD_LOG="/tmp/drone-warehouse-build.log"

MONITOR_SCRIPT="$PROJECT_ROOT/ROS1/src/drone_sorting/scripts/delivery_demo_monitor.py"


validate_arguments() {
    if [[ "$ENABLE_AVOIDANCE" != "false" && "$ENABLE_AVOIDANCE" != "true" ]]; then
        echo "Usage: bash ROS1/scripts/run_warehouse_ros_demo.sh [false|true]"
        echo "  false = ROS navigation + pickup/drop"
        echo "  true  = ROS obstacle avoidance enabled"
        exit 1
    fi
}


enter_ros_container_if_needed() {
    if [[ -f /opt/ros/noetic/setup.bash ]]; then
        return
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


open_unity() {
    echo
    echo "[3/5] Opening MainWarehouse..."

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

    echo "[OK] MainWarehouse requested."
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
enter_ros_container_if_needed

trap cleanup EXIT INT TERM

echo "========================================"
echo " ROS-Controlled Warehouse Delivery Demo"
echo "========================================"
echo "Obstacle avoidance: $ENABLE_AVOIDANCE"

sync_and_build_ros
prepare_ros
open_unity
start_ros
wait_for_ros
run_monitor
