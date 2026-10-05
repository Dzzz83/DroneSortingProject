#!/usr/bin/env bash

set -eo pipefail

PROJECT_ROOT="$HOME/DroneSortingProject"
UNITY_PROJECT="$PROJECT_ROOT/Unity"
ROS_WS="$HOME/drone_sorting_ros1_ws"

UNITY_EDITOR="$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/unityhub-unity-editor-6000.6.0f1"

UNITY_LOG="/tmp/drone-warehouse-unity.log"
ROS_LOG="/tmp/drone-warehouse-ros.log"

MONITOR_SCRIPT="$PROJECT_ROOT/ROS1/src/drone_sorting/scripts/delivery_demo_monitor.py"

cleanup() {
    echo
    echo "[CLEANUP] Stopping ROS..."

    if [[ -n "${ROS_PID:-}" ]]; then
        kill "$ROS_PID" 2>/dev/null || true
    fi

    pkill -f         "[r]oslaunch drone_sorting warehouse_demo.launch"         2>/dev/null || true

    pkill -f         "[d]efault_server_endpoint.py"         2>/dev/null || true

    pkill -x rosmaster 2>/dev/null || true
    pkill -x roscore 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo "========================================"
echo " ROS-Controlled Warehouse Delivery Demo"
echo "========================================"

echo
echo "[1/5] Syncing repository ROS package..."

source /opt/ros/noetic/setup.bash

rm -rf "$ROS_WS/src/drone_sorting"

cp -r     "$PROJECT_ROOT/ROS1/src/drone_sorting"     "$ROS_WS/src/drone_sorting"

cd "$ROS_WS"

catkin_make     >/tmp/drone-warehouse-build.log     2>&1

source devel/setup.bash

echo "[OK] ROS package synced and built."

echo
echo "[2/5] Cleaning previous ROS processes..."

pkill -f     "[r]oslaunch drone_sorting"     2>/dev/null || true

pkill -f     "[d]efault_server_endpoint.py"     2>/dev/null || true

pkill -x rosmaster 2>/dev/null || true
pkill -x roscore 2>/dev/null || true

sleep 1

echo
echo "[3/5] Opening MainWarehouse..."

distrobox-host-exec     pkill -f     "[u]nityhub-unity-editor-6000.6.0f1.*$UNITY_PROJECT"     2>/dev/null || true

sleep 2

rm -f "$UNITY_LOG"

distrobox-host-exec     "$UNITY_EDITOR"     -projectPath "$UNITY_PROJECT"     -executeMethod DeliveryDemoSceneLauncher.OpenMainWarehouse     >"$UNITY_LOG" 2>&1 &

echo "[OK] MainWarehouse requested."

echo
echo "[4/5] Starting ROS warehouse mission..."

rm -f "$ROS_LOG"

roslaunch     --screen     drone_sorting     warehouse_demo.launch     >"$ROS_LOG" 2>&1 &

ROS_PID=$!

ROS_READY=0

for _ in $(seq 1 30); do
    if rosparam get /rosversion         >/dev/null 2>&1
    then
        ROS_READY=1
        break
    fi

    if ! kill -0 "$ROS_PID"         2>/dev/null
    then
        echo
        echo "[FAIL] ROS stopped during startup."
        tail -80 "$ROS_LOG"
        exit 1
    fi

    sleep 1
done

if [[ "$ROS_READY" -ne 1 ]]; then
    echo
    echo "[FAIL] ROS master did not start."
    tail -80 "$ROS_LOG"
    exit 1
fi

echo "[OK] ROS is running."

echo
echo "[5/5] Mission monitor"
echo
echo "========================================"
echo " Scene: MainWarehouse"
echo " Control: ROS"
echo " Avoidance: OFF (stage 1)"
echo
echo " Press PLAY in Unity."
echo
echo " Expected data flow:"
echo " Unity position -> ROS controller"
echo " ROS command -> Unity drone"
echo " ROS PICKUP/DROP -> PackageHandling"
echo "========================================"
echo

if [[ ! -f "$MONITOR_SCRIPT" ]]; then
    echo "[FAIL] Monitor script not found:"
    echo "       $MONITOR_SCRIPT"
    exit 1
fi

python3 -u "$MONITOR_SCRIPT"
