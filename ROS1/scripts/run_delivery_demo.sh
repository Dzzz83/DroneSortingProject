#!/usr/bin/env bash

set -eo pipefail

PROJECT_ROOT="$HOME/DroneSortingProject"
UNITY_PROJECT="$PROJECT_ROOT/Unity"
ROS_WS="$HOME/drone_sorting_ros1_ws"

UNITY_EDITOR="$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/unityhub-unity-editor-6000.6.0f1"

UNITY_LOG="/tmp/drone-unity.log"
ROS_LOG="/tmp/drone-m4-ros.log"

MONITOR_SCRIPT="$PROJECT_ROOT/ROS1/src/drone_sorting/scripts/delivery_demo_monitor.py"


cleanup() {
    echo
    echo "[CLEANUP] Stopping ROS..."

    if [[ -n "${ROS_PID:-}" ]]; then
        kill "$ROS_PID" 2>/dev/null || true
    fi

    pkill -f \
        "[r]oslaunch drone_sorting drone_demo.launch" \
        2>/dev/null || true

    pkill -f \
        "[d]efault_server_endpoint.py" \
        2>/dev/null || true

    pkill -x rosmaster \
        2>/dev/null || true

    pkill -x roscore \
        2>/dev/null || true
}


trap cleanup EXIT INT TERM


echo "========================================"
echo " Drone Sorting Delivery Demo"
echo "========================================"


echo
echo "[1/4] Cleaning previous ROS processes..."

pkill -f \
    "[r]oslaunch drone_sorting drone_demo.launch" \
    2>/dev/null || true

pkill -f \
    "[d]efault_server_endpoint.py" \
    2>/dev/null || true

pkill -x rosmaster \
    2>/dev/null || true

pkill -x roscore \
    2>/dev/null || true

sleep 1


echo
echo "[2/4] Restarting Unity with SampleScene..."

distrobox-host-exec \
    pkill -f \
    "[u]nityhub-unity-editor-6000.6.0f1.*$UNITY_PROJECT" \
    2>/dev/null || true

sleep 2

rm -f "$UNITY_LOG"

distrobox-host-exec \
    "$UNITY_EDITOR" \
    -projectPath "$UNITY_PROJECT" \
    -executeMethod DeliveryDemoSceneLauncher.OpenSampleScene \
    >"$UNITY_LOG" 2>&1 &

echo "Waiting for Unity..."

UNITY_READY=0

for _ in $(seq 1 30); do
    sleep 1

    if distrobox-host-exec \
        pgrep -af \
        "[u]nityhub-unity-editor-6000.6.0f1" \
        2>/dev/null \
        | grep -F "$UNITY_PROJECT" \
        >/dev/null
    then
        UNITY_READY=1
        break
    fi
done

if [[ "$UNITY_READY" -ne 1 ]]; then
    echo
    echo "[FAIL] Unity did not start."
    tail -80 "$UNITY_LOG" 2>/dev/null || true
    exit 1
fi

echo "[OK] Unity started."
echo "[OK] SampleScene requested."


echo
echo "[3/4] Starting ROS..."

source /opt/ros/noetic/setup.bash

cd "$ROS_WS"
source devel/setup.bash

rm -f "$ROS_LOG"

roslaunch \
    --screen \
    drone_sorting \
    drone_demo.launch \
    enable_obstacle_avoidance:=false \
    >"$ROS_LOG" 2>&1 &

ROS_PID=$!

echo "Waiting for ROS master..."

ROS_READY=0

for _ in $(seq 1 30); do
    if rosparam get /rosversion \
        >/dev/null 2>&1
    then
        ROS_READY=1
        break
    fi

    if ! kill -0 "$ROS_PID" \
        2>/dev/null
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
echo "[4/4] Mission monitor"
echo
echo "========================================"
echo " Scene: SampleScene"
echo
echo " Press PLAY in Unity."
echo
echo " The ROS-driven mission will be"
echo " verified in this terminal."
echo "========================================"
echo

if [[ ! -f "$MONITOR_SCRIPT" ]]; then
    echo "[FAIL] Monitor script not found:"
    echo "       $MONITOR_SCRIPT"
    exit 1
fi

python3 -u "$MONITOR_SCRIPT"