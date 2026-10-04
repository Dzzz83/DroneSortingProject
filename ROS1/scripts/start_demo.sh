#!/usr/bin/env bash

set -e

echo "Cleaning old ROS processes..."

distrobox enter --name ros-noetic -- bash -lc '
pkill -f "[r]oslaunch drone_sorting drone_demo.launch" 2>/dev/null || true
pkill -f "[d]efault_server_endpoint.py" 2>/dev/null || true
pkill -x roscore 2>/dev/null || true
pkill -x rosmaster 2>/dev/null || true
'

sleep 1

echo "Opening Unity..."

UNITY="$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/Unity"

if ! pgrep -f "[D]roneSortingSimulation" >/dev/null; then
    "$UNITY" \
        -projectPath "$HOME/DroneSortingSimulation" \
        >/tmp/drone-unity.log 2>&1 &
fi

echo "Starting ROS drone system..."

distrobox enter --name ros-noetic -- bash -lc '
source /opt/ros/noetic/setup.bash
cd "$HOME/drone_sorting_ros1_ws"
source devel/setup.bash

roslaunch drone_sorting drone_demo.launch
'