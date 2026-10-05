#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$HOME/DroneSortingProject"
ROS_WS="$HOME/drone_sorting_ros1_ws"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "[INFO] Entering ros-noetic Distrobox..."

    exec distrobox enter --name ros-noetic -- bash -lc \
        "cd \"$PROJECT_ROOT\" && bash ROS1/scripts/build_ros.sh"
fi

source /opt/ros/noetic/setup.bash

bash "$PROJECT_ROOT/ROS1/scripts/sync_ros_workspace.sh"

cd "$ROS_WS"
catkin_make

echo "ROS workspace built successfully."
