#!/usr/bin/env bash

set -e

PROJECT_ROOT="$HOME/DroneSortingProject"
ROS_WS="$HOME/drone_sorting_ros1_ws"

echo "Syncing repository ROS package into Catkin workspace..."

distrobox enter --name ros-noetic -- bash -lc "
set -e
source /opt/ros/noetic/setup.bash

rm -rf \"$ROS_WS/src/drone_sorting\"
cp -r \"$PROJECT_ROOT/ROS1/src/drone_sorting\" \"$ROS_WS/src/drone_sorting\"

cd \"$ROS_WS\"
catkin_make
"

echo "ROS workspace synced and built successfully."
