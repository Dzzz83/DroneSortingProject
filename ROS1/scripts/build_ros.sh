#!/usr/bin/env bash

set -e

distrobox enter --name ros-noetic -- bash -lc '
source /opt/ros/noetic/setup.bash
cd "$HOME/drone_sorting_ros1_ws"
catkin_make
'

echo "ROS workspace built successfully."