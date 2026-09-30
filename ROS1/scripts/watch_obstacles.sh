#!/usr/bin/env bash
distrobox enter --name ros-noetic -- bash -lc "source /opt/ros/noetic/setup.bash && cd \"$HOME/drone_sorting_ros1_ws\" && source devel/setup.bash && rostopic echo /drone/obstacle_distances"
