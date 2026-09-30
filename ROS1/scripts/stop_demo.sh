#!/usr/bin/env bash

echo "Stopping ROS drone system..."

distrobox enter --name ros-noetic -- bash -lc '
pkill -f "[r]oslaunch drone_sorting drone_demo.launch" 2>/dev/null || true
pkill -f "[d]efault_server_endpoint.py" 2>/dev/null || true
pkill -x roscore 2>/dev/null || true
pkill -x rosmaster 2>/dev/null || true
'

echo "ROS stopped."