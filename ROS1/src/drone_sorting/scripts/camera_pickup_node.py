#!/usr/bin/env python3
"""Run camera-guided pickup or delivery, with optional access from DroneStart."""
import rospy
from drone_sorting.mission.camera_pickup import CameraPickup

if __name__ == "__main__":
    rospy.init_node("camera_pickup")
    CameraPickup().run()
