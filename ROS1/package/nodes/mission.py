#!/usr/bin/env python3
"""Run the complete camera-guided warehouse delivery from the drone start."""
import rospy
from drone_sorting.mission.delivery import DeliveryMission

if __name__ == "__main__":
    rospy.init_node("camera_pickup")
    DeliveryMission().run()
