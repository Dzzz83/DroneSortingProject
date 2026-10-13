#!/usr/bin/env python3
"""Start stationary perception; publishes no movement or gripper commands."""
import rospy
from drone_sorting.perception.package_perception import PackagePerception

if __name__ == "__main__":
    rospy.init_node("package_perception")
    perception = PackagePerception()
    rospy.spin()
