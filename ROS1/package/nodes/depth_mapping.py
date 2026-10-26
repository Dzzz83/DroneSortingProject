#!/usr/bin/env python3
"""Run live depth mapping without mission or movement control."""
import rospy
from drone_sorting.mapping.live_mapping import DepthMappingNode


def main():
    rospy.init_node("depth_mapping")
    DepthMappingNode().run()


if __name__ == "__main__":
    main()
