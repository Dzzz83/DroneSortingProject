#!/usr/bin/env python3

import rospy

from drone_sorting.mission.mission_manager import MissionManager


def main():
    rospy.init_node("mission_manager")

    manager = MissionManager()
    manager.run()


if __name__ == "__main__":
    main()