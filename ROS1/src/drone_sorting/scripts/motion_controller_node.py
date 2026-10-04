#!/usr/bin/env python3

import rospy

from drone_sorting.control.motion_controller import MotionController


def main():
    rospy.init_node("motion_controller")

    controller = MotionController()
    controller.run()


if __name__ == "__main__":
    main()