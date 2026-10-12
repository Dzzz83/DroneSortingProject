#!/usr/bin/env python3
"""Check the camera stream without starting a delivery mission."""

import rospy

from drone_sorting.perception.camera_stream_check import CameraStreamCheck


def main():
    rospy.init_node("camera_stream_check")
    checker = CameraStreamCheck()
    rospy.spin()
    return checker


if __name__ == "__main__":
    main()
