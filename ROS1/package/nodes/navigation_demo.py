#!/usr/bin/env python3
import rospy
from drone_sorting.navigation.demo import NavigationDemo

if __name__ == '__main__':
    rospy.init_node('navigation_demo')
    NavigationDemo().run()
