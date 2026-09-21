#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point


class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager')

        self.publisher = rospy.Publisher(
            '/drone/target_position',
            Point,
            queue_size=10
        )

        self.targets = [
            (0.0, 0.0, 2.0),  # Zone A
            (6.0, 4.0, 2.0),  # Zone B
        ]

        self.target_index = 0

        self.timer = rospy.Timer(
            rospy.Duration(5.0),
            self.publish_next_target
        )

        rospy.loginfo('Mission Manager started.')

        self.publish_next_target()

    def publish_next_target(self, event=None):
        x, y, z = self.targets[self.target_index]

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z

        self.publisher.publish(msg)

        rospy.loginfo(
            f'Published target: ({x}, {y}, {z})'
        )

        self.target_index = (
            self.target_index + 1
        ) % len(self.targets)


if __name__ == '__main__':
    try:
        manager = MissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass