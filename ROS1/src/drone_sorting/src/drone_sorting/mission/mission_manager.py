import math

import rospy
from geometry_msgs.msg import Point, Vector3


class MissionManager:
    """
    Sends the drone back and forth between two endpoints.

    ROS also publishes the stable travel heading so Unity
    sensors always know which direction is "front".
    """

    def __init__(self):
        self.targets = [
            Point(x=18.0, y=0.0, z=2.0),
            Point(x=0.0, y=0.0, z=2.0),
        ]

        # Corresponding stable horizontal headings.
        self.headings = [
            Vector3(x=1.0, y=0.0, z=0.0),   # toward x=18
            Vector3(x=-1.0, y=0.0, z=0.0),  # toward x=0
        ]

        self.target_index = 0

        self.arrival_tolerance = 0.25
        self.switch_delay = 1.0

        self.waiting_to_switch = False

        self.target_publisher = rospy.Publisher(
            "/drone/target_position",
            Point,
            queue_size=1,
            latch=True,
        )

        self.heading_publisher = rospy.Publisher(
            "/drone/travel_heading",
            Vector3,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(
            "/drone/current_position",
            Point,
            self._position_callback,
        )

    def run(self):
        rospy.loginfo("Mission Manager started")

        rospy.sleep(0.5)

        self._publish_mission()

        rospy.spin()

    def _position_callback(self, position):
        if self.waiting_to_switch:
            return

        target = self.targets[self.target_index]

        dx = target.x - position.x
        dy = target.y - position.y
        dz = target.z - position.z

        distance = math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

        if distance <= self.arrival_tolerance:
            rospy.loginfo(
                "Reached target -> "
                "x=%.2f y=%.2f z=%.2f",
                target.x,
                target.y,
                target.z,
            )

            self.waiting_to_switch = True

            rospy.Timer(
                rospy.Duration(self.switch_delay),
                self._switch_target,
                oneshot=True,
            )

    def _switch_target(self, _event):
        self.target_index = (
            self.target_index + 1
        ) % len(self.targets)

        self.waiting_to_switch = False

        self._publish_mission()

    def _publish_mission(self):
        target = self.targets[self.target_index]
        heading = self.headings[self.target_index]

        self.target_publisher.publish(target)
        self.heading_publisher.publish(heading)

        rospy.loginfo(
            "Mission target -> "
            "x=%.2f y=%.2f z=%.2f | "
            "heading x=%.1f",
            target.x,
            target.y,
            target.z,
            heading.x,
        )