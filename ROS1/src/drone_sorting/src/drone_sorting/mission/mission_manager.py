import math

import rospy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String

from drone_sorting.interfaces import ros_topics

from drone_sorting.mission.mission_config import MissionConfig
from drone_sorting.mission.mission_state import MissionState


class MissionManager:
    """Controls the high-level package delivery mission."""

    def __init__(self):
        self.state = MissionState.IDLE

        self.pickup_position = MissionConfig.PICKUP_POSITION
        self.dropoff_position = MissionConfig.DROPOFF_POSITION

        self.current_position = None
        self.current_target = None

        self.arrival_tolerance = MissionConfig.ARRIVAL_TOLERANCE

        self.target_publisher = rospy.Publisher(
            ros_topics.TARGET_POSITION,
            Point,
            queue_size=1,
            latch=True,
        )

        self.heading_publisher = rospy.Publisher(
            ros_topics.TRAVEL_HEADING,
            Vector3,
            queue_size=1,
            latch=True,
        )

        self.package_action_publisher = rospy.Publisher(
            ros_topics.PACKAGE_ACTION,
            String,
            queue_size=1,
            latch=True,
        )

        self.mission_state_publisher = rospy.Publisher(
            ros_topics.MISSION_STATE,
            String,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(
            ros_topics.CURRENT_POSITION,
            Point,
            self._on_position_update,
        )

        rospy.Subscriber(
            ros_topics.PACKAGE_ACTION_STATUS,
            String,
            self._on_package_status,
        )

        self.mission_state_publisher.publish(
            String(data=self.state.name)
        )

    def run(self):
        rospy.loginfo("Mission Manager started")

        rospy.sleep(0.5)

        self._start_pickup_mission()

        rospy.spin()

    def _on_position_update(self, position):
        self.current_position = position

        if self.state == MissionState.GO_TO_PICKUP:
            self._check_pickup_arrival()

        elif self.state == MissionState.GO_TO_DROPOFF:
            self._check_dropoff_arrival()

    def _on_package_status(self, message):
        status = message.data.strip().upper()

        if (
            self.state == MissionState.WAIT_FOR_PICKUP
            and status == "PICKUP_DONE"
        ):
            rospy.loginfo("Pickup confirmed")
            self._start_dropoff_mission()

        elif (
            self.state == MissionState.WAIT_FOR_DROP
            and status == "DROP_DONE"
        ):
            rospy.loginfo("Drop confirmed")
            self._complete_mission()

    def _start_pickup_mission(self):
        self._set_state(MissionState.GO_TO_PICKUP)
        self._set_target(self.pickup_position)

    def _check_pickup_arrival(self):
        if not self._has_reached_target():
            return

        rospy.loginfo("Pickup point reached")

        self._set_state(MissionState.WAIT_FOR_PICKUP)
        self._request_pickup()

    def _start_dropoff_mission(self):
        self._clear_package_action()

        self._set_state(MissionState.GO_TO_DROPOFF)
        self._set_target(self.dropoff_position)

    def _check_dropoff_arrival(self):
        if not self._has_reached_target():
            return

        rospy.loginfo("Drop-off point reached")

        self._set_state(MissionState.WAIT_FOR_DROP)
        self._request_drop()

    def _complete_mission(self):
        self._clear_package_action()

        self._set_state(MissionState.COMPLETE)

        rospy.loginfo("Mission complete")

    def _request_pickup(self):
        self.package_action_publisher.publish(
            String(data="PICKUP")
        )

        rospy.loginfo("Pickup requested")

    def _request_drop(self):
        self.package_action_publisher.publish(
            String(data="DROP")
        )

        rospy.loginfo("Drop requested")

    def _clear_package_action(self):
        self.package_action_publisher.publish(
            String(data="")
        )

    def _has_reached_target(self):
        if self.current_position is None or self.current_target is None:
            return False

        dx = self.current_target.x - self.current_position.x
        dy = self.current_target.y - self.current_position.y
        dz = self.current_target.z - self.current_position.z

        distance = math.sqrt(
            dx * dx
            + dy * dy
            + dz * dz
        )

        return distance <= self.arrival_tolerance

    def _set_target(self, target):
        self.current_target = target

        heading = self._calculate_heading(
            self.current_position,
            target,
        )

        self.target_publisher.publish(target)
        self.heading_publisher.publish(heading)

        rospy.loginfo(
            "Mission target -> x=%.2f y=%.2f z=%.2f",
            target.x,
            target.y,
            target.z,
        )

    def _set_state(self, new_state):
        if self.state == new_state:
            return

        rospy.loginfo(
            "Mission state: %s -> %s",
            self.state.name,
            new_state.name,
        )

        self.state = new_state

        self.mission_state_publisher.publish(
            String(data=self.state.name)
        )

    @staticmethod
    def _calculate_heading(current_position, target):
        if current_position is None:
            return Vector3(x=1.0, y=0.0, z=0.0)

        dx = target.x - current_position.x
        dy = target.y - current_position.y

        length = math.sqrt(dx * dx + dy * dy)

        if length == 0.0:
            return Vector3(x=1.0, y=0.0, z=0.0)

        return Vector3(
            x=dx / length,
            y=dy / length,
            z=0.0,
        )