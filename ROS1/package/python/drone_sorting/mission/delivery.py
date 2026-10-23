"""ROS transport for the full camera-guided warehouse delivery mission."""

import json
import math
import threading
from dataclasses import fields

import rospy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32, String

from drone_sorting.interfaces import ros_topics
from drone_sorting.mission.delivery_guidance import DeliveryGuidance, DeliverySettings, ObservationAccessSettings
from drone_sorting.mission.pickup_guidance import PickupSettings


class DeliveryMission:
    def __init__(self):
        settings = PickupSettings()
        for field in fields(settings):
            value = rospy.get_param("~" + field.name, getattr(settings, field.name))
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("Invalid pickup parameter: " + field.name)
            setattr(settings, field.name, value)
        delivery_defaults = DeliverySettings()
        delivery_values = {field.name: rospy.get_param("~" + field.name, getattr(delivery_defaults, field.name))
                           for field in fields(DeliverySettings)}
        access_defaults = ObservationAccessSettings()
        access_values = {field.name: rospy.get_param("~" + field.name, getattr(access_defaults, field.name))
                         for field in fields(ObservationAccessSettings)}
        self.guidance = DeliveryGuidance(settings, DeliverySettings(**delivery_values),
                                         ObservationAccessSettings(**access_values))
        self.lock = threading.Lock()
        self.position = None
        self.position_received = None
        self.position_timeout = 0.5
        self.last_action = ""
        self.last_phase = None
        self.go_to_pickup_published = False
        self.target = rospy.Publisher(ros_topics.TARGET_POSITION, Point, queue_size=1)
        self.heading = rospy.Publisher(ros_topics.TRAVEL_HEADING, Vector3, queue_size=1)
        self.speed = rospy.Publisher(ros_topics.COMMAND_SPEED, Float32, queue_size=1)
        self.action = rospy.Publisher(ros_topics.PACKAGE_ACTION, String, queue_size=1, latch=True)
        self.state = rospy.Publisher(ros_topics.MISSION_STATE, String, queue_size=1, latch=True)
        self.status = rospy.Publisher(ros_topics.PICKUP_TRIAL_STATUS, String, queue_size=1)
        rospy.Subscriber(ros_topics.CURRENT_POSITION, Point, self.receive_position, queue_size=1)
        rospy.Subscriber(ros_topics.PACKAGE_CANDIDATES, String, self.receive_candidates, queue_size=1)
        rospy.Subscriber(ros_topics.PACKAGE_ACTION_STATUS, String, self.receive_status, queue_size=1)
        self.action.publish(String(data=""))
        self.state.publish(String(data="IDLE"))

    def receive_position(self, message):
        position = (message.x, message.y, message.z)
        if not all(math.isfinite(value) for value in position):
            return
        with self.lock:
            self.position = position
            self.position_received = rospy.get_time()

    def receive_candidates(self, message):
        with self.lock:
            try:
                self.guidance.observe(json.loads(message.data), rospy.get_time())
            except (ValueError, TypeError, KeyError, AttributeError):
                self.guidance.invalidate_observation()

    def receive_status(self, message):
        with self.lock:
            self.guidance.acknowledge(message.data.strip().upper(), rospy.get_time())

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            command = None
            with self.lock:
                now = rospy.get_time()
                position = self.position
                if position is not None:
                    terminal = self.guidance.phase in ("FAILED", "VERIFIED", "COMPLETE")
                    fresh_position = 0 <= now - self.position_received <= self.position_timeout
                    if not terminal and not fresh_position:
                        self.guidance.transition("FAILED", now, "position_feedback_unavailable")
                    command = self.guidance.tick(position, now)
                    center = self.guidance.center
            if command is None:
                rate.sleep()
                continue
            phase = command["phase"]
            if phase != self.last_phase:
                rospy.loginfo("Camera pickup phase: %s (%s)", phase, command["reason"] or "")
                if phase in ("ASCEND_TO_OBSERVATION", "APPROACH"):
                    # Access and visual approach both belong to GO_TO_PICKUP.
                    if not self.go_to_pickup_published:
                        self.state.publish(String(data="GO_TO_PICKUP"))
                        self.go_to_pickup_published = True
                elif phase == "WAIT_FOR_PICKUP":
                    self.state.publish(String(data="WAIT_FOR_PICKUP"))
                elif phase == "CARRY_ASCEND":
                    self.state.publish(String(data="GO_TO_DROPOFF"))
                elif phase == "WAIT_FOR_DROP":
                    self.state.publish(String(data="WAIT_FOR_DROP"))
                elif phase == "COMPLETE":
                    self.state.publish(String(data="COMPLETE"))
                self.last_phase = phase
            self.speed.publish(Float32(data=command["speed"]))
            self.target.publish(Point(*command["target"]))
            delta = [command["target"][axis] - position[axis] for axis in (0, 1)]
            length = math.hypot(*delta)
            self.heading.publish(Vector3(delta[0] / length if length else 0, delta[1] / length if length else 0, 0))
            if command["action"] != self.last_action:
                self.action.publish(String(data=command["action"]))
                self.last_action = command["action"]
            self.status.publish(String(data=json.dumps({"phase": phase, "reason": command["reason"],
                "center": center, "target": command["target"], "speed": command["speed"],
                "stamp": now}, allow_nan=False)))
            rate.sleep()
