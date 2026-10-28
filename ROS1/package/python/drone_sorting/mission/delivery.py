"""ROS transport for the full camera-guided warehouse delivery mission."""

import json
import math
import threading

import rospy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32, String

from drone_sorting.interfaces import ros_topics
from drone_sorting.mission.delivery_guidance import (
    DeliveryGuidance,
    DeliverySettings,
    ObservationAccessSettings,
)
from drone_sorting.mission.pickup_guidance import PickupSettings


def _read_float_parameter(name: str, default: float, *, positive: bool = False) -> float:
    value = rospy.get_param("~" + name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Parameter ~" + name + " must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Parameter ~" + name + " must be finite")
    if positive and number <= 0:
        raise ValueError("Parameter ~" + name + " must be positive")
    return number


def _read_positive_integer_parameter(name: str, default: int) -> int:
    value = rospy.get_param("~" + name, default)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("Parameter ~" + name + " must be a positive integer")
    return value


def _load_pickup_settings() -> PickupSettings:
    defaults = PickupSettings()
    return PickupSettings(
        mount_offset=_read_float_parameter("mount_offset", defaults.mount_offset, positive=True),
        approach_clearance=_read_float_parameter("approach_clearance", defaults.approach_clearance, positive=True),
        lift_clearance=_read_float_parameter("lift_clearance", defaults.lift_clearance, positive=True),
        arrival_tolerance=_read_float_parameter("arrival_tolerance", defaults.arrival_tolerance, positive=True),
        association_radius=_read_float_parameter("association_radius", defaults.association_radius, positive=True),
        stability_tolerance=_read_float_parameter("stability_tolerance", defaults.stability_tolerance, positive=True),
        required_observations=_read_positive_integer_parameter("required_observations", defaults.required_observations),
        maximum_age=_read_float_parameter("maximum_age", defaults.maximum_age, positive=True),
        observation_timeout=_read_float_parameter("observation_timeout", defaults.observation_timeout, positive=True),
        loss_timeout=_read_float_parameter("loss_timeout", defaults.loss_timeout, positive=True),
        phase_timeout=_read_float_parameter("phase_timeout", defaults.phase_timeout, positive=True),
        pickup_timeout=_read_float_parameter("pickup_timeout", defaults.pickup_timeout, positive=True),
        approach_speed=_read_float_parameter("approach_speed", defaults.approach_speed, positive=True),
        descent_speed=_read_float_parameter("descent_speed", defaults.descent_speed, positive=True),
        lift_speed=_read_float_parameter("lift_speed", defaults.lift_speed, positive=True),
    )


def _load_delivery_settings() -> DeliverySettings:
    defaults = DeliverySettings()
    return DeliverySettings(
        dropoff_x=_read_float_parameter("dropoff_x", defaults.dropoff_x),
        dropoff_y=_read_float_parameter("dropoff_y", defaults.dropoff_y),
        release_height=_read_float_parameter("release_height", defaults.release_height),
        carry_height=_read_float_parameter("carry_height", defaults.carry_height),
        carry_speed=_read_float_parameter("carry_speed", defaults.carry_speed),
        lowering_speed=_read_float_parameter("lowering_speed", defaults.lowering_speed),
        travel_timeout=_read_float_parameter("travel_timeout", defaults.travel_timeout),
        drop_timeout=_read_float_parameter("drop_timeout", defaults.drop_timeout),
        settle_time=_read_float_parameter("settle_time", defaults.settle_time),
    )


def _load_access_settings() -> ObservationAccessSettings:
    defaults = ObservationAccessSettings()
    return ObservationAccessSettings(
        observation_x=_read_float_parameter("observation_x", defaults.observation_x),
        observation_y=_read_float_parameter("observation_y", defaults.observation_y),
        observation_height=_read_float_parameter("observation_height", defaults.observation_height),
        access_speed=_read_float_parameter("access_speed", defaults.access_speed),
        access_timeout=_read_float_parameter("access_timeout", defaults.access_timeout),
        hover_time=_read_float_parameter("hover_time", defaults.hover_time),
    )


def horizontal_heading(position, target):
    """Return a unit direction in ROS X/Y; vertical or zero travel has no heading."""
    delta_x = target[0] - position[0]
    delta_y = target[1] - position[1]
    horizontal_distance = math.hypot(delta_x, delta_y)
    if horizontal_distance > 0:
        return Vector3(delta_x / horizontal_distance, delta_y / horizontal_distance, 0)
    return Vector3(0, 0, 0)


class DeliveryMission:
    def __init__(self):
        self.guidance = DeliveryGuidance(
            pickup_settings=_load_pickup_settings(),
            delivery_settings=_load_delivery_settings(),
            access_settings=_load_access_settings(),
        )
        self.lock = threading.Lock()
        self.position = None
        self.position_received_time = None
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
            self.position_received_time = rospy.get_time()

    def receive_candidates(self, message):
        with self.lock:
            try:
                self.guidance.observe(json.loads(message.data), rospy.get_time())
            except (ValueError, TypeError, KeyError, AttributeError):
                self.guidance.invalidate_observation()

    def receive_status(self, message):
        with self.lock:
            self.guidance.acknowledge(message.data.strip().upper(), rospy.get_time())

    def report_phase(self, phase, reason):
        if phase == self.last_phase:
            return

        rospy.loginfo("Camera pickup phase: %s (%s)", phase, reason or "")
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

    def publish_command(self, command, position, center, now):
        self.speed.publish(Float32(data=command["speed"]))
        self.target.publish(Point(*command["target"]))

        heading = horizontal_heading(position, command["target"])
        self.heading.publish(heading)

        if command["action"] != self.last_action:
            self.action.publish(String(data=command["action"]))
            self.last_action = command["action"]

        status = {
            "phase": command["phase"],
            "reason": command["reason"],
            "center": center,
            "target": command["target"],
            "speed": command["speed"],
            "stamp": now,
        }
        self.status.publish(String(data=json.dumps(status, allow_nan=False)))

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            command = None
            with self.lock:
                now = rospy.get_time()
                position = self.position
                position_received_time = self.position_received_time
                if position is not None:
                    is_guidance_finished = self.guidance.phase in ("FAILED", "VERIFIED", "COMPLETE")
                    fresh_position = (
                        position_received_time is not None
                        and 0 <= now - position_received_time <= self.position_timeout
                    )
                    if not is_guidance_finished and not fresh_position:
                        self.guidance.transition("FAILED", now, "position_feedback_unavailable")
                    command = self.guidance.tick(position, now)
                center = self.guidance.center
            if command is None:
                rate.sleep()
                continue
            self.report_phase(command["phase"], command["reason"])
            self.publish_command(command, position, center, now)
            rate.sleep()
