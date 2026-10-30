"""ROS transport and scene startup for the standalone navigation demo."""
import json
import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from drone_sorting.interfaces import ros_topics as topics
from drone_sorting.navigation.guidance import NavigationGuidance


def _read_float_parameter(name: str, default: float) -> float:
    value = rospy.get_param("~" + name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Parameter ~" + name + " must be a number")
    return float(value)


class NavigationDemo:
    def __init__(self):
        self.guidance = NavigationGuidance(
            destination=rospy.get_param('~destination', [34., 119.5, 3.]),
            timeout=_read_float_parameter('timeout', 900.),
            stall_timeout=_read_float_parameter('stall_timeout', 90.),
            arrival_tolerance=_read_float_parameter('arrival_tolerance', .15),
        )
        reset_on_start = rospy.get_param('~reset_on_start', True)
        if not isinstance(reset_on_start, bool):
            raise ValueError("Parameter ~reset_on_start must be true or false")
        self.reset_on_start = reset_on_start
        self.reset_requested_sequence = None
        self.reset_sent_time = -math.inf
        self.target_publisher = rospy.Publisher(topics.TARGET_POSITION, Point, queue_size=1)
        self.status_publisher = rospy.Publisher(topics.NAVIGATION_STATUS, String, queue_size=1, latch=True)
        # This workflow has no package detector; avoid unused GPU capture/readback.
        self.package_camera_publisher = rospy.Publisher(
            topics.PACKAGE_CAMERA_ENABLED, Bool, queue_size=1, latch=True)
        self.package_camera_publisher.publish(Bool(False))
        self.reset_publisher = rospy.Publisher(topics.COURSE_RESET, String, queue_size=1)
        rospy.Subscriber(topics.CURRENT_POSITION, Point, self.receive_position, queue_size=1)
        rospy.Subscriber(topics.COURSE_OBSERVATIONS, String, self.receive_course, queue_size=1)

    def receive_position(self, message):
        self.guidance.receive_position((message.x, message.y, message.z), rospy.get_time())

    def receive_course(self, message):
        try:
            data = json.loads(message.data)
            sequence = data['reset_sequence']
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                return
        except (ValueError, KeyError, TypeError):
            return
        previous_sequence = self.guidance.reset_sequence
        self.guidance.receive_course(sequence, data.get('collision_observed') is True, rospy.get_time())
        if (previous_sequence is not None and previous_sequence != sequence
                and self.reset_requested_sequence is not None):
            self.reset_on_start = False
            self.reset_requested_sequence = None

    def tick(self, now):
        if (self.guidance.state not in ('ARRIVED', 'FAILED') and self.reset_on_start
                and self.guidance.reset_sequence is not None and now - self.reset_sent_time >= 1.):
            self.reset_requested_sequence = self.guidance.reset_sequence
            self.reset_sent_time = now
            self.reset_publisher.publish(String('RESET'))
        target, status = self.guidance.tick(now, waiting_for_reset=self.reset_on_start)
        if target is not None:
            self.target_publisher.publish(Point(*target))
        self.status_publisher.publish(String(json.dumps(status, allow_nan=False)))
        return status

    def run(self):
        rate = rospy.Rate(10)
        previous = None
        while not rospy.is_shutdown():
            status = self.tick(rospy.get_time())
            state = (status['state'], status['reason'])
            if state != previous:
                rospy.loginfo('Navigation %s: %s', *state)
                previous = state
            rate.sleep()
