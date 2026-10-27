"""Warehouse destination coordinator. Coordinates are ROS X/Y horizontal, Z up."""
import json
import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from drone_sorting.interfaces import ros_topics as topics


class NavigationDemo:
    def __init__(self):
        self.destination = tuple(rospy.get_param('~destination', [34.,119.5,3.]))
        self.timeout = float(rospy.get_param('~timeout', 900.))
        self.stall_timeout = float(rospy.get_param('~stall_timeout', 90.))
        self.tolerance = float(rospy.get_param('~arrival_tolerance', .15))
        if (len(self.destination) != 3 or not all(math.isfinite(v) for v in self.destination)
                or not all(math.isfinite(v) and v > 0 for v in
                           (self.timeout,self.stall_timeout,self.tolerance))):
            raise ValueError('Invalid navigation destination or limits')
        self.reset_on_start = rospy.get_param('~reset_on_start', True)
        self.reset_requested = None
        self.reset_sent = -math.inf
        self.sequence = None
        self.position = None
        self.position_received = None
        self.course_received = None
        self.collision = False
        self.started = self.progress_time = self.arrival_since = None
        self.anchor = None
        self.state, self.reason = 'WAITING', 'waiting_for_unity'
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
        position = (message.x,message.y,message.z)
        if all(math.isfinite(v) for v in position):
            self.position = position
            self.position_received = rospy.get_time()

    def receive_course(self, message):
        try:
            data = json.loads(message.data)
            sequence = data['reset_sequence']
            if isinstance(sequence,bool) or not isinstance(sequence,int) or sequence < 0:
                return
        except (ValueError,KeyError,TypeError):
            return
        if self.sequence is not None and self.sequence != sequence:
            self.position = self.position_received = None
            self.started = self.progress_time = self.arrival_since = self.anchor = None
            self.state, self.reason = 'WAITING', 'course_reset'
            self.collision = False
            if self.reset_requested is not None:
                self.reset_on_start = False
                self.reset_requested = None
        self.sequence = sequence
        self.course_received = rospy.get_time()
        self.collision |= data.get('collision_observed') is True

    def tick(self, now):
        fresh = (self.position_received is not None and self.course_received is not None
                 and 0 <= now-self.position_received <= .5
                 and 0 <= now-self.course_received <= .5)
        if self.state in ('ARRIVED','FAILED'):
            pass
        elif self.reset_on_start:
            self.reason = 'waiting_for_course_reset'
            if self.sequence is not None and now-self.reset_sent >= 1.:
                self.reset_requested = self.sequence
                self.reset_sent = now
                self.reset_publisher.publish(String('RESET'))
        elif self.collision:
            self.state,self.reason = 'FAILED','contact_detected'
        elif not fresh:
            self.arrival_since = None
            self.reason = 'waiting_for_fresh_unity_feedback'
            if (self.started is not None and self.position_received is not None
                    and self.course_received is not None
                    and now-min(self.position_received,self.course_received) > 10.):
                self.state,self.reason = 'FAILED','unity_feedback_lost'
        else:
            if self.started is None:
                self.started = self.progress_time = now
                self.anchor = self.position
            if math.dist(self.anchor,self.position) >= .5:
                self.anchor,self.progress_time = self.position,now
            error = math.dist(self.position,self.destination)
            if error <= self.tolerance:
                if self.arrival_since is None:
                    self.arrival_since = now
                if now-self.arrival_since >= 1.:
                    self.state,self.reason = 'ARRIVED','destination_reached'
            else:
                self.arrival_since = None
            if self.state != 'ARRIVED':
                self.state,self.reason = 'NAVIGATING','requesting_destination'
                if now-self.started >= self.timeout:
                    self.state,self.reason = 'FAILED','mission_timeout'
                elif now-self.progress_time >= self.stall_timeout:
                    self.state,self.reason = 'FAILED','no_navigation_progress'
                else:
                    self.target_publisher.publish(Point(*self.destination))
        data = dict(state=self.state,reason=self.reason,destination=self.destination,
                    position=self.position,reset_sequence=self.sequence,
                    elapsed=None if self.started is None else now-self.started,
                    distance=None if self.position is None else math.dist(self.position,self.destination))
        self.status_publisher.publish(String(json.dumps(data,allow_nan=False)))
        return data

    def run(self):
        rate = rospy.Rate(10)
        previous = None
        while not rospy.is_shutdown():
            status = self.tick(rospy.get_time())
            state = (status['state'],status['reason'])
            if state != previous:
                rospy.loginfo('Navigation %s: %s', *state)
                previous = state
            rate.sleep()
