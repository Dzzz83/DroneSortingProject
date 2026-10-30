"""ROS transport for one opt-in depth-control mode; no direct Unity access.

The motion controller remains the single movement-command publisher. The map node
remains the single Unity forward-camera heading publisher.
"""
import json
import math

import rospy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import String

from drone_sorting.control.depth_follower import DepthPathFollower, FollowDecision
from drone_sorting.interfaces import ros_topics as topics


def _read_float_parameter(name: str, default: float) -> float:
    value = rospy.get_param("~" + name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Parameter ~" + name + " must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Parameter ~" + name + " must be finite")
    return number


class DepthNavigation:
    def __init__(self):
        self.follower = DepthPathFollower(maximum_observation_age=
            _read_float_parameter('depth_maximum_observation_age', .5))
        self.reset_sequence = None
        self.package_attached = False
        self.package_received_time = None
        self.mission_target = None
        self.contact_detected = False
        self.course_received = None
        self.local_goal = None
        self.last_goal_publish = -math.inf
        self.last_scan = -math.inf
        self.scan_index = 0
        self.last_hold = None
        self.autonomous_goals = rospy.get_param('~depth_autonomous_goals', False)
        self.allow_vertical = rospy.get_param('~depth_allow_vertical', False)
        self.goal_lookahead = _read_float_parameter('depth_goal_lookahead', 3.0)
        if not 0.5 <= self.goal_lookahead <= 5.0 or not math.isfinite(self.goal_lookahead):
            raise ValueError('depth_goal_lookahead must be between 0.5 and 5 m')
        self.goal_publisher = rospy.Publisher(topics.LOCAL_PLAN_GOAL, Point, queue_size=1)
        self.heading_publisher = rospy.Publisher(topics.LOCAL_PLAN_SCAN_HEADING, Vector3, queue_size=1)
        rospy.Subscriber(topics.LOCAL_PLAN_PATH, Path, self.receive_path, queue_size=1)
        rospy.Subscriber(topics.LOCAL_PLAN_STATUS, String, self.receive_status, queue_size=1)
        rospy.Subscriber(topics.COURSE_OBSERVATIONS, String, self.receive_course, queue_size=1)

    def receive_course(self, message):
        try:
            observation = json.loads(message.data)
            value = observation['reset_sequence']
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                return
        except (ValueError, TypeError, KeyError):
            return
        if self.reset_sequence is not None and self.reset_sequence != value:
            self.contact_detected = False
            self.local_goal = None
            self.mission_target = None
            self.follower.reset()
        self.reset_sequence = value
        self.course_received = rospy.get_time()
        self.contact_detected |= observation.get("collision_observed") is True
        attached = observation.get('package_attached')
        self.package_received_time = rospy.get_time() if isinstance(attached, bool) else None
        if isinstance(attached, bool) and attached != self.package_attached:
            self.package_attached = attached
            self.follower.reset()

    def receive_path(self, message):
        if message.header.frame_id != 'warehouse' or any(p.header.frame_id != 'warehouse'
                                                          for p in message.poses):
            self.follower.receive_path((), math.nan)
            return
        points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                  for p in message.poses]
        self.follower.receive_path(points, message.header.stamp.to_sec())

    def receive_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict) or status.get('package_attached', False) != self.package_attached:
                self.follower.receive_status(None)
                return
            if self.autonomous_goals:
                requested = status.get('request_goal')
                selected = status.get('goal')
                if (requested is None or tuple(requested) != self.mission_target
                        or status.get('reset_sequence') != self.reset_sequence):
                    self.follower.receive_status(None)
                    return
                if selected is not None:
                    if len(selected) != 3 or not all(math.isfinite(v) for v in selected):
                        raise ValueError('invalid selected goal')
                    self.local_goal = tuple(selected)
            self.follower.receive_status(status)
        except (ValueError, TypeError):
            self.follower.receive_status(None)

    def _select_goal(self, observed, target):
        if not self.allow_vertical and abs(target[2] - observed[2]) > 0.125:
            return None
        distance = math.dist(observed, target)
        if distance <= self.goal_lookahead:
            return target
        length = min(distance, self.goal_lookahead)
        return (observed[0] + (target[0]-observed[0])*length/distance,
                observed[1] + (target[1]-observed[1])*length/distance,
                observed[2] + (target[2]-observed[2])*length/distance)

    def _camera_direction(self, now, target, observed, hold_reason):
        dx,dy = target[0]-observed[0], target[1]-observed[1]
        length = math.hypot(dx,dy)
        if length < 1e-6:
            return
        forward = (dx/length,dy/length)
        scanning = hold_reason in ('planner_no_observed_clear_route',
                                   'planner_goal_envelope_not_observed_free',
                                   'planner_start_envelope_not_observed_free',
                                   'awaiting_approved_path')
        if now-self.last_scan < (0.45 if scanning else 0.8):
            return
        self.last_scan = now
        if scanning:
            # +-60-degree yaw sweep: the map keeps a short history of views.
            angle = (0, math.pi/3, -math.pi/3)[self.scan_index % 3]
            self.scan_index += 1
            cosine,sine = math.cos(angle),math.sin(angle)
            direction = (cosine*forward[0]-sine*forward[1],
                         sine*forward[0]+cosine*forward[1])
        else:
            direction = forward
        self.heading_publisher.publish(Vector3(direction[0],direction[1],0))

    def tick(self, command, observed, target, now, step):
        if self.contact_detected:
            return FollowDecision("HOLD", "contact_detected")
        if self.reset_sequence is None:
            return FollowDecision('HOLD', 'awaiting_course_reset_sequence')
        if self.course_received is None or not 0 <= now-self.course_received <= .5:
            return FollowDecision('HOLD', 'course_observations_stale')
        if self.mission_target != target:
            self.mission_target = target
            self.local_goal = None
            self.follower.reset()
        if not self.allow_vertical and abs(target[2]-observed[2]) > .125:
            return FollowDecision('HOLD', 'altitude_change_not_supported')
        if self.autonomous_goals:
            if now-self.last_goal_publish >= .4:
                self.goal_publisher.publish(Point(*target))
                self.last_goal_publish = now
            result = (self.follower.next_step(command, observed, self.local_goal,
                       self.reset_sequence, now, step) if self.local_goal is not None
                      else FollowDecision('HOLD', 'awaiting_approved_path'))
            self._camera_direction(now, target, observed, result.reason)
            return result
        # Only intermediate goals hand off early. Revoking the final route
        # here would prevent the remaining 15 cm of accurate arrival.
        if (self.local_goal is not None and self.local_goal != target
                and math.dist(observed, self.local_goal) <= .15):
            self.local_goal = None
            self.follower.reset()
        if self.local_goal is None:
            self.local_goal = self._select_goal(observed, target)
            self.last_goal_publish = -math.inf
        if self.local_goal is None:
            return FollowDecision('HOLD','no_valid_local_goal')
        if now-self.last_goal_publish >= .4:
            self.goal_publisher.publish(Point(*self.local_goal))
            self.last_goal_publish = now
        result = self.follower.next_step(command, observed, self.local_goal,
                                         self.reset_sequence, now, step)
        self._camera_direction(now, target, observed, result.reason if result.state=='HOLD' else None)
        self.last_hold = result.reason if result.state=='HOLD' else None
        return result
