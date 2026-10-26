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


class DepthNavigation:
    def __init__(self):
        self.follower = DepthPathFollower()
        self.reset_sequence = None
        self.mission_target = None
        self.contact_detected = False
        self.course_received = None
        self.local_goal = None
        self.last_goal_publish = -math.inf
        self.last_scan = -math.inf
        self.scan_index = 0
        self.last_hold = None
        self.goal_lookahead = float(rospy.get_param('~depth_goal_lookahead', 3.0))
        if not 0.5 <= self.goal_lookahead <= 5.0 or not math.isfinite(self.goal_lookahead):
            raise ValueError('depth_goal_lookahead must be between 0.5 and 5 m')
        self.goal_publisher = rospy.Publisher(topics.LOCAL_PLAN_GOAL, Point, queue_size=1)
        self.heading_publisher = rospy.Publisher(topics.LOCAL_PLAN_SCAN_HEADING, Vector3, queue_size=1)
        rospy.Subscriber(topics.LOCAL_PLAN_PATH, Path, self.receive_path, queue_size=1)
        rospy.Subscriber(topics.LOCAL_PLAN_STATUS, String, self.receive_status, queue_size=1)
        rospy.Subscriber(topics.COURSE_OBSERVATIONS, String, self.receive_course, queue_size=1)

    def receive_course(self, message):
        try:
            value = json.loads(message.data)['reset_sequence']
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
        try:
            self.contact_detected |= json.loads(message.data).get("collision_observed") is True
        except (ValueError, TypeError):
            pass

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
            self.follower.receive_status(json.loads(message.data))
        except (ValueError, TypeError):
            self.follower.receive_status(None)

    def _select_goal(self, observed, target):
        if abs(target[2] - observed[2]) > 0.125:
            return None
        distance = math.dist(observed[:2], target[:2])
        if distance <= 0.1:
            return target
        length = min(distance, self.goal_lookahead)
        return (observed[0] + (target[0]-observed[0])*length/distance,
                observed[1] + (target[1]-observed[1])*length/distance, observed[2])

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
        if abs(target[2]-observed[2]) > .125:
            return FollowDecision('HOLD', 'altitude_change_not_supported')
        if self.local_goal is not None and math.dist(observed, self.local_goal) <= .15:
            self.local_goal = None
            self.follower.reset()
        if self.local_goal is None:
            self.local_goal = self._select_goal(observed, target)
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
