"""Passive, ROS-independent gate for following a recently approved local path.

A proposal only grants motion ON the certified polyline, never a direct shortcut.
The ROS controller alone publishes movement. Any missing/mismatched data holds.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowDecision:
    state: str
    reason: str
    next_position: tuple = ()


class DepthPathFollower:
    def __init__(self, maximum_age=0.35, corridor_tolerance=0.05, maximum_pose_gap=0.35,
                 maximum_observation_age=None):
        maximum_observation_age = maximum_age if maximum_observation_age is None else maximum_observation_age
        if not all(math.isfinite(v) and v > 0 for v in
                   (maximum_age, corridor_tolerance, maximum_pose_gap, maximum_observation_age)):
            raise ValueError("Invalid depth follower limits")
        self.maximum_age = maximum_age
        self.maximum_observation_age = maximum_observation_age
        self.corridor_tolerance = corridor_tolerance
        self.maximum_pose_gap = maximum_pose_gap
        self.reset()

    def reset(self):
        self.path = None
        self.path_stamp = None
        self.status = None

    def receive_path(self, points, stamp):
        if (not math.isfinite(stamp) or len(points) < 2 or len(points) > 500
                or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points)):
            self.path, self.path_stamp = None, None
            return
        self.path = tuple(tuple(p) for p in points)
        self.path_stamp = stamp

    def receive_status(self, data):
        if not isinstance(data, dict):
            self.status = None
            return
        self.status = data

    @staticmethod
    def _on_segment(point, a, b):
        delta = tuple(b[i] - a[i] for i in range(3))
        length_sq = sum(d*d for d in delta)
        if length_sq <= 1e-15:
            return math.dist(point, a), 0.0, 0.0
        t = max(0.0, min(1.0, sum((point[i]-a[i])*delta[i] for i in range(3)) / length_sq))
        projected = tuple(a[i]+t*delta[i] for i in range(3))
        return math.dist(point, projected), t, math.sqrt(length_sq)

    def next_step(self, command, observed, goal, reset_sequence, now, distance):
        """Return one short, on-path position or HOLD; does NOT publish anything."""
        status = self.status
        if self.path is None or status is None:
            return FollowDecision("HOLD", "awaiting_approved_path")
        if status.get("state") != "READY":
            return FollowDecision("HOLD", "planner_" + str(status.get("reason", "unavailable")))
        try:
            stamp = float(status["stamp"])
            acquisition = float(status["observation_stamp"])
            expected_goal = tuple(status["goal"])
            sequence = status["reset_sequence"]
            values = (*command, *observed, *goal, stamp, acquisition, now, distance)
            if (len(expected_goal) != 3 or not all(math.isfinite(v) for v in (*values, *expected_goal))
                    or distance <= 0):
                raise ValueError("invalid numerical fields")
        except (KeyError, TypeError, ValueError, OverflowError):
            return FollowDecision("HOLD", "invalid_planner_status")
        if (reset_sequence is None or sequence != reset_sequence
                or not math.isclose(stamp, self.path_stamp, rel_tol=0, abs_tol=1e-5)):
            return FollowDecision("HOLD", "route_epoch_mismatch")
        if not 0 <= now-stamp <= self.maximum_age:
            return FollowDecision("HOLD", "route_plan_stale")
        if not 0 <= now-acquisition <= self.maximum_observation_age:
            return FollowDecision("HOLD", "route_observation_stale")
        if any(abs(a-b) > 1e-5 for a, b in zip(goal, expected_goal)):
            return FollowDecision("HOLD", "route_goal_mismatch")
        if any(abs(a-b) > 1e-5 for a, b in zip(goal, self.path[-1])):
            return FollowDecision("HOLD", "route_endpoint_mismatch")
        if math.dist(observed, self.path[0]) > self.maximum_pose_gap:
            return FollowDecision("HOLD", "route_start_mismatch")
        # Do not rejoin a new route across unobserved space. The command must be
        # on one of the certified segments, with no off-polyline shortcut.
        # A nearby earlier segment must not hide a later exact match. Never
        # move sideways to join: the full commanded step stays on the polyline.
        for index in range(len(self.path)-1):
            a, b = self.path[index:index+2]
            lateral, progress, length = self._on_segment(command, a, b)
            if lateral > 1e-5:
                continue
            if length < 1e-9:
                continue
            if progress >= 1 - 1e-8:
                continue
            step = min(distance, (1-progress)*length)
            t = progress + step / length
            candidate = tuple(a[i]+t*(b[i]-a[i]) for i in range(3))
            return FollowDecision("MOVE", "approved_segment", candidate)
        if math.dist(command, self.path[-1]) <= self.corridor_tolerance:
            return FollowDecision("HOLD", "local_goal_reached")
        return FollowDecision("HOLD", "command_not_on_approved_path")
