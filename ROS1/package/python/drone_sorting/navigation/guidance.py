"""Destination progress checks without ROS or scene startup side effects."""

import math


class NavigationGuidance:
    """Request a destination while physical feedback permits continued travel.

    Coordinates are ROS X/Y horizontal and Z up. All times are seconds.
    Obstacle planning and movement remain the motion controller's responsibility.
    """

    def __init__(self, destination, timeout=900.0, stall_timeout=90.0,
                 arrival_tolerance=0.15):
        self.destination = tuple(destination)
        self.timeout = timeout
        self.stall_timeout = stall_timeout
        self.arrival_tolerance = arrival_tolerance
        if (len(self.destination) != 3
                or not all(math.isfinite(value) for value in self.destination)
                or not all(math.isfinite(value) and value > 0 for value in
                           (timeout, stall_timeout, arrival_tolerance))):
            raise ValueError("Invalid navigation destination or limits")
        self.reset_sequence = None
        self.position = None
        self.position_received_time = None
        self.course_received_time = None
        self.has_collision = False
        self.started_time = None
        self.progress_time = None
        self.arrival_started_time = None
        self.progress_position = None
        self.state = "WAITING"
        self.reason = "waiting_for_unity"

    def receive_position(self, position, now):
        if all(math.isfinite(value) for value in position):
            self.position = tuple(position)
            self.position_received_time = now

    def receive_course(self, reset_sequence, has_collision, now):
        if self.reset_sequence is not None and self.reset_sequence != reset_sequence:
            # A new course run requires fresh position and progress measurements.
            self.position = None
            self.position_received_time = None
            self.started_time = None
            self.progress_time = None
            self.arrival_started_time = None
            self.progress_position = None
            self.state = "WAITING"
            self.reason = "course_reset"
            self.has_collision = False
        self.reset_sequence = reset_sequence
        self.course_received_time = now
        self.has_collision |= has_collision

    def tick(self, now, *, waiting_for_reset=False):
        """Return (requested target or None, status); never publish commands.

        None means no new target is permitted. The controller retains its
        existing command expiry behavior when feedback or navigation fails.
        """
        position_time = self.position_received_time
        course_time = self.course_received_time
        has_fresh_feedback = (
            position_time is not None and course_time is not None
            and 0 <= now - position_time <= 0.5
            and 0 <= now - course_time <= 0.5
        )
        target = None
        if self.state in ("ARRIVED", "FAILED"):
            pass
        elif waiting_for_reset:
            self.reason = "waiting_for_course_reset"
        elif self.has_collision:
            self.state, self.reason = "FAILED", "contact_detected"
        elif not has_fresh_feedback:
            self.arrival_started_time = None
            self.reason = "waiting_for_fresh_unity_feedback"
            if (self.started_time is not None and position_time is not None
                    and course_time is not None
                    and now - min(position_time, course_time) > 10.0):
                self.state, self.reason = "FAILED", "unity_feedback_lost"
        elif self.position is not None:
            if self.started_time is None:
                self.started_time = self.progress_time = now
                self.progress_position = self.position
            if (self.progress_position is not None
                    and math.dist(self.progress_position, self.position) >= 0.5):
                self.progress_position = self.position
                self.progress_time = now
            distance_to_destination = math.dist(self.position, self.destination)
            if distance_to_destination <= self.arrival_tolerance:
                if self.arrival_started_time is None:
                    self.arrival_started_time = now
                if now - self.arrival_started_time >= 1.0:
                    self.state, self.reason = "ARRIVED", "destination_reached"
            else:
                self.arrival_started_time = None
            if self.state != "ARRIVED":
                self.state, self.reason = "NAVIGATING", "requesting_destination"
                if now - self.started_time >= self.timeout:
                    self.state, self.reason = "FAILED", "mission_timeout"
                elif self.progress_time is not None and now - self.progress_time >= self.stall_timeout:
                    self.state, self.reason = "FAILED", "no_navigation_progress"
                else:
                    target = self.destination
        status = dict(
            state=self.state, reason=self.reason, destination=self.destination,
            position=self.position, reset_sequence=self.reset_sequence,
            elapsed=None if self.started_time is None else now - self.started_time,
            distance=None if self.position is None else math.dist(self.position, self.destination),
        )
        return target, status
