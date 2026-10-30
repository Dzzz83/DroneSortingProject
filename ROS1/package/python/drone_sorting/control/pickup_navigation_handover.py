"""Pickup-to-navigation decisions, independent of ROS and movement execution."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class HandoverDecision:
    action: str
    reason: str = ""
    request_stop: bool = False


class PickupNavigationHandover:
    """Allow pickup, then require a confirmed stop before allowing navigation.

    Timestamps and durations are seconds. The controller validates STOP identity
    and freshness; this class never reads ROS messages or publishes commands.
    """

    def __init__(self):
        self.state = "PICKUP"
        self.started_time: Optional[float] = None
        self.phase = ""
        self.phase_received_time: Optional[float] = None
        self.has_verified_pickup = False
        self.failure_reason = ""

    def receive_phase(self, phase, now):
        self.phase = phase
        self.phase_received_time = now
        if phase == "VERIFIED":
            self.has_verified_pickup = True

    def receive_reset(self):
        self.state = "FAILED"
        self.failure_reason = "course_reset_during_mission"

    def confirm_stop(self):
        """Called only after the controller validates Unity's STOP acknowledgement."""
        if self.state == "STOPPING":
            self.state = "NAVIGATING"

    @staticmethod
    def _is_stale(received_time, now):
        return received_time is None or not 0 <= now - received_time <= 0.5

    def _fail(self, reason, *, request_stop=True):
        self.state = "FAILED"
        self.failure_reason = reason
        return HandoverDecision("FAIL", reason, request_stop)

    def tick(self, now, *, has_collision, package_attached,
             package_received_time, course_received_time):
        if self.state == "FAILED":
            return HandoverDecision("FAIL", self.failure_reason, request_stop=True)
        if self.phase_received_time is None:
            return HandoverDecision("HOLD")
        if (self._is_stale(self.phase_received_time, now)
                or has_collision or self.phase == "FAILED"):
            return self._fail("stale_mission_status_collision_or_mission_failure")
        if self.state == "PICKUP":
            if self.phase == "NAVIGATE_TO_DROPOFF":
                if not self.has_verified_pickup:
                    # Preserve the rejected request's stopped behavior; subsequent
                    # updates also request cancellation of pending trajectories.
                    return self._fail("pickup_not_verified", request_stop=False)
                self.state = "STOPPING"
                self.started_time = now
                return HandoverDecision("START_STOP", request_stop=True)
            if self.phase in (
                "ASCEND_TO_OBSERVATION", "GO_TO_OBSERVATION", "HOVER", "OBSERVE",
                "APPROACH", "DESCEND", "WAIT_FOR_PICKUP", "LIFT",
            ):
                return HandoverDecision("ALLOW_PICKUP")
            return HandoverDecision("HOLD")
        if (package_attached is not True
                or self._is_stale(package_received_time, now)
                or self._is_stale(course_received_time, now)
                or self.phase not in ("NAVIGATE_TO_DROPOFF", "HOLD_AT_DROPOFF")):
            return self._fail("invalid_package_course_feedback_or_mission_phase")
        if self.state == "STOPPING":
            if self.started_time is not None and now - self.started_time > 5.0:
                return self._fail("stop_acknowledgement_timeout")
            return HandoverDecision("CHECK_STOP")
        return HandoverDecision("ALLOW_NAVIGATION")
