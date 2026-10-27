"""Fail-closed, ordered short-segment protocol for the opt-in navigation demonstration.

The motion controller is the sole publisher. All coordinates use ROS warehouse axes.
Expiry is calculated from the already validated planner and observation deadlines.
"""
import json
import math
import time


class InsufficientTrajectoryTime(ValueError):
    """Wait for a fresh approval without cancelling already valid queued work."""


class ApprovedTrajectoryStream:
    MAX_TTL = 0.20
    PLAN_AGE = 0.35
    OBSERVATION_AGE = 0.50

    def __init__(self, clock=time.time):
        self.clock = clock
        self.reset(None)

    def reset(self, reset_sequence):
        self.reset_sequence = reset_sequence
        self.sequence = 0
        self.need_restart = True
        self.in_motion = False

    def _check_epoch(self, reset_sequence):
        if (isinstance(reset_sequence, bool) or not isinstance(reset_sequence, int)
                or reset_sequence < 0):
            raise ValueError("invalid reset identity")
        if self.reset_sequence != reset_sequence:
            self.reset(reset_sequence)

    def stop(self, reset_sequence, force=False):
        """Send a stop once when motion was active; do not flood the TCP queue."""
        self._check_epoch(reset_sequence)
        if not self.in_motion and not force:
            return None
        self.sequence += 1
        self.in_motion = False
        self.need_restart = True
        return json.dumps({"reset_sequence": reset_sequence, "sequence": self.sequence,
                           "stop": True}, separators=(",", ":"))

    def move(self, start, end, reset_sequence, speed, now, status, minimum_ttl=0.0):
        """Serialize exactly one previously approved segment; never skip corners."""
        self._check_epoch(reset_sequence)
        if not isinstance(status, dict):
            raise ValueError("missing planner approval")
        try:
            plan_stamp = float(status["stamp"])
            observation_stamp = float(status["observation_stamp"])
            numbers = (*start, *end, speed, now, plan_stamp, observation_stamp, minimum_ttl)
            if (len(start) != 3 or len(end) != 3 or
                    not all(math.isfinite(float(v)) for v in numbers) or
                    speed <= 0 or speed > 10 or minimum_ttl < 0 or
                    math.dist(start, end) > .250001 or
                    math.dist(start, end) <= 1e-9):
                raise ValueError("invalid motion segment")
        except (KeyError, TypeError, OverflowError) as error:
            raise ValueError("invalid motion approval") from error
        ttl = min(self.MAX_TTL, self.PLAN_AGE - (now - plan_stamp),
                  self.OBSERVATION_AGE - (now - observation_stamp))
        if now < plan_stamp or now < observation_stamp:
            raise ValueError("approval is from the future")
        if ttl < .001:
            raise ValueError("approval already expired")
        if ttl < minimum_ttl:
            raise InsufficientTrajectoryTime("approval cannot cover transport and queued execution")
        issued_ms = math.floor(self.clock() * 1000)
        expiry_ms = issued_ms + int(ttl * 1000)
        self.sequence += 1
        packet = {"reset_sequence": reset_sequence, "sequence": self.sequence,
                  "stop": False, "restart": self.need_restart,
                  "issued_ms": issued_ms, "expires_ms": expiry_ms,
                  "speed": speed,
                  "start": dict(zip("xyz", start)), "end": dict(zip("xyz", end))}
        self.in_motion = True
        self.need_restart = False
        return json.dumps(packet, separators=(",", ":"))
