"""One-package guidance, independent of ROS transport and Unity ground truth."""

import math
from dataclasses import dataclass


def distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


@dataclass
class PickupSettings:
    mount_offset: float = 0.95
    approach_clearance: float = 1.4
    lift_clearance: float = 1.0
    arrival_tolerance: float = 0.03
    association_radius: float = 0.25
    stability_tolerance: float = 0.04
    required_observations: int = 3
    maximum_age: float = 1.0
    observation_timeout: float = 15.0
    loss_timeout: float = 3.0
    phase_timeout: float = 35.0
    pickup_timeout: float = 5.0
    approach_speed: float = 0.5
    descent_speed: float = 0.2
    lift_speed: float = 0.3


class PickupGuidance:
    def __init__(self, settings=None):
        self.settings = settings or PickupSettings()
        self.phase = "OBSERVE"
        self.phase_started = None
        self.center = None
        self.stable_count = 0
        self.last_stamp = None
        self.last_seen = None
        self.last_refined = None
        self.observation_valid = False
        self.refinement_count = 0
        self.at_approach = False
        self.reason = None

    def invalidate_observation(self):
        # Keep the package lock and loss timer, but require confirmation again.
        self.observation_valid = False
        self.stable_count = 0
        self.refinement_count = 0

    def observe(self, payload, now):
        if self.phase not in ("OBSERVE", "APPROACH", "DESCEND"):
            return  # Freeze the selected grasp after pickup; transport needs position feedback.
        self.observation_valid = False
        if payload.get("state") != "DETECTIONS" or payload.get("frame_id") != "warehouse":
            self.invalidate_observation()
            return
        stamp = payload.get("stamp")
        if not isinstance(stamp, dict):
            self.invalidate_observation()
            return
        acquired = stamp["secs"] + stamp["nsecs"] / 1e9
        if not 0 <= now - acquired <= self.settings.maximum_age:
            self.invalidate_observation()
            return
        if self.last_stamp is not None and acquired <= self.last_stamp:
            self.invalidate_observation()
            return
        if self.last_seen is not None and acquired - self.last_seen > self.settings.maximum_age:
            self.invalidate_observation()
        self.last_stamp = acquired
        candidates = payload.get("candidates", [])
        ready = [candidate["grasp"]["center"] for candidate in candidates
                 if candidate.get("grasp", {}).get("ready")]
        ready = [center for center in ready if len(center) == 3 and all(math.isfinite(v) for v in center)]
        if self.center is None:
            if not ready:
                self.invalidate_observation()
                return
            # Deterministic initial selection; the locked package cannot be replaced by another.
            selected = min(ready, key=lambda center: (center[0], center[1]))
        else:
            matching = [center for center in ready if distance(center, self.center) <= self.settings.association_radius]
            selected = min(matching, key=lambda center: distance(center, self.center)) if matching else None
        if selected is not None:
            consistent = self.center is not None and distance(selected, self.center) <= self.settings.stability_tolerance
            self.stable_count = self.stable_count + 1 if consistent else 1
            # Freeze the refined center during descent; surface points may remain visible when the top is clipped.
            if self.phase != "DESCEND":
                self.center = tuple(selected)
            if self.at_approach:
                self.refinement_count = self.refinement_count + 1 if consistent else 1
            self.last_refined = acquired
            self.observation_valid = True
        elif self.phase == "DESCEND" and self.center is not None:
            # Continue only while a fresh surface still belongs to the locked box.
            self.observation_valid = any(distance(candidate["position"], self.center) <= 0.6 for candidate in candidates)
        if self.observation_valid:
            self.last_seen = acquired
        else:
            self.invalidate_observation()

    def acknowledge(self, status, now):
        if self.phase == "WAIT_FOR_PICKUP" and status == "PICKUP_DONE":
            self.transition("LIFT", now)

    def transition(self, phase, now, reason=None):
        self.phase, self.phase_started, self.reason = phase, now, reason

    def tick(self, position, now):
        if self.phase_started is None:
            self.phase_started = now
        settings = self.settings
        if self.last_seen is not None and now - self.last_seen > settings.maximum_age:
            self.invalidate_observation()
        elapsed = now - self.phase_started
        target, speed, action = position, 0.0, ""
        if self.phase in ("FAILED", "VERIFIED"):
            return {"target": target, "speed": speed, "action": action, "phase": self.phase, "reason": self.reason}
        limit = settings.observation_timeout if self.phase == "OBSERVE" else settings.pickup_timeout if self.phase == "WAIT_FOR_PICKUP" else settings.phase_timeout
        if elapsed > limit:
            self.transition("FAILED", now, "phase_timeout")
        elif self.phase == "OBSERVE":
            if self.observation_valid and self.last_seen is not None and now - self.last_seen <= settings.maximum_age and self.stable_count >= settings.required_observations:
                self.transition("APPROACH", now)
        elif self.phase in ("APPROACH", "DESCEND"):
            if self.center is None:
                self.transition("FAILED", now, "missing_package_center")
                return {"target": target, "speed": speed, "action": action, "phase": self.phase, "reason": self.reason}
            fresh = self.observation_valid and self.last_seen is not None and now - self.last_seen <= settings.maximum_age
            if not fresh:
                if self.last_seen is None or now - self.last_seen > settings.loss_timeout:
                    self.transition("FAILED", now, "lost_locked_package")
            else:
                root = (self.center[0], self.center[1], self.center[2] + settings.mount_offset)
                if self.phase == "APPROACH":
                    speed = settings.approach_speed
                    target = (root[0], root[1], root[2] + settings.approach_clearance)
                    if distance(position, target) <= settings.arrival_tolerance:
                        self.at_approach = True
                        if self.refinement_count >= settings.required_observations:
                            self.transition("DESCEND", now)
                else:
                    target, speed = root, settings.descent_speed
                    if distance(position, target) <= settings.arrival_tolerance:
                        self.transition("WAIT_FOR_PICKUP", now)
                        action = "PICKUP"
        elif self.phase == "LIFT":
            if self.center is None:
                self.transition("FAILED", now, "missing_package_center")
                return {"target": target, "speed": speed, "action": action, "phase": self.phase, "reason": self.reason}
            target = (self.center[0], self.center[1], self.center[2] + settings.mount_offset + settings.lift_clearance)
            speed = settings.lift_speed
            if distance(position, target) <= settings.arrival_tolerance:
                self.transition("VERIFIED", now)
        return {"target": target, "speed": speed, "action": action, "phase": self.phase, "reason": self.reason}
