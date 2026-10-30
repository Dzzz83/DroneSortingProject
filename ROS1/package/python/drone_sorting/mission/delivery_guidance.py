"""Extend camera pickup with one configured delivery destination, without ground truth."""

import math
from dataclasses import dataclass

from drone_sorting.mission.pickup_guidance import PickupGuidance, distance


@dataclass
class DeliverySettings:
    # Warehouse ROS coordinates: X/Y horizontal, Z vertical. Heights refer to drone root.
    dropoff_x: float = 12.0
    dropoff_y: float = 55.0
    release_height: float = 1.4
    carry_height: float = 6.0
    carry_speed: float = 0.5
    lowering_speed: float = 0.2
    travel_timeout: float = 120.0
    drop_timeout: float = 5.0
    settle_time: float = 2.0

    def __post_init__(self):
        if not all(math.isfinite(value) for value in vars(self).values()):
            raise ValueError("Delivery settings must be finite")
        positive = (self.release_height, self.carry_height, self.carry_speed,
                    self.lowering_speed, self.travel_timeout, self.drop_timeout, self.settle_time)
        if min(positive) <= 0 or self.carry_height <= self.release_height:
            raise ValueError("Delivery needs positive limits and carry height above release height")


@dataclass
class ObservationAccessSettings:
    # ROS warehouse coordinates: X/Y horizontal, Z vertical.
    observation_x: float = 34.0
    observation_y: float = 25.0
    observation_height: float = 6.0
    access_speed: float = 0.5
    access_timeout: float = 60.0
    hover_time: float = 1.0

    def __post_init__(self):
        if not all(math.isfinite(value) for value in vars(self).values()):
            raise ValueError("Observation access settings must be finite")
        if min(self.observation_height, self.access_speed, self.access_timeout, self.hover_time) <= 0:
            raise ValueError("Observation access limits must be positive")


class DeliveryGuidance(PickupGuidance):
    """Optionally fly to observation, then pick up, carry, and release one package."""

    DELIVERY_PHASES = ("CARRY_ASCEND", "CARRY", "LOWER", "WAIT_FOR_DROP", "SETTLE", "COMPLETE")

    def __init__(self, pickup_settings=None, delivery_settings=None, access_settings=None,
                 stop_after_pickup: bool = False):
        super().__init__(pickup_settings)
        self.stop_after_pickup = stop_after_pickup
        self.delivery = delivery_settings or DeliverySettings()
        self.carry_start = None
        self.access = access_settings
        self.ascent_target = None
        self.hover_started = None
        self.observation_ready_at = None
        if self.access is not None:
            self.phase = "ASCEND_TO_OBSERVATION"

    def observe(self, payload, now):
        if self.phase in ("ASCEND_TO_OBSERVATION", "GO_TO_OBSERVATION", "HOVER"):
            return
        if self.observation_ready_at is not None:
            stamp = payload.get("stamp", {})
            acquired = stamp.get("secs", 0) + stamp.get("nsecs", 0) / 1e9
            if acquired < self.observation_ready_at:
                self.invalidate_observation()
                return
        super().observe(payload, now)

    def access_command(self, position, now):
        settings = self.access
        if settings is None:
            raise ValueError("Observation access requires access settings")
        if self.phase_started is None:
            self.phase_started = now
        if self.ascent_target is None:
            self.ascent_target = (position[0], position[1], settings.observation_height)
        observation = (settings.observation_x, settings.observation_y, settings.observation_height)
        target = self.ascent_target if self.phase == "ASCEND_TO_OBSERVATION" else observation
        speed = settings.access_speed
        if now - self.phase_started > settings.access_timeout:
            self.transition("FAILED", now, "observation_access_timeout")
            target, speed = position, 0.0
        elif distance(position, target) <= self.settings.arrival_tolerance:
            speed = 0.0
            if self.phase == "ASCEND_TO_OBSERVATION":
                self.transition("GO_TO_OBSERVATION", now)
            elif self.phase == "GO_TO_OBSERVATION":
                self.transition("HOVER", now)
                self.hover_started = now
            elif self.hover_started is None:
                self.hover_started = now
            elif now - self.hover_started >= settings.hover_time:
                self.observation_ready_at = now
                self.transition("OBSERVE", now)
        elif self.phase == "HOVER":
            # Require continuous arrival tolerance before enabling package selection.
            self.hover_started = None
        return {"target": target, "speed": speed, "action": "", "phase": self.phase, "reason": self.reason}

    def acknowledge(self, status, now):
        if self.phase == "WAIT_FOR_DROP" and status == "DROP_DONE":
            self.transition("SETTLE", now)
        else:
            super().acknowledge(status, now)

    def tick(self, position, now):
        if self.phase in ("ASCEND_TO_OBSERVATION", "GO_TO_OBSERVATION", "HOVER"):
            return self.access_command(position, now)
        if self.phase not in self.DELIVERY_PHASES:
            command = super().tick(position, now)
            if self.phase != "VERIFIED":
                return command

            if self.stop_after_pickup:
                return self._hold_command(position)

            # Pickup has reached its lift target. Pause this tick, then ascend
            # vertically to carry height before allowing lateral travel.
            self.carry_start = (position[0], position[1], self.delivery.carry_height)
            self.transition("CARRY_ASCEND", now)
            return self._hold_command(position)

        return self.delivery_command(position, now)

    def _hold_command(self, position):
        return {"target": position, "speed": 0.0, "action": "",
                "phase": self.phase, "reason": self.reason}

    def delivery_command(self, position, now):
        settings = self.delivery
        elapsed = now - self.phase_started
        if self.phase == "COMPLETE":
            return self._hold_command(position)
        if self.phase == "SETTLE":
            # DROP_DONE confirms release, not placement. The physical evaluator checks settling.
            if elapsed >= settings.settle_time:
                self.transition("COMPLETE", now)
            return self._hold_command(position)
        if self.phase == "WAIT_FOR_DROP":
            if elapsed > settings.drop_timeout:
                self.transition("FAILED", now, "drop_timeout")
            return self._hold_command(position)
        if elapsed > settings.travel_timeout:
            self.transition("FAILED", now, "delivery_timeout")
            return self._hold_command(position)

        if self.phase == "CARRY_ASCEND":
            target = self.carry_start
            speed = settings.carry_speed
            next_phase = "CARRY"
        elif self.phase == "CARRY":
            target = (settings.dropoff_x, settings.dropoff_y, settings.carry_height)
            speed = settings.carry_speed
            next_phase = "LOWER"
        elif self.phase == "LOWER":
            target = (settings.dropoff_x, settings.dropoff_y, settings.release_height)
            speed = settings.lowering_speed
            next_phase = "WAIT_FOR_DROP"
        else:
            raise ValueError("Unexpected delivery phase: " + self.phase)

        action = ""
        if distance(position, target) <= self.settings.arrival_tolerance:
            self.transition(next_phase, now)
            target = position
            speed = 0.0
            if next_phase == "WAIT_FOR_DROP":
                action = "DROP"
        return {"target": target, "speed": speed, "action": action,
                "phase": self.phase, "reason": self.reason}
