"""Synthetic delivery evidence: terminal status alone must never prove physical placement."""

import copy
import unittest

from evaluate_camera_pickup import evaluate, evaluate_access


class DeliveryEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.manifest = {"mission": "delivery", "started": 100, "timeout": 240, "lift_distance": 1,
                         "tolerance": 0.05, "hold_grace": 0.5,
                         "zone_min": [25.5, 52.5, 0.08], "zone_max": [30.5, 57.5, 0.12],
                         "settled_center_height": 0.3}
        self.events = [{"kind": "action", "received": 101, "payload": "PICKUP"},
                       {"kind": "acknowledgement", "received": 102, "payload": "PICKUP_DONE"},
                       {"kind": "guidance", "received": 102.1, "payload": {"phase": "LIFT"}},
                       {"kind": "guidance", "received": 103, "payload": {"phase": "CARRY_ASCEND"}},
                       {"kind": "action", "received": 110, "payload": "DROP"},
                       {"kind": "acknowledgement", "received": 110.1, "payload": "DROP_DONE"}]
        self.samples = [self.sample(101.9, "WAIT_FOR_PICKUP", [32.8, 24, 0.3]),
                        self.sample(102.2, "LIFT", [32.8, 24, 0.4]),
                        self.sample(103, "CARRY_ASCEND", [32.8, 24, 1.3]),
                        self.sample(104, "CARRY", [32.8, 24, 5.05]),
                        self.sample(105, "CARRY", [32, 25, 5.05]),
                        self.sample(109.9, "LOWER", [28, 55, 0.45])]
        for now in (113, 113.3, 113.6):
            sample = self.sample(now, "COMPLETE", [28, 55, 0.3])
            sample.update(attached=False, released=True, held_package="", holding=True)
            sample["drone_position"] = [28, 55, 1.4]
            self.samples.append(sample)

    @staticmethod
    def sample(stamp, phase, position):
        return {"stamp": stamp, "status_age": 0.01, "phase": phase, "selected_package": "package-1",
                "held_package": "package-1", "attached": True, "released": False, "failure": "", "holding": False,
                "drone_position": [position[0], position[1], position[2] + 0.95], "package_position": position,
                "package_bounds_min": [value - 0.3 for value in position],
                "package_bounds_max": [value + 0.3 for value in position],
                "linear_speed": 0, "angular_speed": 0, "collisions": []}

    def report(self):
        return evaluate(self.manifest, self.samples, self.events)

    def test_same_package_carried_released_and_settled_inside_zone(self):
        self.assertTrue(self.report()["passed"], self.report())

    def test_lost_attachment_before_release_fails(self):
        self.samples[4]["attached"] = False
        self.assertIn("attachment_lost", self.report()["failures"])

    def test_package_must_follow_throughout_carry(self):
        self.samples[4]["package_position"][0] += 0.1
        self.assertIn("package_did_not_follow_during_carry", self.report()["failures"])

    def test_missing_duplicate_and_early_drop_acknowledgements_fail(self):
        ack = self.events.pop()
        self.assertIn("expected_one_drop_acknowledgement", self.report()["failures"])
        ack["received"] = 109
        self.events.append(ack)
        self.assertIn("drop_acknowledgement_before_request", self.report()["failures"])
        self.events.append(copy.deepcopy(ack))
        self.assertIn("expected_one_drop_acknowledgement", self.report()["failures"])

    def test_acknowledgement_does_not_prove_release_or_placement(self):
        for field, value in (("released", False), ("attached", True), ("linear_speed", 0.2), ("angular_speed", 1)):
            with self.subTest(field=field):
                sample = self.samples[-1]
                previous = sample[field]
                sample[field] = value
                self.assertIn("invalid_final_package_placement", self.report()["failures"])
                sample[field] = previous
        self.samples[-1]["package_bounds_max"][0] = 31
        self.assertIn("invalid_final_package_placement", self.report()["failures"])

    def test_package_above_floor_or_single_final_sample_fails(self):
        self.samples[-1]["package_position"][2] = 0.6
        self.assertIn("invalid_final_package_placement", self.report()["failures"])
        self.samples = self.samples[:7]
        self.assertIn("drop_not_sustained", self.report()["failures"])

    def test_collision_and_early_completion_fail(self):
        self.samples[4]["collisions"] = ["rack"]
        self.assertIn("physical_collision", self.report()["failures"])
        self.events[-1]["received"] = 114
        self.assertIn("completion_before_drop_acknowledgement", self.report()["failures"])


class AccessEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.manifest = {"tolerance": 0.05, "start_position": [34, 10, 2],
                         "observation_position": [34, 25, 6], "hover_time": 1}
        self.samples = [{"stamp": stamp, "phase": phase, "drone_position": position}
                        for stamp, phase, position in (
                            (100, "ASCEND_TO_OBSERVATION", [34, 10, 2]),
                            (108, "GO_TO_OBSERVATION", [34, 10, 6]),
                            (138, "HOVER", [34, 25, 6]),
                            (139, "OBSERVE", [34, 25, 6]),
                            (141, "APPROACH", [34, 25, 6]))]

    def test_measured_start_route_and_hover_pass(self):
        self.assertEqual(evaluate_access(self.manifest, self.samples, []), [])

    def test_wrong_start_missing_stage_and_hover_drift_fail(self):
        self.samples[0]["drone_position"][0] += 1
        self.samples[2]["drone_position"][0] += 1
        failures = evaluate_access(self.manifest, self.samples, [])
        self.assertIn("wrong_initial_drone_position", failures)
        self.assertIn("observation_hover_not_verified", failures)
        self.samples.pop(1)
        self.assertIn("missing_or_unordered_access_stages", evaluate_access(self.manifest, self.samples, []))

    def test_early_pickup_and_travel_below_height_fail(self):
        self.samples[1]["drone_position"][2] = 3
        events = [{"kind": "action", "received": 137, "payload": "PICKUP"}]
        failures = evaluate_access(self.manifest, self.samples, events)
        self.assertIn("travel_below_observation_height", failures)
        self.assertIn("pickup_before_observation", failures)
