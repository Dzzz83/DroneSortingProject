import unittest

from drone_sorting.mission.pickup_guidance import PickupGuidance


class PickupGuidanceTests(unittest.TestCase):
    def test_missing_package_center_fails_without_moving_or_requesting_pickup(self):
        for phase in ('APPROACH', 'DESCEND', 'LIFT'):
            with self.subTest(phase=phase):
                node = PickupGuidance()
                node.transition(phase, 100)
                command = node.tick((34, 25, 3), 100)
                self.assertEqual(command['phase'], 'FAILED')
                self.assertEqual(command['reason'], 'missing_package_center')
                self.assertEqual(command['target'], (34, 25, 3))
                self.assertEqual(command['speed'], 0)
                self.assertEqual(command['action'], '')

    def observe(self, node, now, center=(34, 25, 0.44), state="DETECTIONS"):
        node.observe({"state": state, "frame_id": "warehouse", "stamp": {"secs": now, "nsecs": 0},
                      "candidates": [{"position": [center[0], center[1], center[2] + 0.3],
                                      "grasp": {"ready": True, "center": center}}]}, now)

    def approach(self, node):
        for now in (100, 100.5, 101):
            self.observe(node, now)
            node.tick((34, 25, 6), now)
        self.assertEqual(node.phase, "APPROACH")

    def test_pickup_requires_refinement_and_real_acknowledgement(self):
        node = PickupGuidance()
        self.approach(node)
        position = (34, 25, 2.79)
        node.tick(position, 101.1)
        for now in (101.5, 102, 102.5):
            self.observe(node, now)
            node.tick(position, now)
        self.assertEqual(node.phase, "DESCEND")
        self.assertEqual(node.tick((34, 25, 1.39), 102.6)["action"], "PICKUP")
        node.acknowledge("DROP_DONE", 102.7)
        self.assertEqual(node.phase, "WAIT_FOR_PICKUP")
        waiting = node.tick((34, 25, 1.39), 102.7)
        self.assertEqual(waiting["speed"], 0)
        self.assertEqual(waiting["action"], "")
        node.acknowledge("PICKUP_DONE", 102.8)
        self.assertEqual(node.phase, "LIFT")
        node.tick((34, 25, 2.39), 103)
        self.assertEqual(node.phase, "VERIFIED")

    def test_lost_perception_holds_then_fails(self):
        node = PickupGuidance()
        self.approach(node)
        self.observe(node, 101.5, state="UNAVAILABLE")
        position = (34, 25, 5)
        hold = node.tick(position, 101.6)
        self.assertEqual(hold["target"], position)
        self.assertEqual(hold["speed"], 0)
        failure = node.tick(position, 105)
        self.assertEqual(failure["speed"], 0)
        self.assertEqual(node.phase, "FAILED")

    def test_other_package_cannot_replace_lock(self):
        node = PickupGuidance()
        self.approach(node)
        self.observe(node, 101.5, center=(35, 25, 0.44))
        self.assertEqual(node.center, (34, 25, 0.44))
        self.assertFalse(node.observation_valid)

    def test_stale_observation_and_pickup_timeout(self):
        node = PickupGuidance()
        self.observe(node, 100)
        node.tick((34, 25, 6), 102)
        self.assertEqual(node.phase, "OBSERVE")
        node.transition("WAIT_FOR_PICKUP", 102)
        node.tick((34, 25, 1.39), 108)
        self.assertEqual(node.phase, "FAILED")

    def test_unsolicited_acknowledgement_does_not_lift(self):
        node = PickupGuidance()
        node.acknowledge("PICKUP_DONE", 100)
        self.assertEqual(node.phase, "OBSERVE")

    def test_interrupted_confirmation_requires_three_new_observations(self):
        for state in ("UNAVAILABLE", "UNRELIABLE", "NO_DETECTIONS"):
            with self.subTest(state=state):
                node = PickupGuidance()
                self.observe(node, 100)
                self.observe(node, 100.5)
                self.observe(node, 100.6, state=state)
                self.assertEqual(node.stable_count, 0)
                self.assertEqual(node.center, (34, 25, 0.44))
                for now in (101, 101.5):
                    self.observe(node, now)
                    node.tick((34, 25, 6), now)
                    self.assertEqual(node.phase, "OBSERVE")
                self.observe(node, 102)
                node.tick((34, 25, 6), 102)
                self.assertEqual(node.phase, "APPROACH")

    def test_interrupted_refinement_holds_and_requires_new_confirmation(self):
        node = PickupGuidance()
        self.approach(node)
        position = (34, 25, 2.79)
        node.tick(position, 101.1)
        for now in (101.5, 102):
            self.observe(node, now)
            node.tick(position, now)
        self.observe(node, 102.1, state="UNRELIABLE")
        self.assertEqual(node.refinement_count, 0)
        self.assertEqual(node.tick(position, 102.1)["target"], position)
        for now in (102.5, 103):
            self.observe(node, now)
            node.tick(position, now)
            self.assertEqual(node.phase, "APPROACH")
        self.observe(node, 103.5)
        node.tick(position, 103.5)
        self.assertEqual(node.phase, "DESCEND")

    def test_expired_observation_resets_confirmation_without_new_messages(self):
        node = PickupGuidance()
        self.observe(node, 100)
        self.observe(node, 100.5)
        node.tick((34, 25, 6), 101.6)
        self.assertEqual(node.stable_count, 0)
        self.assertFalse(node.observation_valid)
        self.observe(node, 101.7)
        self.assertEqual(node.stable_count, 1)

    def test_stale_frame_and_unmatched_package_reset_confirmation(self):
        for interruption in ("stale", "other_package", "missing_grasp", "gap"):
            with self.subTest(interruption=interruption):
                node = PickupGuidance()
                self.observe(node, 100)
                self.observe(node, 100.5)
                if interruption == "stale":
                    node.observe({"state": "DETECTIONS", "frame_id": "warehouse",
                                  "stamp": {"secs": 99, "nsecs": 0}}, 101)
                elif interruption == "other_package":
                    self.observe(node, 101, center=(35, 25, 0.44))
                elif interruption == "missing_grasp":
                    node.observe({"state": "DETECTIONS", "frame_id": "warehouse",
                                  "stamp": {"secs": 101, "nsecs": 0},
                                  "candidates": [{"position": [34, 25, 0.74]}]}, 101)
                else:
                    self.observe(node, 102)
                    self.assertEqual(node.stable_count, 1)
                    continue
                self.assertEqual(node.stable_count, 0)
                self.assertEqual(node.center, (34, 25, 0.44))
