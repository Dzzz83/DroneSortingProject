"""Delivery stages reuse camera pickup and require real, correctly timed acknowledgements."""

import unittest

from drone_sorting.mission.delivery_guidance import DeliveryGuidance, DeliverySettings, ObservationAccessSettings


class DeliveryGuidanceTests(unittest.TestCase):
    def lifted(self):
        node = DeliveryGuidance()
        node.center = (32.8, 24, 0.3)
        node.transition("LIFT", 100)
        command = node.tick((32.8, 24, 2.25), 101)
        self.assertEqual(command["phase"], "CARRY_ASCEND")
        self.assertEqual(command["speed"], 0)
        return node

    def at_release(self):
        node = self.lifted()
        self.assertEqual(node.tick((32.8, 24, 6), 102)["phase"], "CARRY")
        self.assertEqual(node.tick((28, 55, 6), 103)["phase"], "LOWER")
        return node

    def test_vertical_clearance_precedes_lateral_carry_and_lowering(self):
        node = self.lifted()
        command = node.tick((32.8, 24, 3), 101.1)
        self.assertEqual(command["target"], (32.8, 24, 6))
        node.tick((32.8, 24, 6), 102)
        self.assertEqual(node.tick((32.8, 24, 6), 102.1)["target"], (28, 55, 6))
        node.tick((28, 55, 6), 103)
        command = node.tick((28, 55, 5), 103.1)
        self.assertEqual(command["target"], (28, 55, 1.4))
        self.assertEqual(command["speed"], 0.2)
        self.assertEqual(command["action"], "")

    def test_release_once_then_wait_for_drop_acknowledgement_and_settling(self):
        node = self.at_release()
        command = node.tick((28, 55, 1.4), 104)
        self.assertEqual(command["action"], "DROP")
        self.assertEqual(command["speed"], 0)
        for now in (104.1, 104.2):
            self.assertEqual(node.tick((28, 55, 1.4), now)["action"], "")
        node.acknowledge("PICKUP_DONE", 104.3)
        self.assertEqual(node.phase, "WAIT_FOR_DROP")
        node.acknowledge("DROP_DONE", 104.4)
        self.assertEqual(node.tick((28, 55, 1.4), 105)["phase"], "SETTLE")
        self.assertEqual(node.tick((28, 55, 1.4), 106.5)["phase"], "COMPLETE")
        node.acknowledge("DROP_DONE", 107)
        self.assertEqual(node.tick((28, 55, 1.4), 107)["speed"], 0)

    def test_premature_drop_acknowledgement_cannot_skip_carry(self):
        node = self.lifted()
        node.acknowledge("DROP_DONE", 101.1)
        self.assertEqual(node.phase, "CARRY_ASCEND")
        node = self.at_release()
        node.acknowledge("DROP_DONE", 103.1)
        self.assertEqual(node.phase, "LOWER")

    def test_missing_acknowledgement_and_stalled_travel_fail_while_holding(self):
        node = self.at_release()
        node.tick((28, 55, 1.4), 104)
        command = node.tick((28, 55, 1.4), 110)
        self.assertEqual(command["reason"], "drop_timeout")
        self.assertEqual(command["speed"], 0)
        node = self.lifted()
        command = node.tick((32.8, 24, 3), 222)
        self.assertEqual(command["reason"], "delivery_timeout")
        self.assertEqual(command["speed"], 0)

    def test_perception_after_pickup_cannot_move_the_locked_center(self):
        node = self.lifted()
        node.observe({"state": "DETECTIONS", "frame_id": "warehouse", "stamp": {"secs": 102, "nsecs": 0},
                      "candidates": [{"position": [33, 24, 0.3], "grasp": {"ready": True, "center": [33, 24, 0.3]}}]}, 102)
        self.assertEqual(node.center, (32.8, 24, 0.3))

    def test_invalid_settings_are_rejected_but_negative_horizontal_coordinates_are_allowed(self):
        for values in ({"carry_height": 1}, {"drop_timeout": 0}, {"dropoff_x": float("nan")}, {"carry_speed": -1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                DeliverySettings(**values)
        self.assertEqual(DeliverySettings(dropoff_x=-1).dropoff_x, -1)


class ObservationAccessTests(unittest.TestCase):
    def test_ascent_travel_hover_and_only_post_arrival_observations(self):
        node = DeliveryGuidance(access_settings=ObservationAccessSettings())
        payload = {"state": "DETECTIONS", "frame_id": "warehouse", "stamp": {"secs": 100, "nsecs": 0},
                   "candidates": [{"grasp": {"ready": True, "center": [32.8, 24, 0.3]}}]}
        node.observe(payload, 100)
        self.assertIsNone(node.center)
        self.assertEqual(node.tick((34, 10, 2), 100)["target"], (34, 10, 6))
        self.assertEqual(node.tick((34, 10, 6), 108)["phase"], "GO_TO_OBSERVATION")
        self.assertEqual(node.tick((34, 10, 6), 109)["target"], (34, 25, 6))
        self.assertEqual(node.tick((34, 25, 6), 138)["phase"], "HOVER")
        self.assertEqual(node.tick((34, 25, 6), 138.5)["phase"], "HOVER")
        self.assertEqual(node.tick((34, 25, 6), 139)["phase"], "OBSERVE")
        payload["stamp"]["secs"] = 138
        node.observe(payload, 139)
        self.assertIsNone(node.center)
        for stamp in (139, 139.2, 139.4):
            payload["stamp"] = {"secs": int(stamp), "nsecs": round((stamp % 1) * 1e9)}
            node.observe(payload, stamp)
        self.assertEqual(node.tick((34, 25, 6), 139.4)["phase"], "APPROACH")

    def test_hover_drift_resets_settling_and_access_timeout_holds(self):
        node = DeliveryGuidance(access_settings=ObservationAccessSettings())
        node.tick((34, 10, 2), 100)
        node.tick((34, 10, 6), 108)
        node.tick((34, 25, 6), 138)
        self.assertGreater(node.tick((34.1, 25, 6), 138.8)["speed"], 0)
        self.assertEqual(node.tick((34, 25, 6), 139)["phase"], "HOVER")
        self.assertEqual(node.tick((34, 25, 6), 140)["phase"], "OBSERVE")
        node = DeliveryGuidance(access_settings=ObservationAccessSettings())
        node.tick((34, 10, 2), 100)
        node.acknowledge("PICKUP_DONE", 101)
        node.acknowledge("DROP_DONE", 101)
        result = node.tick((34, 10, 2), 161)
        self.assertEqual(result["reason"], "observation_access_timeout")
        self.assertEqual(result["speed"], 0)
        self.assertEqual(result["action"], "")

    def test_invalid_access_settings(self):
        for values in ({"hover_time": 0}, {"observation_height": float("nan")}, {"access_speed": -1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                ObservationAccessSettings(**values)
