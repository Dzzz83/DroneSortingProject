"""Run with: python3 -m unittest discover -s ROS1/tests -p test_approved_trajectory.py"""
import json
import unittest

from drone_sorting.control.approved_trajectory import ApprovedTrajectoryStream, InsufficientTrajectoryTime


class ApprovedTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.stream = ApprovedTrajectoryStream(clock=lambda: 100.0)
        self.status = {"stamp": 10.0, "observation_stamp": 10.0}

    def move(self, start=(0, 0, 3), end=(.1, 0, 3), now=10.1, epoch=1):
        return json.loads(self.stream.move(start, end, epoch, 2.0, now, self.status))

    def test_first_segment_requires_restart(self):
        self.assertTrue(self.move()["restart"])

    def test_subsequent_segment_is_ordered(self):
        self.move()
        step = self.move((.1, 0, 3), (.2, 0, 3))
        self.assertEqual(step["sequence"], 2)
        self.assertFalse(step["restart"])
        self.assertEqual(step["start"]["x"], .1)

    def test_expiry_capped_by_approved_plan(self):
        p = self.move(now=10.30)
        self.assertGreater(p["expires_ms"], p["issued_ms"])
        self.assertLessEqual(p["expires_ms"] - p["issued_ms"], 50)

    def test_expiry_capped_by_observation(self):
        self.status["stamp"] = 10.25
        p = self.move(now=10.45)
        self.assertLessEqual(p["expires_ms"] - p["issued_ms"], 50)

    def test_stale_plan_rejected(self):
        with self.assertRaises(ValueError):
            self.move(now=10.36)

    def test_stale_observation_rejected(self):
        self.status["stamp"] = 10.3
        with self.assertRaises(ValueError):
            self.move(now=10.51)

    def test_no_unbounded_segments(self):
        with self.assertRaises(ValueError):
            self.move(end=(1, 0, 3))

    def test_stop_clears_chain_and_restart(self):
        self.move()
        stop = json.loads(self.stream.stop(1))
        self.assertTrue(stop["stop"])
        self.assertIsNone(self.stream.stop(1))
        step = self.move()
        self.assertTrue(step["restart"])
        self.assertEqual(step["sequence"], stop["sequence"] + 1)

    def test_reset_resets_sequence(self):
        self.move()
        step = self.move(epoch=2)
        self.assertEqual(step["sequence"], 1)
        self.assertTrue(step["restart"])

    def test_invalid_reset_identity(self):
        for epoch in (-1, None, True, "1"):
            with self.subTest(epoch=epoch), self.assertRaises(ValueError):
                self.move(epoch=epoch)

    def test_invalid_numbers_rejected(self):
        for end in ((float("nan"), 0, 3), (float("inf"), 0, 3), (0, 0, 3)):
            with self.subTest(end=end), self.assertRaises(ValueError):
                self.move(end=end)

    def test_future_plan_or_observation_rejected(self):
        for key in ("stamp", "observation_stamp"):
            with self.subTest(key=key):
                self.status[key] = 10.2
                with self.assertRaises(ValueError):
                    self.move(now=10.1)
                self.status[key] = 10.0

    def test_forced_initial_stop_establishes_stationary_anchor(self):
        packet = json.loads(self.stream.stop(1, force=True))
        self.assertTrue(packet["stop"])
        self.assertEqual(packet["sequence"], 1)
        self.assertEqual(self.move()["sequence"], 2)

    def test_sub_millisecond_approval_is_not_extended(self):
        with self.assertRaises(ValueError):
            self.move(now=10.3499)

    def test_short_execution_window_does_not_advance_or_cancel_chain(self):
        self.move()
        with self.assertRaises(InsufficientTrajectoryTime):
            self.stream.move((.1,0,3),(.2,0,3),1,2,10.3,self.status,minimum_ttl=.15)
        self.assertEqual(self.stream.sequence,1)
        self.assertTrue(self.stream.in_motion)
        self.assertFalse(self.stream.need_restart)
        self.status={"stamp":10.3,"observation_stamp":10.3}
        packet=self.move((.1,0,3),(.2,0,3),now=10.3)
        self.assertEqual(packet["sequence"],2)
        self.assertFalse(packet["restart"])

    def test_no_stop_without_movement(self):
        self.assertIsNone(self.stream.stop(1))


if __name__ == "__main__":
    unittest.main()
