"""Reusable navigation decisions run without ROS or scene startup."""

import unittest

from drone_sorting.navigation.guidance import NavigationGuidance


class NavigationGuidanceTests(unittest.TestCase):
    def setUp(self):
        self.guidance = NavigationGuidance((1, 0, 3))

    def feedback(self, now, position=(0, 0, 3), sequence=1, has_collision=False):
        self.guidance.receive_course(sequence, has_collision, now)
        self.guidance.receive_position(position, now)

    def test_no_target_until_feedback_is_fresh_and_startup_reset_is_finished(self):
        self.assertEqual(self.guidance.tick(10)[0], None)
        self.feedback(10)
        target, status = self.guidance.tick(10, waiting_for_reset=True)
        self.assertIsNone(target)
        self.assertEqual(status['reason'], 'waiting_for_course_reset')
        self.assertIsNone(self.guidance.started_time)
        self.assertEqual(self.guidance.tick(10)[0], (1, 0, 3))
        self.assertEqual(self.guidance.tick(10.5)[0], (1, 0, 3))
        self.assertIsNone(self.guidance.tick(10.51)[0])
        self.assertIsNone(self.guidance.tick(9)[0])

    def test_arrival_requires_continuous_fresh_feedback_inside_tolerance(self):
        self.feedback(10, (1, 0, 3))
        self.assertEqual(self.guidance.tick(10)[1]['state'], 'NAVIGATING')
        self.guidance.tick(10.6)  # Missing feedback interrupts arrival.
        self.feedback(11, (1, 0, 3))
        self.assertEqual(self.guidance.tick(11)[1]['state'], 'NAVIGATING')
        self.feedback(11.5)  # Leaving tolerance also interrupts arrival.
        self.guidance.tick(11.5)
        self.feedback(12, (1, 0, 3))
        self.guidance.tick(12)
        self.feedback(13, (1, 0, 3))
        target, status = self.guidance.tick(13)
        self.assertIsNone(target)
        self.assertEqual(status['state'], 'ARRIVED')
        self.assertIsNone(self.guidance.tick(14)[0])

    def test_collision_latches_until_course_reset_and_new_position(self):
        self.feedback(10, has_collision=True)
        target, status = self.guidance.tick(10)
        self.assertIsNone(target)
        self.assertEqual(status['reason'], 'contact_detected')
        self.feedback(11)
        self.assertEqual(self.guidance.tick(11)[1]['state'], 'FAILED')
        self.guidance.receive_course(2, False, 12)
        self.assertIsNone(self.guidance.tick(12)[0])
        self.guidance.receive_position((0, 0, 3), 12)
        self.assertEqual(self.guidance.tick(12)[0], (1, 0, 3))

    def test_lost_feedback_and_travel_limits_stop_targets(self):
        for timeout, stall_timeout, now, reason in (
            (900, 90, 100, 'no_navigation_progress'),
            (2, 90, 12, 'mission_timeout'),
        ):
            with self.subTest(reason=reason):
                self.guidance = NavigationGuidance((1, 0, 3), timeout, stall_timeout)
                self.feedback(10)
                self.guidance.tick(10)
                self.feedback(now)
                target, status = self.guidance.tick(now)
                self.assertIsNone(target)
                self.assertEqual(status['reason'], reason)
        self.guidance = NavigationGuidance((1, 0, 3))
        self.feedback(10)
        self.guidance.tick(10)
        self.assertEqual(self.guidance.tick(20)[1]['state'], 'NAVIGATING')
        target, status = self.guidance.tick(20.01)
        self.assertIsNone(target)
        self.assertEqual(status['reason'], 'unity_feedback_lost')

    def test_invalid_settings_and_nonfinite_position(self):
        for values in ({'destination': (1, 2)}, {'destination': (1, 2, float('nan'))},
                       {'timeout': 0}, {'stall_timeout': -1}, {'arrival_tolerance': float('inf')}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                NavigationGuidance(**dict({'destination': (1, 0, 3)}, **values))
        self.feedback(10)
        self.guidance.receive_position((float('nan'), 0, 3), 11)
        self.assertEqual(self.guidance.position_received_time, 10)


if __name__ == '__main__':
    unittest.main()
