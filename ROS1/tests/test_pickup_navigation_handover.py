"""Handover state transitions without ROS or a movement executor."""

import unittest

from drone_sorting.control.pickup_navigation_handover import PickupNavigationHandover


class PickupNavigationHandoverTests(unittest.TestCase):
    def setUp(self):
        self.handover = PickupNavigationHandover()

    def tick(self, now=100, **overrides):
        feedback = dict(has_collision=False, package_attached=True,
                        package_received_time=now, course_received_time=now)
        feedback.update(overrides)
        return self.handover.tick(now, **feedback)

    def start_handover(self):
        self.handover.receive_phase('VERIFIED', 100)
        self.handover.receive_phase('NAVIGATE_TO_DROPOFF', 100)
        return self.tick()

    def test_startup_pickup_and_verified_hold(self):
        self.assertEqual(self.tick().action, 'HOLD')
        self.handover.receive_phase('LIFT', 100)
        self.assertEqual(self.tick().action, 'ALLOW_PICKUP')
        self.handover.receive_phase('VERIFIED', 100)
        self.assertEqual(self.tick().action, 'HOLD')
        self.assertEqual(self.handover.state, 'PICKUP')

    def test_navigation_requires_stop_confirmation_and_cannot_return_to_pickup(self):
        self.assertTrue(self.start_handover().request_stop)
        self.assertEqual(self.handover.state, 'STOPPING')
        self.assertEqual(self.tick().action, 'CHECK_STOP')
        self.assertEqual(self.tick().action, 'CHECK_STOP')
        self.handover.confirm_stop()
        self.assertEqual(self.tick().action, 'ALLOW_NAVIGATION')
        self.handover.receive_phase('HOLD_AT_DROPOFF', 100)
        self.assertEqual(self.tick().action, 'ALLOW_NAVIGATION')
        self.handover.receive_phase('LIFT', 100)
        self.assertEqual(self.tick().action, 'FAIL')
        self.handover.receive_phase('NAVIGATE_TO_DROPOFF', 100)
        self.assertEqual(self.tick().action, 'FAIL')

    def test_unverified_navigation_is_rejected_and_cannot_be_resumed_by_late_verification(self):
        self.handover.confirm_stop()
        self.assertEqual(self.handover.state, 'PICKUP')
        self.handover.receive_phase('NAVIGATE_TO_DROPOFF', 100)
        rejection = self.tick()
        self.assertEqual(rejection.reason, 'pickup_not_verified')
        self.assertFalse(rejection.request_stop)
        self.handover.receive_phase('VERIFIED', 100)
        self.assertTrue(self.tick().request_stop)
        self.assertEqual(self.handover.state, 'FAILED')

    def test_phase_freshness_boundary_and_future_timestamps(self):
        self.handover.receive_phase('LIFT', 100)
        self.assertEqual(self.tick(100.5).action, 'ALLOW_PICKUP')
        self.assertEqual(self.tick(100.501).action, 'FAIL')
        self.handover = PickupNavigationHandover()
        self.handover.receive_phase('LIFT', 101)
        self.assertEqual(self.tick(100).action, 'FAIL')

    def test_attachment_course_and_collision_failures(self):
        for feedback in ({'package_attached': False}, {'package_received_time': None},
                         {'package_received_time': 99.49}, {'course_received_time': None},
                         {'course_received_time': 101}, {'has_collision': True}):
            with self.subTest(feedback=feedback):
                self.handover = PickupNavigationHandover()
                self.start_handover()
                self.assertTrue(self.tick(**feedback).request_stop)
                self.assertEqual(self.handover.state, 'FAILED')

    def test_stop_timeout_boundary_and_reset_latch(self):
        self.start_handover()
        self.handover.receive_phase('NAVIGATE_TO_DROPOFF', 105)
        self.assertEqual(self.tick(105).action, 'CHECK_STOP')
        failure = self.tick(105.001)
        self.assertEqual(failure.reason, 'stop_acknowledgement_timeout')
        self.handover.confirm_stop()
        self.assertEqual(self.handover.state, 'FAILED')
        self.handover = PickupNavigationHandover()
        self.handover.receive_reset()
        self.assertEqual(self.tick().reason, 'course_reset_during_mission')
        self.assertTrue(self.tick().request_stop)


if __name__ == '__main__':
    unittest.main()
