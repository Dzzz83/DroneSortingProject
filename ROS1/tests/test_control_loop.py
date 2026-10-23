"""Exercise the pickup transport/control boundary without a ROS master or Unity."""

import json
import threading
import unittest
from unittest.mock import Mock, patch

try:
    import rospy
    from geometry_msgs.msg import Point
    from std_msgs.msg import Float32
    from drone_sorting.control.motion_controller import MotionController
    from drone_sorting.mission.delivery import DeliveryMission
except ImportError:
    rospy = None


@unittest.skipIf(rospy is None, "Requires ROS Noetic message packages")
class ControlLoopTests(unittest.TestCase):
    def controller(self, dynamic=True, **overrides):
        parameters = {"~enable_obstacle_avoidance": False, "~speed": 0.5,
                      "~enable_dynamic_speed": dynamic, "~position_timeout": 0.5,
                      "~target_timeout": 0.5, "~speed_timeout": 0.5}
        parameters.update(overrides)
        with patch.object(rospy, "get_param", side_effect=lambda key, default: parameters.get(key, default)), \
                patch.object(rospy, "Publisher"), patch.object(rospy, "Subscriber"), \
                patch.object(rospy, "loginfo"):
            node = MotionController()
        node._log_status = Mock()
        with patch.object(rospy, "get_time", return_value=100):
            node._current_position_callback(Point(0, 0, 6))
            node._target_position_callback(Point(1, 0, 6))
        return node

    def update(self, node, now=100):
        with patch.object(rospy, "get_time", return_value=now):
            node._update_command()

    def speed(self, node, value, now=100):
        with patch.object(rospy, "get_time", return_value=now):
            node._speed_callback(Float32(value))

    def test_phase_uses_current_mission_status_and_ignores_malformed_payloads(self):
        from std_msgs.msg import String
        node = self.controller()
        node._phase_callback(String(data=json.dumps({"phase": "LOWER"})))
        self.assertEqual(node.current_phase, "LOWER")
        for payload in ("invalid", "null", "[]", "{}", '{"phase":4}'):
            node._phase_callback(String(data=payload))
            self.assertEqual(node.current_phase, "LOWER")

    def test_sensor_order_matches_unity_seven_element_array(self):
        from std_msgs.msg import Float32MultiArray
        node = self.controller()
        node._obstacle_callback(Float32MultiArray(data=[1, 2, 3, 4, 5, 6, 7]))
        sensors = node._parse_lateral_sensors()
        self.assertEqual((sensors["F"], sensors["L"], sensors["R"]), (1, 4, 5))
        self.assertEqual(set(sensors), {"F", "L", "R"})

    def test_dynamic_control_waits_for_speed_and_stops_on_speed_loss(self):
        node = self.controller()
        self.update(node)
        node.command_publisher.publish.assert_not_called()
        self.speed(node, 0.2)
        self.update(node)
        self.assertAlmostEqual(node.command_position.x, 0.01)
        node.command_publisher.reset_mock()
        node.position_received = node.target_received = 101
        self.update(node, 101)
        node.command_publisher.publish.assert_not_called()

    def test_zero_speed_holds_last_command_despite_lagging_feedback(self):
        node = self.controller()
        node.command_position = Point(0.1, 0, 6)
        node.target_position = Point(0, 0, 6)
        self.speed(node, 0)
        self.update(node)
        self.assertEqual(node.command_position.x, 0.1)
        node.command_publisher.publish.assert_not_called()

    def test_delayed_five_hz_feedback_does_not_rewind_twenty_hz_commands(self):
        node = self.controller()
        commands = [0.0]
        for tick in range(1, 41):
            now = 100 + tick / 20
            if tick % 4 == 0:
                # Feedback arrives at 5 Hz and represents a command three ticks old.
                with patch.object(rospy, "get_time", return_value=now):
                    node._current_position_callback(Point(commands[max(0, tick - 3)], 0, 6))
            node.target_received = now
            self.speed(node, 0.5, now)
            self.update(node, now)
            commands.append(node.command_position.x)
        self.assertTrue(all(b >= a for a, b in zip(commands, commands[1:])))
        self.assertAlmostEqual(commands[-1], 1.0)

    def test_fresh_but_stationary_feedback_bounds_command_lead(self):
        node = self.controller()
        for tick in range(40):
            now = 100 + tick / 20
            with patch.object(rospy, "get_time", return_value=now):
                node._current_position_callback(Point(0, 0, 6))
            node.target_received = now
            self.speed(node, 0.5, now)
            self.update(node, now)
        self.assertGreater(node.command_position.x, 0)
        self.assertLessEqual(node.command_position.x, 0.25 + 1e-9)
        count = node.command_publisher.publish.call_count
        self.update(node, now)
        self.assertEqual(node.command_publisher.publish.call_count, count)

    def test_tracking_wait_resumes_when_unity_catches_up(self):
        node = self.controller()
        node.command_position = Point(0.4, 0, 6)
        node.target_position = Point(0.4, 0, 6)
        self.speed(node, 0.5)
        self.update(node)
        node.command_publisher.publish.assert_not_called()
        with patch.object(rospy, "get_time", return_value=100):
            node._current_position_callback(Point(0.4, 0, 6))
            node._target_position_callback(Point(1, 0, 6))
        self.update(node)
        self.assertAlmostEqual(node.command_position.x, 0.425)
        node.command_publisher.publish.assert_called_once()

    def test_stop_and_target_reversal_preserve_last_command_on_delayed_feedback(self):
        node = self.controller()
        self.speed(node, 0.5)
        for _ in range(4): self.update(node)
        with patch.object(rospy, "get_time", return_value=100):
            node._current_position_callback(Point(0.05, 0, 6))
        self.speed(node, 0)
        self.update(node)
        self.assertAlmostEqual(node.command_position.x, 0.1)
        with patch.object(rospy, "get_time", return_value=100):
            node._target_position_callback(Point(0, 0, 6))
        self.speed(node, 0.5)
        self.update(node)
        self.assertAlmostEqual(node.command_position.x, 0.075)

    def test_invalid_speed_stops_instead_of_retaining_previous_speed(self):
        for value in (-1, 0.6, float("nan"), float("inf")):
            with self.subTest(value=value):
                node = self.controller()
                self.speed(node, 0.2)
                self.speed(node, value)
                self.update(node)
                node.command_publisher.publish.assert_not_called()

    def test_position_and_target_timeouts_stop_publication(self):
        for missing in ("position_received", "target_received"):
            with self.subTest(missing=missing):
                node = self.controller()
                self.speed(node, 0.2)
                setattr(node, missing, 99)
                self.update(node)
                node.command_publisher.publish.assert_not_called()

    def test_invalid_position_and_target_do_not_refresh_feedback(self):
        node = self.controller()
        with patch.object(rospy, "get_time", return_value=101):
            node._current_position_callback(Point(float("nan"), 0, 6))
            node._target_position_callback(Point(0, float("inf"), 6))
        self.assertEqual(node.position_received, 100)
        self.assertEqual(node.target_received, 100)

    def test_fixed_speed_mode_and_short_final_step(self):
        node = self.controller(dynamic=False, **{"~speed": 2.0})
        self.update(node)
        self.assertAlmostEqual(node.command_position.x, 0.1)
        node.command_position.x = 0.985
        node.current_position = Point(0.985, 0, 6)
        self.update(node)
        self.assertAlmostEqual(node.command_position.x, 1.0)

    def test_invalid_command_lead_is_rejected(self):
        for value in (0, -1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.controller(**{"~maximum_command_lead": value})

    def test_invalid_timeout_configuration_is_rejected(self):
        for name in ("~position_timeout", "~target_timeout", "~speed_timeout"):
            for value in (-1, float("nan"), float("inf")):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    self.controller(**{name: value})

    def test_waiting_for_first_position_does_not_hold_callback_lock(self):
        node = DeliveryMission.__new__(DeliveryMission)
        node.lock = threading.Lock()
        node.position = None
        rate = Mock()
        rate.sleep.side_effect = lambda: self.assertFalse(node.lock.locked())
        with patch.object(rospy, "Rate", return_value=rate), \
                patch.object(rospy, "is_shutdown", side_effect=[False, True]), \
                patch.object(rospy, "get_time", return_value=100):
            node.run()
        rate.sleep.assert_called_once()

    def camera_node(self):
        with patch.object(rospy, "get_param", side_effect=lambda key, default: default), \
                patch.object(rospy, "Publisher", side_effect=lambda *args, **kwargs: Mock()), patch.object(rospy, "Subscriber"):
            node = DeliveryMission()
        node.position = (12, 55, 6)
        node.position_received = 99
        return node

    def test_default_mission_always_starts_with_observation_access(self):
        node = self.camera_node()
        self.assertEqual(node.guidance.phase, "ASCEND_TO_OBSERVATION")
        self.assertEqual(node.guidance.delivery.dropoff_x, 12.0)
        self.assertIsNotNone(node.guidance.access)

    def test_mission_uses_configured_destination_and_observation_point(self):
        parameters = {"~dropoff_x": -4.0, "~dropoff_y": 42.0, "~observation_x": 30.0}
        with patch.object(rospy, "get_param", side_effect=lambda key, default: parameters.get(key, default)), \
                patch.object(rospy, "Publisher"), patch.object(rospy, "Subscriber"):
            node = DeliveryMission()
        self.assertEqual(node.guidance.delivery.dropoff_x, -4.0)
        self.assertEqual(node.guidance.delivery.dropoff_y, 42.0)
        self.assertEqual(node.guidance.access.observation_x, 30.0)

    def run_camera_once(self, node):
        with patch.object(rospy, "Rate"), patch.object(rospy, "loginfo"), \
                patch.object(rospy, "is_shutdown", side_effect=[False, True]), \
                patch.object(rospy, "get_time", return_value=100):
            node.run()

    def test_delivery_stops_on_stale_feedback(self):
        from drone_sorting.mission.delivery_guidance import DeliveryGuidance
        node = self.camera_node()
        node.guidance = DeliveryGuidance()
        node.guidance.center = (32.8, 24, 0.3)
        node.guidance.transition("CARRY", 99)
        self.run_camera_once(node)
        self.assertEqual(node.guidance.phase, "FAILED")
        self.assertEqual(node.guidance.reason, "position_feedback_unavailable")
        self.assertEqual(node.speed.publish.call_args[0][0].data, 0)

    def test_completed_delivery_is_not_overwritten_after_unity_exits(self):
        from drone_sorting.mission.delivery_guidance import DeliveryGuidance
        node = self.camera_node()
        node.guidance = DeliveryGuidance()
        node.guidance.transition("COMPLETE", 99)
        self.run_camera_once(node)
        self.assertEqual(node.guidance.phase, "COMPLETE")
        self.assertEqual(node.speed.publish.call_args[0][0].data, 0)

    def test_delivery_publishes_the_existing_public_mission_sequence(self):
        node = self.camera_node()
        node.position_received = 100
        node.guidance = Mock(center=(32.8, 24, 0.3))
        for phase in ("APPROACH", "WAIT_FOR_PICKUP", "LIFT", "CARRY_ASCEND", "CARRY",
                      "LOWER", "WAIT_FOR_DROP", "SETTLE", "COMPLETE"):
            node.guidance.phase = phase
            node.guidance.tick.return_value = {"phase": phase, "target": node.position,
                                               "speed": 0, "action": "", "reason": None}
            self.run_camera_once(node)
        states = [call[0][0].data for call in node.state.publish.call_args_list]
        self.assertEqual(states, ["IDLE", "GO_TO_PICKUP", "WAIT_FOR_PICKUP", "GO_TO_DROPOFF",
                                  "WAIT_FOR_DROP", "COMPLETE"])


    def test_access_stops_on_stale_feedback(self):
        from drone_sorting.mission.delivery_guidance import DeliveryGuidance, ObservationAccessSettings
        node = self.camera_node()
        node.guidance = DeliveryGuidance(access_settings=ObservationAccessSettings())
        self.run_camera_once(node)
        self.assertEqual(node.guidance.phase, "FAILED")
        self.assertEqual(node.guidance.reason, "position_feedback_unavailable")
        self.assertEqual(node.speed.publish.call_args[0][0].data, 0)

    def test_access_and_approach_publish_go_to_pickup_once(self):
        node = self.camera_node()
        node.position_received = 100
        node.guidance = Mock(center=None)
        for phase in ("ASCEND_TO_OBSERVATION", "GO_TO_OBSERVATION", "HOVER", "OBSERVE", "APPROACH"):
            node.guidance.phase = phase
            node.guidance.tick.return_value = {"phase": phase, "target": node.position,
                                               "speed": 0, "action": "", "reason": None}
            self.run_camera_once(node)
        states = [call[0][0].data for call in node.state.publish.call_args_list]
        self.assertEqual(states, ["IDLE", "GO_TO_PICKUP"])
