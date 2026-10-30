"""Exercise the pickup transport/control boundary without a ROS master or Unity."""

import json
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
    def test_numeric_parameters_require_numbers_before_conversion(self):
        for name in ('speed', 'update_rate', 'maximum_command_lead',
                     'position_timeout', 'target_timeout', 'speed_timeout'):
            for value in (True, False, '2', None, [], {}):
                with self.subTest(parameter=name, value=value):
                    with self.assertRaisesRegex(ValueError, '~' + name):
                        self.controller(**{'~' + name: value})
            for value in (1, 1.5):
                with self.subTest(parameter=name, value=value):
                    node = self.controller(dynamic=False, **{'~' + name: value})
                    self.assertEqual(getattr(node, name), float(value))
                    self.assertIsInstance(getattr(node, name), float)

    def controller(self, dynamic=True, **overrides):
        parameters = {"~enable_obstacle_avoidance": False, "~speed": 0.5,
                      "~enable_dynamic_speed": dynamic, "~position_timeout": 0.5,
                      "~target_timeout": 0.5, "~speed_timeout": 0.5}
        parameters.update(overrides)
        with patch.object(rospy, "get_param", side_effect=lambda key, default: parameters.get(key, default)), \
                patch.object(rospy, "Publisher", side_effect=lambda *args, **kwargs: Mock()), \
                patch.object(rospy, "Subscriber"), \
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

    def smooth_controller(self):
        with patch("drone_sorting.control.depth_navigation.DepthNavigation"):
            node = self.controller(dynamic=False, **{
                "~avoidance_mode": "depth", "~depth_smooth_trajectory_enabled": True})
        node.depth_navigation.reset_sequence = 1
        node.depth_reset_seen = 1
        return node

    def combined_controller(self):
        with patch("drone_sorting.control.depth_navigation.DepthNavigation"):
            node = self.controller(**{
                "~avoidance_mode": "depth", "~depth_smooth_trajectory_enabled": True,
                "~depth_after_pickup": True})
        depth = node.depth_navigation
        depth.reset_sequence = node.depth_reset_seen = 1
        depth.contact_detected = False
        depth.package_attached = True
        depth.package_received_time = depth.course_received = 100
        self.speed(node, .5)
        node._update_depth_command = Mock()
        return node

    def phase(self, node, phase, now=100):
        from std_msgs.msg import String
        with patch.object(rospy, "get_time", return_value=now):
            node._phase_callback(String(json.dumps({"phase": phase})))

    def test_combined_pickup_uses_points_without_entering_trajectory_mode(self):
        node = self.combined_controller()
        self.update(node)
        self.assertEqual(node.handover.state, "PICKUP")
        self.phase(node, "LIFT")
        self.update(node)
        node.command_publisher.publish.assert_called_once()
        node.trajectory_publisher.publish.assert_not_called()
        node._update_depth_command.assert_not_called()

    def test_combined_mode_requires_smooth_depth_and_dynamic_speed(self):
        for parameters in ({'~depth_after_pickup': 'true'},
                           {'~depth_after_pickup': True},
                           {'~depth_after_pickup': True, '~avoidance_mode': 'depth'},
                           {'~depth_after_pickup': True, '~avoidance_mode': 'depth',
                            '~depth_smooth_trajectory_enabled': True, '~enable_dynamic_speed': False}):
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                self.controller(**parameters)

    def test_combined_pickup_retains_heuristic_avoidance_during_access(self):
        node = self.combined_controller()
        self.phase(node, "GO_TO_OBSERVATION")
        node._choose_avoidance_action = Mock(return_value='HOLD')
        self.update(node)
        node._choose_avoidance_action.assert_called_once()
        self.assertEqual(node.command_position.x, 0)

    def test_combined_handover_requires_matching_stopped_ack_and_never_returns_to_points(self):
        node = self.combined_controller()
        self.phase(node, "VERIFIED")
        self.update(node)
        self.phase(node, "NAVIGATE_TO_DROPOFF")
        self.update(node)
        stop = json.loads(node.trajectory_publisher.publish.call_args[0][0].data)
        self.assertTrue(stop['stop'])
        node.depth_navigation.follower.reset.assert_called_once()
        self.trajectory_status(node, stop['sequence'] - 1)
        self.update(node)
        self.assertEqual(node.handover.state, "STOPPING")
        node._update_depth_command.assert_not_called()
        self.trajectory_status(node, stop['sequence'], stopped=False)
        self.update(node)
        self.assertEqual(node.handover.state, "STOPPING")
        self.trajectory_status(node, stop['sequence'], x=.03)
        self.update(node)
        node._update_depth_command.assert_not_called()
        self.update(node)
        self.assertEqual(node.handover.state, "NAVIGATING")
        self.assertEqual(node.command_position.x, .03)
        node._update_depth_command.assert_called_once()
        self.phase(node, "LIFT")
        self.update(node)
        self.assertEqual(node.handover.state, "FAILED")
        node.command_publisher.publish.assert_not_called()

    def test_combined_handover_rejects_unverified_pickup_or_missing_package_feedback(self):
        for problem in ('unverified', 'detached', 'missing_package_stamp', 'stale_course'):
            with self.subTest(problem=problem):
                node = self.combined_controller()
                if problem != 'unverified':
                    self.phase(node, "VERIFIED")
                self.phase(node, "NAVIGATE_TO_DROPOFF")
                self.update(node)
                if problem == 'detached':
                    node.depth_navigation.package_attached = False
                elif problem == 'missing_package_stamp':
                    node.depth_navigation.package_received_time = None
                elif problem == 'stale_course':
                    node.depth_navigation.course_received = 99
                self.update(node)
                self.assertEqual(node.handover.state, "FAILED")
                node._update_depth_command.assert_not_called()
                node.command_publisher.publish.assert_not_called()

    def test_combined_handover_timeout_or_reset_latches_failure(self):
        for reset in (False, True):
            with self.subTest(reset=reset):
                node = self.combined_controller()
                self.phase(node, "VERIFIED")
                self.phase(node, "NAVIGATE_TO_DROPOFF")
                self.update(node)
                if reset:
                    node.depth_navigation.reset_sequence = 2
                    self.update(node)
                else:
                    self.phase(node, "NAVIGATE_TO_DROPOFF", now=106)
                    node.depth_navigation.package_received_time = node.depth_navigation.course_received = 106
                    self.update(node, now=106)
                self.assertEqual(node.handover.state, "FAILED")
                node._update_depth_command.assert_not_called()
                node.command_publisher.publish.assert_not_called()

    def trajectory_status(self, node, sequence, stopped=True, x=0, now=100, epoch=1):
        from std_msgs.msg import String
        with patch.object(rospy, "get_time", return_value=now):
            node._trajectory_status_callback(String(data=json.dumps({
                "reset_sequence": epoch, "sequence": sequence, "stopped": stopped,
                "stamp_ms": now * 1000, "position": {"x": x, "y": 0, "z": 6}})))

    def test_smooth_initial_sync_requires_matching_stop_acknowledgement(self):
        node = self.smooth_controller()
        self.update(node)
        stop = json.loads(node.trajectory_publisher.publish.call_args[0][0].data)
        self.assertTrue(stop["stop"])
        self.trajectory_status(node, stop["sequence"] - 1, x=.02)
        self.update(node)
        self.assertIsNotNone(node.trajectory_stop_sequence)
        self.trajectory_status(node, stop["sequence"], x=.03)
        self.update(node)
        self.assertIsNone(node.trajectory_stop_sequence)
        self.assertEqual(node.command_position.x, .03)
        node.depth_navigation.tick.assert_not_called()

    def test_missing_depth_positions_stop_without_planning_or_moving(self):
        for name in ('command_position', 'current_position', 'target_position'):
            with self.subTest(position=name):
                node = self.smooth_controller()
                node.trajectory_stream.move((0, 0, 6), (.1, 0, 6), 1, 2, 100,
                                            {'stamp': 100, 'observation_stamp': 100})
                setattr(node, name, None)
                node._update_depth_command(100)
                node.depth_navigation.tick.assert_not_called()
                node.command_publisher.publish.assert_not_called()
                packet = json.loads(node.trajectory_publisher.publish.call_args[0][0].data)
                self.assertTrue(packet['stop'])

    def test_missing_depth_navigation_cannot_authorize_or_publish_trajectory(self):
        node = self.smooth_controller()
        node.depth_navigation = None
        self.assertFalse(node._trajectory_ready(100))
        node._stop_trajectory(force=True, now=100)
        node._update_depth_command(100)
        node.trajectory_publisher.publish.assert_not_called()
        node.command_publisher.publish.assert_not_called()

    def test_smooth_stop_does_not_rewind_to_delayed_position(self):
        node = self.smooth_controller()
        node.trajectory_needs_sync = False
        node.trajectory_stream.move((0, 0, 6), (.1, 0, 6), 1, 2, 100,
                                    {"stamp": 100, "observation_stamp": 100})
        node.command_position = Point(.1, 0, 6)
        node._stop_trajectory(now=100)
        self.assertEqual(node.command_position.x, .1)
        self.trajectory_status(node, node.trajectory_stop_sequence, x=.075)
        self.assertFalse(node._trajectory_ready(100))
        self.assertEqual(node.command_position.x, .075)

    def test_smooth_lost_stop_retries_same_packet_then_recovers_from_ack(self):
        node = self.smooth_controller()
        self.update(node)
        original = node.trajectory_publisher.publish.call_args[0][0].data
        self.trajectory_status(node, 0, now=100.1)
        self.update(node, 100.1)
        self.assertEqual(node.trajectory_publisher.publish.call_count, 1)
        self.trajectory_status(node, 0, now=100.6)
        self.update(node, 100.6)
        self.assertEqual(node.trajectory_publisher.publish.call_count, 2)
        self.assertEqual(node.trajectory_publisher.publish.call_args[0][0].data, original)
        self.assertEqual(node.trajectory_stream.sequence, 1)
        node.depth_navigation.tick.assert_not_called()
        # A lost acknowledgement also keeps the same pending stop identity.
        self.update(node, 101.2)
        self.assertEqual(node.trajectory_publisher.publish.call_count, 3)
        self.assertEqual(node.trajectory_publisher.publish.call_args[0][0].data, original)
        self.trajectory_status(node, 1, x=.075, now=101.3)
        self.update(node, 101.3)
        self.assertIsNone(node.trajectory_stop_sequence)
        self.assertEqual(node.command_position.x, .075)
        node.trajectory_publisher.reset_mock()
        with patch.object(node, '_update_depth_command') as advance:
            node.position_received = node.target_received = 101.4
            self.update(node, 101.4)
        advance.assert_called_once()
        node.trajectory_publisher.publish.assert_not_called()

    def test_smooth_reset_discards_pending_stop_and_rejects_old_ack(self):
        node = self.smooth_controller()
        self.update(node)
        original = node.trajectory_publisher.publish.call_args[0][0].data
        node.depth_navigation.reset_sequence = 2
        self.update(node, 100.6)
        replacement = node.trajectory_publisher.publish.call_args[0][0].data
        self.assertNotEqual(original, replacement)
        self.assertEqual(json.loads(replacement)['reset_sequence'], 2)
        self.trajectory_status(node, 1, now=100.7, epoch=1)
        self.update(node, 100.7)
        self.assertIsNotNone(node.trajectory_stop_sequence)
        self.trajectory_status(node, 1, now=100.8, epoch=2)
        self.update(node, 100.8)
        self.assertIsNone(node.trajectory_stop_sequence)

    def heuristic_sensors(self, node, front, left, right):
        from std_msgs.msg import Float32MultiArray
        node._obstacle_callback(Float32MultiArray(data=[front, -1, -1, left, right, -1, -1]))

    def test_heuristic_holds_when_front_and_both_sides_are_blocked(self):
        node = self.controller(dynamic=False, **{'~avoidance_mode': 'heuristic'})
        self.heuristic_sensors(node, .1, .1, .1)
        self.update(node)
        self.assertEqual(node.command_position, Point(0, 0, 6))
        self.assertIsNone(node.bypass_direction)

    def test_heuristic_selects_only_clear_side_and_keeps_right_preference(self):
        for left, right, expected in ((-1, -1, 'RIGHT'), (.1, -1, 'RIGHT'), (-1, .1, 'LEFT')):
            with self.subTest(left=left, right=right):
                node = self.controller(dynamic=False, **{'~avoidance_mode': 'heuristic'})
                self.heuristic_sensors(node, .1, left, right)
                self.update(node)
                self.assertEqual(node.bypass_direction, expected)
                self.assertNotEqual(node.command_position, Point(0, 0, 6))

    def test_heuristic_rechecks_latched_side_before_any_further_movement(self):
        for direction, front in (('LEFT', .1), ('RIGHT', .1), ('LEFT', -1), ('RIGHT', -1)):
            with self.subTest(direction=direction, front=front):
                node = self.controller(dynamic=False, **{'~avoidance_mode': 'heuristic'})
                node.bypass_direction = direction
                node.closest_target_dist = 0.  # Divergence must not bypass the blocked-side gate.
                node.max_divergence_dist = .5
                self.heuristic_sensors(node, front, .1, .1)
                self.update(node)
                if front >= 0:
                    self.assertEqual(node.command_position, Point(0, 0, 6))
                else:
                    # Once the front is clear, abandon the blocked bypass.
                    self.assertIsNone(node.bypass_direction)
                    self.assertGreater(node.command_position.x, 0)

    def test_smooth_local_rejection_triggers_stop_and_recovery(self):
        node = self.smooth_controller()
        node.trajectory_needs_sync = False
        node.trajectory_stream.move((0, 0, 6), (.1, 0, 6), 1, 2, 100,
                                    {"stamp": 100, "observation_stamp": 100})
        node.trajectory_first_move = 1
        self.trajectory_status(node, 1)
        self.assertFalse(node._trajectory_ready(100))
        self.assertEqual(node.trajectory_stop_sequence, 2)

    def test_smooth_candidate_lead_wait_recovers_when_feedback_stalls(self):
        node = self.smooth_controller()
        node.trajectory_stream.move((0, 0, 6), (.2, 0, 6), 1, 2, 100,
                                    {"stamp": 100, "observation_stamp": 100})
        node.command_position = Point(.2, 0, 6)
        node.position_progress_at = 99
        node.depth_navigation.tick.return_value = Mock(state="MOVE", next_position=(.3, 0, 6))
        node._update_depth_command(100)
        self.assertIsNotNone(node.trajectory_stop_sequence)
        self.assertEqual(node.command_position.x, .2)

    def test_smooth_short_window_waits_without_rewind_or_stop(self):
        node = self.smooth_controller()
        node.trajectory_stream.move((0,0,6),(.1,0,6),1,2,100,
                                    {"stamp":100,"observation_stamp":100})
        node.command_position = Point(.1,0,6)
        node.depth_navigation.tick.return_value = Mock(state="MOVE",next_position=(.125,0,6))
        node.depth_navigation.follower.status = {"stamp":99.7,"observation_stamp":99.7}
        node._update_depth_command(100)
        node.trajectory_publisher.publish.assert_not_called()
        self.assertEqual(node.command_position.x,.1)
        self.assertEqual(node.trajectory_stream.sequence,1)
        self.assertIsNone(node.trajectory_stop_sequence)

    def test_frontier_at_goal_does_not_cancel_still_approved_queued_motion(self):
        node = self.smooth_controller()
        node.trajectory_stream.move((0,0,6),(.1,0,6),1,2,100,
                                    {"stamp":100,"observation_stamp":100})
        node.depth_navigation.tick.return_value = Mock(state="HOLD",reason="local_goal_reached")
        node._update_depth_command(100)
        node.trajectory_publisher.publish.assert_not_called()
        self.assertTrue(node.trajectory_stream.in_motion)
        node.depth_navigation.tick.return_value = Mock(state="HOLD",reason="route_plan_stale")
        node._update_depth_command(100)
        self.assertIsNotNone(node.trajectory_stop_sequence)

    def test_configured_fifty_hz_keeps_requested_speed_with_smaller_steps(self):
        node = self.controller(dynamic=False, **{"~speed":2.,"~update_rate":50.})
        self.update(node)
        self.assertAlmostEqual(node.command_position.x,.04)
        for rate in (0,float("nan"),101):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                self.controller(**{"~update_rate":rate})

    def test_smooth_wrong_epoch_and_stale_ack_cannot_restart(self):
        node = self.smooth_controller()
        self.update(node)
        self.trajectory_status(node, node.trajectory_stop_sequence, epoch=0)
        self.assertFalse(node._trajectory_ready(100))
        self.trajectory_status(node, node.trajectory_stop_sequence)
        self.assertFalse(node._trajectory_ready(101))
        self.assertIsNotNone(node.trajectory_stop_sequence)

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
        node = self.camera_node()
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
        node.position_received_time = 99
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
        node.position_received_time = 100
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
        node.position_received_time = 100
        node.guidance = Mock(center=None)
        for phase in ("ASCEND_TO_OBSERVATION", "GO_TO_OBSERVATION", "HOVER", "OBSERVE", "APPROACH"):
            node.guidance.phase = phase
            node.guidance.tick.return_value = {"phase": phase, "target": node.position,
                                               "speed": 0, "action": "", "reason": None}
            self.run_camera_once(node)
        states = [call[0][0].data for call in node.state.publish.call_args_list]
        self.assertEqual(states, ["IDLE", "GO_TO_PICKUP"])
