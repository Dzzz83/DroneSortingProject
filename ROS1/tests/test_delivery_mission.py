"""Mission configuration validation and ROS publication contracts."""

from dataclasses import asdict, fields
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

try:
    import rospy
except ImportError:
    rospy = None

if rospy is not None:
    from geometry_msgs.msg import Point
    from std_msgs.msg import String

    from drone_sorting.mission import delivery
    from drone_sorting.mission.delivery_guidance import DeliverySettings, ObservationAccessSettings
    from drone_sorting.mission.pickup_guidance import PickupSettings


@unittest.skipIf(rospy is None, "Requires ROS Noetic")
class MissionParameterTests(unittest.TestCase):
    def setUp(self):
        self.parameters = {}
        parameter_patch = patch.object(
            rospy, "get_param", side_effect=lambda name, default: self.parameters.get(name, default)
        )
        self.get_param = parameter_patch.start()
        self.addCleanup(parameter_patch.stop)
        self.loaders = (
            (PickupSettings, delivery._load_pickup_settings),
            (DeliverySettings, delivery._load_delivery_settings),
            (ObservationAccessSettings, delivery._load_access_settings),
        )

    def test_defaults_preserve_settings_and_read_every_parameter(self):
        for settings_type, load in self.loaders:
            with self.subTest(settings=settings_type.__name__):
                self.get_param.reset_mock()
                self.assertEqual(load(), settings_type())
                requested = [call.args[0] for call in self.get_param.call_args_list]
                expected = ["~" + field.name for field in fields(settings_type)]
                self.assertCountEqual(requested, expected)

    def test_pickup_only_requires_a_boolean(self):
        for invalid in ("true", "false", 0, 1, None, [], {}):
            with self.subTest(value=invalid):
                self.parameters = {"~stop_after_pickup": invalid}
                with self.assertRaisesRegex(ValueError, "~stop_after_pickup"):
                    delivery.DeliveryMission()

    def test_supported_profiles_load_all_configured_values(self):
        import yaml

        config_directory = Path(__file__).resolve().parents[1] / "package/config"
        for profile in ("demo.yaml", "course.yaml"):
            mission = yaml.safe_load((config_directory / profile).read_text())["camera_pickup"]
            self.parameters = {"~" + name: value for name, value in mission.items()}
            for settings_type, load in self.loaders:
                with self.subTest(profile=profile, settings=settings_type.__name__):
                    expected = {field.name: mission[field.name] for field in fields(settings_type)}
                    self.assertEqual(asdict(load()), expected)

    def test_navigate_after_pickup_requires_a_boolean(self):
        for value in ("true", "false", 0, 1, None, [], {}):
            with self.subTest(value=value):
                self.parameters = {"~navigate_after_pickup": value}
                with self.assertRaisesRegex(ValueError, "~navigate_after_pickup"):
                    delivery.DeliveryMission()

    def test_invalid_numeric_types_name_the_parameter(self):
        for settings_type, load in self.loaders:
            for field in fields(settings_type):
                for invalid in (True, False, "1.5", None, [], {}):
                    with self.subTest(parameter=field.name, value=invalid):
                        self.parameters = {"~" + field.name: invalid}
                        with self.assertRaisesRegex(ValueError, "~" + field.name):
                            load()

    def test_nonfinite_values_are_rejected(self):
        for settings_type, load in self.loaders:
            for field in fields(settings_type):
                for invalid in (float("nan"), float("inf"), -float("inf")):
                    with self.subTest(parameter=field.name, value=invalid):
                        self.parameters = {"~" + field.name: invalid}
                        with self.assertRaisesRegex(ValueError, "~" + field.name):
                            load()

    def test_pickup_limits_remain_positive(self):
        for field in fields(PickupSettings):
            for invalid in (0, -1):
                with self.subTest(parameter=field.name, value=invalid):
                    self.parameters = {"~" + field.name: invalid}
                    with self.assertRaisesRegex(ValueError, "~" + field.name):
                        delivery._load_pickup_settings()

    def test_observation_count_requires_a_positive_integer(self):
        for invalid in (2.5, 3.0, 0, -1):
            with self.subTest(value=invalid):
                self.parameters = {"~required_observations": invalid}
                with self.assertRaisesRegex(ValueError, "~required_observations"):
                    delivery._load_pickup_settings()
        self.parameters = {"~required_observations": 4}
        self.assertEqual(delivery._load_pickup_settings().required_observations, 4)

    def test_horizontal_coordinates_allow_zero_and_negative_values(self):
        self.parameters = {
            "~dropoff_x": -4, "~dropoff_y": 0,
            "~observation_x": 0, "~observation_y": -5,
        }
        destination = delivery._load_delivery_settings()
        access = delivery._load_access_settings()
        self.assertEqual((destination.dropoff_x, destination.dropoff_y), (-4.0, 0.0))
        self.assertEqual((access.observation_x, access.observation_y), (0.0, -5.0))
        self.assertIsInstance(destination.dropoff_x, float)

    def test_settings_classes_still_enforce_delivery_and_access_ranges(self):
        cases = (
            (delivery._load_delivery_settings, {"~carry_height": 1.0}),
            (delivery._load_delivery_settings, {"~drop_timeout": 0}),
            (delivery._load_access_settings, {"~hover_time": 0}),
        )
        for load, parameters in cases:
            with self.subTest(parameters=parameters):
                self.parameters = parameters
                with self.assertRaises(ValueError):
                    load()


@unittest.skipIf(rospy is None, "Requires ROS Noetic")
class MissionTransportTests(unittest.TestCase):
    def setUp(self):
        self.published = []
        self.now = 100.0
        patches = (
            patch.object(rospy, "get_param", side_effect=lambda name, default: default),
            patch.object(rospy, "Publisher", side_effect=self.make_publisher),
            patch.object(rospy, "Subscriber"),
            patch.object(rospy, "get_time", side_effect=lambda: self.now),
            patch.object(rospy, "loginfo"),
            patch.object(rospy, "Rate"),
        )
        for patcher in patches:
            patcher.start()
            self.addCleanup(patcher.stop)
        self.mission = delivery.DeliveryMission()
        self.published.clear()

    def make_publisher(self, topic, message_type, **options):
        publisher = Mock()
        publisher.publish.side_effect = lambda message: self.published.append((topic, message))
        return publisher

    def run_ticks(self, count):
        with patch.object(rospy, "is_shutdown", side_effect=[False] * count + [True]):
            self.mission.run()

    def messages(self, topic):
        return [message for published_topic, message in self.published if published_topic == topic]

    def enable_navigation(self):
        parameters = {"~navigate_after_pickup": True, "~dropoff_x": 34,
                      "~dropoff_y": 119.5, "~carry_height": 3}
        with patch.object(rospy, "get_param", side_effect=lambda name, default: parameters.get(name, default)):
            self.mission = delivery.DeliveryMission()
        self.published.clear()
        self.mission.receive_course(String('{"reset_sequence": 1}'))
        self.mission.receive_position(Point(34, 55, 2.25))

    def finish_pickup(self):
        self.mission.guidance.center = (34, 55, 0.3)
        self.mission.guidance.transition("WAIT_FOR_PICKUP", self.now)
        self.mission.receive_status(String("PICKUP_DONE"))
        self.run_ticks(1)

    def test_combined_mission_switches_once_after_confirmed_pickup_and_lift(self):
        self.enable_navigation()
        self.assertTrue(self.mission.guidance.stop_after_pickup)
        self.mission.guidance.transition("WAIT_FOR_PICKUP", self.now)
        self.run_ticks(1)
        self.assertEqual(self.mission.stage, "PICKUP")
        self.finish_pickup()
        self.assertEqual(self.mission.stage, "NAVIGATE_TO_DROPOFF")
        self.assertIsNone(self.mission.navigation.started_time)
        self.assertEqual(self.messages(delivery.ros_topics.COMMAND_SPEED)[-1].data, 0)
        with patch.object(self.mission.guidance, "tick", side_effect=AssertionError("pickup restarted")):
            self.run_ticks(2)
        self.assertEqual(self.messages(delivery.ros_topics.TARGET_POSITION)[-1], Point(34, 119.5, 3))
        self.assertGreater(self.messages(delivery.ros_topics.COMMAND_SPEED)[-1].data, 0)
        self.assertEqual([message.data for message in self.messages(delivery.ros_topics.MISSION_STATE)],
                         ["WAIT_FOR_PICKUP", "GO_TO_DROPOFF"])
        self.assertIsNone(self.mission.guidance.carry_start)

    def test_failed_pickup_and_stale_verified_position_cannot_start_navigation(self):
        for stale_position in (False, True):
            with self.subTest(stale_position=stale_position):
                self.enable_navigation()
                if stale_position:
                    self.mission.guidance.transition("VERIFIED", self.now)
                    self.now += 0.6
                else:
                    self.mission.guidance.transition("FAILED", self.now, "pickup_failed")
                self.run_ticks(2)
                self.assertEqual(self.mission.stage, "FAILED")
                self.assertIsNone(self.mission.navigation.started_time)
                self.assertTrue(all(message.data == 0 for message in self.messages(delivery.ros_topics.COMMAND_SPEED)))

    def test_navigation_arrival_holds_without_drop_or_complete(self):
        self.enable_navigation()
        self.finish_pickup()
        for elapsed in (0.1, 0.6, 1.2, 2.0):
            self.now = 100 + elapsed
            self.mission.receive_course(String('{"reset_sequence": 1}'))
            self.mission.receive_position(Point(34, 119.5, 3))
            self.run_ticks(1)
        self.assertEqual(self.mission.stage, "HOLD_AT_DROPOFF")
        self.assertEqual(self.messages(delivery.ros_topics.COMMAND_SPEED)[-1].data, 0)
        self.assertEqual(self.messages(delivery.ros_topics.PACKAGE_ACTION), [])
        self.assertNotIn("COMPLETE", [message.data for message in self.messages(delivery.ros_topics.MISSION_STATE)])

    def test_collision_or_reset_during_navigation_latches_failure(self):
        for report, reason in (({'reset_sequence': 1, 'collision_observed': True}, 'contact_detected'),
                               ({'reset_sequence': 2}, 'course_reset_during_mission')):
            with self.subTest(reason=reason):
                self.enable_navigation()
                self.finish_pickup()
                self.mission.receive_course(String(json.dumps(report)))
                self.mission.receive_position(Point(34, 55, 2.25))
                self.run_ticks(2)
                self.assertEqual(self.mission.stage, "FAILED")
                self.assertEqual(self.mission.guidance.reason, reason)
                self.assertEqual(self.messages(delivery.ros_topics.COMMAND_SPEED)[-1].data, 0)

    def test_navigation_timeout_and_missing_course_feedback_stop_commands(self):
        self.enable_navigation()
        self.finish_pickup()
        self.run_ticks(1)
        self.mission.navigation.timeout = 1
        self.now += 1.1
        self.mission.receive_course(String('{"reset_sequence": 1}'))
        self.mission.receive_position(Point(34, 55, 2.25))
        self.run_ticks(1)
        self.assertEqual(self.mission.stage, "FAILED")
        self.assertEqual(self.mission.guidance.reason, "mission_timeout")
        self.enable_navigation()
        self.finish_pickup()
        self.now += 0.6
        self.mission.receive_position(Point(34, 55, 2.25))
        self.run_ticks(1)
        self.assertEqual(self.messages(delivery.ros_topics.COMMAND_SPEED)[-1].data, 0)
        self.assertIsNone(self.mission.navigation.started_time)

    def test_missing_position_does_not_publish_movement(self):
        self.run_ticks(1)
        self.assertEqual(self.published, [])

    def test_profiles_select_pickup_only_and_publish_a_hold(self):
        import yaml

        config_directory = Path(__file__).resolve().parents[1] / "package/config"
        self.assertFalse(self.mission.guidance.stop_after_pickup)
        for profile, stops_after_pickup in (("demo.yaml", False), ("course.yaml", True)):
            with self.subTest(profile=profile):
                parameters = yaml.safe_load((config_directory / profile).read_text())["camera_pickup"]
                with patch.object(rospy, "get_param", side_effect=lambda name, default: parameters.get(name[1:], default)):
                    self.mission = delivery.DeliveryMission()
                self.assertEqual(self.mission.guidance.stop_after_pickup, stops_after_pickup)
                if not stops_after_pickup:
                    continue
                self.published.clear()
                position = Point(34, 55, 2.25)
                self.mission.receive_position(position)
                self.mission.guidance.transition("VERIFIED", self.now)
                self.run_ticks(2)
                topics = delivery.ros_topics
                self.assertEqual(self.mission.guidance.phase, "VERIFIED")
                self.assertEqual([message.data for message in self.messages(topics.COMMAND_SPEED)], [0, 0])
                self.assertEqual(self.messages(topics.TARGET_POSITION), [position, position])
                self.assertEqual(self.messages(topics.PACKAGE_ACTION), [])
                self.assertEqual(self.messages(topics.MISSION_STATE), [])

    def test_stale_position_fails_and_publishes_a_stop(self):
        topics = delivery.ros_topics
        self.mission.receive_position(Point(34, 10, 2))
        self.now += 0.6
        self.run_ticks(1)
        self.assertEqual(self.messages(topics.COMMAND_SPEED)[0].data, 0)
        self.assertEqual(self.messages(topics.TARGET_POSITION)[0], Point(34, 10, 2))
        status = json.loads(self.messages(topics.PICKUP_TRIAL_STATUS)[0].data)
        self.assertEqual(status["phase"], "FAILED")
        self.assertEqual(status["reason"], "position_feedback_unavailable")
        self.assertEqual(self.messages(topics.PACKAGE_ACTION), [])

    def test_position_without_a_timestamp_fails_and_publishes_a_stop(self):
        topics = delivery.ros_topics
        self.mission.position = (34, 10, 2)
        self.run_ticks(1)
        self.assertEqual(self.messages(topics.COMMAND_SPEED)[0].data, 0)
        self.assertEqual(self.messages(topics.TARGET_POSITION)[0], Point(34, 10, 2))
        status = json.loads(self.messages(topics.PICKUP_TRIAL_STATUS)[0].data)
        self.assertEqual(status["phase"], "FAILED")
        self.assertEqual(status["reason"], "position_feedback_unavailable")
        self.assertEqual(self.messages(topics.PACKAGE_ACTION), [])

    def test_command_publication_preserves_order_and_horizontal_heading(self):
        topics = delivery.ros_topics
        self.mission.receive_position(Point(34, 10, 6))
        self.mission.guidance.transition("GO_TO_OBSERVATION", self.now)
        self.run_ticks(1)
        self.assertEqual([topic for topic, message in self.published], [
            topics.COMMAND_SPEED, topics.TARGET_POSITION,
            topics.TRAVEL_HEADING, topics.PICKUP_TRIAL_STATUS,
        ])
        self.assertEqual(self.messages(topics.TARGET_POSITION)[0], Point(34, 25, 6))
        heading = self.messages(topics.TRAVEL_HEADING)[0]
        self.assertEqual((heading.x, heading.y, heading.z), (0, 1, 0))
        self.assertEqual(self.messages(topics.COMMAND_SPEED)[0].data, 0.5)
        status = json.loads(self.messages(topics.PICKUP_TRIAL_STATUS)[0].data)
        self.assertEqual(status["target"], [34, 25, 6])
        self.assertEqual(status["stamp"], self.now)

    def test_repeated_commands_do_not_repeat_actions_or_phase_reports(self):
        topics = delivery.ros_topics
        self.mission.receive_position(Point(34, 25, 1.4))
        pickup = {"phase": "WAIT_FOR_PICKUP", "reason": None,
                  "target": (34, 25, 1.4), "speed": 0, "action": "PICKUP"}
        lift = dict(pickup, phase="LIFT", action="")
        self.mission.guidance.tick = Mock(side_effect=[pickup, pickup, lift, lift])
        self.run_ticks(4)
        self.assertEqual([message.data for message in self.messages(topics.MISSION_STATE)],
                         ["WAIT_FOR_PICKUP"])
        self.assertEqual([message.data for message in self.messages(topics.PACKAGE_ACTION)],
                         ["PICKUP", ""])
        self.assertEqual([topic for topic, message in self.published[:6]], [
            topics.MISSION_STATE, topics.COMMAND_SPEED, topics.TARGET_POSITION,
            topics.TRAVEL_HEADING, topics.PACKAGE_ACTION, topics.PICKUP_TRIAL_STATUS,
        ])
        for heading in self.messages(topics.TRAVEL_HEADING):
            self.assertEqual((heading.x, heading.y, heading.z), (0, 0, 0))

    def test_pickup_acknowledgement_starts_lift_without_releasing(self):
        topics = delivery.ros_topics
        self.mission.receive_position(Point(34, 25, 1.4))
        self.mission.guidance.center = (34, 25, 0.45)
        self.mission.guidance.transition("WAIT_FOR_PICKUP", self.now)
        self.mission.receive_status(String(" pickup_done "))
        self.run_ticks(1)
        self.assertEqual(self.mission.guidance.phase, "LIFT")
        self.assertEqual(self.messages(topics.TARGET_POSITION)[0], Point(34, 25, 2.4))
        self.assertGreater(self.messages(topics.COMMAND_SPEED)[0].data, 0)
        self.assertEqual(self.messages(topics.PACKAGE_ACTION), [])
