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

    def test_missing_position_does_not_publish_movement(self):
        self.run_ticks(1)
        self.assertEqual(self.published, [])

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
