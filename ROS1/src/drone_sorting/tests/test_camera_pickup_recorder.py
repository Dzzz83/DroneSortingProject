"""Check recorder callbacks and files using mocked ROS transport only."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

try:
    import rospy
    from std_msgs.msg import Float32, String
except ImportError:
    rospy = None


@unittest.skipIf(rospy is None, "Requires ROS Noetic message packages")
class CameraPickupRecorderTests(unittest.TestCase):
    def test_recorder_only_subscribes_and_preserves_events(self):
        path = Path(__file__).resolve().parents[3] / "tools/record_camera_pickup.py"
        spec = importlib.util.spec_from_file_location("pickup_recorder_under_test", path)
        recorder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(recorder)
        callbacks = {}

        def subscribe(topic, message_type, callback, **kwargs):
            callbacks[topic] = callback

        def messages():
            callbacks["/drone/pickup_trial_status"](String(json.dumps({"phase": "APPROACH", "stamp": 100, "speed": 0.5})))
            callbacks["/drone/package_action"](String("PICKUP"))
            callbacks["/drone/package_action_status"](String("PICKUP_DONE"))
            callbacks["/drone/command_speed"](Float32(0))
            callbacks["/drone/pickup_trial_status"](String("[]"))

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(rospy, "myargv", return_value=["recorder", "--output", directory]), \
                patch.object(rospy, "init_node"), patch.object(rospy, "Subscriber", side_effect=subscribe), \
                patch.object(rospy, "Publisher") as publisher, patch.object(rospy, "logwarn") as warning, \
                patch.object(rospy, "spin", side_effect=messages):
            recorder.main()
            publisher.assert_not_called()
            warning.assert_called_once()
            rows = [json.loads(line) for line in (Path(directory) / "ros-events.jsonl").read_text().splitlines()]
            self.assertEqual([row["kind"] for row in rows], ["guidance", "action", "acknowledgement", "speed"])
            self.assertEqual(rows[2]["payload"], "PICKUP_DONE")
            self.assertEqual(json.loads((Path(directory) / "latest-guidance.json").read_text())["phase"], "APPROACH")
            with self.assertRaises(FileExistsError):
                recorder.main()
