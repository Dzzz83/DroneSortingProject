"""ROS freshness and unavailable-state checks without sensor or mission side effects."""

import json
import threading
import unittest
from unittest.mock import Mock, patch

try:
    import rospy
    from sensor_msgs.msg import Image
    from drone_sorting.perception.package_perception import PackagePerception
except ImportError:
    rospy = None


@unittest.skipIf(rospy is None, "Requires ROS Noetic message packages")
class PackagePerceptionTests(unittest.TestCase):
    def setUp(self):
        self.node = PackagePerception.__new__(PackagePerception)
        self.node.maximum_age = 1.5
        self.node.publisher = Mock()
        self.node.lock = threading.Lock()

    def test_freshness_rejects_old_and_future_frames(self):
        frame = Image()
        with patch.object(rospy.Time, "now", return_value=rospy.Time(100)):
            for stamp in (98, 101):
                frame.header.stamp = rospy.Time(stamp)
                with self.assertRaises(ValueError):
                    self.node.check_age(frame)
            frame.header.stamp = rospy.Time(99)
            self.node.check_age(frame)

    def test_missing_camera_does_not_report_empty(self):
        self.node.last_received = None
        self.node.check_health(None)
        payload = json.loads(self.node.publisher.publish.call_args[0][0].data)
        self.assertEqual(payload["state"], "UNAVAILABLE")
        self.assertEqual(payload["candidates"], [])
        self.assertIsNone(payload["stamp"])

    def test_publish_preserves_observation_identity(self):
        frame = Image()
        frame.header.seq = 42
        frame.header.stamp = rospy.Time(100, 23)
        self.node.publish("UNRELIABLE", frame=frame, reason="invalid_depth")
        payload = json.loads(self.node.publisher.publish.call_args[0][0].data)
        self.assertEqual(payload["sequence"], 42)
        self.assertEqual(payload["stamp"], {"secs": 100, "nsecs": 23})
        self.assertEqual(payload["reason"], "invalid_depth")

    def process_response(self, response=None, failure=None):
        from drone_sorting.perception import package_perception
        frame = Image()
        frame.header.seq = 42
        self.node.latest = (frame, None, None, None)
        self.node.ready = threading.Event()
        self.node.ready.set()
        self.node.port = 10001
        connection = Mock()
        connection.getresponse.side_effect = failure
        if response is not None:
            connection.getresponse.return_value.status = 200
            connection.getresponse.return_value.read.return_value = json.dumps(response).encode()
        with patch.object(rospy, "is_shutdown", side_effect=[False, True]), \
                patch.object(package_perception.CameraStreamCheck, "_validate_frame"), \
                patch.object(self.node, "check_age"), \
                patch.object(package_perception.http.client, "HTTPConnection", return_value=connection):
            self.node.process()
        connection.close.assert_called_once()
        return json.loads(self.node.publisher.publish.call_args[0][0].data)

    def test_detector_failure_is_unavailable(self):
        payload = self.process_response(failure=OSError("Connection refused"))
        self.assertEqual(payload["state"], "UNAVAILABLE")
        self.assertEqual(payload["candidates"], [])

    def test_mismatched_detector_sequence_is_unreliable(self):
        payload = self.process_response(response={"sequence": "41", "detections": []})
        self.assertEqual(payload["state"], "UNRELIABLE")
        self.assertEqual(payload["candidates"], [])


if __name__ == "__main__":
    unittest.main()
