import struct
import unittest
from unittest.mock import patch

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo, Image
    from drone_sorting.perception.camera_stream_check import CameraStreamCheck
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires the ROS Noetic Python environment")
class CameraStreamContractTests(unittest.TestCase):
    def setUp(self):
        self.rgb = Image(width=2, height=2, encoding="rgb8", step=6, data=bytes(12))
        self.depth = Image(width=2, height=2, encoding="32FC1", step=8,
                           data=struct.pack("<4f", 1, 2, 3, 4))
        self.info = CameraInfo(width=2, height=2, distortion_model="plumb_bob",
                               D=[0] * 5, K=[1, 0, 0.5, 0, 1, 0.5, 0, 0, 1])
        self.pose = PoseStamped()
        self.pose.pose.orientation.w = 1
        self.messages = (self.rgb, self.depth, self.info, self.pose)
        for message in self.messages:
            message.header.seq = 7
            message.header.stamp = rospy.Time(100, 0)
            message.header.frame_id = "package_camera_optical"
        self.pose.header.frame_id = "warehouse"

    def validate(self):
        with patch.object(rospy.Time, "now", return_value=rospy.Time(100, 0)):
            CameraStreamCheck._validate_frame(*self.messages)

    def test_valid_frame(self):
        self.validate()

    def test_missing_centre_depth_is_valid_sensor_data(self):
        self.depth.data = struct.pack("<4f", 1, 2, 3, float("nan"))
        self.validate()

    def test_reject_unsynchronized_timestamp(self):
        self.depth.header.stamp = rospy.Time(99, 0)
        with self.assertRaises(ValueError):
            self.validate()

    def test_reject_bad_encoding(self):
        self.rgb.encoding = "bgr8"
        with self.assertRaises(ValueError):
            self.validate()

    def test_reject_stale_frame(self):
        for message in self.messages:
            message.header.stamp = rospy.Time(90, 0)
        with self.assertRaises(ValueError):
            self.validate()

    def test_reject_short_depth_payload(self):
        self.depth.data = b""
        with self.assertRaises(ValueError):
            self.validate()

    def test_reject_invalid_pose(self):
        self.pose.pose.orientation.w = 0
        with self.assertRaises(ValueError):
            self.validate()


if __name__ == "__main__":
    unittest.main()
