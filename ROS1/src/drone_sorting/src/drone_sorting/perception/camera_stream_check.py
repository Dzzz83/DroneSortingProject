"""Read-only validation of synchronized Unity camera observations."""

import math

import message_filters
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

from drone_sorting.interfaces import ros_topics
from drone_sorting.perception.camera_geometry import depth_at_pixel, deproject_pixel, transform_point


class CameraStreamCheck:
    """Reports sensor health without publishing movement or package commands."""

    def __init__(self):
        self.valid_frames = 0
        self.last_received = None
        self.required_frames = int(rospy.get_param("~required_frames", 0))
        self.subscribers = [
            message_filters.Subscriber(topic, message_type, queue_size=1, buff_size=8 * 1024 * 1024)
            for topic, message_type in (
                (ros_topics.RGB_IMAGE, Image),
                (ros_topics.DEPTH_IMAGE, Image),
                (ros_topics.CAMERA_INFO, CameraInfo),
                (ros_topics.CAMERA_POSE, PoseStamped),
            )
        ]
        self.synchronizer = message_filters.TimeSynchronizer(self.subscribers, queue_size=5)
        self.synchronizer.registerCallback(self._on_frame)
        self.health_timer = rospy.Timer(rospy.Duration(2), self._check_health)

    def _on_frame(self, rgb, depth, calibration, camera_pose):
        try:
            self._validate_frame(rgb, depth, calibration, camera_pose)
        except ValueError as error:
            rospy.logwarn_throttle(5, "Camera frame rejected: %s", error)
            return
        self.valid_frames += 1
        self.last_received = rospy.Time.now()
        rospy.loginfo_throttle(5, "CAMERA_STREAM_VALID frames=%d size=%dx%d", self.valid_frames, rgb.width, rgb.height)
        if self.required_frames > 0 and self.valid_frames >= self.required_frames:
            rospy.loginfo("CAMERA_STREAM_CHECK_PASSED frames=%d", self.valid_frames)
            rospy.signal_shutdown("Required synchronized frames verified")

    @staticmethod
    def _validate_frame(rgb, depth, calibration, camera_pose):
        if rgb.encoding != "rgb8" or depth.encoding != "32FC1":
            raise ValueError("Expected rgb8 and 32FC1 encodings")
        if rgb.width == 0 or rgb.height == 0:
            raise ValueError("Empty image")
        if (rgb.width, rgb.height) != (depth.width, depth.height):
            raise ValueError("RGB/depth dimensions differ")
        if (rgb.width, rgb.height) != (calibration.width, calibration.height):
            raise ValueError("Calibration dimensions differ")
        if rgb.step != rgb.width * 3 or len(rgb.data) != rgb.step * rgb.height:
            raise ValueError("Invalid packed RGB layout")
        if rgb.header.frame_id != "package_camera_optical" or depth.header.frame_id != rgb.header.frame_id:
            raise ValueError("Incorrect optical frame")
        if calibration.header.frame_id != rgb.header.frame_id or camera_pose.header.frame_id != "warehouse":
            raise ValueError("Calibration or pose frame mismatch")
        if len({message.header.seq for message in (rgb, depth, calibration, camera_pose)}) != 1:
            raise ValueError("Sequence numbers differ")
        if any(message.header.stamp != rgb.header.stamp for message in (depth, calibration, camera_pose)):
            raise ValueError("Acquisition timestamps differ")
        age = (rospy.Time.now() - rgb.header.stamp).to_sec()
        if age < -0.5 or age > 3:
            raise ValueError("Stale observation or mismatched wall clocks")
        if calibration.distortion_model != "plumb_bob" or any(value != 0 for value in calibration.D):
            raise ValueError("Expected an undistorted simulated pinhole camera")

        column, row = rgb.width // 2, rgb.height // 2
        depth_value = depth_at_pixel(depth.data, depth.width, depth.height,
                                     depth.step, depth.is_bigendian, column, row)
        # A missing centre measurement is allowed; it is never treated as an empty area.
        optical_point = deproject_pixel(column, row, depth_value or 1.0, calibration.K)
        position = camera_pose.pose.position
        rotation = camera_pose.pose.orientation
        world_point = transform_point(optical_point, (position.x, position.y, position.z),
                                      (rotation.x, rotation.y, rotation.z, rotation.w))
        if not all(math.isfinite(value) for value in world_point):
            raise ValueError("Invalid camera/world transform")

    def _check_health(self, _event):
        if self.last_received is None or (rospy.Time.now() - self.last_received).to_sec() > 3:
            rospy.logwarn_throttle(5, "Waiting for fresh synchronized camera data; do not infer an empty pickup area")
