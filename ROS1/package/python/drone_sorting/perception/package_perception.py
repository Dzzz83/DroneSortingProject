"""Read-only ROS synchronization, freshness checks, inference, and localization."""

import http.client
import json
import math
import threading
import time

import message_filters
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from drone_sorting.interfaces import ros_topics
from drone_sorting.interfaces.parameters import read_float, read_int, read_float_list
from drone_sorting.perception.camera_validation import validate_frame
from drone_sorting.perception.package_localization import localize_detections
from drone_sorting.perception.package_grasp import estimate_grasp


class PackagePerception:
    def __init__(self):
        self.port = read_int("detector_port", 10001)
        self.maximum_age = read_float("maximum_age", 1.5)
        area = read_float_list("pickup_bounds", [31.5, 36.5, 22.5, 27.5])
        height_range = read_float_list("surface_height_range", [0.2, 1.2])
        if area is None or height_range is None:
            raise ValueError("Perception bounds are required")
        self.area = tuple(area)
        self.height_range = tuple(height_range)
        if (not 0 < self.port < 65536 or not math.isfinite(self.maximum_age) or self.maximum_age <= 0
                or len(self.area) != 4 or len(self.height_range) != 2
                or not all(math.isfinite(value) for value in self.area + self.height_range)
                or self.area[0] >= self.area[1] or self.area[2] >= self.area[3]
                or self.height_range[0] >= self.height_range[1]):
            raise ValueError("Invalid perception limits")
        self.publisher = rospy.Publisher(ros_topics.PACKAGE_CANDIDATES, String, queue_size=1)
        self.latest = None
        self.last_received = None
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.subscribers = [message_filters.Subscriber(topic, kind, queue_size=1, buff_size=8 * 1024 * 1024)
                            for topic, kind in [(ros_topics.RGB_IMAGE, Image), (ros_topics.DEPTH_IMAGE, Image),
                                               (ros_topics.CAMERA_INFO, CameraInfo), (ros_topics.CAMERA_POSE, PoseStamped)]]
        self.synchronizer = message_filters.TimeSynchronizer(self.subscribers, queue_size=5)
        self.synchronizer.registerCallback(self.receive)
        self.worker = threading.Thread(target=self.process, daemon=True)
        self.worker.start()
        self.health = rospy.Timer(rospy.Duration.from_sec(0.5), self.check_health)

    def publish(self, state, frame=None, reason=None, **fields):
        result = {"schema_version": 1, "state": state, "frame_id": "warehouse", "candidates": [],
                  "sequence": None, "stamp": None, "reason": reason, **fields}
        if frame is not None:
            result["sequence"] = frame.header.seq
            result["stamp"] = {"secs": frame.header.stamp.secs, "nsecs": frame.header.stamp.nsecs}
        self.publisher.publish(String(data=json.dumps(result, allow_nan=False)))

    def receive(self, *frame):
        with self.lock:
            self.latest = frame  # One pending observation; newer input replaces older work.
            self.last_received = time.monotonic()
        self.ready.set()

    def check_health(self, _event):
        with self.lock:
            last_received = self.last_received
        if last_received is None or time.monotonic() - last_received > self.maximum_age:
            self.publish("UNAVAILABLE", reason="missing_synchronized_camera_data")

    def process(self):
        while not rospy.is_shutdown():
            if not self.ready.wait(0.2):
                continue
            with self.lock:
                frame, self.latest = self.latest, None
                self.ready.clear()
            if frame is None:
                continue
            rgb, depth, calibration, pose = frame
            started = time.monotonic()
            connection = None
            try:
                validate_frame(rgb, depth, calibration, pose)
                self.check_age(rgb)
                connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=2)
                connection.request("POST", "/detect", body=bytes(rgb.data), headers={
                    "X-Width": str(rgb.width), "X-Height": str(rgb.height), "X-Sequence": str(rgb.header.seq)})
                response = connection.getresponse()
                if response.status != 200:
                    raise OSError("Detector service returned HTTP %s" % response.status)
                payload = json.loads(response.read(1024 * 1024))
                if payload["sequence"] != str(rgb.header.seq) or not isinstance(payload["detections"], list):
                    raise ValueError("Detector response does not match observation")
                self.check_age(rgb)
                position, rotation = pose.pose.position, pose.pose.orientation
                result = localize_detections(payload["detections"],
                    (depth.width, depth.height, depth.step, depth.is_bigendian, depth.data), calibration.K,
                    (position.x, position.y, position.z), (rotation.x, rotation.y, rotation.z, rotation.w),
                    self.area, self.height_range)
                for candidate in result["candidates"]:
                    candidate["grasp"] = estimate_grasp(candidate["bbox"],
                        (depth.width, depth.height, depth.step, depth.is_bigendian, depth.data), calibration.K,
                        (position.x, position.y, position.z), (rotation.x, rotation.y, rotation.z, rotation.w),
                        self.area, self.height_range)
                self.check_age(rgb)
                self.publish(frame=rgb, processing_ms=(time.monotonic() - started) * 1000,
                             observation_age_seconds=(rospy.Time.now() - rgb.header.stamp).to_sec(),
                             raw_detections=len(payload["detections"]), **result)
            except (ValueError, KeyError, TypeError) as error:
                self.publish("UNRELIABLE", frame=rgb, reason=str(error))
            except (OSError, http.client.HTTPException) as error:
                self.publish("UNAVAILABLE", frame=rgb, reason="detector_unavailable: " + str(error))
            finally:
                if connection is not None:
                    connection.close()

    def check_age(self, rgb):
        age = (rospy.Time.now() - rgb.header.stamp).to_sec()
        if age < -0.5 or age > self.maximum_age:
            raise ValueError("Observation exceeds perception freshness limit")
