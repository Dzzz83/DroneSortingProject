"""Read-only live pose/TF consistency check; starts no movement or mission nodes."""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from drone_sorting.interfaces import ros_topics
from drone_sorting.perception.camera_geometry import transform_point


def quaternion(value):
    return value.x, value.y, value.z, value.w


def translation(value):
    return value.x, value.y, value.z


def verify():
    rospy.init_node("verify_live_camera_transforms", anonymous=True)
    observations = {}
    lock = threading.Lock()
    complete = threading.Event()
    errors = []

    def receive(kind, stamp, message):
        with lock:
            key = stamp.to_nsec()
            frame = observations.setdefault(key, {})
            frame[kind] = message
            while len(observations) > 20:
                del observations[min(observations)]
            if len(frame) != 3 or complete.is_set():
                return
            try:
                check_frame(frame["tf"], frame["drone"], frame["camera"])
            except (AssertionError, ValueError) as error:
                errors.append(str(error))
            complete.set()

    def receive_tf(message):
        transforms = {transform.child_frame_id: transform for transform in message.transforms}
        if "drone_base" in transforms and "package_camera_optical" in transforms:
            receive("tf", transforms["drone_base"].header.stamp, transforms)

    subscribers = [
        rospy.Subscriber(ros_topics.TRANSFORMS, TFMessage, receive_tf, queue_size=5),
        rospy.Subscriber(ros_topics.CURRENT_POSE, PoseStamped,
                         lambda message: receive("drone", message.header.stamp, message), queue_size=5),
        rospy.Subscriber(ros_topics.CAMERA_POSE, PoseStamped,
                         lambda message: receive("camera", message.header.stamp, message), queue_size=5),
    ]
    try:
        if not complete.wait(15):
            raise RuntimeError("No synchronized live pose/TF frame received; keep Unity running and focused")
        if errors:
            raise AssertionError(errors[0])
        print("PASS: live warehouse/drone/optical TF chain agrees with both stamped poses and optical axes")
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        rospy.signal_shutdown("Transform verification finished")


def check_frame(transforms, drone, camera):
    body = transforms["drone_base"]
    mount = transforms["package_camera_optical"]
    assert body.header.frame_id == "warehouse", "Incorrect body parent"
    assert mount.header.frame_id == "drone_base", "Incorrect camera parent"
    assert mount.header.stamp == body.header.stamp, "TF timestamps differ"
    assert drone.header.frame_id == camera.header.frame_id == "warehouse", "Incorrect pose frames"
    body_translation = translation(body.transform.translation)
    body_rotation = quaternion(body.transform.rotation)
    mount_translation = translation(mount.transform.translation)
    mount_rotation = quaternion(mount.transform.rotation)
    assert all(math.isclose(a, b, abs_tol=0.0001)
               for a, b in zip(body_translation, translation(drone.pose.position))), "Body positions differ"
    # Check the origin and three axes to verify translation and rotation together.
    for point in ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)):
        body_direct = transform_point(point, translation(drone.pose.position), quaternion(drone.pose.orientation))
        body_from_tf = transform_point(point, body_translation, body_rotation)
        assert all(math.isclose(a, b, abs_tol=0.0001) for a, b in zip(body_direct, body_from_tf)), "Body TF/pose differs"
        camera_local = transform_point(point, mount_translation, mount_rotation)
        composed = transform_point(camera_local, body_translation, body_rotation)
        direct = transform_point(point, translation(camera.pose.position), quaternion(camera.pose.orientation))
        assert all(math.isclose(a, b, abs_tol=0.0001) for a, b in zip(composed, direct)), "Camera TF/pose differs"


if __name__ == "__main__":
    verify()
