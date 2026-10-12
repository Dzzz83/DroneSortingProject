"""Independently deserialize Unity's saved ROS1 fixtures; requires ROS Noetic."""

import argparse
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from drone_sorting.perception.camera_geometry import depth_at_pixel, transform_point


def verify(directory):
    messages = {}
    for name, message_type in (
        ("rgb", Image), ("depth", Image), ("camera-info", CameraInfo),
        ("camera-pose", PoseStamped), ("drone-pose", PoseStamped), ("tf", TFMessage),
    ):
        messages[name] = message_type().deserialize((directory / (name + ".bin")).read_bytes())

    rgb, depth, calibration = messages["rgb"], messages["depth"], messages["camera-info"]
    assert (rgb.width, rgb.height, rgb.step, rgb.encoding, len(rgb.data)) == (2, 2, 6, "rgb8", 12)
    assert (depth.width, depth.height, depth.step, depth.encoding) == (2, 2, 8, "32FC1")
    assert depth_at_pixel(depth.data, 2, 2, 8, depth.is_bigendian, 1, 0) == 2
    assert depth_at_pixel(depth.data, 2, 2, 8, depth.is_bigendian, 0, 1) is None
    assert calibration.K[0] > 0 and calibration.K[4] > 0
    for name in ("rgb", "depth", "camera-info", "camera-pose", "drone-pose"):
        header = messages[name].header
        assert (header.seq, header.stamp.secs, header.stamp.nsecs) == (7, 123, 456)

    body, mount = messages["tf"].transforms
    assert (body.header.frame_id, body.child_frame_id) == ("warehouse", "drone_base")
    assert (mount.header.frame_id, mount.child_frame_id) == ("drone_base", "package_camera_optical")
    translation = body.transform.translation
    rotation = body.transform.rotation
    camera_translation = mount.transform.translation
    composed = transform_point(
        (camera_translation.x, camera_translation.y, camera_translation.z),
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )
    position = messages["camera-pose"].pose.position
    assert all(math.isclose(actual, expected, abs_tol=0.0001)
               for actual, expected in zip(composed, (position.x, position.y, position.z)))
    print("PASS: ROS Noetic decoded all six Unity messages; metadata, depth, calibration, and TF translation agree")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    verify(parser.parse_args().directory)
