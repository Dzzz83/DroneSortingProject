"""Validate synchronized Unity RGB-depth observations before localization."""

import math

import rospy

from drone_sorting.perception.camera_geometry import depth_at_pixel, deproject_pixel, transform_point

def validate_frame(rgb, depth, calibration, camera_pose):
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

