"""Camera geometry and depth decoding, independent of ROS and detector libraries."""

import math
import struct


def depth_at_pixel(data, width, height, row_stride, bigendian, column, row):
    """Read one 32FC1 optical-axis depth value in metres; invalid values return None."""
    if width <= 0 or height <= 0 or row_stride < width * 4:
        raise ValueError("Invalid depth image dimensions or row stride")
    if len(data) != row_stride * height:
        raise ValueError("Depth byte count does not match its dimensions")
    if not (0 <= column < width and 0 <= row < height):
        raise ValueError("Depth pixel lies outside the image")
    value = struct.unpack_from(
        ">f" if bigendian else "<f", data, row * row_stride + column * 4
    )[0]
    return value if math.isfinite(value) and value > 0 else None


def deproject_pixel(column, row, depth_metres, intrinsic_matrix):
    """Return optical coordinates: X right, Y down, Z forward (all in metres)."""
    if len(intrinsic_matrix) != 9:
        raise ValueError("Camera intrinsics must be a 3 by 3 matrix")
    fx, fy = intrinsic_matrix[0], intrinsic_matrix[4]
    cx, cy = intrinsic_matrix[2], intrinsic_matrix[5]
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy, depth_metres)):
        raise ValueError("Camera parameters and depth must be finite")
    if fx <= 0 or fy <= 0 or depth_metres <= 0:
        raise ValueError("Focal lengths and depth must be positive")
    return (
        (column - cx) * depth_metres / fx,
        (row - cy) * depth_metres / fy,
        depth_metres,
    )


def transform_point(point, translation, quaternion):
    """Apply a rigid transform; quaternion order is ROS (x, y, z, w)."""
    if not all(math.isfinite(value) for value in (*point, *translation, *quaternion)):
        raise ValueError("Transform inputs must be finite")
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if abs(norm - 1.0) > 0.001:
        raise ValueError("Transform quaternion must be normalized")
    # q * point * inverse(q), expanded to avoid a matrix-library dependency.
    px, py, pz = point
    tx = 2 * (y * pz - z * py)
    ty = 2 * (z * px - x * pz)
    tz = 2 * (x * py - y * px)
    return (
        px + w * tx + y * tz - z * ty + translation[0],
        py + w * ty + z * tx - x * tz + translation[1],
        pz + w * tz + x * ty - y * tx + translation[2],
    )
