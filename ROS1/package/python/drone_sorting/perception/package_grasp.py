"""Estimate upright demo-box centers from an adequately visible square top face.

Only RGB-selected depth and calibration enter this module. Known box dimensions
are a geometry prior, never a lookup of package positions. Tilted or obscured
boxes are unsupported; a surface candidate may remain usable without a grasp.
"""

import math
import statistics

from drone_sorting.perception.camera_geometry import depth_at_pixel, deproject_pixel, transform_point


def fit_top_face(points, side=0.6):
    """Fit a yaw-rotated square; require both dimensions and broad face coverage."""
    if len(points) < 30:
        return {"ready": False, "reason": "insufficient_top_face"}
    top = statistics.median(point[2] for point in points)
    points = [point for point in points if abs(point[2] - top) <= 0.015]
    if len(points) < 30:
        return {"ready": False, "reason": "nonplanar_top_face"}
    best = None
    for degrees in range(0, 90, 2):
        angle = math.radians(degrees)
        cosine, sine = math.cos(angle), math.sin(angle)
        rotated = [(cosine * p[0] + sine * p[1], -sine * p[0] + cosine * p[1]) for p in points]
        left, right = min(p[0] for p in rotated), max(p[0] for p in rotated)
        bottom, upper = min(p[1] for p in rotated), max(p[1] for p in rotated)
        area = (right - left) * (upper - bottom)
        if best is None or area < best[0]:
            best = (area, cosine, sine, left, right, bottom, upper, rotated)
    if best is None:
        raise ValueError("No grasp orientation evaluated")
    _, cosine, sine, left, right, bottom, upper, rotated = best
    spans = (right - left, upper - bottom)
    if not all(side - 0.06 <= span <= side + 0.04 for span in spans):
        return {"ready": False, "reason": "incomplete_or_wrong_size_top_face"}
    cells = {(min(5, int(6 * (x - left) / spans[0])), min(5, int(6 * (y - bottom) / spans[1])))
             for x, y in rotated}
    if len(cells) < 28:
        return {"ready": False, "reason": "occluded_top_face"}
    x, y = (left + right) / 2, (bottom + upper) / 2
    return {"ready": True, "center": [cosine * x - sine * y, sine * x + cosine * y, top - side / 2],
            "reference": "upright_box_center", "top_height": top, "side_meters": side,
            "top_samples": len(points), "covered_cells": len(cells)}


def estimate_grasp(box, depth, intrinsics, translation, rotation, area, height_range):
    width, height, stride, bigendian, data = depth
    left, top, right, bottom = box
    if left < 2 or top < 2 or right > width - 2 or bottom > height - 2:
        return {"ready": False, "reason": "clipped_package_view"}
    points = []
    step = max(1, int(max(right - left, bottom - top) / 60))
    for row in range(max(0, int(top)), min(height, math.ceil(bottom)), step):
        for column in range(max(0, int(left)), min(width, math.ceil(right)), step):
            distance = depth_at_pixel(data, width, height, stride, bigendian, column, row)
            if distance is None or not 0.2 <= distance <= 15:
                continue
            world = transform_point(deproject_pixel(column, row, distance, intrinsics), translation, rotation)
            if (area[0] <= world[0] <= area[1] and area[2] <= world[1] <= area[3]
                    and height_range[0] <= world[2] <= height_range[1]):
                points.append(world)
    if not points:
        return {"ready": False, "reason": "missing_package_depth"}
    # The upper quartile isolates the horizontal top from visible vertical sides.
    ordered_heights = sorted(point[2] for point in points)
    top_height = ordered_heights[int(0.75 * (len(points) - 1))]
    return fit_top_face([point for point in points if abs(point[2] - top_height) <= 0.015])
