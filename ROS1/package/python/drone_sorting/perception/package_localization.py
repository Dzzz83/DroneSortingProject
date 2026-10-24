"""Conservative visible-surface estimates from RGB boxes and synchronized depth.

World coordinates are ROS warehouse (Unity x,z,y). A returned surface sample is
not a package center or grasp target. No scene ground truth is used here.
"""

import math
import statistics

from drone_sorting.perception.camera_geometry import depth_at_pixel, deproject_pixel, transform_point


def validate_detection(detection, width, height):
    if not isinstance(detection, dict):
        raise ValueError("Detection must be an object")
    box = detection.get("xyxy", [])
    confidence = detection.get("confidence")
    if len(box) != 4 or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in box):
        raise ValueError("Invalid detection box")
    if not isinstance(confidence, (int, float)) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError("Invalid detection confidence")
    left, top, right, bottom = box
    if not (0 <= left < right <= width and 0 <= top < bottom <= height):
        raise ValueError("Detection lies outside image")


def localize_detections(detections, depth, intrinsics, translation, rotation,
                        area=(31.5, 36.5, 22.5, 27.5), height_range=(0.2, 1.2), minimum_samples=6):
    """Select an actual supported depth pixel, rather than averaging off the surface.

    Inset grid samples suppress box-edge/background contamination. Height and
    area bounds are domain constraints for 0.6 m packages on the pickup floor.
    Median-height support rejects mixed floor/occluder depth, but is not general
    segmentation. Results include the selected pixel for independent evaluation.
    """
    width, height, stride, bigendian, data = depth
    # Validate the full buffer even when detections are empty.
    depth_at_pixel(data, width, height, stride, bigendian, 0, 0)
    valid_depth = 0
    checked = 0
    for row in range(0, height, max(1, height // 20)):
        for column in range(0, width, max(1, width // 20)):
            checked += 1
            value = depth_at_pixel(data, width, height, stride, bigendian, column, row)
            valid_depth += value is not None and 0.2 <= value <= 15
    if valid_depth < max(1, checked * 0.05):
        return {"state": "UNRELIABLE", "candidates": [], "rejections": ["insufficient_valid_depth"]}
    candidates, rejections = [], []
    for detection in detections:
        validate_detection(detection, width, height)
    for detection in sorted(detections, key=lambda item: item.get("confidence", -1), reverse=True):
        left, top, right, bottom = detection["xyxy"]
        samples = []
        pixels = set()
        for vertical in range(9):
            for horizontal in range(9):
                column = min(width - 1, int(left + (right - left) * (0.2 + horizontal * 0.6 / 8)))
                row = min(height - 1, int(top + (bottom - top) * (0.2 + vertical * 0.6 / 8)))
                if (column, row) in pixels:
                    continue
                pixels.add((column, row))
                value = depth_at_pixel(data, width, height, stride, bigendian, column, row)
                if value is None or not 0.2 <= value <= 15:
                    continue
                optical = deproject_pixel(column, row, value, intrinsics)
                world = transform_point(optical, translation, rotation)
                if area[0] <= world[0] <= area[1] and area[2] <= world[1] <= area[3] and height_range[0] <= world[2] <= height_range[1]:
                    samples.append((world, column, row))
        if len(samples) < minimum_samples:
            rejections.append({"bbox": detection["xyxy"], "reason": "insufficient_package_surface_depth"})
            continue
        median = tuple(statistics.median(sample[0][axis] for sample in samples) for axis in range(3))
        supported = [sample for sample in samples if abs(sample[0][2] - median[2]) <= 0.12 and
                     math.hypot(sample[0][0] - median[0], sample[0][1] - median[1]) <= 0.35]
        if len(supported) < minimum_samples:
            rejections.append({"bbox": detection["xyxy"], "reason": "inconsistent_surface_depth"})
            continue
        selected = min(supported, key=lambda sample: sum((sample[0][axis] - median[axis]) ** 2 for axis in range(3)))
        if any(sum((selected[0][axis] - candidate["position"][axis]) ** 2 for axis in range(3)) < 0.2 ** 2
               for candidate in candidates):
            rejections.append({"bbox": detection["xyxy"], "reason": "duplicate_surface_candidate"})
            continue
        candidates.append({"bbox": detection["xyxy"], "confidence": detection["confidence"],
                           "position": list(selected[0]), "pixel": [selected[1], selected[2]],
                           "supported_samples": len(supported), "reference": "visible_surface"})
    state = "DETECTIONS" if candidates else "UNRELIABLE" if detections else "NO_DETECTIONS"
    return {"state": state, "candidates": candidates, "rejections": rejections}
