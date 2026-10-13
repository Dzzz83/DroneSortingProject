#!/usr/bin/env python3
"""Check editor-exported YOLO labels against visible-pixel masks (requires Pillow).

Scene metadata and masks are evaluation ground truth, never detector inputs.
"""

import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

from PIL import Image


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify(directory):
    directory = Path(directory)
    require((directory / "COMPLETE.txt").is_file(), "Export did not complete.")
    manifest = json.loads((directory / "manifest.json").read_text())
    require(manifest["classId"] == 0 and manifest["className"] == "package", "Unexpected class contract.")
    size = (manifest["imageWidth"], manifest["imageHeight"])
    observations = manifest["observations"]
    names = [observation["name"] for observation in observations]
    training_trial = manifest.get("datasetKind", "pilot") == "training-trial"
    expected_images = 200 if training_trial else 16
    require(len(names) == expected_images and len(set(names)) == expected_images,
            f"Expected {expected_images} distinct views.")
    splits = verify_splits(observations) if training_trial else None
    image_hashes = {}
    total_boxes = 0
    empty_views = 0
    for observation in observations:
        name = observation["name"]
        parts = Path(name).parts
        if training_trial:
            require(len(parts) == 2 and parts[0] == observation["split"] and
                    parts[1] not in [".", ".."], "Invalid observation filename.")
        else:
            require(Path(name).name == name, "Invalid observation filename.")
        image_path = directory / "images" / (name + ".png")
        if training_trial:
            digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
            require(digest not in image_hashes, name + ": duplicate RGB capture: " + image_hashes.get(digest, ""))
            image_hashes[digest] = name
        with Image.open(directory / "images" / (name + ".png")) as image:
            require(image.size == size, name + ": wrong image dimensions.")
        with Image.open(directory / "masks" / (name + ".png")) as image:
            require(image.size == size, name + ": wrong mask dimensions.")
            mask = image.convert("RGB")
        with Image.open(directory / "overlays" / (name + ".png")) as image:
            require(image.size == size, name + ": wrong overlay dimensions.")
        lines = (directory / "labels" / (name + ".txt")).read_text().splitlines()
        boxes = observation["boxes"]
        require(len(lines) == len(boxes), name + ": label count differs from manifest.")
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        palette = mask.getcolors(maxcolors=4)
        require(palette is not None and all(color == (0, 0, 0) or color in colors for _, color in palette),
                name + ": unexpected mask color.")
        channels = mask.split()
        present = {index for index, channel in enumerate(channels) if channel.getbbox() is not None}
        require(present == {box["packageIndex"] for box in boxes}, name + ": mask/box IDs differ.")
        for line, box in zip(lines, boxes):
            fields = line.split()
            require(len(fields) == 5 and fields[0] == "0", name + ": invalid YOLO class/row.")
            coordinates = [float(value) for value in fields[1:]]
            require(all(0 <= value <= 1 for value in coordinates), name + ": invalid normalized coordinate.")
            x, y, width, height = coordinates
            require(width > 0 and height > 0, name + ": empty bounding box.")
            require(x - width / 2 >= -1e-7 and x + width / 2 <= 1 + 1e-7 and
                    y - height / 2 >= -1e-7 and y + height / 2 <= 1 + 1e-7,
                    name + ": box outside image.")
            channel = channels[box["packageIndex"]]
            left, top, right, bottom = channel.getbbox()
            mask_width = right - left
            mask_height = bottom - top
            require((left, top, mask_width, mask_height) ==
                    (box["left"], box["top"], box["width"], box["height"]),
                    name + ": manifest bounds differ from visible pixels.")
            require(channel.histogram()[255] == box["visiblePixels"], name + ": visible-pixel count differs.")
            expected = [(left + mask_width / 2) / size[0], (top + mask_height / 2) / size[1],
                        mask_width / size[0], mask_height / size[1]]
            require(all(abs(actual - reference) < 1e-6 for actual, reference in zip(coordinates, expected)),
                    name + ": YOLO bounds differ from mask (including image row orientation).")
        if not boxes:
            empty_views += 1
        if "empty" in name or "full-occlusion" in name:
            require(not boxes, name + ": hidden/absent objects must not be labeled.")
        if "partial-occlusion" in name:
            require(len(boxes) == 1, name + ": partially visible package missing.")
        if "image-edge" in name:
            require(len(boxes) == 1, name + ": edge package missing.")
            box = boxes[0]
            require(box["left"] == 0 or box["left"] + box["width"] == size[0],
                    name + ": expected clipped edge box.")
        total_boxes += len(boxes)
    result = {"images": len(observations), "boxes": total_boxes, "empty_views": empty_views}
    if training_trial:
        result["splits"] = splits
    return result


def verify_splits(observations):
    """Reject shared configurations/placements across train, validation, and test."""
    expected = {"train": 140, "val": 30, "test": 30}
    counts = Counter(observation["split"] for observation in observations)
    require(dict(counts) == expected, "Expected train/val/test counts of 140/30/30.")
    groups = defaultdict(list)
    signatures = {}
    for observation in observations:
        groups[observation["configurationId"]].append(observation)
        signature = json.dumps([observation["packagePositionsUnity"], observation["packageRotationsUnity"],
                                observation["packageActive"]], sort_keys=True)
        prior = signatures.setdefault(signature, observation["configurationId"])
        require(prior == observation["configurationId"], "Package configuration duplicated under another ID.")
    require(len(groups) == 100, "Expected 100 independent configuration groups.")
    for group in groups.values():
        require(len(group) == 2 and {item["view"] for item in group} == {"observation", "approach"},
                "Each configuration must have one observation and one approach view.")
        require(len({item["split"] for item in group}) == 1, "Configuration leaks across dataset splits.")
        for key in ["packagePositionsUnity", "packageRotationsUnity", "packageActive"]:
            require(group[0][key] == group[1][key], "Paired views must share package configuration.")
    return dict(counts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    arguments = parser.parse_args()
    print("PACKAGE_DATASET_PILOT_VERIFIED", json.dumps(verify(arguments.directory)))
