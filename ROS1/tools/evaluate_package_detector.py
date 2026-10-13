#!/usr/bin/env python3
"""Held-out RGB detection evaluation. No localization, ROS, or movement commands."""

import argparse
import hashlib
import json
import statistics
import time
from pathlib import Path

from PIL import Image, ImageDraw


def intersection_over_union(first, second):
    width = max(0, min(first[2], second[2]) - max(first[0], second[0]))
    height = max(0, min(first[3], second[3]) - max(first[1], second[1]))
    intersection = width * height
    first_area = max(0, first[2] - first[0]) * max(0, first[3] - first[1])
    second_area = max(0, second[2] - second[0]) * max(0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


def match_detections(detections, targets, threshold=0.5):
    """Confidence-ordered one-to-one matching; duplicate detections count as false positives."""
    remaining = set(range(len(targets)))
    matched_ious = []
    for detection in sorted(detections, key=lambda item: item["confidence"], reverse=True):
        if not remaining:
            continue
        best = max(remaining, key=lambda index: intersection_over_union(detection["xyxy"], targets[index]))
        overlap = intersection_over_union(detection["xyxy"], targets[best])
        if overlap >= threshold:
            remaining.remove(best)
            matched_ious.append(overlap)
    true_positives = len(matched_ious)
    return {"tp": true_positives, "fp": len(detections) - true_positives,
            "fn": len(targets) - true_positives, "matched_ious": matched_ious}


def summarize(rows):
    tp = sum(row["tp"] for row in rows)
    fp = sum(row["fp"] for row in rows)
    fn = sum(row["fn"] for row in rows)
    overlaps = [value for row in rows for value in row["matched_ious"]]
    return {"images": len(rows), "tp": tp, "fp": fp, "fn": fn,
            "precision": tp / (tp + fp) if tp + fp else 0,
            "recall": tp / (tp + fn) if tp + fn else 0,
            "mean_matched_iou": statistics.mean(overlaps) if overlaps else None,
            "empty_images": sum(not row["targets"] for row in rows),
            "empty_images_with_false_detections": sum(not row["targets"] and bool(row["detections"]) for row in rows)}


def main():
    import torch
    import ultralytics
    from ultralytics import YOLO

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--weights", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()
    dataset, weights, output = (path.resolve() for path in [arguments.dataset, arguments.weights, arguments.output])
    if output.exists():
        raise FileExistsError("Choose a fresh evaluation directory; test results must not be overwritten.")
    if not weights.is_file():
        raise FileNotFoundError("Local trained checkpoint required.")
    torch.set_num_threads(6)
    model = YOLO(str(weights))
    if model.names != {0: "package"}:
        raise ValueError("Expected the single package class.")
    # AP metrics sweep confidence; the diagnostic operating threshold below is fixed beforehand.
    metrics = model.val(data=str(dataset / "packages.yaml"), split="test", imgsz=640, batch=4,
                        device=0, workers=0, plots=True, project=str(output.parent), name=output.name,
                        exist_ok=False, conf=0.001, half=False)
    manifest = json.loads((dataset / "manifest.json").read_text())
    rows = []
    (output / "annotated").mkdir()
    for observation in manifest["observations"]:
        if observation["split"] != "test":
            continue
        name = observation["name"]
        image_path = dataset / "images" / (name + ".png")
        with Image.open(image_path) as source:
            image = source.convert("RGB")
        targets = []
        for line in (dataset / "labels" / (name + ".txt")).read_text().splitlines():
            _, x, y, width, height = map(float, line.split())
            targets.append([(x - width / 2) * image.width, (y - height / 2) * image.height,
                            (x + width / 2) * image.width, (y + height / 2) * image.height])
        result = model.predict(str(image_path), imgsz=640, conf=0.25, device=0, verbose=False)[0]
        detections = [{"xyxy": box.xyxy[0].tolist(), "confidence": float(box.conf.item())} for box in result.boxes]
        row = {"name": name, "view": observation["view"], "targets": targets, "detections": detections,
               **match_detections(detections, targets)}
        rows.append(row)
        drawing = ImageDraw.Draw(image)
        for target in targets:
            drawing.rectangle(target, outline="lime", width=2)
        for detection in detections:
            drawing.rectangle(detection["xyxy"], outline="red", width=2)
            drawing.text(detection["xyxy"][:2], f"package {detection['confidence']:.2f}", fill="red")
        image.save(output / "annotated" / (Path(name).name + ".png"))
    reference_row = next(row for row in rows if row["view"] == "observation" and row["targets"])
    reference = str(dataset / "images" / (reference_row["name"] + ".png"))
    for _ in range(3):
        model.predict(reference, imgsz=640, conf=0.25, device=0, verbose=False)
    timings = []
    for _ in range(20):
        torch.cuda.synchronize()
        start = time.perf_counter()
        model.predict(reference, imgsz=640, conf=0.25, device=0, verbose=False)
        torch.cuda.synchronize()
        timings.append((time.perf_counter() - start) * 1000)
    report = {"weights": str(weights), "weights_sha256": hashlib.sha256(weights.read_bytes()).hexdigest(),
              "manifest_sha256": hashlib.sha256((dataset / "manifest.json").read_bytes()).hexdigest(),
              "ultralytics": ultralytics.__version__, "torch": torch.__version__,
              "confidence_threshold": 0.25, "matching_iou_threshold": 0.5,
              "standard_ap_metrics": {key: float(value) for key, value in metrics.results_dict.items()},
              "fixed_threshold": summarize(rows),
              "by_view": {view: summarize([row for row in rows if row["view"] == view])
                          for view in ["observation", "approach"]},
              "warmed_predict_ms": {"image": reference_row["name"], "frames": 20, "mean": statistics.mean(timings),
                                    "median": statistics.median(timings), "min": min(timings), "max": max(timings)},
              "images": rows}
    (output / "evaluation-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print("PACKAGE_DETECTOR_TEST_COMPLETE", json.dumps(report["fixed_threshold"]), flush=True)


if __name__ == "__main__":
    main()
