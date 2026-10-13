#!/usr/bin/env python3
"""Offline YOLO fine-tuning; never connects to ROS or uses test images for tuning."""

import argparse
import hashlib
import json
import time
from pathlib import Path

import torch
import ultralytics
from ultralytics import YOLO

from verify_package_dataset_pilot import verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--weights", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=50)
    arguments = parser.parse_args()
    dataset = arguments.dataset.resolve()
    weights = arguments.weights.resolve()
    output = arguments.output.resolve()
    if not weights.is_file():
        raise FileNotFoundError("Local pretrained weights are required; no automatic download: " + str(weights))
    if output.exists():
        raise FileExistsError("Choose a new training output directory: " + str(output))
    if not torch.cuda.is_available():
        raise RuntimeError("This trial requires the verified CUDA environment.")
    checks = verify(dataset)
    if checks.get("splits") != {"train": 140, "val": 30, "test": 30}:
        raise ValueError("Training requires the checked 200-image dataset, not the pilot.")
    torch.set_num_threads(6)
    configuration = dict(data=str(dataset / "packages.yaml"), epochs=arguments.epochs,
                         imgsz=640, batch=4, device=0, workers=0, cache=False,
                         seed=20261012, deterministic=True, optimizer="AdamW", lr0=0.001,
                         patience=arguments.epochs, amp=False, mosaic=0.5, close_mosaic=10,
                         plots=True, project=str(output.parent), name=output.name, exist_ok=False)
    model = YOLO(str(weights))
    started = time.perf_counter()
    model.train(**configuration)
    report = {"elapsed_seconds": time.perf_counter() - started, "configuration": configuration,
              "dataset_checks": checks, "torch": torch.__version__, "ultralytics": ultralytics.__version__,
              "gpu": torch.cuda.get_device_name(0), "source_weights": str(weights),
              "source_weights_sha256": hashlib.sha256(weights.read_bytes()).hexdigest(),
              "dataset_manifest_sha256": hashlib.sha256((dataset / "manifest.json").read_bytes()).hexdigest(),
              "best_checkpoint": str(output / "weights/best.pt")}
    (output / "training-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print("PACKAGE_DETECTOR_TRAINING_COMPLETE", report["best_checkpoint"], flush=True)


if __name__ == "__main__":
    main()
