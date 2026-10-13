#!/usr/bin/env python3
"""Check independent stationary Unity ground truth against the agreed error limit."""

import argparse
import json
import math
import statistics
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--maximum-error", type=float, default=0.1)
    args = parser.parse_args()
    if not math.isfinite(args.maximum_error) or args.maximum_error <= 0:
        parser.error("Maximum error must be a finite positive number")
    rows = json.loads((args.output / "unity-evaluation.json").read_text())["observations"]
    expected = {"three_packages": 3, "changed_placements": 3, "approach": 3, "partial_occlusion": 3,
                "empty": 0, "missing_camera": 0}
    cases, failures = {}, []
    for name, count in expected.items():
        observations = [row for row in rows if row["testCase"] == name]
        errors = [error for row in observations for error in row.get("errorsMeters", []) or []]
        state = "DETECTIONS" if count else "NO_DETECTIONS" if name == "empty" else "UNAVAILABLE"
        minimum_frames = 1 if name == "missing_camera" else 5
        valid = len(observations) >= minimum_frames and all(
            row["state"] == state and row["candidates"] == count and row["matchedPackages"] == count
            and not row.get("failure") and len(row.get("errorsMeters", []) or []) == count
            for row in observations)
        valid = valid and all(0 <= error <= args.maximum_error for error in errors)
        cases[name] = {"frames": len(observations), "expected_candidates": count, "state": state,
                       "surface_samples": len(errors), "maximum_error_meters": max(errors) if errors else None,
                       "passed": valid}
        if not valid:
            failures.append(name)
    # Sequences restart in each Play Mode run. Use the latest record for each sequence.
    latest = {}
    for line in (args.output / "observations.jsonl").read_text().splitlines():
        observation = json.loads(line)
        if observation.get("stamp") is not None:
            latest[observation["sequence"]] = observation
    matched = [latest[row["sequence"]] for row in rows if row["testCase"] != "missing_camera"]
    report = {"passed": not failures, "failed_cases": failures, "maximum_allowed_error_meters": args.maximum_error,
              "position_reference": "visible_surface", "cases": cases,
              "processing_ms_median": statistics.median(row["processing_ms"] for row in matched) if matched else None,
              "observation_age_seconds_maximum": max(row["observation_age_seconds"] for row in matched) if matched else None}
    (args.output / "evaluation-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(bool(failures))


if __name__ == "__main__":
    main()
