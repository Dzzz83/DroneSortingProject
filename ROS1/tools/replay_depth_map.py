#!/usr/bin/env python3
"""Replay this project's forward-camera recordings into a local map; never commands motion."""
import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from drone_sorting.mapping.depth_map import DepthObservation, LocalDepthMap, MapSettings


def vector(value, fields):
    return tuple(value[field] for field in fields) if isinstance(value, dict) else tuple(value)


def load_observation(path):
    # Both repository formats use ROS warehouse poses and forward optical axes.
    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"]))
            return DepthObservation(metadata["stamp"], data["depth"], tuple(metadata["intrinsics"]),
                                    tuple(metadata["position"]), tuple(metadata["quaternion"]),
                                    tuple(metadata["drone_position"]))
    metadata = json.loads(path.read_text())
    if (metadata["frame_id"] != "warehouse" or metadata["optical_frame"] != "forward_camera_optical"
            or metadata["maximum_depth_m"] != 15 or metadata["evaluation_only"]):
        raise ValueError("Only production forward observations can enter this map")
    depth = np.fromfile(path.with_suffix(".depth.f32"), dtype="<f4").reshape(metadata["height"],metadata["width"])
    return DepthObservation(metadata["seconds"]+metadata["nanoseconds"]/1e9, depth,
                            tuple(metadata["calibration"]), vector(metadata["camera_ros"],"xyz"),
                            vector(metadata["optical_rotation_ros"],"xyzw"), vector(metadata["drone_ros"],"xyz"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--maximum-age", type=float, default=2)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; choose a new evidence directory")
    paths = sorted(args.input.glob("frame-*.npz"))
    if not paths:
        paths = [path.with_name(path.name.replace(".depth.f32", ".json"))
                 for path in args.input.glob("*.depth.f32")
                 if "diagnostic" not in path.name and "probe" not in path.name and "known_target" not in path.name]
    observations = sorted((load_observation(path) for path in paths), key=lambda observation: observation.stamp)
    if not observations:
        parser.error("No project forward-camera recordings found")
    settings = MapSettings(pixel_stride=args.stride, maximum_age=args.maximum_age)
    local = LocalDepthMap(settings)
    durations = []
    for observation in observations:
        start = time.perf_counter()
        local.integrate(observation, observation.stamp)
        durations.append(time.perf_counter()-start)
    records = local.snapshot(observations[-1].stamp)
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output/"cells.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell_x","cell_y","cell_z","ros_x","ros_y","ros_z","state","age_seconds"])
        for cell,state,age in records:
            writer.writerow([*cell,*[(value+.5)*settings.resolution for value in cell],state,age])
    summary = {"frames":len(observations),"settings":asdict(settings),
               "frame_id":"warehouse","centre":list(observations[-1].drone_position),
               "occupied_cells":sum(state=="occupied" for _,state,_ in records),
               "free_ray_cells":sum(state=="free" for _,state,_ in records),
               "integrate_p95_ms":float(np.percentile(durations,95)*1000),
               "integrate_max_ms":max(durations)*1000,
               "cells_after_expiry":len(local.snapshot(observations[-1].stamp+settings.maximum_age+.001)),
               "limits":["Unknown is implicit: absent or expired cells are unknown",
                         "Free cells record rays, not clearance for a drone/package volume",
                         "No-hit and invalid depth never clear cells",
                         "Recorded-time replay only; no ROS publication, planner or movement integration"]}
    (args.output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
