#!/usr/bin/env python3
"""Judge physical pickup or delivery evidence independently of terminal status.

Unity (x, y, z) positions are saved as ROS (x, z, y): ROS Z is vertical.
Receipt timestamps share UTC wall time with Unity samples. In-flight hold motion
is measured after a grace interval, rather than assumed to be instantaneous.
"""

import argparse
import json
import math
from pathlib import Path


def distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def evaluate(manifest, samples, events):
    """Return explicit failure reasons and measurements for one bounded trial."""
    failures = []
    measurements = {}
    delivery = manifest.get("mission") == "delivery"
    if not samples:
        return {"passed": False, "failures": ["missing_unity_samples"], "measurements": {}}
    numeric_values = [manifest[key] for key in ("started", "timeout", "lift_distance", "tolerance", "hold_grace")]
    for sample in samples:
        numeric_values.extend([sample["stamp"], sample["status_age"], *sample["drone_position"], *sample["package_position"]])
        if len(sample["drone_position"]) != 3 or len(sample["package_position"]) != 3:
            failures.append("invalid_position_dimensions")
    numeric_values.extend(event["received"] for event in events)
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in numeric_values):
        failures.append("nonfinite_evidence")
    if failures:
        return {"passed": False, "failures": failures, "measurements": measurements}
    if not (0 < manifest["tolerance"] < manifest["lift_distance"] and manifest["timeout"] > 0 and manifest["hold_grace"] >= 0):
        return {"passed": False, "failures": ["invalid_trial_limits"], "measurements": {}}
    if any(a["stamp"] > b["stamp"] for a, b in zip(samples, samples[1:])):
        failures.append("unordered_unity_samples")
    started = manifest["started"]
    events = sorted((event for event in events if event["received"] >= started), key=lambda event: event["received"])
    requests = [event for event in events if event["kind"] == "action" and event["payload"] == "PICKUP"]
    acknowledgements = [event for event in events if event["kind"] == "acknowledgement" and event["payload"] == "PICKUP_DONE"]
    if len(requests) != 1:
        failures.append("expected_one_pickup_request")
    if not acknowledgements:
        failures.append("missing_pickup_acknowledgement")
    elif not requests or acknowledgements[0]["received"] < requests[0]["received"]:
        failures.append("acknowledgement_before_request")
    selected_ids = {sample["selected_package"] for sample in samples if sample["selected_package"]}
    if len(selected_ids) != 1:
        failures.append("missing_or_changed_selected_package")
    if any(sample.get("failure") for sample in samples):
        failures.append("unity_evaluation_failure")
    if any(sample["held_package"] and sample["held_package"] != sample["selected_package"] for sample in samples):
        failures.append("wrong_package_attached")
    attached = [sample for sample in samples if sample["attached"]]
    drop_requests = [event for event in events if event["kind"] == "action" and event["payload"] == "DROP"]
    release_time = drop_requests[0]["received"] if delivery and drop_requests else math.inf
    if not attached:
        failures.append("selected_package_never_attached")
    elif any(not sample["attached"] for sample in samples if attached[0]["stamp"] <= sample["stamp"] < release_time):
        failures.append("attachment_lost")
    terminal = samples[-1]
    expected_terminal = "COMPLETE" if delivery else "VERIFIED"
    if terminal["phase"] != expected_terminal:
        failures.append("guidance_did_not_finish")
    if terminal["status_age"] > 0.5:
        failures.append("stale_terminal_status")
    if terminal["stamp"] - started > manifest["timeout"]:
        failures.append("trial_timeout")
    lift_phases = ("LIFT", "CARRY_ASCEND", "CARRY") if delivery else ("LIFT", "VERIFIED")
    lifting = [sample for sample in samples if sample["phase"] in lift_phases]
    if lifting and (not acknowledgements or lifting[0]["stamp"] < acknowledgements[0]["received"]):
        failures.append("lift_before_acknowledgement")
    phases = [event["payload"].get("phase") for event in events if event["kind"] == "guidance"]
    lift_finish_phase = "CARRY_ASCEND" if delivery else "VERIFIED"
    if "LIFT" not in phases or lift_finish_phase not in phases:
        failures.append("missing_guidance_events")
    if attached:
        baseline = attached[0]
        lift_terminal = terminal
        if delivery:
            lift_terminal = next((sample for sample in samples if sample["phase"] == lift_finish_phase), terminal)
        package_rise = lift_terminal["package_position"][2] - baseline["package_position"][2]
        drone_rise = lift_terminal["drone_position"][2] - baseline["drone_position"][2]
        measurements.update(package_rise_meters=package_rise, drone_rise_meters=drone_rise)
        if acknowledgements and any(sample["package_position"][2] - baseline["package_position"][2] > manifest["tolerance"]
                                    for sample in attached if sample["stamp"] < acknowledgements[0]["received"]):
            failures.append("physical_lift_before_acknowledgement")
        required = manifest["lift_distance"] - manifest["tolerance"]
        if package_rise < required or drone_rise < required:
            failures.append("insufficient_physical_lift")
        if abs(package_rise - drone_rise) > manifest["tolerance"]:
            failures.append("package_did_not_follow_drone")
        # Require a short sustained result, not one favorable sample.
        sustained_phases = ("CARRY_ASCEND", "CARRY") if delivery else ("VERIFIED",)
        settled = [sample for sample in samples if sample["phase"] in sustained_phases and sample["attached"]
                   and sample["package_position"][2] - baseline["package_position"][2] >= required]
        if len(settled) < 3 or settled[-1]["stamp"] - settled[0]["stamp"] < 0.2:
            failures.append("lift_not_sustained")
    if delivery:
        failures.extend(evaluate_delivery(manifest, samples, events, attached, drop_requests, measurements))
    if manifest.get("start_from_drone_start"):
        failures.extend(evaluate_access(manifest, samples, events))
    hold_distances = []
    hold_start = None
    hold_position = None
    for sample in samples:
        if sample["holding"]:
            if hold_start is None:
                hold_start = sample["stamp"]
            if sample["stamp"] - hold_start >= manifest["hold_grace"]:
                if hold_position is None:
                    hold_position = sample["drone_position"]
                hold_distances.append(distance(sample["drone_position"], hold_position))
        else:
            hold_start = hold_position = None
    measurements["maximum_hold_drift_meters"] = max(hold_distances, default=0)
    if measurements["maximum_hold_drift_meters"] > manifest["tolerance"]:
        failures.append("movement_during_hold")
    return {"passed": not failures, "failures": failures, "measurements": measurements}


def evaluate_access(manifest, samples, events):
    """Require measured start, ordered access, and sustained observation hover."""
    failures = []
    tolerance = manifest["tolerance"]
    observation = manifest["observation_position"]
    if distance(samples[0]["drone_position"], manifest["start_position"]) > tolerance:
        failures.append("wrong_initial_drone_position")
    phases = []
    for sample in samples:
        if not phases or phases[-1] != sample["phase"]:
            phases.append(sample["phase"])
    required = ["ASCEND_TO_OBSERVATION", "GO_TO_OBSERVATION", "HOVER", "OBSERVE", "APPROACH"]
    relevant = [phase for phase in phases if phase in required]
    if relevant != required:
        failures.append("missing_or_unordered_access_stages")
    access = [sample for sample in samples if sample["phase"] in required[:3]]
    start = manifest["start_position"]
    for sample in access:
        position = sample["drone_position"]
        if sample["phase"] == "ASCEND_TO_OBSERVATION" and distance(position[:2], start[:2]) > tolerance:
            failures.append("lateral_motion_before_ascent")
            break
        if sample["phase"] == "GO_TO_OBSERVATION" and abs(position[2] - observation[2]) > tolerance:
            failures.append("travel_below_observation_height")
            break
    hovering = [sample for sample in samples if sample["phase"] in ("HOVER", "OBSERVE")]
    if (not hovering or hovering[-1]["stamp"] - hovering[0]["stamp"] < manifest["hover_time"]
            or any(distance(sample["drone_position"], observation) > tolerance for sample in hovering)):
        failures.append("observation_hover_not_verified")
    observe = next((sample["stamp"] for sample in samples if sample["phase"] == "OBSERVE"), math.inf)
    if any(event["received"] < observe for event in events if event["kind"] == "action" and event["payload"] == "PICKUP"):
        failures.append("pickup_before_observation")
    return failures


def evaluate_delivery(manifest, samples, events, attached, requests, measurements):
    """Require continuous carry, ordered release, and sustained placement inside the zone."""
    failures = []
    acknowledgements = [event for event in events if event["kind"] == "acknowledgement" and event["payload"] == "DROP_DONE"]
    if len(requests) != 1:
        failures.append("expected_one_drop_request")
    if len(acknowledgements) != 1:
        failures.append("expected_one_drop_acknowledgement")
    request_time = requests[0]["received"] if requests else math.inf
    acknowledgement_time = acknowledgements[0]["received"] if acknowledgements else math.inf
    if acknowledgement_time < request_time:
        failures.append("drop_acknowledgement_before_request")
    if requests and (not attached or request_time <= attached[0]["stamp"]):
        failures.append("drop_before_pickup")
    if any(sample["phase"] in ("SETTLE", "COMPLETE") and sample["stamp"] < acknowledgement_time for sample in samples):
        failures.append("completion_before_drop_acknowledgement")
    if attached:
        offset = [a - b for a, b in zip(attached[0]["package_position"], attached[0]["drone_position"])]
        drift = max(distance([a - b for a, b in zip(sample["package_position"], sample["drone_position"])], offset)
                    for sample in attached)
        measurements["maximum_carry_drift_meters"] = drift
        if drift > manifest["tolerance"]:
            failures.append("package_did_not_follow_during_carry")
    if any(sample.get("collisions") for sample in samples):
        failures.append("physical_collision")
    zone_min, zone_max = manifest["zone_min"], manifest["zone_max"]
    if len(zone_min) != 3 or len(zone_max) != 3 or not all(math.isfinite(value) for value in
            (*zone_min, *zone_max, manifest["settled_center_height"])) or any(zone_min[axis] >= zone_max[axis] for axis in (0, 1)):
        raise ValueError("Invalid destination geometry")
    settled = []
    invalid_placement = False
    for sample in samples:
        if sample["phase"] != "COMPLETE":
            continue
        numeric = (*sample["package_bounds_min"], *sample["package_bounds_max"], sample["linear_speed"], sample["angular_speed"])
        if len(sample["package_bounds_min"]) != 3 or len(sample["package_bounds_max"]) != 3 or not all(math.isfinite(value) for value in numeric):
            raise ValueError("Invalid physical drop evidence")
        inside = all(sample["package_bounds_min"][axis] >= zone_min[axis] and
                     sample["package_bounds_max"][axis] <= zone_max[axis] for axis in (0, 1))
        on_floor = abs(sample["package_position"][2] - manifest["settled_center_height"]) <= manifest["tolerance"]
        detached = sample["released"] and not sample["attached"] and not sample["held_package"]
        stopped = sample["linear_speed"] <= 0.05 and sample["angular_speed"] <= 0.1
        if detached and inside and on_floor and stopped:
            settled.append(sample)
        else:
            invalid_placement = True
    if invalid_placement:
        failures.append("invalid_final_package_placement")
    if len(settled) < 3 or settled[-1]["stamp"] - settled[0]["stamp"] < 0.5:
        failures.append("drop_not_sustained")
    measurements["final_package_position"] = samples[-1]["package_position"]
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        if not (args.output / "COMPLETE.txt").exists():
            raise ValueError("Trial did not finish scene restoration")
        if (args.output / "harness-failure.txt").exists():
            raise ValueError("Unity harness reported a failure; inspect harness-failure.txt")
        manifest = json.loads((args.output / "manifest.json").read_text())
        samples = [json.loads(line) for line in (args.output / "unity-samples.jsonl").read_text().splitlines()]
        events = [json.loads(line) for line in (args.output / "ros-events.jsonl").read_text().splitlines()]
        report = evaluate(manifest, samples, events)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        report = {"passed": False, "failures": ["invalid_or_incomplete_evidence"], "detail": str(error)}
    (args.output / "evaluation-report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(not report["passed"])


if __name__ == "__main__":
    main()
