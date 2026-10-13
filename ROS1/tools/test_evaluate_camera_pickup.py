"""Synthetic evidence cases for the physical checker; none represent a live pickup."""

import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from evaluate_camera_pickup import evaluate


class PhysicalPickupEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.manifest = {"started": 100, "timeout": 150, "lift_distance": 1,
                         "tolerance": 0.05, "hold_grace": 0.5}
        self.events = [
            {"kind": "action", "received": 101, "payload": "PICKUP"},
            {"kind": "acknowledgement", "received": 102, "payload": "PICKUP_DONE"},
            {"kind": "guidance", "received": 102.1, "payload": {"phase": "LIFT"}},
            {"kind": "guidance", "received": 103, "payload": {"phase": "VERIFIED"}},
        ]
        self.samples = [self.sample(101.9, "WAIT_FOR_PICKUP", 0, holding=True),
                        self.sample(102.1, "LIFT", 0.03),
                        self.sample(103, "VERIFIED", 1),
                        self.sample(103.2, "VERIFIED", 1),
                        self.sample(103.4, "VERIFIED", 1)]

    @staticmethod
    def sample(stamp, phase, rise, holding=False):
        return {"stamp": stamp, "status_age": 0.01, "phase": phase,
                "selected_package": "package-a", "held_package": "package-a", "attached": True,
                "drone_position": [34, 25, 1.39 + rise], "package_position": [34, 25, 0.44 + rise],
                "holding": holding, "failure": ""}

    def report(self):
        return evaluate(self.manifest, self.samples, self.events)

    def test_selected_package_attaches_and_rises_with_drone(self):
        report = self.report()
        self.assertTrue(report["passed"], report)
        self.assertAlmostEqual(report["measurements"]["package_rise_meters"], 1)

    def test_terminal_phase_without_attachment_cannot_pass(self):
        for sample in self.samples:
            sample["attached"] = False
            sample["held_package"] = ""
        self.assertIn("selected_package_never_attached", self.report()["failures"])

    def test_wrong_package_and_changed_selection_fail(self):
        self.samples[-1]["held_package"] = "package-b"
        self.assertIn("wrong_package_attached", self.report()["failures"])
        self.samples[-1]["selected_package"] = "package-b"
        self.assertIn("missing_or_changed_selected_package", self.report()["failures"])

    def test_lift_requires_request_and_acknowledgement(self):
        self.events = [event for event in self.events if event["kind"] != "acknowledgement"]
        self.assertIn("missing_pickup_acknowledgement", self.report()["failures"])
        self.assertIn("lift_before_acknowledgement", self.report()["failures"])

    def test_duplicate_pickup_and_early_acknowledgement_fail(self):
        self.events.append(copy.deepcopy(self.events[0]))
        self.assertIn("expected_one_pickup_request", self.report()["failures"])
        self.events[1]["received"] = 100.5
        self.assertIn("acknowledgement_before_request", self.report()["failures"])

    def test_insufficient_lift_or_package_not_following_drone_fails(self):
        self.samples[-1]["package_position"][2] = 0.54
        failures = self.report()["failures"]
        self.assertIn("insufficient_physical_lift", failures)
        self.assertIn("package_did_not_follow_drone", failures)

    def test_attachment_loss_and_single_favorable_sample_fail(self):
        self.samples[-1]["attached"] = False
        self.assertIn("attachment_lost", self.report()["failures"])
        self.samples = self.samples[:3]
        self.assertIn("lift_not_sustained", self.report()["failures"])

    def test_hold_allows_inflight_grace_then_measures_drift(self):
        hold = [self.sample(100.1, "OBSERVE", 0, True),
                self.sample(100.4, "OBSERVE", 0.1, True),
                self.sample(100.7, "OBSERVE", 0.1, True),
                self.sample(100.9, "OBSERVE", 0.1, True)]
        for sample in hold:
            sample["attached"] = False
            sample["held_package"] = ""
        approach = self.sample(101, "APPROACH", 0)
        approach["attached"] = False
        approach["held_package"] = ""
        self.samples = hold + [approach] + self.samples
        self.assertTrue(self.report()["passed"])
        self.samples[3]["drone_position"][0] += 0.1
        self.assertIn("movement_during_hold", self.report()["failures"])

    def test_stale_status_timeout_and_unity_failure_fail(self):
        self.samples[-1]["status_age"] = 2
        self.samples[-1]["stamp"] = 251
        self.samples[-1]["failure"] = "association failed"
        failures = self.report()["failures"]
        self.assertIn("stale_terminal_status", failures)
        self.assertIn("trial_timeout", failures)
        self.assertIn("unity_evaluation_failure", failures)

    def test_missing_nonfinite_and_invalid_dimensions_fail(self):
        self.assertFalse(evaluate(self.manifest, [], self.events)["passed"])
        self.samples[-1]["package_position"][0] = float("nan")
        self.assertIn("nonfinite_evidence", self.report()["failures"])
        self.samples[-1]["package_position"] = [0, 1]
        self.assertIn("invalid_position_dimensions", self.report()["failures"])

    def test_physical_rise_before_acknowledgement_fails(self):
        self.samples.insert(1, self.sample(101.95, "WAIT_FOR_PICKUP", 0.2))
        self.assertIn("physical_lift_before_acknowledgement", self.report()["failures"])

    def test_missing_guidance_events_fails(self):
        self.events = [event for event in self.events if event["kind"] != "guidance"]
        self.assertIn("missing_guidance_events", self.report()["failures"])

    def test_cli_accepts_complete_synthetic_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "COMPLETE.txt").write_text("Synthetic fixture only")
            (output / "manifest.json").write_text(json.dumps(self.manifest))
            for name, rows in (("unity-samples.jsonl", self.samples), ("ros-events.jsonl", self.events)):
                (output / name).write_text("".join(json.dumps(row) + "\n" for row in rows))
            completed = subprocess.run([sys.executable, str(Path(__file__).with_name("evaluate_camera_pickup.py")), directory],
                                       capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_cli_rejects_incomplete_export(self):
        with tempfile.TemporaryDirectory() as directory:
            completed = subprocess.run([sys.executable, str(Path(__file__).with_name("evaluate_camera_pickup.py")), directory],
                                       capture_output=True, text=True)
            self.assertNotEqual(completed.returncode, 0)
            report = json.loads((Path(directory) / "evaluation-report.json").read_text())
            self.assertEqual(report["failures"], ["invalid_or_incomplete_evidence"])


if __name__ == "__main__":
    unittest.main()
