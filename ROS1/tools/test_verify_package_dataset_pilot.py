"""Small independent fixtures for the offline label checker (requires Pillow)."""

import json
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from verify_package_dataset_pilot import verify, verify_splits


class PilotVerificationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        for folder in ["images", "masks", "overlays", "labels"]:
            (self.root / folder).mkdir()
        observations = []
        for index in range(16):
            suffix = {10: "empty", 11: "empty", 12: "partial-occlusion",
                      13: "full-occlusion", 14: "image-edge", 15: "image-edge"}.get(index, "placement")
            name = f"{index:02}-{suffix}"
            mask = Image.new("RGB", (8, 6))
            boxes = []
            label = ""
            if index not in [10, 11, 13]:
                # Deliberately off-center vertically, to expose an inverted row convention.
                x = 0 if index == 14 else 7 if index == 15 else 2
                mask.putpixel((x, 1), (255, 0, 0))
                boxes = [{"packageIndex": 0, "visiblePixels": 1, "left": x,
                          "top": 1, "width": 1, "height": 1}]
                label = f"0 {(x + 0.5) / 8:.8f} 0.25000000 0.12500000 0.16666667\n"
            mask.save(self.root / "masks" / (name + ".png"))
            for folder in ["images", "overlays"]:
                Image.new("RGB", (8, 6)).save(self.root / folder / (name + ".png"))
            (self.root / "labels" / (name + ".txt")).write_text(label)
            observations.append({"name": name, "boxes": boxes})
        (self.root / "manifest.json").write_text(json.dumps({"classId": 0, "className": "package",
            "imageWidth": 8, "imageHeight": 6, "observations": observations}))
        (self.root / "COMPLETE.txt").touch()

    def test_valid_visible_empty_occluded_and_edge_labels(self):
        self.assertEqual(verify(self.root), {"images": 16, "boxes": 13, "empty_views": 3})

    def test_rejects_binary_classification_label(self):
        (self.root / "labels/00-placement.txt").write_text("1 0.3125 0.25 0.125 0.16666667\n")
        with self.assertRaisesRegex(ValueError, "class/row"):
            verify(self.root)

    def test_rejects_bottom_up_coordinate(self):
        (self.root / "labels/00-placement.txt").write_text("0 0.3125 0.75 0.125 0.16666667\n")
        with self.assertRaisesRegex(ValueError, "row orientation"):
            verify(self.root)

    def test_rejects_nan_and_out_of_bounds_coordinates(self):
        for coordinate in ["nan", "1.1"]:
            with self.subTest(coordinate=coordinate):
                (self.root / "labels/00-placement.txt").write_text(f"0 {coordinate} 0.25 0.125 0.16666667\n")
                with self.assertRaisesRegex(ValueError, "normalized coordinate"):
                    verify(self.root)

    def test_rejects_label_on_empty_image(self):
        (self.root / "labels/10-empty.txt").write_text("0 0.5 0.5 0.1 0.1\n")
        with self.assertRaisesRegex(ValueError, "label count"):
            verify(self.root)

    def test_rejects_incomplete_export(self):
        (self.root / "COMPLETE.txt").unlink()
        with self.assertRaisesRegex(ValueError, "did not complete"):
            verify(self.root)

    def trial_observations(self):
        observations = []
        for group in range(100):
            split = "train" if group < 70 else "val" if group < 85 else "test"
            for view in ["observation", "approach"]:
                observations.append({"split": split, "configurationId": str(group), "view": view,
                                     "packagePositionsUnity": [group], "packageRotationsUnity": [0],
                                     "packageActive": [True]})
        return observations

    def test_valid_configuration_split(self):
        self.assertEqual(verify_splits(self.trial_observations()), {"train": 140, "val": 30, "test": 30})

    def test_rejects_configuration_leakage_even_with_correct_split_counts(self):
        observations = self.trial_observations()
        observations[0]["split"], observations[-1]["split"] = observations[-1]["split"], observations[0]["split"]
        with self.assertRaisesRegex(ValueError, "leaks"):
            verify_splits(observations)

    def test_rejects_duplicate_placement_under_different_configuration_id(self):
        observations = self.trial_observations()
        observations[-1]["packagePositionsUnity"] = observations[0]["packagePositionsUnity"]
        with self.assertRaisesRegex(ValueError, "duplicated"):
            verify_splits(observations)


if __name__ == "__main__":
    unittest.main()
