import math
import struct
import unittest

from drone_sorting.perception.package_localization import localize_detections


class PackageLocalizationTests(unittest.TestCase):
    def frame(self, surface=True, invalid=False):
        pixels = [float("nan") if invalid else 1.88] * 32 * 32
        if surface:
            for y in range(8, 24):
                for x in range(8, 24):
                    pixels[y * 32 + x] = 1.3
        return (32, 32, 128, False, struct.pack("<1024f", *pixels))

    def localize(self, detections=None, **kwargs):
        if detections is None:
            detections = [{"xyxy": [7, 7, 25, 25], "confidence": 0.9}]
        return localize_detections(detections, self.frame(**kwargs),
                                  [100, 0, 15.5, 0, 100, 15.5, 0, 0, 1],
                                  (34, 25, 2), (1, 0, 0, 0))

    def test_selects_real_surface_sample_and_ignores_floor(self):
        result = self.localize()
        self.assertEqual(result["state"], "DETECTIONS")
        candidate = result["candidates"][0]
        self.assertAlmostEqual(candidate["position"][2], 0.7, places=5)
        self.assertEqual(candidate["reference"], "visible_surface")
        self.assertGreaterEqual(candidate["supported_samples"], 6)
        self.assertTrue(all(math.isfinite(value) for value in candidate["position"]))

    def test_floor_only_detection_is_unreliable_not_empty(self):
        result = self.localize(surface=False)
        self.assertEqual(result["state"], "UNRELIABLE")
        self.assertFalse(result["candidates"])

    def test_valid_no_detection_is_distinct_from_invalid_depth(self):
        self.assertEqual(self.localize([], surface=False)["state"], "NO_DETECTIONS")
        self.assertEqual(self.localize([], surface=False, invalid=True)["state"], "UNRELIABLE")

    def test_invalid_depth_cannot_localize(self):
        self.assertEqual(self.localize(surface=False, invalid=True)["state"], "UNRELIABLE")

    def test_duplicate_candidates_are_removed(self):
        boxes = [{"xyxy": [7, 7, 25, 25], "confidence": confidence} for confidence in [0.8, 0.9]]
        result = self.localize(boxes)
        self.assertEqual(len(result["candidates"]), 1)
        self.assertEqual(result["candidates"][0]["confidence"], 0.9)

    def test_outside_pickup_area_is_rejected(self):
        result = localize_detections([{"xyxy": [7, 7, 25, 25], "confidence": 0.9}], self.frame(),
                                    [100, 0, 15.5, 0, 100, 15.5, 0, 0, 1],
                                    (0, 0, 2), (1, 0, 0, 0))
        self.assertEqual(result["state"], "UNRELIABLE")

    def test_invalid_detection_and_truncated_depth_fail(self):
        for malformed in [None, 42, {"xyxy": [7, 7, 25, 25], "confidence": None}]:
            with self.assertRaises(ValueError):
                self.localize([malformed])
        with self.assertRaises(ValueError):
            self.localize([{"xyxy": [0, 0, 33, 20], "confidence": 0.9}])
        with self.assertRaises(ValueError):
            localize_detections([], (32, 32, 128, False, b""), [1] * 9, (0, 0, 0), (0, 0, 0, 1))


if __name__ == "__main__":
    unittest.main()
