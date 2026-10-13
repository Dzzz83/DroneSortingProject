import unittest

from evaluate_package_detector import intersection_over_union, match_detections


class DetectionMatchingTests(unittest.TestCase):
    def test_overlap_and_non_overlap(self):
        self.assertAlmostEqual(intersection_over_union([0, 0, 2, 2], [1, 0, 3, 2]), 1 / 3)
        self.assertEqual(intersection_over_union([0, 0, 1, 1], [2, 2, 3, 3]), 0)

    def test_duplicate_detection_is_false_positive(self):
        detections = [{"confidence": 0.9, "xyxy": [0, 0, 2, 2]},
                      {"confidence": 0.8, "xyxy": [0, 0, 2, 2]}]
        result = match_detections(detections, [[0, 0, 2, 2]])
        self.assertEqual((result["tp"], result["fp"], result["fn"]), (1, 1, 0))

    def test_empty_image_false_positive(self):
        result = match_detections([{"confidence": 0.8, "xyxy": [0, 0, 2, 2]}], [])
        self.assertEqual((result["tp"], result["fp"], result["fn"]), (0, 1, 0))

    def test_missed_target_and_insufficient_overlap(self):
        result = match_detections([{"confidence": 0.8, "xyxy": [1, 0, 3, 2]}], [[0, 0, 2, 2]])
        self.assertEqual((result["tp"], result["fp"], result["fn"]), (0, 1, 1))


if __name__ == "__main__":
    unittest.main()
