import unittest

from package_detector_service import restore_image_boxes


class PaddedDetectionGeometryTests(unittest.TestCase):
    def test_original_sensor_coordinates_and_confidence_are_preserved(self):
        restored = restore_image_boxes([{"xyxy": [220, 165, 260, 185], "confidence": 0.9}],
                                       200, 150, 800, 600)
        self.assertEqual(restored, [{"xyxy": [20, 15, 60, 35], "confidence": 0.9}])

    def test_boxes_crossing_padding_are_clipped_to_sensor_bounds(self):
        restored = restore_image_boxes([{"xyxy": [100, 100, 1100, 900], "confidence": 0.8}],
                                       200, 150, 800, 600)
        self.assertEqual(restored[0]["xyxy"], [0, 0, 800, 600])

    def test_padding_only_boxes_are_discarded(self):
        detections = [{"xyxy": [0, 0, 100, 100], "confidence": 0.9},
                      {"xyxy": [1001, 200, 1100, 400], "confidence": 0.9}]
        self.assertEqual(restore_image_boxes(detections, 200, 150, 800, 600), [])
