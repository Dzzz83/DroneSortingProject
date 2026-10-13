import math
import unittest

from drone_sorting.perception.package_grasp import fit_top_face


class PackageGraspTests(unittest.TestCase):
    def square(self, angle=0, cropped=False):
        cosine, sine = math.cos(angle), math.sin(angle)
        points = []
        for row in range(25):
            for column in range(25):
                x, y = column * 0.025 - 0.3, row * 0.025 - 0.3
                if cropped and x < 0:
                    continue
                points.append((34 + cosine * x - sine * y, 25 + sine * x + cosine * y, 0.74))
        return points

    def test_rotated_box_center(self):
        for angle in (0, 0.31, 0.8):
            result = fit_top_face(self.square(angle))
            self.assertTrue(result["ready"])
            for value, expected in zip(result["center"], (34, 25, 0.44)):
                self.assertAlmostEqual(value, expected, places=5)

    def test_partial_face_is_not_a_grasp(self):
        self.assertFalse(fit_top_face(self.square(cropped=True))["ready"])
        self.assertFalse(fit_top_face([])["ready"])

    def test_hollow_face_is_rejected(self):
        points = [p for p in self.square() if abs(p[0] - 34) > 0.25 or abs(p[1] - 25) > 0.25]
        self.assertFalse(fit_top_face(points)["ready"])
