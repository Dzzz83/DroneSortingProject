import math
import struct
import unittest

from drone_sorting.perception.camera_geometry import depth_at_pixel, deproject_pixel, transform_point


class CameraGeometryTests(unittest.TestCase):
    def test_top_down_depth_and_endianness(self):
        for bigendian, prefix in ((False, "<"), (True, ">")):
            data = struct.pack(prefix + "4f", 1, 2, 3, 4)
            self.assertEqual(depth_at_pixel(data, 2, 2, 8, bigendian, 1, 0), 2)
            self.assertEqual(depth_at_pixel(data, 2, 2, 8, bigendian, 0, 1), 3)

    def test_padded_depth_row(self):
        data = struct.pack("<2f", 1, 2) + b"xxxx" + struct.pack("<2f", 3, 4) + b"xxxx"
        self.assertEqual(depth_at_pixel(data, 2, 2, 12, False, 1, 1), 4)

    def test_invalid_depth(self):
        for value in (float("nan"), float("inf"), 0, -1):
            self.assertIsNone(depth_at_pixel(struct.pack("<f", value), 1, 1, 4, False, 0, 0))

    def test_reject_bad_layout_and_bounds(self):
        for arguments in ((b"", 1, 1, 4, False, 0, 0),
                          (bytes(4), 1, 1, 3, False, 0, 0),
                          (bytes(4), 1, 1, 4, False, 1, 0)):
            with self.assertRaises(ValueError):
                depth_at_pixel(*arguments)

    def test_deprojection(self):
        k = (200, 0, 399.5, 0, 200, 299.5, 0, 0, 1)
        self.assertEqual(deproject_pixel(399.5, 299.5, 2, k), (0, 0, 2))
        self.assertEqual(deproject_pixel(499.5, 349.5, 2, k), (1, 0.5, 2))

    def test_reject_bad_intrinsics(self):
        for k in ((), (0,) * 9, (float("nan"),) * 9):
            with self.assertRaises(ValueError):
                deproject_pixel(0, 0, 2, k)

    def test_rigid_transform(self):
        q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        result = transform_point((1, 0, 0), (34, 25, 6), q)
        for actual, expected in zip(result, (34, 26, 6)):
            self.assertAlmostEqual(actual, expected)

    def test_reject_invalid_quaternion(self):
        with self.assertRaises(ValueError):
            transform_point((0, 0, 1), (0, 0, 0), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
