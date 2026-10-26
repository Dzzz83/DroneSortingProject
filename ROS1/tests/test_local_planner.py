"""Focused planning checks using the same snapshot/cell contract as LocalDepthMap."""
import math
import unittest
from types import SimpleNamespace

from drone_sorting.mapping.local_planner import LocalPlanner, VehicleEnvelope


class SyntheticDepthMap:
    """Known-free voxels; every cell not supplied is genuinely unknown."""

    def __init__(self, resolution=0.5):
        self.settings = SimpleNamespace(resolution=resolution)
        self.bounds = ((-20,20),(-20,20),(-8,8))
        self.evidence = {}

    def cell_at(self, point):
        return tuple(math.floor(v/self.settings.resolution) for v in point)

    def snapshot(self, now):
        return [(cell, state, 0) for cell, state in self.evidence.items()]

    def observed(self, xlo, xhi, ylo, yhi, zlo, zhi):
        for x in range(xlo,xhi+1):
            for y in range(ylo,yhi+1):
                for z in range(zlo,zhi+1):
                    self.evidence[(x,y,z)] = "free"

    def block(self, x, y, z):
        self.evidence[(x,y,z)] = "occupied"


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.map = SyntheticDepthMap()
        self.start = (0.25, 0.25, 1.25)
        self.goal = (2.25, 0.25, 1.25)
        self.small = VehicleEnvelope(.1,.1,.1,.1)

    def test_direct_observed_route(self):
        self.map.observed(0, 4, 0, 0, 2, 2)
        result = LocalPlanner(self.map, self.small).plan(self.start, self.goal, 10)
        self.assertEqual(result.state, "READY", result.reason)
        self.assertEqual(result.path[0], self.start)
        self.assertEqual(result.path[-1], self.goal)
        self.assertTrue(all(abs(p[1]-.25) < 1e-9 for p in result.path))

    def test_blocked_route_with_observed_bypass(self):
        self.map.observed(0, 4, 0, 2, 2, 2)
        self.map.block(2, 0, 2)
        result = LocalPlanner(self.map, self.small).plan(self.start, self.goal, 10)
        self.assertEqual(result.state, "READY", result.reason)
        self.assertGreater(max(p[1] for p in result.path), .25)

    def test_unknown_bypass_causes_hold(self):
        self.map.observed(0, 4, 0, 0, 2, 2)
        self.map.block(2, 0, 2)
        result = LocalPlanner(self.map, self.small).plan(self.start, self.goal, 10)
        self.assertEqual((result.state,result.reason), ("HOLD", "no_observed_clear_route"))
        self.assertFalse(result.path)

    def test_passage_narrower_than_vehicle_envelope_holds(self):
        self.map.observed(0, 4, 0, 0, 2, 2)
        wide = VehicleEnvelope(.3,.3,.3,.3)
        result = LocalPlanner(self.map, wide).plan(self.start, self.goal, 10)
        self.assertEqual(result.state, "HOLD")
        self.assertIn("envelope", result.reason)

    def test_unknown_around_single_free_ray_is_not_clearance(self):
        self.map.observed(0, 4, 0, 0, 2, 2)
        result = LocalPlanner(self.map, VehicleEnvelope(.1,.3,.1,.1)).plan(self.start, self.goal, 10)
        self.assertEqual(result.state, "HOLD")

    def test_direct_segment_checks_between_cell_centers(self):
        self.map.observed(0, 4, 0, 0, 2, 2)
        # Even if endpoints are free, crossing an unknown voxel must fail.
        self.map.evidence.pop((2,0,2))
        result = LocalPlanner(self.map, self.small).plan(self.start, self.goal, 10)
        self.assertEqual(result.state, "HOLD")

    def test_no_map_and_unsupported_vertical_goal_hold(self):
        self.map.bounds = None
        result = LocalPlanner(self.map, self.small).plan(self.start, self.goal, 10)
        self.assertEqual(result.reason, "empty_map")
        self.map.bounds = ((-20,20),(-20,20),(-8,8))
        self.map.observed(0, 4, 0, 0, 2, 2)
        result = LocalPlanner(self.map, self.small).plan(self.start, (2.25,.25,2.25), 10)
        self.assertEqual(result.reason, "altitude_change_not_supported")

    def test_invalid_envelope_is_rejected(self):
        with self.assertRaises(ValueError):
            VehicleEnvelope(1,1,1,0)


if __name__ == "__main__":
    unittest.main()
