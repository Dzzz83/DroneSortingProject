"""Clearance explanations must agree with conservative planning, without writes."""
import math
import unittest

import numpy as np

from drone_sorting.mapping.depth_map import DepthObservation, LocalDepthMap, MapSettings
from drone_sorting.mapping.local_planner import LocalPlanner, VehicleEnvelope


class ClearanceDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.local = LocalDepthMap()
        self.local.bounds = ((-32, 32), (-32, 32), (-12, 12))
        self.planner = LocalPlanner(self.local, VehicleEnvelope(.25, .25, .25, .25))
        self.start = (0, 0, 1)

    def fill(self):
        for cell in self.planner.envelope_cells(self.start):
            self.local.cells[cell] = [10., -math.inf]

    def test_complete_start_matches_approval_without_mutating_evidence(self):
        self.fill()
        # The planner also certifies its connector to the grid-cell centre.
        for x in range(-1, 2):
            for y in range(-1, 2):
                for z in range(3, 6):
                    self.local.cells[(x, y, z)] = [10., -math.inf]
        before = self.local.snapshot(10)
        report = self.planner.clearance_diagnostics(self.start, 10)
        self.assertEqual((report['total'], report['free'], report['unknown']), (8, 8, 0))
        self.assertEqual(report['outside_bounds'], 0)
        self.assertEqual(self.planner.plan(self.start, self.start, 10).state, 'READY')
        self.assertEqual(self.local.snapshot(10), before)

    def test_unknown_and_occupied_failures_are_separate_and_hold(self):
        self.fill()
        self.local.cells.pop((-1, -1, 3))
        self.local.cells[(0, 0, 4)] = [10., 10.]
        report = self.planner.clearance_diagnostics(self.start, 10, (0, 0, .5), (0, 1, 0))
        self.assertEqual((report['free'], report['unknown'], report['occupied']), (6, 1, 1))
        self.assertEqual(report['failed_samples']['unknown'][0]['cell'], (-1, -1, 3))
        directions = report['failed_by_camera_direction']
        self.assertEqual(directions['behind']['unknown'], 1)
        self.assertEqual(directions['left']['unknown'], 1)
        self.assertEqual(directions['above']['occupied'], 1)
        result = self.planner.plan(self.start, self.start, 10)
        self.assertEqual(result.reason, 'start_envelope_not_observed_free')
        self.assertFalse(result.path)

    def test_expiry_is_unknown_and_only_retained_history_is_identifiable(self):
        self.fill()
        report = self.planner.clearance_diagnostics(self.start, 12.01)
        self.assertEqual((report['unknown'], report['retained_expired_unknown']), (8, 8))
        self.assertEqual(self.planner.plan(self.start, self.start, 12.01).state, 'HOLD')
        self.local.cells.clear()  # Same loss of provenance as integration pruning.
        report = self.planner.clearance_diagnostics(self.start, 12.01)
        self.assertEqual((report['unknown'], report['retained_expired_unknown']), (8, 0))

    def test_bounds_reset_and_sample_limit(self):
        self.local.bounds = ((0, 32), (0, 32), (-12, 12))
        report = self.planner.clearance_diagnostics(self.start, 10)
        self.assertEqual(report['outside_bounds'], 6)
        self.assertEqual(len(report['failed_samples']['unknown']), 6)
        self.fill()
        self.local.reset()
        report = self.planner.clearance_diagnostics(self.start, 10)
        self.assertEqual((report['unknown'], report['outside_bounds']), (8, 8))
        self.assertEqual(report['retained_expired_unknown'], 0)
        self.assertEqual(self.planner.plan(self.start, self.start, 10).reason, 'empty_map')

    def test_exact_and_negative_voxel_boundaries(self):
        self.assertEqual(set(self.planner.envelope_cells(self.start)),
                         {(x, y, z) for x in (-1, 0) for y in (-1, 0) for z in (3, 4)})
        shifted = tuple(v-1e-5 for v in self.start)
        self.assertEqual(len(list(self.planner.envelope_cells(shifted))), 27)

    def test_real_lens_yaw_views_cannot_observe_near_axis_vertical_volume(self):
        # Unity ConfigureLens(320, 240, 90) sets VERTICAL FOV. Supply finite
        # surfaces in every pixel (more useful returns than an empty/no-hit view).
        local = LocalDepthMap(MapSettings(pixel_stride=8))
        root, camera = (34., 50., 2.), (34., 50., 1.5)
        intrinsics = (120., 0., 160., 0., 120., 120., 0., 0., 1.)
        image = np.full((240, 320), 4., dtype=np.float32)
        for index, yaw in enumerate((0, math.pi/2, math.pi, 3*math.pi/2)):
            # World-Z yaw composed with optical-forward -> ROS +Y rotation.
            s, c = math.sin(yaw/2), math.cos(yaw/2)
            q = (-c*math.sqrt(.5), -s*math.sqrt(.5), s*math.sqrt(.5), c*math.sqrt(.5))
            stamp = 10 + index*.1
            local.integrate(DepthObservation(stamp, image, intrinsics, camera, q, root), stamp)
        planner = LocalPlanner(local, VehicleEnvelope(1.55, 1.40, 1.26, .772))
        report = planner.clearance_diagnostics(root, 10.3, camera, (0, 1, 0))
        self.assertEqual(report['total'], 1680)
        self.assertGreater(report['free'], 0)
        self.assertGreater(report['unknown'], 0)
        self.assertEqual(report['occupied'], 0)
        self.assertEqual(report['outside_bounds'], 0)
        self.assertEqual(report['retained_expired_unknown'], 0)
        # This whole voxel is inside the envelope, outside every level yaw view:
        # minimum vertical separation 1 m > max horizontal distance sqrt(2)*.25.
        self.assertIn((136, 200, 10), set(planner.envelope_cells(root)))
        self.assertEqual(local.state_at((34.125, 50.125, 2.625), 10.3), 'unknown')
        self.assertEqual(planner.plan(root, (34, 51, 2), 10.3).reason,
                         'start_envelope_not_observed_free')

        # Two additional genuine pitched observations resolve the synthetic
        # blind spot, without bootstrapping any cells around the vehicle.
        for index, q in enumerate(((0,0,0,1), (1,0,0,0))):
            stamp = 10.4+index*.1
            local.integrate(DepthObservation(stamp, image, intrinsics, camera, q, root), stamp)
        report = planner.clearance_diagnostics(root, 10.5)
        self.assertEqual((report['free'], report['unknown']), (1680, 0))
        self.assertEqual(planner.plan(root, (34,51,2), 10.5).state, 'READY')


if __name__ == '__main__':
    unittest.main()
