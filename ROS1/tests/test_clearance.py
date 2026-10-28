"""Continuous envelope clearance, including brief diagonal voxel intersections."""
import itertools
import random
import unittest
from types import SimpleNamespace

from drone_sorting.mapping.clearance import ObservedClearance
from drone_sorting.mapping.local_planner import VehicleEnvelope


class SweptClearanceTests(unittest.TestCase):
    def setUp(self):
        self.map = SimpleNamespace(settings=SimpleNamespace(resolution=.25),
                                   bounds=((-16, 16),) * 3)
        self.evidence = {cell: 'free' for cell in itertools.product(range(-16, 17), repeat=3)}
        self.envelope = VehicleEnvelope(1.55, 1.40, 1.26, .772)

    def corner_segment(self, entering_axis, leaving_axis, enter_after_exit=False):
        below = (self.envelope.half_x, self.envelope.half_y, self.envelope.below)
        above = (self.envelope.half_x, self.envelope.half_y, self.envelope.above)
        start, end = [.125] * 3, [.125] * 3
        start[entering_axis] = -above[entering_axis] - .02
        end[entering_axis] = -above[entering_axis] + (.005 if enter_after_exit else .04)
        start[leaving_axis] = below[leaving_axis] + .23
        end[leaving_axis] = below[leaving_axis] + .28
        return tuple(start), tuple(end)

    def test_short_diagonals_cannot_skip_occupied_or_unknown_voxels(self):
        for state in ('occupied', 'unknown'):
            self.evidence.pop((0, 0, 0), None)
            if state == 'occupied':
                self.evidence[(0, 0, 0)] = state
            for axes in itertools.permutations(range(3), 2):
                with self.subTest(state=state, axes=axes):
                    clearance = ObservedClearance(self.map, self.envelope, self.evidence)
                    start, end = self.corner_segment(*axes)
                    self.assertTrue(clearance.volume_free(start))
                    self.assertTrue(clearance.volume_free(end))
                    self.assertFalse(clearance.sweep_free(start, end))
                    self.assertFalse(clearance.sweep_free(end, start))

    def test_diagonal_does_not_require_unused_bounding_box_corners_free(self):
        self.evidence[(0, 0, 0)] = 'occupied'
        for axes in itertools.permutations(range(3), 2):
            with self.subTest(axes=axes):
                clearance = ObservedClearance(self.map, self.envelope, self.evidence)
                start, end = self.corner_segment(*axes, enter_after_exit=True)
                self.assertTrue(clearance.sweep_free(start, end))
                self.assertTrue(clearance.sweep_free(end, start))

    def test_face_contact_and_stationary_envelope_keep_boundary_convention(self):
        self.evidence[(1, 1, 0)] = 'occupied'
        clearance = ObservedClearance(self.map, VehicleEnvelope(.125, .125, .125, .125),
                                      self.evidence)
        start, end = (.125, .125, .125), (.375, .125, .125)
        self.assertTrue(clearance.sweep_free(start, end))
        self.assertTrue(clearance.sweep_free(start, start))
        blocked = (.375, .375, .125)
        self.assertFalse(clearance.sweep_free(blocked, blocked))

    def test_axis_aligned_sweep_checks_every_cell_and_map_bounds(self):
        clearance = ObservedClearance(self.map, VehicleEnvelope(.1, .1, .1, .1),
                                      self.evidence)
        self.assertTrue(clearance.sweep_free((.125, .125, .125), (.125, .125, 2.125)))
        self.evidence.pop((0, 0, 4))
        clearance = ObservedClearance(self.map, VehicleEnvelope(.1, .1, .1, .1),
                                      self.evidence)
        self.assertFalse(clearance.sweep_free((.125, .125, .125), (.125, .125, 2.125)))
        self.assertFalse(clearance.sweep_free((.125, .125, .125), (5., .125, .125)))

    def test_sweeps_match_independent_segment_box_intersection(self):
        self.evidence[(0, 0, 0)] = 'occupied'
        clearance = ObservedClearance(self.map, self.envelope, self.evidence)
        e = self.envelope
        # Expand the blocked voxel by the vehicle extents. A root segment
        # intersects this box exactly when the moving envelope overlaps it.
        epsilon = self.map.settings.resolution*1e-8
        limits = ((-e.half_x+epsilon, .25+e.half_x-epsilon),
                  (-e.half_y+epsilon, .25+e.half_y-epsilon),
                  (-e.above+epsilon, .25+e.below-epsilon))
        generator = random.Random(73)
        for _ in range(250):
            start = tuple(generator.uniform(-2.4, 2.4) for _axis in range(3))
            end = tuple(generator.uniform(-2.4, 2.4) for _axis in range(3))
            enter, leave = 0., 1.
            for a, b, (low, high) in zip(start, end, limits):
                crossing = sorted(((low-a)/(b-a), (high-a)/(b-a)))
                enter, leave = max(enter, crossing[0]), min(leave, crossing[1])
            with self.subTest(start=start, end=end):
                self.assertEqual(clearance.sweep_free(start, end), enter >= leave)
