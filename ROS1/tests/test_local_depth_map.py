"""Geometric and temporal safety boundaries of the recorded-data local map."""
import math
import unittest

import numpy as np

from drone_sorting.mapping.depth_map import DepthObservation, LocalDepthMap, MapSettings


class LocalDepthMapTests(unittest.TestCase):
    def observation(self, depth=3., stamp=10., quaternion=None, position=(.125,.125,1.125), root=(.125,.125,1.625)):
        image = np.full((3,3), np.nan, dtype=np.float32)
        image[1,1] = depth
        q = quaternion or (-math.sqrt(.5),0,0,math.sqrt(.5))
        return DepthObservation(stamp,image,(2,0,1,0,2,1,0,0,1),position,q,root)

    def local(self):
        return LocalDepthMap(MapSettings(pixel_stride=1))

    def test_finite_surface_free_ray_and_occluded_unknown_have_correct_world_positions(self):
        local = self.local()
        local.integrate(self.observation(),10)
        self.assertEqual(local.state_at((.125,3.125,1.125),10),"occupied")
        self.assertEqual(local.state_at((.125,1.125,1.125),10),"free")
        self.assertEqual(local.state_at((.125,4.125,1.125),10),"unknown")
        self.assertEqual(local.state_at((1.125,1.125,1.125),10),"unknown")

    def test_invalid_and_no_hit_do_not_grant_free_space_or_a_vehicle_bootstrap(self):
        for value in (np.nan,np.inf):
            with self.subTest(value=value):
                local=self.local(); local.integrate(self.observation(value),10)
                self.assertEqual(local.snapshot(10),[])
                self.assertEqual(local.state_at((.125,.125,1.625),10),"unknown")

    def test_optical_right_down_forward_transform_into_ros_axes(self):
        observation=self.observation()
        image=np.full((3,3),np.nan,dtype=np.float32); image[0,2]=3
        observation=DepthObservation(10,image,observation.intrinsics,observation.position,
                                     observation.quaternion,observation.drone_position)
        points,_=observation.surface_points(1)
        np.testing.assert_allclose(points,[[1.625,3.125,2.625]],atol=1e-6)

    def test_surface_uncertainty_blocks_neighbouring_cells(self):
        # Place the surface near a voxel boundary so its uncertainty spans two cells.
        local=self.local(); local.integrate(self.observation(position=(.24,.125,1.125)),10)
        self.assertEqual(local.state_at((.04,3.12,1.12),10),"occupied")
        self.assertEqual(local.state_at((.3,3.12,1.12),10),"occupied")

    def test_recent_views_survive_camera_turn_but_expire_to_unknown(self):
        local=self.local();local.integrate(self.observation(),10)
        local.integrate(self.observation(stamp=10.2,quaternion=(0,0,0,1)),10.2)
        self.assertEqual(local.state_at((.125,3.125,1.125),10.2),"occupied")
        self.assertEqual(local.snapshot(12.201),[])

    def test_occupied_evidence_wins_until_its_own_age_expires(self):
        local=self.local();local.integrate(self.observation(),10)
        local.integrate(self.observation(depth=5,stamp=11),11)
        self.assertEqual(local.state_at((.125,3.125,1.125),11),"occupied")
        self.assertEqual(local.state_at((.125,3.125,1.125),12.01),"free")
        self.assertEqual(local.state_at((.125,3.125,1.125),13.01),"unknown")

    def test_stale_future_and_replayed_data_cannot_refresh_map(self):
        local=self.local();local.integrate(self.observation(),10)
        before=local.snapshot(10)
        for observation,now in [(self.observation(stamp=9),10),(self.observation(stamp=11),10),
                                (self.observation(stamp=11),14)]:
            with self.subTest(stamp=observation.stamp,now=now),self.assertRaises(ValueError):
                local.integrate(observation,now)
        self.assertEqual(local.snapshot(10),before)

    def test_reset_discards_previous_trial_and_accepts_a_new_acquisition_epoch(self):
        local=self.local();local.integrate(self.observation(),10)
        local.reset()
        self.assertEqual(local.snapshot(10),[])
        local.integrate(self.observation(stamp=1),1)
        self.assertEqual(local.state_at((.125,3.125,1.125),1),"occupied")

    def test_moving_window_removes_old_geometry(self):
        local=self.local();local.integrate(self.observation(),10)
        local.integrate(self.observation(stamp=11,position=(100,100,2.5),root=(100,100,3)),11)
        self.assertEqual(local.state_at((.125,3.125,1.125),11),"unknown")

    def test_negative_cell_coordinates_and_corner_traversal(self):
        local=self.local()
        self.assertEqual(local.cell_at((-.001,.25,-.25)),(-1,1,-1))
        cells=list(local._ray_cells((.125,.125,.125),(.625,.625,.625)))
        self.assertEqual(cells,[(0,0,0),(1,1,1),(2,2,2)])
        self.assertEqual(list(local._ray_cells((0,0,0),(-.3,0,0))),[(-1,0,0),(-2,0,0)])

    def test_observation_owns_depth_copy_and_rejects_bad_metadata(self):
        observation=self.observation()
        with self.assertRaises(ValueError): observation.depth[0,0]=1
        for depth in (-1,.1,16,-np.inf):
            with self.subTest(depth=depth),self.assertRaises(ValueError): self.observation(depth)
        with self.assertRaises(ValueError):self.observation(quaternion=(0,0,0,4))
        with self.assertRaises(ValueError):MapSettings(maximum_age=-1)


if __name__=="__main__":unittest.main()
