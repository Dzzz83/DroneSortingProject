"""Observed-space goal selection with no scene or obstacle ground truth inputs."""
import math
import unittest
from test_local_planner import SyntheticDepthMap
from drone_sorting.mapping.local_planner import LocalPlanner, VehicleEnvelope
from drone_sorting.mapping.local_goals import LocalGoalPlanner
from drone_sorting.mapping.clearance import ObservedClearance


class LocalGoalTests(unittest.TestCase):
    def setUp(self):
        self.map = SyntheticDepthMap()
        self.map.observed(-8,12,-8,12,2,2)
        self.planner = LocalPlanner(self.map, VehicleEnvelope(.1,.1,.1,.1))
        self.selector = LocalGoalPlanner(self.planner, radius=3.)

    def test_blocked_straight_goal_selects_reachable_detour_and_progresses(self):
        for y in range(-2,3):
            self.map.block(3,y,2)
        start = (.25,.25,1.25)
        target = (5.25,.25,1.25)
        clearance = ObservedClearance(self.map,self.planner.envelope,self.map.evidence)
        goals=[]
        route_points=[]
        for i in range(12):
            result = self.selector.plan(start,target,10+i,self.map.evidence)
            self.assertEqual(result.state,'READY',result.reason)
            self.assertTrue(all(clearance.sweep_free(a,b) for a,b in zip(result.path,result.path[1:])))
            route_points.extend(result.path)
            start = result.path[-1]
            goals.append(start)
            if math.dist(start,target)<1e-6:
                break
        self.assertEqual(start,target,goals)
        self.assertGreater(max(abs(p[1]-.25) for p in route_points), .5)

    def test_unknown_and_expired_evidence_cannot_supply_a_goal(self):
        self.map.evidence.clear()
        result = self.selector.plan((.25,.25,1.25),(5.25,.25,1.25),10,{})
        self.assertEqual(result.state,'HOLD')
        self.assertFalse(result.path)

    def test_cached_goal_is_rechecked_and_reset_discards_history(self):
        result = self.selector.plan((.25,.25,1.25),(10.,.25,1.25),10,self.map.evidence)
        self.assertEqual(result.state,'READY')
        self.assertEqual(self.selector.plan((.25,.25,1.25),(10.,.25,1.25),11,{}).state,'HOLD')
        self.selector.visited.append(result.path[-1])
        self.selector.reset()
        self.assertEqual(self.selector.visited,[])
        self.assertIsNone(self.selector.goal)

    def test_integral_clearance_matches_authoritative_envelope_cells(self):
        self.map.block(1,1,2)
        clearance = ObservedClearance(self.map,self.planner.envelope,self.map.evidence)
        for x in (-.0001,0,.1,.25,.4,.5,.99999,1.,5.9):
            for y in (-.1,.25,.49,.5,.6,1.):
                p = (x,y,1.25)
                expected = all(self.map.evidence.get(c)=='free' for c in self.planner.envelope_cells(p))
                self.assertEqual(clearance.volume_free(p),expected,p)

if __name__=='__main__': unittest.main()
