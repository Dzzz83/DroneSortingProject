"""No ROS dependency: explicit path/goal/epoch freshness and segment safety."""
import unittest
from drone_sorting.control.depth_follower import DepthPathFollower


class DepthFollowerTests(unittest.TestCase):
    def setUp(self):
        self.follower = DepthPathFollower()
        self.start = (0.0, 0.0, 1.0)
        self.goal = (1.0, 1.0, 1.0)
        self.path = (self.start, (1.0, 0.0, 1.0), self.goal)
        self.follower.receive_path(self.path, 10.0)
        self.follower.receive_status({"state":"READY", "stamp":10.0,
            "observation_stamp":10.0, "goal":list(self.goal),
            "reset_sequence":5, "reason":"observed_route"})

    def decide(self, command=None, observed=None, goal=None, seq=5, now=10.1, step=.1):
        return self.follower.next_step(command or self.start, observed or self.start,
                                       goal or self.goal, seq, now, step)

    def test_follows_certified_segments_not_corner_cutting(self):
        x=self.decide()
        self.assertEqual(x.state, "MOVE")
        self.assertEqual(x.next_position,(.1,0,1))
        x=self.decide(command=(.95,0,1),observed=(0,0,1),step=.2)
        self.assertEqual(x.next_position,(1,0,1))
        x=self.decide(command=(1,0,1),observed=(0,0,1))
        self.assertEqual(x.next_position,(1,.1,1))

    def test_no_unapproved_diagonal_join(self):
        self.assertEqual(self.decide(command=(.25,.1,1)).reason,"command_not_on_approved_path")

    def test_stale_plan_and_stale_sensor_hold(self):
        self.assertEqual(self.decide(now=10.4).reason,"route_stale")
        self.follower.status["observation_stamp"]=9
        self.assertEqual(self.decide().reason,"route_stale")

    def test_old_epoch_mismatched_goal_and_stamp_hold(self):
        self.assertEqual(self.decide(seq=6).reason,"route_epoch_mismatch")
        self.assertEqual(self.decide(goal=(1,2,1)).reason,"route_goal_mismatch")
        self.follower.receive_path(self.path,9.8)
        self.assertEqual(self.decide().reason,"route_epoch_mismatch")

    def test_invalid_missing_or_hold_status_fails_closed(self):
        self.follower.receive_status({"state":"HOLD", "reason":"no_observed_clear_route"})
        self.assertEqual(self.decide().state,"HOLD")
        self.follower.receive_status({"state":"READY"})
        self.assertEqual(self.decide().reason,"invalid_planner_status")
        self.follower.reset()
        self.assertEqual(self.decide().reason,"awaiting_approved_path")

    def test_pose_mismatch_and_invalid_path_fail_closed(self):
        self.assertEqual(self.decide(observed=(2,2,1)).reason,"route_start_mismatch")
        self.follower.receive_path(((0,0,float('nan')), (1,1,1)), 10)
        self.assertEqual(self.decide().state,"HOLD")

if __name__ == '__main__': unittest.main()
