"""ROS adapter tests: planner is a passive consumer of the existing depth map."""
import json
import math
import unittest
from unittest.mock import Mock, patch

try:
    import rospy
    from geometry_msgs.msg import Point
    from drone_sorting.interfaces import ros_topics as topics
    from drone_sorting.mapping.live_mapping import DepthMappingNode
except ImportError:
    rospy = None


@unittest.skipIf(rospy is None, "Requires sourced ROS Noetic")
class LiveLocalPlannerTests(unittest.TestCase):
    def create_node(self, envelope=None, carried=None, select_goals=False):
        self.now = 100.0
        self.publishers = {}

        def publisher(topic, *args, **kwargs):
            self.publishers[topic] = Mock()
            return self.publishers[topic]

        def param(name, default):
            if name == "~select_local_goals":
                return select_goals
            if name == "~planner_envelope":
                return envelope
            if name == "~planner_carried_envelope":
                return carried
            return default

        for name, replacement in (("Publisher", publisher), ("Subscriber", Mock()),
                                  ("Timer", Mock()), ("on_shutdown", Mock()),
                                  ("get_time", lambda: self.now), ("get_param", param)):
            p = patch.object(rospy, name, replacement)
            p.start()
            self.addCleanup(p.stop)
        return DepthMappingNode()

    def plan_status(self):
        message = self.publishers[topics.LOCAL_PLAN_STATUS].publish.call_args[0][0]
        return json.loads(message.data)

    def test_no_configured_envelope_returns_hold_and_no_motion_publishers(self):
        node = self.create_node()
        node.receive_goal(Point(.875, .125, .625))
        node.process_once(self.now)
        self.assertEqual(self.plan_status()["reason"], "envelope_not_configured")
        self.assertEqual(self.publishers[topics.LOCAL_PLAN_PATH].publish.call_args[0][0].poses, [])
        for name in (topics.COMMAND_POSITION, topics.TARGET_POSITION,
                     topics.COMMAND_SPEED, topics.PACKAGE_ACTION):
            self.assertNotIn(name, self.publishers)

    def test_observed_route_published_only_as_nav_path(self):
        node = self.create_node([.05, .05, .05, .05])
        node.map.bounds = ((0, 4), (0, 0), (2, 2))
        for x in range(4):
            node.map.cells[(x, 0, 2)] = [100.0, -math.inf]
        node.latest_position = (.125, .125, .625)
        node.last_stamp = 100.0
        node.problem = None
        node.receive_goal(Point(.875, .125, .625))
        node.process_once(100)
        self.assertEqual(self.plan_status()["state"], "READY", self.plan_status())
        self.assertEqual(self.plan_status()["start_clearance"]["free"], 1)
        route = self.publishers[topics.LOCAL_PLAN_PATH].publish.call_args[0][0]
        self.assertEqual(route.header.frame_id, "warehouse")
        self.assertGreaterEqual(len(route.poses), 2)
        self.assertNotIn(topics.COMMAND_POSITION, self.publishers)

        # A stale camera pose prevents route publication even though voxels have
        # not reached their longer evidence expiry time.
        node.process_once(100.6)
        self.assertEqual(self.plan_status()["state"], "HOLD")
        self.assertEqual(self.plan_status()["reason"], "map_stale_observation")
        self.assertEqual(self.publishers[topics.LOCAL_PLAN_PATH].publish.call_args[0][0].poses, [])

    def test_autonomous_plan_distinguishes_destination_from_selected_endpoint(self):
        node = self.create_node([.05,.05,.05,.05],select_goals=True)
        node.map.bounds = ((0,31),(0,0),(2,2))
        node.map.cells = {(x,0,2):[100.,-math.inf] for x in range(32)}
        node.latest_position = (.125,.125,.625)
        node.last_stamp = 100.
        node.problem = None
        node.receive_goal(Point(10.125,.125,.625))
        node.process_once(100.)
        status = self.plan_status()
        self.assertEqual(status['state'],'READY',status)
        self.assertEqual(status['request_goal'],[10.125,.125,.625])
        self.assertLess(status['goal'][0],10.125)
        route = self.publishers[topics.LOCAL_PLAN_PATH].publish.call_args[0][0]
        self.assertEqual(route.poses[-1].pose.position.x,status['goal'][0])

    def test_start_hold_publishes_bounded_authoritative_diagnostics(self):
        node = self.create_node([.25, .25, .25, .25])
        node.map.bounds = ((0, 280), (0, 280), (0, 30))
        node.map.cells[(136, 200, 8)] = [100., -math.inf]
        node.latest_position = (34., 50., 2.)
        node.latest_camera = ((34., 50., 1.5), (0., 1., 0.))
        node.last_stamp = 100.
        node.problem = None
        node.receive_goal(Point(34., 51., 2.))
        node.process_once(100.)
        status = self.plan_status()
        self.assertEqual(status['reason'], 'start_envelope_not_observed_free')
        report = status['start_clearance']
        self.assertEqual((report['total'], report['free'], report['unknown']), (8, 1, 7))
        self.assertEqual(report['camera_position'], [34., 50., 1.5])
        self.assertLessEqual(len(report['failed_samples']['unknown']), 6)
        self.assertEqual(self.publishers[topics.LOCAL_PLAN_PATH].publish.call_args[0][0].poses, [])

    def test_planning_cannot_publish_ready_after_the_acquisition_expires(self):
        node = self.create_node([.05, .05, .05, .05])
        node.map.bounds = ((0, 4), (0, 0), (2, 2))
        for x in range(4):
            node.map.cells[(x, 0, 2)] = [100., -math.inf]
        node.latest_position = (.125, .125, .625)
        node.last_stamp = 100.
        node.problem = None
        node.receive_goal(Point(.875, .125, .625))
        plan = node.planner.plan
        def slow_plan(*args, **kwargs):
            result = plan(*args, **kwargs)
            self.assertEqual(result.state, 'READY')
            self.now = 100.6
            return result
        with patch.object(node.planner, 'plan', side_effect=slow_plan):
            node.process_once()
        status = self.plan_status()
        self.assertEqual((status['state'], status['reason']), ('HOLD', 'map_stale_observation'))
        self.assertEqual(status['observation_stamp'], 100.)
        self.assertEqual(status['stamp'], 100.)
        self.assertAlmostEqual(status['timing_ms']['observation_age_at_publish_ms'], 600.)
        self.assertEqual(self.publishers[topics.LOCAL_PLAN_PATH].publish.call_args[0][0].poses, [])

    def test_reset_revokes_previous_goal(self):
        from std_msgs.msg import String
        node = self.create_node([.05, .05, .05, .05])
        node.receive_course(String('{"reset_sequence":0}'))
        node.receive_goal(Point(.875, .125, .625))
        node.receive_course(String('{"reset_sequence":1}'))
        self.assertIsNone(node.local_goal)
        node.process_once(self.now)
        self.assertEqual(self.plan_status()["reason"], "no_local_goal")
        self.assertNotIn('start_clearance', self.plan_status())
        self.assertIsNone(node.latest_camera)

    def test_attached_package_requires_its_own_envelope(self):
        from std_msgs.msg import String
        node = self.create_node([.25, .25, .25, .25])
        node.receive_course(String('{"reset_sequence":0,"package_attached":true}'))
        node.process_once(self.now)
        self.assertEqual(self.plan_status()['reason'], 'carried_envelope_not_configured')
        self.assertTrue(self.plan_status()['package_attached'])

    def test_package_feedback_switches_between_measured_envelopes(self):
        from std_msgs.msg import String
        node = self.create_node([.25, .25, .25, .25], [.25, .25, .5, .25])
        node.receive_course(String('{"reset_sequence":0,"package_attached":true}'))
        self.assertEqual(node.planner.envelope.below, .5)
        node.receive_course(String('{"reset_sequence":0,"package_attached":false}'))
        self.assertEqual(node.planner.envelope.below, .25)

    def test_carried_envelope_cannot_reduce_the_unloaded_extents(self):
        with self.assertRaises(ValueError):
            self.create_node([.25, .25, .25, .25], [.1, .25, .5, .25])


if __name__ == "__main__":
    unittest.main()
