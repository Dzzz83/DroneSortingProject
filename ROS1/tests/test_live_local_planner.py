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
    def create_node(self, envelope=None):
        self.now = 100.0
        self.publishers = {}

        def publisher(topic, *args, **kwargs):
            self.publishers[topic] = Mock()
            return self.publishers[topic]

        def param(name, default):
            if name == "~planner_envelope":
                return envelope
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

    def test_reset_revokes_previous_goal(self):
        from std_msgs.msg import String
        node = self.create_node([.05, .05, .05, .05])
        node.receive_course(String('{"reset_sequence":0}'))
        node.receive_goal(Point(.875, .125, .625))
        node.receive_course(String('{"reset_sequence":1}'))
        self.assertIsNone(node.local_goal)
        node.process_once(self.now)
        self.assertEqual(self.plan_status()["reason"], "no_local_goal")


if __name__ == "__main__":
    unittest.main()
