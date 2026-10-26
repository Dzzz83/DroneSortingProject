"""Coordinator uses physical feedback, resets and timeouts; never package ACKs."""
import json
import unittest
from unittest.mock import Mock,patch
try:
    import rospy
    from geometry_msgs.msg import Point
    from std_msgs.msg import String
    from drone_sorting.navigation.demo import NavigationDemo
except ImportError:
    rospy=None


@unittest.skipIf(rospy is None,'Requires sourced ROS Noetic')
class NavigationDemoTests(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.params={'~reset_on_start':False,'~destination':[1,0,3]}
        self.pubs={}
        def publisher(topic,*args,**kwargs):
            self.pubs[topic]=Mock(); return self.pubs[topic]
        for name,value in [('get_time',lambda:self.now),('get_param',lambda n,d:self.params.get(n,d)),
                           ('Publisher',publisher),('Subscriber',Mock())]:
            p=patch.object(rospy,name,value);p.start();self.addCleanup(p.stop)
        self.node=NavigationDemo()

    def feedback(self,position=(0,0,3),sequence=1,contact=False):
        self.node.receive_course(String(json.dumps(dict(reset_sequence=sequence,collision_observed=contact))))
        self.node.receive_position(Point(*position))

    def test_waits_for_feedback_and_requires_sustained_physical_arrival(self):
        self.assertEqual(self.node.tick(10)['state'],'WAITING')
        self.feedback()
        self.assertEqual(self.node.tick(10)['state'],'NAVIGATING')
        self.now=10.1;self.feedback((1,0,3))
        self.assertNotEqual(self.node.tick(self.now)['state'],'ARRIVED')
        self.now=11.2;self.feedback((1,0,3))
        self.assertEqual(self.node.tick(self.now)['state'],'ARRIVED')

    def test_contact_latches_failure_and_reset_allows_new_run(self):
        self.feedback(contact=True)
        self.assertEqual(self.node.tick(10)['reason'],'contact_detected')
        self.now=11;self.feedback(sequence=2)
        self.assertEqual(self.node.tick(11)['state'],'NAVIGATING')

    def test_stale_position_cannot_confirm_arrival_or_publish_target(self):
        self.feedback((1,0,3));self.node.tick(10)
        count=self.node.target_publisher.publish.call_count
        self.assertNotEqual(self.node.tick(11.2)['state'],'ARRIVED')
        self.assertEqual(self.node.target_publisher.publish.call_count,count)

    def test_start_reset_waits_for_new_epoch_and_new_position(self):
        self.node.reset_on_start=True
        self.feedback();self.node.tick(10)
        self.node.reset_publisher.publish.assert_called_once()
        self.node.receive_course(String('{"reset_sequence":2}'))
        self.assertFalse(self.node.reset_on_start)
        self.assertIsNone(self.node.position)
        self.assertEqual(self.node.tick(10)['state'],'WAITING')

    def test_stall_and_mission_timeouts_stop_target_publication(self):
        self.feedback();self.node.tick(10)
        self.now=101.;self.feedback()
        self.assertEqual(self.node.tick(self.now)['reason'],'no_navigation_progress')
        self.now=102.;self.feedback(sequence=2);self.node.tick(self.now)
        self.node.timeout=1.
        self.now=104.;self.feedback((.6,0,3),sequence=2)
        self.assertEqual(self.node.tick(self.now)['reason'],'mission_timeout')
