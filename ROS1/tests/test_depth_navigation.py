"""ROS-independent transport smoke test via bounded message stubs."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class FakeStamp:
    def __init__(self,v): self.v=v
    def to_sec(self): return self.v


class FakePublisher:
    def __init__(self): self.sent=[]
    def publish(self,v): self.sent.append(v)


class FakePoint:
    def __init__(self,x=0,y=0,z=0): self.x=x;self.y=y;self.z=z


class FakeVector(FakePoint): pass


class FakeHeader:
    def __init__(self,frame_id='warehouse',stamp=10):
        self.frame_id=frame_id;self.stamp=FakeStamp(stamp)


class FakePath:
    def __init__(self,points,stamp=10):
        self.header=FakeHeader(stamp=stamp)
        self.poses=[]
        for p in points:
            item=types.SimpleNamespace(header=FakeHeader(stamp=stamp),
                 pose=types.SimpleNamespace(position=FakePoint(*p)))
            self.poses.append(item)


class FakeString:
    def __init__(self,data):self.data=data


class DepthNavigationTests(unittest.TestCase):
    def setUp(self):
        self.clock=10.0
        self.pubs={}
        rospy=types.ModuleType('rospy')
        rospy.get_param=lambda name,default:default
        rospy.get_time=lambda:self.clock
        rospy.Publisher=lambda name,*args,**kw:self.pubs.setdefault(name,FakePublisher())
        rospy.Subscriber=lambda *args,**kw:None
        geometry=types.ModuleType('geometry_msgs');geommsg=types.ModuleType('geometry_msgs.msg')
        geommsg.Point=FakePoint;geommsg.Vector3=FakeVector
        nav=types.ModuleType('nav_msgs');navmsg=types.ModuleType('nav_msgs.msg');navmsg.Path=FakePath
        std=types.ModuleType('std_msgs');stdmsg=types.ModuleType('std_msgs.msg');stdmsg.String=FakeString
        interfaces=types.ModuleType('drone_sorting.interfaces')
        topics=types.ModuleType('drone_sorting.interfaces.ros_topics')
        for name in ('LOCAL_PLAN_GOAL', 'LOCAL_PLAN_SCAN_HEADING', 'LOCAL_PLAN_PATH', 'LOCAL_PLAN_STATUS', 'COURSE_OBSERVATIONS'):
            setattr(topics,name,name)
        sources={'drone_sorting.interfaces':interfaces,'drone_sorting.interfaces.ros_topics':topics,
                 'rospy':rospy,'geometry_msgs':geometry,'geometry_msgs.msg':geommsg,
                 'nav_msgs':nav,'nav_msgs.msg':navmsg,'std_msgs':std,'std_msgs.msg':stdmsg}
        with patch.dict(sys.modules,sources):
            source=Path(__file__).parents[1]/'package/python/drone_sorting/control/depth_navigation.py'
            spec=importlib.util.spec_from_file_location('depth_nav_test_'+self._testMethodName,source)
            self.module=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.node=self.module.DepthNavigation()
        self.topics=self.module.topics
        self.node.receive_course(FakeString('{"reset_sequence":1,"collision_observed":false}'))

    def test_no_path_holds_and_requests_local_goal_and_camera(self):
        d=self.node.tick((0,0,1),(0,0,1),(4,0,1),10,.01)
        self.assertEqual(d.state,'HOLD')
        self.assertEqual(self.pubs[self.topics.LOCAL_PLAN_GOAL].sent[-1].x,3)
        self.assertEqual(len(self.pubs[self.topics.LOCAL_PLAN_SCAN_HEADING].sent),1)

    def test_approved_path_only_and_epoch_reset(self):
        self.node.tick((0,0,1),(0,0,1),(4,0,1),10,.01)
        self.node.receive_path(FakePath([(0,0,1),(3,0,1)],stamp=10))
        self.node.receive_status(FakeString(json.dumps({'state':'READY','stamp':10,
            'observation_stamp':10,'goal':[3,0,1],'reset_sequence':1})))
        d=self.node.tick((0,0,1),(0,0,1),(4,0,1),10.1,.01)
        self.assertEqual(d.state,'MOVE')
        self.assertEqual(d.next_position,(.01,0,1))
        self.node.receive_course(FakeString('{"reset_sequence":2,"collision_observed":false}'))
        self.assertEqual(self.node.tick((0,0,1),(0,0,1),(4,0,1),10.2,.01).state,'HOLD')

    def test_final_goal_keeps_approval_inside_intermediate_arrival_radius(self):
        target = (1, 0, 1)
        start = (.9, 0, 1)
        self.node.tick(start, start, target, 10, .01)
        self.node.receive_path(FakePath([start, target], stamp=10))
        self.node.receive_status(FakeString(json.dumps({'state':'READY', 'stamp':10,
            'observation_stamp':10, 'goal':list(target), 'reset_sequence':1})))
        decision = self.node.tick(start, start, target, 10.1, .01)
        self.assertEqual(decision.state, 'MOVE', decision.reason)
        self.assertAlmostEqual(decision.next_position[0], .91)

    def test_intermediate_goal_requests_next_leg_and_revokes_old_approval(self):
        self.node.tick((0,0,1), (0,0,1), (4,0,1), 10, .01)
        self.node.receive_path(FakePath([(0,0,1),(3,0,1)], stamp=10))
        self.node.receive_status(FakeString(json.dumps({'state':'READY', 'stamp':10,
            'observation_stamp':10, 'goal':[3,0,1], 'reset_sequence':1})))
        self.clock = 10.1
        decision = self.node.tick((2.9,0,1), (2.9,0,1), (4,0,1), 10.1, .01)
        self.assertEqual(decision.state, 'HOLD')
        self.assertEqual(self.node.local_goal, (4,0,1))
        self.assertEqual(self.pubs[self.topics.LOCAL_PLAN_GOAL].sent[-1].x, 4)

    def test_autonomous_mode_accepts_only_matching_destination_and_epoch(self):
        self.node.autonomous_goals = True
        target = (10,0,1)
        self.node.tick((0,0,1),(0,0,1),target,10,.01)
        self.assertEqual(self.pubs[self.topics.LOCAL_PLAN_GOAL].sent[-1].x,10)
        self.node.receive_path(FakePath([(0,0,1),(1,1,1)]))
        status = dict(state='READY',stamp=10,observation_stamp=10,
                      request_goal=list(target),goal=[1,1,1],reset_sequence=1)
        self.node.receive_status(FakeString(json.dumps(status)))
        decision = self.node.tick((0,0,1),(0,0,1),target,10.1,.01)
        self.assertEqual(decision.state,'MOVE',decision.reason)
        status['request_goal'] = [11,0,1]
        self.node.receive_status(FakeString(json.dumps(status)))
        self.assertEqual(self.node.tick((0,0,1),(0,0,1),target,10.1,.01).state,'HOLD')
        status['request_goal'] = list(target)
        status['reset_sequence'] = 2
        self.node.receive_status(FakeString(json.dumps(status)))
        self.assertIsNone(self.node.follower.status)

    def test_contact_latches_until_reset(self):
        self.node.receive_course(FakeString('{"reset_sequence":1,"collision_observed":true}'))
        self.assertEqual(self.node.tick((0,0,1),(0,0,1),(4,0,1),10,.01).reason,'contact_detected')
        self.node.receive_course(FakeString('{"reset_sequence":2,"collision_observed":false}'))
        self.assertNotEqual(self.node.tick((0,0,1),(0,0,1),(4,0,1),10,.01).reason,'contact_detected')

    def test_stale_course_telemetry_holds(self):
        self.assertEqual(self.node.tick((0,0,1),(0,0,1),(4,0,1),10.6,.01).reason,
                         'course_observations_stale')

    def test_vertical_goal_holds(self):
        self.assertEqual(self.node.tick((0,0,1),(0,0,1),(4,0,2),10,.01).reason,
                         'altitude_change_not_supported')

    def test_opt_in_vertical_goal_requests_a_three_dimensional_plan(self):
        self.node.allow_vertical = True
        decision = self.node.tick((0,0,1),(0,0,1),(0,0,5),10,.01)
        self.assertEqual(decision.state, 'HOLD')
        goal = self.pubs[self.topics.LOCAL_PLAN_GOAL].sent[-1]
        self.assertEqual((goal.x, goal.y, goal.z), (0, 0, 4))

    def test_package_attachment_revokes_unloaded_approval(self):
        self.node.tick((0,0,1),(0,0,1),(4,0,1),10,.01)
        self.node.receive_path(FakePath([(0,0,1),(3,0,1)],stamp=10))
        status={'state':'READY','stamp':10,'observation_stamp':10,
                'goal':[3,0,1],'reset_sequence':1,'package_attached':False}
        self.node.receive_status(FakeString(json.dumps(status)))
        self.node.receive_course(FakeString('{"reset_sequence":1,"package_attached":true}'))
        self.assertIsNone(self.node.follower.path)
        self.node.receive_status(FakeString(json.dumps(status)))
        self.assertIsNone(self.node.follower.status)

if __name__ == '__main__':unittest.main()
