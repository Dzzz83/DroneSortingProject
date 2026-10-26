"""Focused ROS adapter checks without a master or Unity."""
import json
import math
import struct
import unittest
from unittest.mock import Mock, patch

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Float32, Header, String
    from drone_sorting.mapping.live_mapping import DepthMappingNode
    from drone_sorting.interfaces import ros_topics as topics
except ImportError:
    rospy = None


@unittest.skipIf(rospy is None, "Requires ROS Noetic message packages")
class LiveMappingTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.publishers = {}
        def publisher(topic, *args, **kwargs):
            self.publishers[topic] = Mock()
            return self.publishers[topic]
        for name, replacement in [('Publisher',publisher),('Subscriber',Mock()),('Timer',Mock()),
                                  ('on_shutdown',Mock()),('get_time',lambda:self.now),
                                  ('get_param',lambda name,default:1 if name=='~pixel_stride' else default)]:
            context = patch.object(rospy,name,replacement)
            context.start()
            self.addCleanup(context.stop)
        clock = patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        self.node = DepthMappingNode()
        self.node.receive_range(Float32(15))

    def messages(self, stamp=100):
        stamp = rospy.Time.from_sec(stamp)
        depth = Image(header=Header(seq=1,stamp=stamp,frame_id='forward_camera_optical'),
                      width=3,height=3,encoding='32FC1',step=12,
                      data=struct.pack('<9f',*([3.]*9)))
        info = CameraInfo(header=Header(seq=17,stamp=stamp,frame_id='forward_camera_optical'),
                          width=3,height=3,distortion_model='plumb_bob',D=[0]*5,K=[2,0,1,0,2,1,0,0,1])
        camera = PoseStamped(header=Header(seq=22,stamp=stamp,frame_id='warehouse'))
        camera.pose.position.x,camera.pose.position.y,camera.pose.position.z=34,50,2.5
        camera.pose.orientation.x,camera.pose.orientation.w=-math.sqrt(.5),math.sqrt(.5)
        drone = PoseStamped(header=Header(seq=3,stamp=stamp,frame_id='warehouse'))
        drone.pose.position.x,drone.pose.position.y,drone.pose.position.z=34,50,3
        drone.pose.orientation.w=1
        return depth,info,camera,drone

    def status(self):
        return json.loads(self.publishers[topics.LOCAL_MAP_STATUS].publish.call_args[0][0].data)

    def accept(self, stamp=100):
        self.node.receive_bundle(*self.messages(stamp))
        self.node.process_once(self.now)

    def test_exact_stamp_sync_ignores_independent_topic_sequence_counters(self):
        messages=self.messages()
        for index in (3,0,2):self.node.subscribers[index].signalMessage(messages[index])
        self.assertIsNone(self.node.pending)
        unmatched=self.messages(99)[1]
        self.node.subscribers[1].signalMessage(unmatched)
        self.assertIsNone(self.node.pending)
        self.node.subscribers[1].signalMessage(messages[1])
        self.node.process_once(self.now)
        self.assertEqual(self.status()['accepted'],1)
        self.assertEqual(self.status()['state'],'ACTIVE')
        for topic in (topics.COMMAND_POSITION,topics.TARGET_POSITION,topics.PACKAGE_ACTION):
            self.assertNotIn(topic,self.publishers)
        self.assertEqual(self.publishers[topics.LOCAL_MAP_OCCUPIED].publish.call_args[0][0].header.frame_id,'warehouse')

    def test_newest_pending_bundle_replaces_backlog(self):
        self.node.receive_bundle(*self.messages(99.9))
        self.node.receive_bundle(*self.messages(100))
        self.node.process_once(100)
        self.assertEqual(self.node.last_stamp,100)
        self.assertEqual(self.status()['superseded'],1)
        self.assertEqual(self.status()['accepted'],1)

    def test_malformed_observations_do_not_refresh_accepted_age(self):
        for issue in ('layout','calibration','pose','stamp','optical_frame','root_rotation'):
            with self.subTest(issue=issue):
                self.node.map.reset();self.node.last_stamp=None
                self.accept()
                messages=self.messages(100.1)
                depth,info,camera,drone=messages
                if issue=='layout':depth.data=b''
                if issue=='calibration':info.K=[0]*9
                if issue=='pose':camera.pose.orientation.w=4
                if issue=='stamp':drone.header.stamp=rospy.Time(99)
                if issue=='optical_frame':depth.header.frame_id='package_camera_optical'
                if issue=='root_rotation':drone.pose.orientation.w=float('nan')
                self.node.receive_bundle(*messages)
                self.node.process_once(100.1)
                self.assertEqual(self.node.last_stamp,100)
                self.assertEqual(self.status()['state'],'WAITING')

    def test_stale_stream_expires_clouds_without_new_callbacks(self):
        self.accept()
        self.node.process_once(100.6)
        self.assertEqual(self.status()['reason'],'stale_observation')
        self.assertGreater(self.status()['occupied_cells'],0)
        self.node.process_once(102.1)
        self.assertEqual(self.status()['occupied_cells'],0)
        self.assertEqual(self.status()['free_cells'],0)
        self.assertEqual(self.publishers[topics.LOCAL_MAP_OCCUPIED].publish.call_args[0][0].width,0)

    def test_course_reset_clears_pending_map_and_rejects_earlier_acquisitions(self):
        self.node.receive_course(String('{"reset_sequence":0}'))
        self.accept()
        self.node.receive_bundle(*self.messages(100.1))
        self.now=100.2
        self.node.receive_course(String('{"reset_sequence":1,"package_position":{"x":999}}'))
        self.node.process_once(self.now)
        self.assertEqual(self.node.map.snapshot(self.now),[])
        self.node.receive_bundle(*self.messages(100.1))
        self.node.process_once(self.now)
        self.assertEqual(self.status()['reason'],'observation_precedes_reset')
        self.now=100.3
        self.accept(100.3)
        self.assertEqual(self.status()['state'],'ACTIVE')
        self.assertEqual(self.status()['reset_sequence'],1)

    def test_stream_gap_discards_old_trial_history_on_recovery(self):
        self.accept()
        old_point=(34,53,2.5)
        self.assertEqual(self.node.map.state_at(old_point,100),'occupied')
        messages=self.messages(101)
        messages[2].pose.orientation.x=0
        messages[2].pose.orientation.w=1
        self.node.receive_bundle(*messages)
        self.node.process_once(101)
        self.assertEqual(self.node.map.state_at(old_point,101),'unknown')

    def test_range_declaration_and_acquisition_age_are_required(self):
        self.node.receive_range(Float32(10))
        self.accept()
        self.assertEqual(self.status()['accepted'],0)
        self.node.receive_range(Float32(15))
        self.node.receive_bundle(*self.messages(99))
        self.node.process_once(100)
        self.assertEqual(self.status()['reason'],'stale_or_future_observation')

    def test_startup_retries_and_shutdown_release_camera(self):
        self.node.request_camera()
        self.assertTrue(self.publishers[topics.FORWARD_CAMERA_ENABLED].publish.call_args[0][0].data)
        self.node.shutdown()
        self.assertFalse(self.publishers[topics.FORWARD_CAMERA_ENABLED].publish.call_args[0][0].data)
        self.node.request_timer.shutdown.assert_called_once()
        count=self.publishers[topics.FORWARD_CAMERA_ENABLED].publish.call_count
        self.node.request_camera()
        self.assertEqual(self.publishers[topics.FORWARD_CAMERA_ENABLED].publish.call_count,count)
