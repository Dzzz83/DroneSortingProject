"""Camera monitoring must report failure without implying physical placement."""
import importlib.util
import json
import runpy
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

try:
    import rospy
except ImportError:
    rospy = None


@unittest.skipIf(rospy is None, 'Requires ROS Noetic')
class DeliveryMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('demo_monitor',
            Path(__file__).resolve().parents[1] / 'package/nodes/monitor.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        self.subscriber = patch.object(rospy, 'Subscriber').start()
        self.shutdown = patch.object(rospy, 'signal_shutdown').start()
        patch.object(rospy, 'loginfo').start()
        self.warn = patch.object(rospy, 'logwarn').start()
        patch.object(rospy, 'spin').start()
        self.print = patch('builtins.print').start()
        self.addCleanup(patch.stopall)
        self.monitor = self.module.DemoMonitor()

    def message(self, **status):
        return SimpleNamespace(data=json.dumps(status))

    def test_failed_guidance_reports_reason_and_returns_nonzero(self):
        message = self.message(phase='FAILED', reason='lost_locked_package')
        self.monitor._on_guidance(message)
        self.monitor._on_guidance(message)
        self.shutdown.assert_called_once_with('lost_locked_package')
        self.assertEqual(self.monitor.run(), 1)

    def test_cli_propagates_failure_exit_status(self):
        def subscribe(topic, message_type, callback):
            if topic == self.module.ros_topics.PICKUP_TRIAL_STATUS:
                callback(self.message(phase='FAILED', reason='phase_timeout'))
        self.subscriber.side_effect = subscribe
        with patch.object(rospy, 'myargv', return_value=['monitor']), \
                patch.object(rospy, 'init_node'), self.assertRaises(SystemExit) as exited:
            runpy.run_path(self.module.__file__, run_name='__main__')
        self.assertEqual(exited.exception.code, 1)

    def test_malformed_status_does_not_stop_monitoring(self):
        for data in ('not json', 'null', '[]', '{}', '{"phase": 4}', '{"phase":"FAILED","reason":4}'):
            self.monitor._on_guidance(SimpleNamespace(data=data))
        self.assertEqual(self.warn.call_count, 6)
        self.shutdown.assert_not_called()

    def test_completion_waits_for_both_acknowledgements(self):
        self.monitor._on_state(SimpleNamespace(data='COMPLETE'))
        self.monitor._on_status(SimpleNamespace(data='PICKUP_DONE'))
        self.shutdown.assert_not_called()
        self.monitor._on_status(SimpleNamespace(data='DROP_DONE'))
        self.shutdown.assert_called_once()
        self.assertEqual(self.monitor.run(), 0)

    def test_perception_rejection_is_visible_and_duplicate_is_suppressed(self):
        message = self.message(state='UNRELIABLE', reason='Observation exceeds perception freshness limit')
        self.monitor._on_perception(message)
        self.monitor._on_perception(message)
        self.assertEqual(self.print.call_count, 1)
        self.assertIn('freshness limit', self.print.call_args.args[0])
        self.monitor._on_guidance(self.message(phase='FAILED', reason='phase_timeout'))
        self.assertIn('freshness limit', self.print.call_args.args[0])

    def test_malformed_perception_does_not_replace_last_result(self):
        self.monitor._on_perception(self.message(state='NO_DETECTIONS', reason=None))
        for data in ('null', '[]', 'not json', '{"state": 4}', '{"state":"UNAVAILABLE","reason":4}'):
            self.monitor._on_perception(SimpleNamespace(data=data))
        self.assertEqual(self.monitor.last_perception, ('NO_DETECTIONS', None))
        self.assertEqual(self.warn.call_count, 5)

    def test_monitor_always_observes_camera_guidance(self):
        self.subscriber.reset_mock()
        self.module.DemoMonitor()
        topics = [call.args[0] for call in self.subscriber.call_args_list]
        self.assertIn(self.module.ros_topics.PICKUP_TRIAL_STATUS, topics)
        self.assertIn(self.module.ros_topics.PACKAGE_CANDIDATES, topics)
