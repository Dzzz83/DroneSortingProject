"""Verify restart ownership, stale PID safety and real process shutdown."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import camera_demo_processes as lifecycle


def process(arguments=(), environment=(), scopes=(), started='123'):
    return lifecycle.Process(12345, started, tuple(arguments), frozenset(environment), frozenset(scopes))


class CameraDemoProcessTests(unittest.TestCase):
    def test_editor_match_requires_exact_project(self):
        project = Path('/tmp/demo')
        self.assertTrue(lifecycle.is_project_editor(
            process(['/opt/Unity', '-projectPath', '/tmp/demo/Unity']), project))
        self.assertFalse(lifecycle.is_project_editor(
            process(['/opt/Unity', '-projectPath', '/tmp/demo-other/Unity']), project))
        self.assertFalse(lifecycle.is_project_editor(
            process(['/usr/bin/python3', '-projectPath', '/tmp/demo/Unity']), project))

    def test_ros_match_requires_exact_run_identity(self):
        run = Path('/tmp/demo/ROS1/build/camera-demo-one')
        self.assertTrue(lifecycle.owns_ros_process(process(environment=[f'ROS_LOG_DIR={run}/ros-logs']), [run]))
        self.assertTrue(lifecycle.owns_ros_process(process(arguments=['bash', str(run / 'start_camera_demo_ros.sh')]), [run]))
        self.assertFalse(lifecycle.owns_ros_process(process(environment=[f'ROS_LOG_DIR={run}-other/ros-logs']), [run]))
        self.assertFalse(lifecycle.owns_ros_process(process(arguments=['roslaunch', 'camera_delivery.launch']), [run]))

    def test_reused_pid_is_not_signalled(self):
        old = process(started='100')
        with patch.object(lifecycle, 'read_process', return_value=process(started='200')), \
                patch.object(lifecycle.os, 'kill') as kill:
            lifecycle.send_signal(old, lifecycle.signal.SIGTERM)
            kill.assert_not_called()

    def test_manual_mode_preserves_editor_scope(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = root / 'ROS1/build/camera-demo-old'
            run.mkdir(parents=True)
            (run / 'systemd-scope.txt').write_text('camera-delivery-1-2.scope')
            editor = process(['/opt/Unity', '-projectPath', str(root / 'Unity')])
            with patch.object(lifecycle, 'processes', return_value=[editor]), \
                    patch.object(lifecycle, 'stop_scope') as scope, \
                    patch.object(lifecycle, 'stop_processes') as stop:
                lifecycle.restart(root, '', unity_mode='manual')
                scope.assert_not_called()
                stop.assert_not_called()

    def test_restart_without_scope_stops_editor_descendants(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            editor = process(['/opt/Unity', '-projectPath', str(root / 'Unity')])
            worker = lifecycle.Process(23456, '124', ('worker',), frozenset(), frozenset(), editor.pid)
            unrelated = lifecycle.Process(34567, '125', ('worker',), frozenset(), frozenset(), 1)
            with patch.object(lifecycle, 'processes', return_value=[editor, worker, unrelated]), \
                    patch.object(lifecycle, 'still_running', return_value=True), \
                    patch.object(lifecycle, 'stop_processes') as stop:
                lifecycle.restart(root, '')
                stop.assert_called_once_with([editor, worker])

    def test_current_scope_and_invalid_records_are_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for index, scope in enumerate(('camera-delivery-1-2.scope', 'camera-delivery-3-4.scope', 'unrelated.service')):
                directory = root / 'ROS1/build' / f'camera-demo-{index}'
                directory.mkdir(parents=True)
                (directory / 'systemd-scope.txt').write_text(scope)
            with patch.object(lifecycle, 'processes', return_value=[]), \
                    patch.object(lifecycle, 'stop_scope') as stop:
                lifecycle.restart(root, 'camera-delivery-3-4.scope')
                stop.assert_called_once_with('camera-delivery-1-2.scope')

    def test_restart_stops_owned_process_and_preserves_unrelated_process(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = root / 'ROS1/build/camera-demo-test'
            run.mkdir(parents=True)
            command = [sys.executable, '-c', 'import time; time.sleep(60)']
            owned = subprocess.Popen(command, env=dict(os.environ, CAMERA_DEMO_RUN_DIR=str(run)))
            unrelated = subprocess.Popen(command)
            try:
                lifecycle.restart(root, 'camera-delivery-0-0.scope')
                owned.wait(timeout=5)
                self.assertIsNone(unrelated.poll())
            finally:
                for child in (owned, unrelated):
                    if child.poll() is None:
                        child.terminate()
                    child.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
