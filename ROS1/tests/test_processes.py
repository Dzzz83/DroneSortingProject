"""Verify restart ownership, stale PID safety and real process shutdown."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import processes as lifecycle


def process(arguments=(), environment=(), scopes=(), started='123'):
    return lifecycle.Process(12345, started, tuple(arguments), frozenset(environment), frozenset(scopes))


class ProcessTests(unittest.TestCase):
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
        self.assertTrue(lifecycle.owns_ros_process(process(arguments=['bash', str(run / 'start_ros.sh')]), [run]))
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


class RunCleanupTests(unittest.TestCase):
    def test_removes_finished_runs_but_keeps_dependencies_and_symlink_targets(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            build = root / 'ROS1/build'
            completed = build / 'camera-demo-old'
            completed.mkdir(parents=True)
            (completed / 'systemd-scope.txt').touch()
            (completed / 'unity.log').write_text('old log')
            dependency = build / 'detector-venv'
            dependency.mkdir()
            outside = root / 'user-data'
            outside.mkdir()
            (outside / 'keep.txt').write_text('keep')
            (build / 'camera-demo-link').symlink_to(outside, target_is_directory=True)
            with patch.object(lifecycle, 'processes', return_value=[]):
                lifecycle.discard_finished_runs(root)
            self.assertFalse(completed.exists())
            self.assertTrue(dependency.exists())
            self.assertEqual((outside / 'keep.txt').read_text(), 'keep')
            self.assertTrue((build / 'camera-demo-link').is_symlink())

    def test_preserves_ros_and_unity_active_runs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ros_run = root / 'ROS1/build/camera-demo-ros'
            unity_run = root / 'ROS1/build/camera-demo-unity'
            ros_run.mkdir(parents=True)
            unity_run.mkdir()
            (ros_run / 'systemd-scope.txt').touch()
            (unity_run / 'systemd-scope.txt').touch()
            snapshot = [
                process(environment=[f'CAMERA_DEMO_RUN_DIR={ros_run}']),
                process(['/opt/Unity', '-projectPath', str(root / 'Unity'),
                         '-logFile', str(unity_run / 'unity.log')]),
            ]
            with patch.object(lifecycle, 'processes', return_value=snapshot):
                lifecycle.discard_finished_runs(root)
            self.assertTrue(ros_run.exists())
            self.assertTrue(unity_run.exists())

    def test_preserves_scene_recovery_outside_run_history(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = root / 'ROS1/build/camera-demo-old'
            recovery = run / 'unity-scene-backups'
            recovery.mkdir(parents=True)
            (run / 'systemd-scope.txt').touch()
            (recovery / '0.backup').write_text('unsaved scene')
            with patch.object(lifecycle, 'processes', return_value=[]):
                lifecycle.discard_finished_runs(root)
            saved = root / 'Unity/UserSettings/SceneBackups/camera-demo-old/0.backup'
            self.assertEqual(saved.read_text(), 'unsaved scene')
            self.assertFalse(run.exists())

    def test_existing_recovery_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = root / 'ROS1/build/camera-demo-old'
            (run / 'unity-scene-backups').mkdir(parents=True)
            (run / 'systemd-scope.txt').touch()
            destination = root / 'Unity/UserSettings/SceneBackups/camera-demo-old'
            destination.mkdir(parents=True)
            with patch.object(lifecycle, 'processes', return_value=[]):
                with self.assertRaisesRegex(RuntimeError, 'already exists'):
                    lifecycle.discard_finished_runs(root)
            self.assertTrue(run.exists())


if __name__ == '__main__':
    unittest.main()
