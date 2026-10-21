"""Exercise real launcher refusal paths without starting Unity or ROS services."""
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / 'ROS1/scripts/run_camera_delivery_demo.sh'


class CameraLauncherTests(unittest.TestCase):
    def test_unknown_option_is_rejected_before_startup(self):
        result = subprocess.run(['bash', str(LAUNCHER), '--automatic-play'],
                                capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('always restarts the demo', result.stderr)

    def run_launcher(self, with_cache=True, failed_import=False, stop_at_restart=False, **overrides):
        environment = dict(os.environ, UNITY_EDITOR='/bin/true', DETECTOR_PYTHON='/bin/true',
                           CAMERA_DEMO_MEMORY_MODE='off', CAMERA_DEMO_MIN_AVAILABLE_GIB='0.01')
        environment.update(overrides)
        # Restart tests must never target the developer's real open project.
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary)
            scene = project / 'Unity/Assets/Scenes/MainWarehouse.unity'
            scene.parent.mkdir(parents=True)
            scene.touch()
            if with_cache:
                for name in ('ArtifactDB', 'SourceAssetDB', 'ScriptAssemblies/Assembly-CSharp.dll'):
                    cache = project / 'Unity/Library' / name
                    cache.parent.mkdir(parents=True, exist_ok=True)
                    cache.touch()
            tools = project / 'ROS1/tools'
            tools.mkdir(parents=True)
            if failed_import:
                (project / 'ROS1/build/unity-import-stopped').mkdir(parents=True)
            restart_source = (ROOT / 'ROS1/tools/camera_demo_processes.py').read_text()
            if stop_at_restart:
                # Stop before any services start; only exercise startup admission.
                restart_source = 'raise SystemExit(73)\n'
            (tools / 'camera_demo_processes.py').write_text(restart_source)
            environment['PROJECT_ROOT'] = temporary
            return subprocess.run(['bash', str(LAUNCHER)], env=environment,
                                  capture_output=True, text=True, timeout=20)


    def test_missing_unity_has_actionable_error(self):
        result = self.run_launcher(UNITY_EDITOR='/nonexistent/camera-demo-unity')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('set UNITY_EDITOR', result.stderr)

    def test_missing_weights_fails_before_starting_services(self):
        result = self.run_launcher(DETECTOR_WEIGHTS='/nonexistent/camera-demo-weights.pt')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('set DETECTOR_WEIGHTS', result.stderr)

    def test_low_memory_refuses_before_restarting_project_processes(self):
        with tempfile.NamedTemporaryFile() as weights:
            result = self.run_launcher(DETECTOR_WEIGHTS=weights.name,
                                       CAMERA_DEMO_MIN_AVAILABLE_GIB='1000000')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Close memory-heavy applications', result.stderr)
            self.assertNotIn('[RESTART]', result.stdout)

    def test_fresh_and_interrupted_imports_do_not_block_normal_startup(self):
        with tempfile.NamedTemporaryFile() as weights:
            for with_cache, failed_import in ((False, False), (True, True)):
                with self.subTest(with_cache=with_cache, failed_import=failed_import):
                    result = self.run_launcher(
                        with_cache=with_cache, failed_import=failed_import, stop_at_restart=True,
                        DETECTOR_WEIGHTS=weights.name, CAMERA_DEMO_MIN_AVAILABLE_GIB='')
                    self.assertEqual(result.returncode, 73, result.stderr)
                    self.assertIn('[RESTART]', result.stdout)
                    self.assertNotIn('prepare_camera_demo_unity.sh', result.stderr)

    def test_memory_modes_without_systemd_user_manager(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'camera_demo_linux.sh').write_text((ROOT / 'ROS1/scripts/camera_demo_linux.sh').read_text())
            definitions = root / 'launcher-functions.sh'
            definitions.write_text(LAUNCHER.read_text().split('\ncheck_dependencies "$@"', 1)[0])
            for mode, succeeds in (('auto', True), ('off', True), ('required', False)):
                with self.subTest(mode=mode):
                    result = subprocess.run(
                        ['bash', '-c', 'source "$1"; systemctl() { return 1; }; isolate_demo_processes',
                         'bash', str(definitions)],
                        env=dict(os.environ, CAMERA_DEMO_MEMORY_MODE=mode, CAMERA_DEMO_SCOPE=''),
                        capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode == 0, succeeds, result.stderr)

    def test_normal_scope_keeps_main_memory_and_swap_limits(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'camera_demo_linux.sh').write_text((ROOT / 'ROS1/scripts/camera_demo_linux.sh').read_text())
            definitions = root / 'launcher-functions.sh'
            definitions.write_text(LAUNCHER.read_text().split('\ncheck_dependencies "$@"', 1)[0])
            arguments = root / 'scope-arguments'
            command = '''source "$1"
systemctl() { return 0; }
systemd-run() { printf '%s\\n' "$@" > "$TEST_SCOPE_ARGUMENTS"; }
isolate_demo_processes
'''
            result = subprocess.run(
                ['bash', '-c', command, 'bash', str(definitions)],
                env=dict(os.environ, CAMERA_DEMO_MEMORY_MODE='required', CAMERA_DEMO_SCOPE='',
                         CAMERA_DEMO_MEMORY_MAX='', CAMERA_DEMO_SWAP_MAX='',
                         TEST_SCOPE_ARGUMENTS=str(arguments)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            settings = arguments.read_text().splitlines()
            self.assertIn('--property=MemoryMax=8G', settings)
            self.assertIn('--property=MemorySwapMax=2G', settings)
            self.assertFalse(any('MemoryHigh=' in setting for setting in settings))

    def test_manual_unity_does_not_start_editor(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'camera_demo_linux.sh').write_text((ROOT / 'ROS1/scripts/camera_demo_linux.sh').read_text())
            definitions = root / 'launcher-functions.sh'
            definitions.write_text(LAUNCHER.read_text().split('\ncheck_dependencies "$@"', 1)[0])
            result = subprocess.run(['bash', '-c', 'source "$1"; open_unity; [[ -z "$UNITY_PID" ]]',
                                     'bash', str(definitions)],
                                    env=dict(os.environ, UNITY_MODE='manual', UNITY_EDITOR='/nonexistent'),
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('press Play', result.stdout)

    def test_existing_service_is_not_terminated(self):
        with tempfile.NamedTemporaryFile() as weights, socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                server.bind(('127.0.0.1', 10001))
            except OSError:
                self.skipTest('Port 10001 already occupied')
            server.listen()
            result = self.run_launcher(DETECTOR_WEIGHTS=weights.name)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Port 10001 is unavailable', result.stderr)
            with socket.create_connection(('127.0.0.1', 10001), timeout=1):
                pass

    def test_unity_recovery_backups_are_preserved_before_opening(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            backups = root / 'Unity/Temp/__Backupscenes'
            backups.mkdir(parents=True)
            (backups / '0.backup').write_text('recoverable scene')
            run = root / 'run'
            run.mkdir()
            definitions = root / 'launcher-functions.sh'
            (root / 'camera_demo_linux.sh').write_text((ROOT / 'ROS1/scripts/camera_demo_linux.sh').read_text())
            definitions.write_text(LAUNCHER.read_text().split('\ncheck_dependencies "$@"', 1)[0])
            editor = root / 'editor'
            editor.write_text('''#!/usr/bin/env python3
import pathlib, sys, time
log = pathlib.Path(sys.argv[sys.argv.index('-logFile') + 1])
assert '-refreshImportMode' not in sys.argv
assert '-gc-helper-count' not in sys.argv
assert sys.argv[sys.argv.index('-job-worker-count') + 1] == '4'
assert sys.argv[sys.argv.index('-desiredWorkerCount') + 1] == '1'
assert sys.argv[sys.argv.index('-standbyWorkerCount') + 1] == '1'
assert sys.argv[sys.argv.index('-idleWorkerShutdownDelay') + 1] == '1000'
assert sys.argv[sys.argv.index('-executeMethod') + 1] == 'DeliveryDemoSceneLauncher.PrepareMainWarehouseForCameraDemo'
log.write_text('Delivery demo scene ready: Assets/Scenes/MainWarehouse.unity\\n')
time.sleep(1)
log.write_text(log.read_text() + 'CAMERA_DEMO_EDITOR_IDLE\\n')
time.sleep(30)
''')
            editor.chmod(0o755)
            command = '''source "$1"
RUN_DIR="$2"
DETECTOR_PID=$$
ROS_WRAPPER_PID=$$
trap '[[ -z "$UNITY_PID" ]] || kill -TERM "$UNITY_PID" 2>/dev/null || true' EXIT
open_unity
'''
            result = subprocess.run(['bash', '-c', command, 'bash', str(definitions), str(run)],
                                    env=dict(os.environ, PROJECT_ROOT=str(root), UNITY_EDITOR=str(editor)),
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(backups.exists())
            self.assertEqual((run / 'unity-scene-backups/0.backup').read_text(), 'recoverable scene')
            preferences = root / 'Unity/UserSettings/Search.settings'
            self.assertEqual(preferences.read_text(), 'indexOnEditorStartup = false\n')

            # A scene-open log alone must not start services, and user settings
            # must survive repeated launches even when indexing is enabled.
            preferences.write_text('indexOnEditorStartup = true\nfetchPreview = false\n')
            editor.write_text('''#!/usr/bin/env python3
import pathlib, sys, time
pathlib.Path(sys.argv[sys.argv.index('-logFile') + 1]).write_text(
    'Delivery demo scene ready: Assets/Scenes/MainWarehouse.unity\\n')
time.sleep(30)
''')
            (run / 'unity.log').unlink()
            result = subprocess.run(['bash', '-c', command, 'bash', str(definitions), str(run)],
                                    env=dict(os.environ, PROJECT_ROOT=str(root), UNITY_EDITOR=str(editor),
                                             UNITY_STARTUP_TIMEOUT='1'),
                                    capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('ROS and YOLO have not started', result.stderr)
            self.assertEqual(preferences.read_text(),
                             'indexOnEditorStartup = true\nfetchPreview = false\n')

    @unittest.skipUnless(Path('/opt/ros/noetic/setup.bash').exists(), 'Requires ROS Noetic')
    def test_existing_workspace_package_is_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / 'workspace'
            destination = workspace / 'src/drone_sorting'
            destination.mkdir(parents=True)
            sentinel = destination / 'user-owned.txt'
            sentinel.write_text('keep me')
            result = subprocess.run(['bash', str(ROOT / 'ROS1/scripts/prepare_camera_demo_ros.sh'),
                                     str(ROOT), str(workspace),
                                     str(Path.home() / 'drone_sorting_ros1_ws/src/ROS-TCP-Endpoint'), temporary],
                                    capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('already owned by another package', result.stdout)
            self.assertEqual(sentinel.read_text(), 'keep me')

    @unittest.skipUnless(Path('/opt/ros/noetic/setup.bash').exists(), 'Requires ROS Noetic')
    def test_finished_monitor_preserves_ros_until_supervisor_is_stopped(self):
        for status in (0, 1):
            with self.subTest(monitor_exit_status=status):
                self.check_finished_monitor(status)

    def check_finished_monitor(self, monitor_status):
        # Replace external ROS commands, but exercise the actual supervisor's
        # failure handling and signal cleanup without taking over real ports.
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            commands = root / 'commands'
            commands.mkdir()
            devel = root / 'workspace/devel'
            devel.mkdir(parents=True)
            (devel / 'setup.bash').write_text(f'export PATH="{commands}:$PATH"\n')
            scripts = {
                'rosnode': '#!/bin/bash\nprintf "%s\\n" /unity_endpoint /package_perception /camera_pickup /motion_controller\n',
                'python3': '#!/bin/bash\nif [[ "$1" == - ]]; then cat >/dev/null; exit 0; fi\nexit "$MONITOR_EXIT_STATUS"\n',
                'roslaunch': '''#!/usr/bin/python3
import os, signal, sys, time
from pathlib import Path
root = Path(os.environ['SUPERVISOR_TEST_ROOT'])
(root / 'launch.pid').write_text(str(os.getpid()))
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
try:
    while True:
        time.sleep(0.1)
finally:
    (root / 'launch-stopped').touch()
''',
            }
            for name, content in scripts.items():
                executable = commands / name
                executable.write_text(content)
                executable.chmod(0o755)
            with (root / 'supervisor.log').open('w') as output:
                process = subprocess.Popen(['bash', str(ROOT / 'ROS1/scripts/start_camera_demo_ros.sh'),
                                            str(ROOT), str(root / 'workspace'), str(root)],
                                           env=dict(os.environ, SUPERVISOR_TEST_ROOT=str(root),
                                                    MONITOR_EXIT_STATUS=str(monitor_status)),
                                           stdout=output, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 10
                    while time.monotonic() < deadline:
                        if 'ROS remains available' in (root / 'supervisor.log').read_text():
                            break
                        self.assertIsNone(process.poll(), (root / 'supervisor.log').read_text())
                        time.sleep(0.1)
                    self.assertIn('ROS remains available', (root / 'supervisor.log').read_text())
                    self.assertIsNone(process.poll())
                    os.kill(int((root / 'launch.pid').read_text()), 0)
                finally:
                    process.terminate()
                    process.wait(timeout=10)
                self.assertTrue((root / 'launch-stopped').exists())
                self.assertFalse((root / 'ros-supervisor.pid').exists())
