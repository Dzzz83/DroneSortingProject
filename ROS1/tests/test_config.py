"""Linux configuration boundaries, without installing dependencies or starting services."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'ROS1/scripts'


class LinuxConfigurationTests(unittest.TestCase):
    def run_config(self, project, command, **overrides):
        environment = dict(os.environ, PROJECT_ROOT=str(project), ROS_ENVIRONMENT='auto')
        environment.update(overrides)
        return subprocess.run(
            ['bash', '-c', 'set -euo pipefail; fail() { echo "$*" >&2; exit 1; }; '
             'source "$1"; ' + command, 'bash', str(SCRIPTS / 'internal/config.sh')],
            env=environment, capture_output=True, text=True, timeout=10)

    def test_defaults_work_in_project_path_with_spaces(self):
        with tempfile.TemporaryDirectory(prefix='camera project ') as temporary:
            result = self.run_config(temporary, 'validate_camera_workspace; printf "%s\\n" "$WORKSPACE" "$DETECTOR_WEIGHTS" "$DETECTOR_DEVICE"')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.splitlines(), [
                temporary + '/ROS1/build/linux-camera-workspace',
                temporary + '/ROS1/models/package_detector.pt', 'cpu'])

    def test_source_and_unity_workspaces_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            for relative in ('', '/Unity', '/Unity/nested', '/ROS1/package'):
                with self.subTest(relative=relative):
                    result = self.run_config(temporary, 'validate_camera_workspace',
                                             CAMERA_DEMO_WORKSPACE=temporary + relative)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn('separate CAMERA_DEMO_WORKSPACE', result.stderr)

    def test_invalid_ros_environment_is_rejected(self):
        result = self.run_config('/tmp/project', 'select_ros_environment', ROS_ENVIRONMENT='unknown')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('ROS_ENVIRONMENT must be', result.stderr)

    def test_unfetched_lfs_asset_has_actionable_error(self):
        with tempfile.TemporaryDirectory() as temporary:
            asset = Path(temporary) / 'Unity/Assets/model.fbx'
            asset.parent.mkdir(parents=True)
            asset.write_text('version https://git-lfs.github.com/spec/v1\noid sha256:abc\nsize 2048\n')
            result = self.run_config(temporary, 'check_unity_assets')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('git lfs pull', result.stderr)
            self.assertEqual(asset.read_text().splitlines()[0], 'version https://git-lfs.github.com/spec/v1')

    def test_unity_hub_discovery_and_explicit_override(self):
        with tempfile.TemporaryDirectory() as temporary:
            editor = Path(temporary) / 'Unity/Hub/Editor/6000.6.0f1/Editor/Unity'
            editor.parent.mkdir(parents=True)
            editor.touch()
            editor.chmod(0o755)
            result = self.run_config('/tmp/project', 'find_unity_editor; printf "%s" "$UNITY_EDITOR"',
                                     HOME=temporary, UNITY_EDITOR='')
            self.assertEqual(result.stdout, str(editor))
            explicit = self.run_config('/tmp/project', 'find_unity_editor; printf "%s" "$UNITY_EDITOR"',
                                       HOME=temporary, UNITY_EDITOR='/custom/editor')
            self.assertEqual(explicit.stdout, '/custom/editor')

    def test_setup_unknown_option_has_no_side_effects(self):
        with tempfile.TemporaryDirectory() as temporary:
            result = subprocess.run(['bash', str(SCRIPTS / 'setup_demo.sh'), '--unknown'],
                                    env=dict(os.environ, PROJECT_ROOT=temporary),
                                    capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Usage:', result.stderr)
            self.assertEqual(list(Path(temporary).iterdir()), [])


class MissionConfigurationTests(unittest.TestCase):
    def test_supported_mission_settings_are_known_and_valid(self):
        for profile in ('demo.yaml', 'course.yaml'):
            with self.subTest(profile=profile):
                self.check_mission_settings(profile)

    def check_mission_settings(self, profile):
        # Validate editable configuration without requiring it to equal Python fallbacks.
        from dataclasses import fields
        import math
        import yaml
        from drone_sorting.mission.delivery_guidance import DeliverySettings, ObservationAccessSettings
        from drone_sorting.mission.pickup_guidance import PickupSettings
        config = yaml.safe_load((ROOT / 'ROS1/package/config' / profile).read_text())
        mission = config['camera_pickup']
        known_fields = set()
        for kind in (DeliverySettings, ObservationAccessSettings, PickupSettings):
            names = {field.name for field in fields(kind)}
            known_fields.update(names)
            values = {name: mission[name] for name in names}
            kind(**values)  # Validate delivery/access limits, allowing negative horizontal positions.
            if kind is PickupSettings:
                self.assertTrue(all(isinstance(value, (int, float)) and math.isfinite(value) and value > 0
                                    for value in values.values()))
        self.assertEqual(set(mission), known_fields)


if __name__ == '__main__':
    unittest.main()
