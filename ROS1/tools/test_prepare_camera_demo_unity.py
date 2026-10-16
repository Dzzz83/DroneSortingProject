"""Test bounded import supervision with fake Unity and systemd commands."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'ROS1/scripts/prepare_camera_demo_unity.sh'


class UnityPreparationTests(unittest.TestCase):
    def run_preparation(self, root, pressure=False, editor_status=0, timeout=False):
        commands = root / 'commands'
        commands.mkdir()
        editor = commands / 'Unity'
        editor.write_text(f'''#!{sys.executable}
import os, pathlib, sys, time
root = pathlib.Path(os.environ['PROJECT_ROOT'])
args = sys.argv
assert args[args.index('-refreshImportMode') + 1] == 'InProcess'
assert '-batchmode' in args and '-quit' in args
if os.environ.get('TEST_PRESSURE') == 'yes' or os.environ.get('TEST_TIMEOUT') == 'yes':
    time.sleep(30)
for name in ('ArtifactDB', 'SourceAssetDB', 'ScriptAssemblies/Assembly-CSharp.dll'):
    path = root / 'Unity/Library' / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.touch()
pathlib.Path(args[args.index('-logFile') + 1]).write_text('Delivery demo scene ready: Assets/Scenes/MainWarehouse.unity\\n')
sys.exit({editor_status})
''')
        wrappers = {
            'systemd-run': '''#!/bin/bash
printf '%s\\n' "$@" > "$PROJECT_ROOT/scope-arguments"
echo $$ > "$PROJECT_ROOT/import.pid"
while [[ "$1" != "$UNITY_EDITOR" ]]; do shift; done
exec "$@"
''',
            'systemctl': '''#!/bin/bash
if [[ " $* " == *" stop "* ]]; then
    touch "$PROJECT_ROOT/scope-stopped"
    if [[ -f "$PROJECT_ROOT/import.pid" && ( "${TEST_PRESSURE:-}" == yes || "${TEST_TIMEOUT:-}" == yes ) ]]; then
        kill -TERM "$(cat "$PROJECT_ROOT/import.pid")" 2>/dev/null || true
    fi
fi
exit 0
''',
            'python3': f'''#!/bin/bash
if [[ "$1" == *camera_demo_resources.py ]]; then
    if [[ " $* " == *--check-import-pressure* && "${{TEST_PRESSURE:-}}" == yes && -f "$PROJECT_ROOT/import.pid" ]]; then
        echo 'fixture pressure' >&2
        exit 1
    fi
    exit 0
fi
exec '{sys.executable}' "$@"
''',
        }
        editor.chmod(0o755)
        for name, source in wrappers.items():
            path = commands / name
            path.write_text(source)
            path.chmod(0o755)
        return subprocess.run(
            ['bash', str(SCRIPT)],
            env=dict(os.environ, PATH=f'{commands}:{os.environ["PATH"]}', PROJECT_ROOT=str(root),
                     UNITY_EDITOR=str(editor), TEST_PRESSURE='yes' if pressure else 'no',
                     TEST_TIMEOUT='yes' if timeout else 'no', UNITY_IMPORT_TIMEOUT='1' if timeout else '600'),
            capture_output=True, text=True, timeout=15)

    def test_success_uses_bounded_scope_and_leaves_cache(self):
        with tempfile.TemporaryDirectory(prefix='unity import ') as temporary:
            root = Path(temporary)
            result = self.run_preparation(root)
            self.assertEqual(result.returncode, 0, result.stderr)
            args = (root / 'scope-arguments').read_text()
            for setting in ('MemoryHigh=4G', 'MemoryMax=5G', 'MemorySwapMax=512M', 'RuntimeMaxSec=600'):
                self.assertIn(setting, args)
            self.assertTrue((root / 'scope-stopped').exists())
            self.assertTrue((root / 'Unity/Library/ArtifactDB').exists())
            self.assertIn('scene opening completed', result.stdout)
            self.assertEqual(len(list((root / 'ROS1/build').glob('unity-import-*/COMPLETE'))), 1)

    def test_pressure_stops_import_without_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            result = self.run_preparation(root, pressure=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('five consecutive pressure checks', result.stderr)
            self.assertTrue((root / 'scope-stopped').exists())
            self.assertFalse((root / 'Unity/Library/ArtifactDB').exists())

    def test_editor_failure_is_reported_and_scope_is_stopped(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            result = self.run_preparation(root, editor_status=1)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('exited unsuccessfully', result.stderr)
            self.assertTrue((root / 'scope-stopped').exists())

    def test_timeout_stops_import(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            result = self.run_preparation(root, timeout=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('exceeded 1 seconds', result.stderr)
            self.assertTrue((root / 'scope-stopped').exists())
