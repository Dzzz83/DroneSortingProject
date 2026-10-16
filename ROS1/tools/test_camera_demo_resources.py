"""Exercise memory boundaries using fixtures, without allocating test memory."""
from pathlib import Path
import tempfile
import unittest

from camera_demo_resources import check_available_memory, check_startup, has_unity_cache, import_pressure_reason


class MemoryPreflightTests(unittest.TestCase):
    def test_uses_available_ram_instead_of_free_ram_or_swap(self):
        with tempfile.TemporaryDirectory() as temporary:
            meminfo = Path(temporary) / 'meminfo'
            meminfo.write_text('MemFree: 1 kB\nMemAvailable: 10485760 kB\nSwapFree: 99999999 kB\n')
            self.assertEqual(check_available_memory(10, meminfo), 10)
            with self.assertRaisesRegex(ValueError, 'Close memory-heavy applications'):
                check_available_memory(10.1, meminfo)

    def test_missing_available_memory_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            meminfo = Path(temporary) / 'meminfo'
            meminfo.write_text('MemFree: 10485760 kB\n')
            with self.assertRaisesRegex(ValueError, 'Cannot read MemAvailable'):
                check_available_memory(1, meminfo)

    def test_invalid_thresholds_are_rejected(self):
        for minimum in (0, -1, float('nan'), float('inf')):
            with self.subTest(minimum=minimum), self.assertRaisesRegex(ValueError, 'positive number'):
                check_available_memory(minimum)


class StartupPolicyTests(unittest.TestCase):
    def fixtures(self, root, available_kib, cached):
        meminfo = root / 'meminfo'
        meminfo.write_text(f'MemAvailable: {available_kib} kB\n')
        if cached:
            for name in ('ArtifactDB', 'SourceAssetDB', 'ScriptAssemblies/Assembly-CSharp.dll'):
                path = root / 'Unity/Library' / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
        return meminfo

    def test_cached_startup_accepts_8_point_7_gib_without_explicit_minimum(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            info = self.fixtures(root, int(8.7 * 1024 ** 2), True)
            messages = check_startup(root, 'auto', meminfo=info)
            self.assertIn('cache: present', messages[0])
            self.assertFalse(any('[WARN]' in message for message in messages))

    def test_low_memory_warns_but_explicit_minimum_blocks(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            info = self.fixtures(root, 3 * 1024 ** 2, True)
            self.assertTrue(any('[WARN]' in message for message in check_startup(root, 'auto', meminfo=info)))
            with self.assertRaisesRegex(ValueError, 'configured minimum'):
                check_startup(root, 'auto', 4, info)

    def test_fresh_and_partial_cache_require_separate_import(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            info = self.fixtures(root, 12 * 1024 ** 2, False)
            for partial in (False, True):
                with self.subTest(partial=partial):
                    if partial:
                        path = root / 'Unity/Library/ArtifactDB'
                        path.parent.mkdir(parents=True)
                        path.touch()
                    self.assertFalse(has_unity_cache(root))
                    with self.assertRaisesRegex(ValueError, 'prepare_camera_demo_unity.sh'):
                        check_startup(root, 'auto', meminfo=info)

    def test_manual_backend_can_start_without_unity_cache(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            info = self.fixtures(root, 8 * 1024 ** 2, False)
            self.assertIn('cache: absent/incomplete', check_startup(root, 'manual', meminfo=info)[0])

    def test_import_pressure_boundaries(self):
        healthy = 'some avg10=70.0 avg60=20 total=1\nfull avg10=9.99 avg60=0 total=0\n'
        self.assertIsNone(import_pressure_reason(2, healthy))
        self.assertIn('below 2 GiB', import_pressure_reason(1.99, healthy))
        self.assertIn('10%', import_pressure_reason(3, healthy.replace('9.99', '10.00')))

    def test_interrupted_recorded_import_overrides_cache_file_presence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            info = self.fixtures(root, 8 * 1024 ** 2, True)
            run = root / 'ROS1/build/unity-import-trial'
            run.mkdir(parents=True)
            self.assertFalse(has_unity_cache(root))
            with self.assertRaisesRegex(ValueError, 'prepare_camera_demo_unity.sh'):
                check_startup(root, 'auto', meminfo=info)
            (run / 'COMPLETE').touch()
            self.assertTrue(has_unity_cache(root))
