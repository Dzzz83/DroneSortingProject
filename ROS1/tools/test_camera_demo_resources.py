"""Exercise memory boundaries using fixtures, without allocating test memory."""
from pathlib import Path
import tempfile
import unittest

from camera_demo_resources import check_available_memory


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
