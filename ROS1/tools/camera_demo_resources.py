"""Refuse demo startup when Linux reports insufficient available host RAM."""
import argparse
import math
from pathlib import Path

GIB = 1024 ** 3


def check_available_memory(minimum_gib, meminfo=Path('/proc/meminfo')):
    if not math.isfinite(minimum_gib) or minimum_gib <= 0:
        raise ValueError('Minimum available RAM must be a positive number of GiB.')
    for line in meminfo.read_text().splitlines():
        if line.startswith('MemAvailable:'):
            available_gib = int(line.split()[1]) * 1024 / GIB
            if available_gib < minimum_gib:
                raise ValueError(
                    f'Only {available_gib:.1f} GiB RAM is available; startup requires '
                    f'{minimum_gib:g} GiB. Close memory-heavy applications first. '
                    'CAMERA_DEMO_MIN_AVAILABLE_GIB adjusts this preflight threshold; '
                    'lowering it can cause memory pressure in other applications.')
            return available_gib
    raise ValueError('Cannot read MemAvailable from /proc/meminfo; refusing startup.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--minimum-gib', type=float, default=10)
    args = parser.parse_args()
    try:
        available = check_available_memory(args.minimum_gib)
    except (OSError, ValueError) as error:
        parser.exit(1, f'[FAIL] {error}\n')
    print(f'[OK] Available RAM: {available:.1f} GiB (preflight only, not a reservation).')


if __name__ == '__main__':
    main()
