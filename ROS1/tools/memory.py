"""Check optional host RAM headroom before restarting the demo."""
import argparse
import math
from pathlib import Path

GIB = 1024 ** 3


def available_memory_gib(meminfo=Path('/proc/meminfo')):
    for line in meminfo.read_text().splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) * 1024 / GIB
    raise ValueError('Cannot read MemAvailable from /proc/meminfo; refusing startup.')


def check_available_memory(minimum_gib, meminfo=Path('/proc/meminfo')):
    if not math.isfinite(minimum_gib) or minimum_gib <= 0:
        raise ValueError('Minimum available RAM must be a positive number of GiB.')
    available = available_memory_gib(meminfo)
    if available < minimum_gib:
        raise ValueError(
            f'Only {available:.1f} GiB RAM is available; the configured minimum is '
            f'{minimum_gib:g} GiB. Close memory-heavy applications first.')
    return available



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--minimum-gib', required=True, type=float)
    args = parser.parse_args()
    try:
        available = check_available_memory(args.minimum_gib)
    except (OSError, ValueError) as error:
        parser.exit(1, f'[FAIL] {error}\n')
    print(f'[OK] Available RAM: {available:.1f} GiB (not a reservation).')


if __name__ == '__main__':
    main()
