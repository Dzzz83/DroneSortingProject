"""Inspect Unity cache and Linux memory without starting application processes."""
import argparse
import math
from pathlib import Path

GIB = 1024 ** 3
CACHE_FILES = ('ArtifactDB', 'SourceAssetDB', 'ScriptAssemblies/Assembly-CSharp.dll')


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
            f'{minimum_gib:g} GiB. Close memory-heavy applications first. '
            'CAMERA_DEMO_MIN_AVAILABLE_GIB adjusts this optional hard minimum.')
    return available


def has_unity_cache_files(project):
    return all((project / 'Unity/Library' / name).is_file() for name in CACHE_FILES)


def has_unity_cache(project):
    # A stopped import can create every cache file before the scene is ready.
    imports = [path for path in (project / 'ROS1/build').glob('unity-import-*') if path.is_dir()]
    if imports:
        latest = max(imports, key=lambda path: path.stat().st_mtime_ns)
        if not (latest / 'COMPLETE').is_file():
            return False
    # Older, previously working checkouts may predate the import helper.
    return has_unity_cache_files(project)


def check_startup(project, unity_mode, minimum_gib=None, meminfo=Path('/proc/meminfo')):
    cached = has_unity_cache(project)
    if not cached and unity_mode == 'auto':
        raise ValueError('Unity cache is missing or its latest recorded import did not complete. Run '
                         'bash ROS1/scripts/prepare_camera_demo_unity.sh first. '
                         'It imports Unity without starting ROS or YOLO. '
                         'UNITY_MODE=manual can check the backend without opening Unity.')
    available = (available_memory_gib(meminfo) if minimum_gib is None
                 else check_available_memory(minimum_gib, meminfo))
    messages = [f'[INFO] Unity cache: {"present" if cached else "absent/incomplete"}; '
                f'available RAM: {available:.1f} GiB.']
    if minimum_gib is None and available < 4:
        messages.append('[WARN] Less than 4 GiB RAM is available. This is provisional '
                        'guidance, not a measured requirement; consider closing other applications.')
    messages.append('[INFO] Cache presence does not rule out asset reimports; memory is not reserved.')
    return messages


def import_pressure_reason(available_gib, pressure_text):
    if available_gib < 2:
        return 'available host RAM is below 2 GiB'
    for line in pressure_text.splitlines():
        fields = line.split()
        if fields and fields[0] == 'full':
            values = dict(field.split('=', 1) for field in fields[1:])
            if float(values['avg10']) >= 10:
                return 'host full memory pressure avg10 is at least 10%'
            return None
    raise ValueError('Cannot read full memory pressure; refusing to continue import.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path)
    parser.add_argument('--unity-mode', choices=('auto', 'manual'), default='auto')
    parser.add_argument('--minimum-gib', type=float)
    parser.add_argument('--check-import-pressure', action='store_true')
    args = parser.parse_args()
    try:
        if args.check_import_pressure:
            reason = import_pressure_reason(available_memory_gib(), Path('/proc/pressure/memory').read_text())
            if reason:
                raise ValueError(reason)
        elif args.project:
            for message in check_startup(args.project, args.unity_mode, args.minimum_gib):
                print(message)
        elif args.minimum_gib is not None:
            available = check_available_memory(args.minimum_gib)
            print(f'[OK] Available RAM: {available:.1f} GiB (not a reservation).')
        else:
            parser.error('Specify --project, --minimum-gib, or --check-import-pressure.')
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f'[FAIL] {error}\n')


if __name__ == '__main__':
    main()
