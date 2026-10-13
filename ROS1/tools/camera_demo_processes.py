"""Stop this project's previous camera demo, including container processes.

Linux /proc exposes host PIDs even for Distrobox processes. Match run-specific
arguments/environment before signalling; never kill a process by port or name.
"""
import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal
import subprocess
import time

SCOPE_NAME = re.compile(r'camera-delivery-\d+-\d+\.scope')


@dataclass(frozen=True)
class Process:
    pid: int
    started: str
    arguments: tuple
    environment: frozenset
    scopes: frozenset


def read_process(pid, proc_root=Path('/proc')):
    directory = proc_root / str(pid)
    try:
        if directory.stat().st_uid != os.getuid():
            return None
        # comm can contain spaces/parentheses; fields after its last ')' start
        # with state (field 3), so starttime (field 22) has index 19.
        fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        arguments = tuple(filter(None, (directory / 'cmdline').read_bytes().decode().split('\0')))
        environment = frozenset((directory / 'environ').read_bytes().decode().split('\0'))
        scopes = frozenset(part for line in (directory / 'cgroup').read_text().splitlines()
                           for part in line.split('/') if SCOPE_NAME.fullmatch(part))
        return Process(int(pid), fields[19], arguments, environment, scopes)
    except (OSError, UnicodeError, IndexError):
        return None


def processes():
    return [process for entry in Path('/proc').iterdir() if entry.name.isdigit()
            if (process := read_process(entry.name)) is not None]


def is_project_editor(process, project):
    arguments = process.arguments
    if not arguments or not Path(arguments[0]).name.startswith(('Unity', 'unityhub-unity-editor')):
        return False
    if '-projectPath' not in arguments:
        return False
    index = arguments.index('-projectPath') + 1
    return index < len(arguments) and Path(arguments[index]).resolve() == project / 'Unity'


def owns_ros_process(process, run_directories):
    for directory in run_directories:
        if (f'CAMERA_DEMO_RUN_DIR={directory}' in process.environment or
                f'ROS_LOG_DIR={directory}/ros-logs' in process.environment):
            return True
        for helper in ('prepare_camera_demo_ros.sh', 'start_camera_demo_ros.sh'):
            if str(directory / helper) in process.arguments:
                return True
    return False


def still_running(process):
    current = read_process(process.pid)
    return current is not None and current.started == process.started


def send_signal(process, action):
    if still_running(process):
        try:
            os.kill(process.pid, action)
        except ProcessLookupError:
            pass


def stop_processes(selected, grace=20):
    for process in selected:
        send_signal(process, signal.SIGTERM)
    deadline = time.monotonic() + grace
    while any(still_running(process) for process in selected) and time.monotonic() < deadline:
        time.sleep(0.1)
    for process in selected:
        send_signal(process, signal.SIGKILL)
    deadline = time.monotonic() + 5
    while any(still_running(process) for process in selected) and time.monotonic() < deadline:
        time.sleep(0.1)
    if any(still_running(process) for process in selected):
        raise RuntimeError('Previous demo processes did not stop; inspect their run logs.')


def stop_scope(scope):
    # systemd scopes include Unity import workers and the host detector. Ignore
    # absent units left over in old log directories, but report actual failures.
    result = subprocess.run(['systemctl', '--user', 'show', scope, '--property=ActiveState', '--value'],
                            capture_output=True, text=True, check=False)
    if result.stdout.strip() not in ('active', 'activating', 'deactivating'):
        return
    print(f'[RESTART] Closing previous demo scope: {scope}', flush=True)
    subprocess.run(['systemctl', '--user', 'stop', '--no-block', scope], check=True)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        result = subprocess.run(['systemctl', '--user', 'show', scope, '--property=ActiveState', '--value'],
                                capture_output=True, text=True, check=False)
        if result.stdout.strip() not in ('active', 'activating', 'deactivating'):
            return
        time.sleep(0.2)
    subprocess.run(['systemctl', '--user', 'kill', '--signal=KILL', '--kill-whom=all', scope], check=True)
    # The following port/editor preflight also verifies shutdown before startup.


def restart(project, current_scope):
    project = project.resolve()
    run_directories = [path.resolve() for path in (project / 'ROS1/build').glob('camera-demo-*')
                       if path.is_dir() and not path.is_symlink()]
    scopes = set()
    for directory in run_directories:
        record = directory / 'systemd-scope.txt'
        if record.exists():
            scope = record.read_text().strip()
            if SCOPE_NAME.fullmatch(scope):
                scopes.add(scope)
    snapshot = processes()
    editors = [process for process in snapshot if is_project_editor(process, project)]
    # Recover an older project editor even if it predates scope log recording.
    for editor in editors:
        scopes.update(editor.scopes)
    scopes.discard(current_scope)
    ros = [process for process in snapshot if owns_ros_process(process, run_directories)
           and current_scope not in process.scopes]
    if ros:
        print('[RESTART] Stopping previous demo ROS/build processes (including Distrobox).', flush=True)
        stop_processes(ros)
    for scope in sorted(scopes):
        stop_scope(scope)
    remaining_editors = [process for process in editors if still_running(process)
                         and current_scope not in process.scopes]
    if remaining_editors:
        print('[RESTART] Closing this project\'s Unity editor and import workers.', flush=True)
        stop_processes(remaining_editors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', type=Path)
    parser.add_argument('current_scope')
    args = parser.parse_args()
    try:
        restart(args.project, args.current_scope)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(1, f'[FAIL] Restart failed: {error}\n')


if __name__ == '__main__':
    main()
