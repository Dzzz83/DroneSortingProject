#!/usr/bin/env bash
# Host coordinator: ROS helpers run in Noetic; Unity and YOLO stay on the host.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "${PROJECT_ROOT:-$SCRIPT_DIR/../..}")"
source "$SCRIPT_DIR/internal/config.sh"
WORKSPACE="$(realpath -m "$WORKSPACE")"
ENDPOINT_SOURCE="$(realpath -m "$ENDPOINT_SOURCE")"
UNITY_MODE="${UNITY_MODE:-auto}"
MEMORY_MODE="${CAMERA_DEMO_MEMORY_MODE:-auto}"
CAMERA_DEMO_SCOPE="${CAMERA_DEMO_SCOPE:-}"
DETECTOR_PID=''
ROS_WRAPPER_PID=''
LOG_TAIL_PID=''
UNITY_PID=''
RUN_DIR=''

fail() { echo "[FAIL] $*" >&2; exit 1; }

isolate_demo_processes() {
    # Unity and its workers must not share the terminal/Codex OOM group. The
    # scope lives until its last process exits, including an editor left open.
    if [[ -n "${CAMERA_DEMO_SCOPE:-}" ]] && grep -Fq "/$CAMERA_DEMO_SCOPE" /proc/self/cgroup 2>/dev/null; then
        return
    fi
    [[ "$MEMORY_MODE" != off ]] || return 0

    # Probe systemd-run and polkit authorization active test
    if ! command -v systemd-run >/dev/null || ! systemctl --user show-environment >/dev/null 2>&1 || ! systemd-run --user --scope --quiet true 2>/dev/null; then
        [[ "$MEMORY_MODE" != required ]] || fail 'Memory isolation requires a running systemd user manager with polkit authorization.'
        echo '[INFO] Running without a systemd memory scope (polkit/systemd-run authorization unavailable); monitoring host memory usage.'
        return 0
    fi

    local scope="camera-delivery-$(date +%s)-$$.scope"
    echo "[SETUP] Isolating demo processes (${CAMERA_DEMO_MEMORY_MAX:-8G} RAM, ${CAMERA_DEMO_SWAP_MAX:-2G} swap limit)."
    local exit_status
    if systemd-run --user --scope --quiet --unit="$scope" \
        --property="MemoryMax=${CAMERA_DEMO_MEMORY_MAX:-8G}" --property="MemorySwapMax=${CAMERA_DEMO_SWAP_MAX:-2G}" \
        env CAMERA_DEMO_SCOPE="$scope" bash "$SCRIPT_DIR/run_demo.sh" "$@"; then
        exit 0
    else
        exit_status=$?
    fi
    if [[ "$(systemctl --user show "$scope" --property=Result --value 2>/dev/null)" == oom-kill ]]; then
        echo "[FAIL] Demo scope exhausted its ${CAMERA_DEMO_MEMORY_MAX:-8G} RAM / ${CAMERA_DEMO_SWAP_MAX:-2G} swap allowance; Linux killed a demo process. Inspect unity.log and journalctl --user -u $scope." >&2
    fi
    exit "$exit_status"
}

check_dependencies() {
    [[ $# == 0 ]] || fail 'Usage: bash ROS1/scripts/run_demo.sh (always restarts the demo)'
    [[ "$(uname -s)" == Linux ]] || fail 'This launcher supports native Linux with ROS Noetic or Distrobox.'
    if grep -qi microsoft /proc/version; then fail 'Use the documented Windows/WSL workflow; this launcher opens host Linux Unity.'; fi
    for command_name in python3 flock realpath; do
        command -v "$command_name" >/dev/null || fail "Missing command: $command_name"
    done
    case "$UNITY_MODE" in auto|manual) ;; *) fail 'UNITY_MODE must be auto or manual.' ;; esac
    case "$MEMORY_MODE" in auto|required|off) ;; *) fail 'CAMERA_DEMO_MEMORY_MODE must be auto, required, or off.' ;; esac
    find_unity_editor
    [[ "$UNITY_MODE" == manual || -x "$UNITY_EDITOR" ]] || fail "Unity Editor not found: $UNITY_EDITOR (set UNITY_EDITOR or UNITY_MODE=manual)."
    [[ -x "$DETECTOR_PYTHON" ]] || fail "Detector Python not found: $DETECTOR_PYTHON (run setup_demo.sh or set DETECTOR_PYTHON)."
    [[ -f "$DETECTOR_WEIGHTS" ]] || fail "Trained package weights not found: $DETECTOR_WEIGHTS (set DETECTOR_WEIGHTS)."
    [[ -f "$PROJECT_ROOT/Unity/Assets/Scenes/MainWarehouse.unity" ]] || fail 'MainWarehouse scene is missing.'
    check_unity_assets
    validate_camera_workspace
    select_ros_environment
}

check_available_resources() {
    # Wait briefly for sockets to finish closing after the previous run exits.
    python3 - "$PROJECT_ROOT/Unity" "$UNITY_MODE" <<'PYTHON'
import socket, sys, time
from pathlib import Path
project = str(Path(sys.argv[1]).resolve())
for port in (11311, 10000, 10001):
    deadline = time.monotonic() + 5
    while True:
        try:
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind(('0.0.0.0', port))
            break
        except OSError as error:
            if time.monotonic() >= deadline:
                raise SystemExit(f'Port {port} is unavailable: {error}. An untracked service must be stopped by its owner.')
            time.sleep(0.1)
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        args = (entry / 'cmdline').read_bytes().decode().split('\0')
    except (OSError, UnicodeError):
        continue
    if '-adb2' in args:
        continue
    if sys.argv[2] != 'manual' and args and Path(args[0]).name.startswith(('Unity', 'unityhub-unity-editor')) and '-projectPath' in args:
        index = args.index('-projectPath')
        if index + 1 < len(args) and str(Path(args[index + 1]).resolve()) == project:
            raise SystemExit('Previous Unity editor did not close; inspect its run logs.')
PYTHON
}

cleanup() {
    trap - EXIT INT TERM HUP
    [[ -z "$RUN_DIR" ]] || echo "[CLEANUP] Stopping this run's ROS and detector. Logs: $RUN_DIR"
    for supervisor in prepare-supervisor ros-supervisor; do
        if [[ -n "$RUN_DIR" && -f "$RUN_DIR/$supervisor.pid" ]]; then
            # Signal inside the container, not just its host-side wrapper.
            "${ROS_RUNNER[@]}" -c 'kill -TERM "$(cat "$1/$2.pid")"' bash "$RUN_DIR" "$supervisor" 2>/dev/null || true
        fi
    done
    if [[ -n "$ROS_WRAPPER_PID" ]]; then
        for ((attempt=0; attempt<35; attempt++)); do
            kill -0 "$ROS_WRAPPER_PID" 2>/dev/null || break
            sleep 1
        done
        kill -TERM "$ROS_WRAPPER_PID" 2>/dev/null || true
        wait "$ROS_WRAPPER_PID" 2>/dev/null || true
    fi
    [[ -z "$DETECTOR_PID" ]] || kill -TERM "$DETECTOR_PID" 2>/dev/null || true
    [[ -z "$DETECTOR_PID" ]] || wait "$DETECTOR_PID" 2>/dev/null || true
    [[ -z "$LOG_TAIL_PID" ]] || kill "$LOG_TAIL_PID" 2>/dev/null || true
    # Unity remains open, including when the user exits the launcher.
}

wait_for_detector() {
    for ((attempt=0; attempt<60; attempt++)); do
        kill -0 "$DETECTOR_PID" 2>/dev/null || {
            [[ -f "$RUN_DIR/detector.log" ]] && tail -30 "$RUN_DIR/detector.log"
            fail 'Detector exited during startup.'
        }
        if grep -q '^PACKAGE_DETECTOR_SERVICE_READY 10001$' "$RUN_DIR/detector.log" 2>/dev/null; then return 0; fi
        sleep 1
    done
    fail 'Detector readiness timed out; inspect detector.log.'
}

wait_for_ros() {
    for ((attempt=0; attempt<90; attempt++)); do
        kill -0 "$ROS_WRAPPER_PID" 2>/dev/null || {
            [[ -f "$RUN_DIR/ros-supervisor.log" ]] && tail -30 "$RUN_DIR/ros-supervisor.log"
            fail 'ROS supervisor exited during startup.'
        }
        [[ ! -f "$RUN_DIR/ros-ready" ]] || return 0
        sleep 1
    done
    fail 'ROS readiness timed out; inspect ros-supervisor.log and ros.log.'
}

open_unity() {
    if [[ "$UNITY_MODE" == manual ]]; then
        echo "[INFO] Open $PROJECT_ROOT/Unity in Unity 6000.6.0f1, select MainWarehouse, connect to 127.0.0.1:10000, then press Play."
        return
    fi
    mkdir -p "$PROJECT_ROOT/Unity/UserSettings"
    if [[ ! -e "$PROJECT_ROOT/Unity/UserSettings/Search.settings" ]]; then
        printf 'indexOnEditorStartup = false\n' > "$PROJECT_ROOT/Unity/UserSettings/Search.settings"
    fi
    if [[ -d "$PROJECT_ROOT/Unity/Temp/__Backupscenes" ]]; then
        mv "$PROJECT_ROOT/Unity/Temp/__Backupscenes" "$RUN_DIR/unity-scene-backups"
        echo "[RESTART] Preserved Unity recovery scenes: $RUN_DIR/unity-scene-backups"
    fi
    "$UNITY_EDITOR" -projectPath "$PROJECT_ROOT/Unity" -job-worker-count 4 \
        -desiredWorkerCount 1 -standbyWorkerCount 1 -idleWorkerShutdownDelay 1000 \
        -executeMethod DemoLauncher.Prepare \
        -logFile "$RUN_DIR/unity.log" > "$RUN_DIR/unity-start.log" 2>&1 &
    UNITY_PID=$!
    for ((attempt=0; attempt<${UNITY_STARTUP_TIMEOUT:-600}; attempt++)); do
        if ! kill -0 "$UNITY_PID" 2>/dev/null; then
            if [[ -f "$RUN_DIR/unity.log" ]]; then
                tail -30 "$RUN_DIR/unity.log"
            elif [[ -f "$RUN_DIR/unity-start.log" ]]; then
                tail -30 "$RUN_DIR/unity-start.log"
            fi
            fail 'Unity exited before startup activity settled; inspect unity.log or unity-start.log.'
        fi
        if [[ -f "$RUN_DIR/unity.log" ]] && grep -q 'CAMERA_DEMO_EDITOR_IDLE' "$RUN_DIR/unity.log"; then return 0; fi
        sleep 1
    done
    fail 'Unity did not finish startup activity before the timeout; inspect unity.log. ROS and YOLO have not started.'
}

check_dependencies "$@"
if [[ -n "${CAMERA_DEMO_MIN_AVAILABLE_GIB:-}" ]]; then
    python3 "$SCRIPT_DIR/../tools/memory.py" \
        --minimum-gib "$CAMERA_DEMO_MIN_AVAILABLE_GIB"
fi
isolate_demo_processes "$@"
mkdir -p "$PROJECT_ROOT/ROS1/build"
exec 8> "$PROJECT_ROOT/ROS1/build/camera-demo-restart.lock"
flock -n 8 || fail 'Another demo restart is in progress; wait until it is READY.'
echo '[RESTART] Closing the previous demo before fresh startup.'
python3 "$PROJECT_ROOT/ROS1/tools/processes.py" "$PROJECT_ROOT" "$CAMERA_DEMO_SCOPE" --unity-mode "$UNITY_MODE"
check_available_resources
exec 9> "$PROJECT_ROOT/ROS1/build/camera-demo.lock"
flock -n 9 || fail 'Another camera demo launcher is running.'
RUN_DIR="$(mktemp -d "$PROJECT_ROOT/ROS1/build/camera-demo-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
printf '%s\n' "$CAMERA_DEMO_SCOPE" > "$RUN_DIR/systemd-scope.txt"
cp "$SCRIPT_DIR/internal/build_workspace.sh" "$SCRIPT_DIR/internal/start_ros.sh" "$RUN_DIR/"
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

echo "[1/4] Preparing ROS workspace. Logs: $RUN_DIR"
"${ROS_RUNNER[@]}" "$RUN_DIR/build_workspace.sh" "$PROJECT_ROOT" "$WORKSPACE" "$ENDPOINT_SOURCE" "$RUN_DIR" \
    > "$RUN_DIR/build.log" 2>&1 8>&- 9>&- &
ROS_WRAPPER_PID=$!
wait "$ROS_WRAPPER_PID" || { [[ -f "$RUN_DIR/build.log" ]] && tail -40 "$RUN_DIR/build.log"; fail 'ROS preparation failed.'; }
ROS_WRAPPER_PID=''
echo '[2/4] Opening MainWarehouse; waiting for editor background work. Wait for READY before pressing Play.'
open_unity 8>&- 9>&-
echo '[3/4] Starting the YOLO detector on the host.'
CAMERA_DEMO_RUN_DIR="$RUN_DIR" "$DETECTOR_PYTHON" "$PROJECT_ROOT/ROS1/tools/detector.py" \
    --weights "$DETECTOR_WEIGHTS" --device "$DETECTOR_DEVICE" --confidence 0.25 --image-size 640 --context-scale 1.5 \
    > "$RUN_DIR/detector.log" 2>&1 8>&- 9>&- &
DETECTOR_PID=$!
wait_for_detector

echo '[4/4] Starting ROS camera delivery.'
"${ROS_RUNNER[@]}" "$RUN_DIR/start_ros.sh" "$PROJECT_ROOT" "$WORKSPACE" "$RUN_DIR" \
    > "$RUN_DIR/ros-supervisor.log" 2>&1 8>&- 9>&- &
ROS_WRAPPER_PID=$!
wait_for_ros

flock -u 8
exec 8>&-
echo '[READY] Press Play in Unity. Keep this terminal open; Ctrl+C stops ROS and YOLO and leaves Unity open.'
tail -n +1 -F "$RUN_DIR/monitor.log" 9>&- &
LOG_TAIL_PID=$!
while kill -0 "$ROS_WRAPPER_PID" 2>/dev/null; do
    [[ "$UNITY_MODE" == manual ]] || kill -0 "$UNITY_PID" 2>/dev/null || fail 'Unity exited during the demo; inspect unity.log and the system journal.'
    kill -0 "$DETECTOR_PID" 2>/dev/null || fail 'Detector exited during the demo; inspect detector.log.'
    sleep 1
done
wait "$ROS_WRAPPER_PID"
