#!/usr/bin/env bash
# Same ROS navigation, rendered by a standalone course player for recording.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "$SCRIPT_DIR/../..")"
source "$SCRIPT_DIR/internal/config.sh"
fail() { echo "[FAIL] $*" >&2; exit 1; }
REBUILD=false
BUILD_ONLY=false
ROS_ARGUMENTS=()
for argument in "$@"; do
    case "$argument" in
        --rebuild) REBUILD=true ;;
        --build-only) BUILD_ONLY=true ;;
        --help)
            cat <<'HELP'
Usage: bash ROS1/scripts/run_recording_demo.sh [--rebuild] [--build-only] [name:=value ...]

Runs the existing A-to-B navigation in a 1280x720, 30 FPS standalone window.
The player keeps running while recorder controls have focus. Unity Play is not needed.
The first run builds the player; --rebuild refreshes it after code/scene changes.
With Unity open, stop Play and press Ctrl+R so its build helper can handle the request.
Record the CourseDemo window. Camera controls are unchanged. Ctrl+C stops the demo.
HELP
            exit 0 ;;
        *:=*) ROS_ARGUMENTS+=("$argument") ;;
        *) fail "Unknown argument: $argument (use --help)." ;;
    esac
done
if [[ "$BUILD_ONLY" != true ]]; then
    select_ros_environment
    check_ros_python_dependencies
fi
PLAYER="$PROJECT_ROOT/Unity/Builds/CourseRecording/CourseDemo.x86_64"
BUILD_RESULT="$PROJECT_ROOT/Unity/Logs/course-recording-build.result"
BUILD_REQUEST="$PROJECT_ROOT/Unity/Temp/course-recording-build.request"
# Cancelling a pending launch must not leave a surprise build request in the Editor.
trap 'rm -f "$BUILD_REQUEST"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ ! -x "$PLAYER" || "$REBUILD" == true ]]; then
    mkdir -p "$PROJECT_ROOT/Unity/Temp" "$PROJECT_ROOT/Unity/Logs"
    rm -f "$BUILD_RESULT"
    if command -v fuser >/dev/null && fuser -s "$PROJECT_ROOT/Unity/Temp/UnityLockfile" 2>/dev/null; then
        touch "$BUILD_REQUEST"
        echo '[BUILD] In Unity, stop Play and press Ctrl+R. The course player will build automatically.'
        for ((attempt=0; attempt<1800; attempt++)); do
            [[ ! -f "$BUILD_RESULT" ]] || break
            sleep 1
        done
        [[ -f "$BUILD_RESULT" ]] || fail 'Timed out waiting for Unity to finish the build.'
    else
        find_unity_editor
        [[ -n "$UNITY_EDITOR" ]] || fail 'Unity 6000.6.0f1 was not found; set UNITY_EDITOR.'
        "$UNITY_EDITOR" -batchmode -projectPath "$PROJECT_ROOT/Unity" \
            -executeMethod CourseRecordingBuild.Build -quit \
            -logFile "$PROJECT_ROOT/Unity/Logs/course-recording-build.log"
    fi
    [[ "$(head -n 1 "$BUILD_RESULT")" == OK ]] || fail "$(cat "$BUILD_RESULT")"
fi
[[ -x "$PLAYER" ]] || fail 'The recording player is missing.'
[[ "$BUILD_ONLY" != true ]] || exit 0
if command -v fuser >/dev/null && fuser -s "$PROJECT_ROOT/Unity/Temp/UnityLockfile" 2>/dev/null; then
    fail 'Player build is ready. Close the Unity Editor to free GPU memory, then rerun this command.'
fi
# Avoid starting a second simulation when the ROS port is already in use.
python3 - <<'PY'
import socket
with socket.socket() as probe:
    if probe.connect_ex(('127.0.0.1',11311)) == 0:
        raise SystemExit('[FAIL] ROS is already running. Stop that run with Ctrl+C first.')
PY
# Unity's native Wayland backend hits a compositor FIFO protocol error here.
# XWayland retains the same Vulkan renderer and remains capturable by wf-recorder.
env SDL_VIDEODRIVER=x11 "$PLAYER" -screen-fullscreen 0 -screen-width 1280 -screen-height 720 \
    -logFile "$PROJECT_ROOT/Unity/Builds/CourseRecording/player.log" &
PLAYER_PID=$!
cleanup() { kill "$PLAYER_PID" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
bash "$SCRIPT_DIR/run_navigation.sh" "${ROS_ARGUMENTS[@]}"
