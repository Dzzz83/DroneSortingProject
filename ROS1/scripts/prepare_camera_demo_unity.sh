#!/usr/bin/env bash
# Import Unity alone under bounded resources; never start ROS or the detector.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "${PROJECT_ROOT:-$SCRIPT_DIR/../..}")"
source "$SCRIPT_DIR/camera_demo_linux.sh"
fail() { echo "[FAIL] $*" >&2; exit 1; }
IMPORT_TIMEOUT="${UNITY_IMPORT_TIMEOUT:-600}"
[[ "$IMPORT_TIMEOUT" =~ ^[1-9][0-9]{0,2}$ ]] && ((IMPORT_TIMEOUT <= 600)) \
    || fail 'UNITY_IMPORT_TIMEOUT must be an integer from 1 to 600 seconds.'
[[ $# == 0 ]] || fail 'Usage: bash ROS1/scripts/prepare_camera_demo_unity.sh'
[[ "$(uname -s)" == Linux ]] || fail 'Host Linux is required.'
find_unity_editor
[[ -x "$UNITY_EDITOR" ]] || fail 'Unity 6000.6.0f1 is missing; set UNITY_EDITOR.'
command -v systemd-run >/dev/null || fail 'Bounded import requires systemd-run.'
systemctl --user show-environment >/dev/null 2>&1 || fail 'Bounded import requires a systemd user manager.'
[[ -r /proc/pressure/memory ]] || fail 'Bounded import requires Linux memory pressure reporting.'
check_unity_assets
# Budget: at most 5 GiB RAM for this import plus 2 GiB of initial host headroom.
# This is a provisional experiment budget, not a measured Unity requirement.
python3 "$SCRIPT_DIR/../tools/camera_demo_resources.py" --minimum-gib 7
python3 "$SCRIPT_DIR/../tools/camera_demo_resources.py" --check-import-pressure
mkdir -p "$PROJECT_ROOT/ROS1/build"
exec 9> "$PROJECT_ROOT/ROS1/build/camera-demo.lock"
flock -n 9 || fail 'Stop the camera launcher before preparing Unity.'
# Never close another editor, including one opened manually for this checkout.
python3 - "$PROJECT_ROOT/Unity" <<'PYTHON'
from pathlib import Path
import sys
project = Path(sys.argv[1]).resolve()
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        args = (entry / 'cmdline').read_bytes().decode().split('\0')
        if '-projectPath' in args and Path(args[args.index('-projectPath') + 1]).resolve() == project:
            raise SystemExit('Close this project in Unity before preparing it.')
    except (OSError, UnicodeError, IndexError):
        continue
PYTHON
RUN_DIR="$(mktemp -d "$PROJECT_ROOT/ROS1/build/unity-import-XXXXXX")"
SCOPE="camera-import-$(date +%s)-$$.scope"
IMPORT_PID=''
cleanup() {
    trap - EXIT INT TERM HUP
    systemctl --user stop --no-block "$SCOPE" 2>/dev/null || true
    if [[ -n "$IMPORT_PID" ]]; then
        # Do not wait indefinitely for a swapping or unresponsive import.
        for ((attempt=0; attempt<5; attempt++)); do
            kill -0 "$IMPORT_PID" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "$IMPORT_PID" 2>/dev/null; then
            systemctl --user kill --signal=KILL --kill-whom=all "$SCOPE" 2>/dev/null || true
        fi
        wait "$IMPORT_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
printf '%s\n' "$SCOPE" > "$RUN_DIR/systemd-scope.txt"
echo "[IMPORT] Unity only: RAM high=4G/max=5G; swap max=512M; timeout=${IMPORT_TIMEOUT}s. Logs: $RUN_DIR"
systemd-run --user --scope --quiet --unit="$SCOPE" \
    --property=MemoryHigh=4G --property=MemoryMax=5G --property=MemorySwapMax=512M \
    --property="RuntimeMaxSec=$IMPORT_TIMEOUT" \
    "$UNITY_EDITOR" -projectPath "$PROJECT_ROOT/Unity" -batchmode -quit \
    -refreshImportMode InProcess -job-worker-count 2 -gc-helper-count 1 \
    -executeMethod DeliveryDemoSceneLauncher.OpenMainWarehouse \
    -logFile "$RUN_DIR/unity.log" > "$RUN_DIR/scope.log" 2>&1 9>&- &
IMPORT_PID=$!
started=$SECONDS
pressure_samples=0
while kill -0 "$IMPORT_PID" 2>/dev/null; do
    if ((SECONDS - started >= IMPORT_TIMEOUT)); then
        fail "Import exceeded $IMPORT_TIMEOUT seconds; inspect $RUN_DIR."
    fi
    if python3 "$SCRIPT_DIR/../tools/camera_demo_resources.py" --check-import-pressure >> "$RUN_DIR/memory.log" 2>&1; then
        pressure_samples=0
    else
        pressure_samples=$((pressure_samples + 1))
    fi
    if ((pressure_samples >= 5)); then
        fail "Stopping import after five consecutive pressure checks; inspect $RUN_DIR/memory.log."
    fi
    # Read-only samples: no additional application workload is started.
    printf '%(%FT%T%z)T\n' -1 >> "$RUN_DIR/memory.log"
    awk '/MemAvailable:|SwapFree:/ {print}' /proc/meminfo >> "$RUN_DIR/memory.log"
    cat /proc/pressure/memory >> "$RUN_DIR/memory.log"
    systemctl --user show "$SCOPE" --property=MemoryCurrent --property=MemoryPeak \
        >> "$RUN_DIR/memory.log" 2>/dev/null || true
    sleep 1
done
if wait "$IMPORT_PID"; then
    IMPORT_PID=''
else
    IMPORT_PID=''
    fail "Unity import exited unsuccessfully; inspect $RUN_DIR/unity.log and scope.log."
fi
grep -q 'Delivery demo scene ready: Assets/Scenes/MainWarehouse.unity' "$RUN_DIR/unity.log" \
    || fail 'Unity did not confirm warehouse scene readiness.'
# A zero exit status alone must not mark a partial import as ready.
python3 - "$PROJECT_ROOT" "$SCRIPT_DIR/../tools" <<'PYTHON'
from pathlib import Path
import sys
sys.path.insert(0, sys.argv[2])
from camera_demo_resources import has_unity_cache_files
if not has_unity_cache_files(Path(sys.argv[1])):
    raise SystemExit('Unity exited without the expected cache files.')
PYTHON
touch "$RUN_DIR/COMPLETE"
echo "[OK] Unity import and scene opening completed. Logs: $RUN_DIR"
