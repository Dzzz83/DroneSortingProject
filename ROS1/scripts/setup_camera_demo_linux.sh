#!/usr/bin/env bash
# Install project-local runtime dependencies without modifying user workspaces.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "${PROJECT_ROOT:-$SCRIPT_DIR/../..}")"
source "$SCRIPT_DIR/camera_demo_linux.sh"
fail() { echo "[FAIL] $*" >&2; exit 1; }

BACKEND=cpu
case "${1:-}" in
    '') ;;
    --gpu) BACKEND=cu124 ;;
    *) fail 'Usage: bash ROS1/scripts/setup_camera_demo_linux.sh [--gpu]' ;;
esac
[[ $# -le 1 ]] || fail 'Expected at most one option.'
[[ "$(uname -s)" == Linux ]] || fail 'Linux is required.'
grep -qi microsoft /proc/version && fail 'This setup targets host Linux, not WSL.'
for tool in git flock sha256sum python3; do
    command -v "$tool" >/dev/null || fail "Missing command: $tool"
done
check_unity_assets
SETUP_PYTHON="${DETECTOR_SETUP_PYTHON:-python3.11}"
command -v "$SETUP_PYTHON" >/dev/null || fail 'Install Python 3.11 with venv support, or set DETECTOR_SETUP_PYTHON.'
"$SETUP_PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 11), "Detector setup requires Python 3.11"'
select_ros_environment
validate_camera_workspace
"${ROS_RUNNER[@]}" -c 'source /opt/ros/noetic/setup.bash && command -v catkin_make >/dev/null' || fail 'Selected ROS environment needs ROS Noetic and catkin_make; see README.'
mkdir -p "$PROJECT_ROOT/ROS1/build"
exec 9> "$PROJECT_ROOT/ROS1/build/camera-demo-setup.lock"
flock -n 9 || fail 'Another setup is running.'
# Setup must not update the environment while the launcher is using it.
exec 8> "$PROJECT_ROOT/ROS1/build/camera-demo.lock"
flock -n 8 || fail 'Stop the camera launcher before setup.'
(cd "$PROJECT_ROOT/ROS1/models" && sha256sum -c package_detector.sha256)

if [[ ! -e "$ENDPOINT_SOURCE" ]]; then
    mkdir -p "$(dirname "$ENDPOINT_SOURCE")"
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git "$ENDPOINT_SOURCE"
    git -C "$ENDPOINT_SOURCE" checkout --detach "$ENDPOINT_REVISION"
fi
[[ "$(git -C "$ENDPOINT_SOURCE" rev-parse HEAD)" == "$ENDPOINT_REVISION" ]] || fail 'Endpoint revision differs from the tested revision; use a separate ROS_TCP_ENDPOINT_SOURCE.'
[[ -z "$(git -C "$ENDPOINT_SOURCE" status --porcelain)" ]] || fail 'Endpoint has local changes; use a clean checkout.'

VENV="$(dirname "$(dirname "$DETECTOR_PYTHON")")"
[[ "$DETECTOR_PYTHON" == "$VENV/bin/python" && "$VENV" != / ]] || fail 'DETECTOR_PYTHON must name a dedicated venv/bin/python.'
if [[ ! -e "$VENV" ]]; then
    "$SETUP_PYTHON" -m venv "$VENV"
fi
[[ -f "$VENV/pyvenv.cfg" && -x "$DETECTOR_PYTHON" ]] || fail 'Detector environment is not a usable virtual environment.'
"$DETECTOR_PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 11), "Detector venv requires Python 3.11"'
"$DETECTOR_PYTHON" -m pip install "torch==2.5.1+$BACKEND" "torchvision==0.20.1+$BACKEND" \
    -c "$PROJECT_ROOT/ROS1/requirements/detector.txt" --index-url "https://download.pytorch.org/whl/$BACKEND"
"$DETECTOR_PYTHON" -m pip install -r "$PROJECT_ROOT/ROS1/requirements/detector.txt"
"$DETECTOR_PYTHON" -m pip check
# Load and infer on CPU even for a GPU install, before claiming setup succeeded.
"$DETECTOR_PYTHON" - "$DETECTOR_WEIGHTS" <<'PYTHON'
import sys
import numpy as np
from ultralytics import YOLO
model = YOLO(sys.argv[1])
assert model.names == {0: 'package'}, model.names
model.predict(np.zeros((600, 800, 3), dtype=np.uint8), device='cpu', imgsz=640, verbose=False)
PYTHON
RUN_DIR="$(mktemp -d "$PROJECT_ROOT/ROS1/build/setup-camera-XXXXXX")"
trap 'rm -rf "$RUN_DIR"' EXIT
"${ROS_RUNNER[@]}" "$SCRIPT_DIR/prepare_camera_demo_ros.sh" "$PROJECT_ROOT" "$WORKSPACE" "$ENDPOINT_SOURCE" "$RUN_DIR" 8>&- 9>&-
echo '[OK] Setup complete. Prepare Unity once: bash ROS1/scripts/prepare_camera_demo_unity.sh'
echo 'Then run: bash ROS1/scripts/run_camera_delivery_demo.sh'
