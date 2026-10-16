#!/usr/bin/env bash
# Shared Linux defaults; callers set PROJECT_ROOT before sourcing this file.
ENDPOINT_REVISION=993d366b8900bf9f3d2da444fde64c0379b4dc7c
WORKSPACE="${CAMERA_DEMO_WORKSPACE:-$PROJECT_ROOT/ROS1/build/linux-camera-workspace}"
ENDPOINT_SOURCE="${ROS_TCP_ENDPOINT_SOURCE:-$PROJECT_ROOT/ROS1/build/ROS-TCP-Endpoint}"
DETECTOR_PYTHON="${DETECTOR_PYTHON:-$PROJECT_ROOT/ROS1/build/detector-venv/bin/python}"
DETECTOR_WEIGHTS="${DETECTOR_WEIGHTS:-$PROJECT_ROOT/ROS1/models/package_detector.pt}"
DETECTOR_DEVICE="${DETECTOR_DEVICE:-cpu}"
ROS_CONTAINER="${ROS_CONTAINER:-ros-noetic}"
ROS_RUNNER=(bash)

select_ros_environment() {
    case "${ROS_ENVIRONMENT:-auto}" in
        auto|native|distrobox) ;;
        *) fail 'ROS_ENVIRONMENT must be auto, native, or distrobox.' ;;
    esac
    if [[ -f /opt/ros/noetic/setup.bash && "${ROS_ENVIRONMENT:-auto}" != distrobox ]]; then
        return
    fi
    [[ "${ROS_ENVIRONMENT:-auto}" != native ]] || fail 'Native ROS Noetic is missing.'
    command -v distrobox >/dev/null || fail 'Install ROS Noetic or Distrobox; see README Linux setup.'
    ROS_RUNNER=(distrobox enter --name "$ROS_CONTAINER" -- bash)
}

validate_camera_workspace() {
    WORKSPACE="$(realpath -m "$WORKSPACE")"
    ENDPOINT_SOURCE="$(realpath -m "$ENDPOINT_SOURCE")"
    case "$WORKSPACE/" in
        "$PROJECT_ROOT/"|"$PROJECT_ROOT/Unity/"*|"$PROJECT_ROOT/ROS1/src/"*)
            fail 'Choose a separate CAMERA_DEMO_WORKSPACE directory.' ;;
    esac
}

check_unity_assets() {
    python3 - "$PROJECT_ROOT/Unity/Assets" <<'PYTHON'
from pathlib import Path
import sys
missing = []
for path in Path(sys.argv[1]).rglob('*'):
    if path.is_file() and path.stat().st_size <= 1024:
        with path.open('rb') as asset:
            if asset.read(43).startswith(b'version https://git-lfs.github.com/spec/v1'):
                missing.append(str(path))
if missing:
    raise SystemExit('Unity Git LFS assets are missing. Install Git LFS and run git lfs pull.\n'
                     + '\n'.join(missing[:5]))
PYTHON
}

find_unity_editor() {
    if [[ -n "${UNITY_EDITOR:-}" ]]; then
        return
    fi
    local editor
    for editor in "$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/Unity" \
                  "$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/unityhub-unity-editor-6000.6.0f1" \
                  /opt/unity/Editor/Unity; do
        if [[ -x "$editor" ]]; then
            UNITY_EDITOR="$editor"
            return
        fi
    done
    UNITY_EDITOR=''
}
