#!/usr/bin/env bash
# Install pinned checker tooling without modifying ROS system Python packages.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "$SCRIPT_DIR/../..")"
fail() { echo "[FAIL] $*" >&2; exit 1; }
source "$SCRIPT_DIR/internal/config.sh"
select_ros_environment
command -v npm >/dev/null || fail 'Install Node.js and npm on the host first.'
npm ci --prefix "$PROJECT_ROOT/ROS1/requirements/typecheck"
"${ROS_RUNNER[@]}" -c '
    set -e
    cd "$1"
    /usr/bin/python3 -m venv --system-site-packages ROS1/build/typecheck-venv
    exec ROS1/build/typecheck-venv/bin/python -m pip install -r ROS1/requirements/typecheck.txt
' bash "$PROJECT_ROOT"
