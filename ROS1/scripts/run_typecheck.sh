#!/usr/bin/env bash
# Run the same project configuration used by VS Code/Pylance inside Noetic.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "$SCRIPT_DIR/../..")"
fail() { echo "[FAIL] $*" >&2; exit 1; }
source "$SCRIPT_DIR/internal/config.sh"
select_ros_environment
[[ -f "$PROJECT_ROOT/ROS1/requirements/typecheck/node_modules/pyright/index.js" ]] ||
    fail 'Run bash ROS1/scripts/setup_typecheck.sh first.'
exec "${ROS_RUNNER[@]}" -c '
    set -e
    source /opt/ros/noetic/setup.bash
    cd "$1"
    shift
    test -x ROS1/build/typecheck-venv/bin/python || {
        echo "Run bash ROS1/scripts/setup_typecheck.sh first." >&2
        exit 1
    }
    exec node ROS1/requirements/typecheck/node_modules/pyright/index.js \
        --project pyrightconfig.json --pythonpath ROS1/build/typecheck-venv/bin/python "$@"
' bash "$PROJECT_ROOT" "$@"
