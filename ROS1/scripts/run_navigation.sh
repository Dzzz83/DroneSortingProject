#!/usr/bin/env bash
# Reuse the prepared Noetic workspace; leave the user's Unity editor open.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(realpath "$SCRIPT_DIR/../..")"
source "$SCRIPT_DIR/internal/config.sh"
fail() { echo "[FAIL] $*" >&2; exit 1; }
if [[ "${1:-}" == --help ]]; then
    cat <<'HELP'
Usage: bash ROS1/scripts/run_navigation.sh [name:=value ...]

Open Unity's TwoWarehouseDelivery scene, run this command, then press Play.
The launcher builds ROS and starts navigation only, using verified launch defaults.
Examples: speed:=1.5   reset_on_start:=false   'destination:=[34.0,85.0,3.0]'
Keep this terminal open. Ctrl+C stops ROS and leaves Unity open.
Use Unity's Reset physical trial button to replay while ROS remains running.
HELP
    exit 0
fi
for argument in "$@"; do
    [[ "$argument" == *:=* ]] || fail "Expected a ROS launch argument such as speed:=2.0 (use --help)."
done
select_ros_environment
check_ros_python_dependencies
[[ -f "$WORKSPACE/devel/setup.bash" ]] || fail 'Prepare the ROS workspace with setup_demo.sh first.'
echo '[NAVIGATION] Open TwoWarehouseDelivery in Unity and press Play. No detector or package mission is started.'
exec "${ROS_RUNNER[@]}" -c '
    set -e
    source /opt/ros/noetic/setup.bash
    if ! python3 -c "import rosgraph,sys; sys.exit(int(rosgraph.is_master_online()))"; then
        echo "[FAIL] ROS is already running. Stop that run with Ctrl+C before starting another." >&2
        exit 1
    fi
    echo "[BUILD] Preparing the navigation workspace."
    cd "$1"
    catkin_make -j2 -l2 >/dev/null || exit
    source devel/setup.bash
    shift
    echo "[READY] Press Play in Unity. Navigation starts when Unity feedback arrives. Ctrl+C stops this run."
    exec roslaunch --screen drone_sorting navigation_demo.launch "$@"
' bash "$WORKSPACE" "$@"
