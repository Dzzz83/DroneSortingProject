#!/usr/bin/env bash
# Own the ROS launch and monitor inside ROS/Distrobox, where their PIDs are valid.
set -eo pipefail

PROJECT_ROOT="$1"
WORKSPACE="$2"
RUN_DIR="$3"
export CAMERA_DEMO_RUN_DIR="$RUN_DIR"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
export ROS_LOG_DIR="$RUN_DIR/ros-logs"

LAUNCH_PID=''
MONITOR_PID=''
cleanup() {
    trap - EXIT INT TERM
    [[ -z "$MONITOR_PID" ]] || kill -TERM "$MONITOR_PID" 2>/dev/null || true
    # roslaunch shuts down its own nodes and auto-started master on SIGINT.
    if [[ -n "$LAUNCH_PID" ]]; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" || true
    fi
    [[ -z "$MONITOR_PID" ]] || wait "$MONITOR_PID" 2>/dev/null || true
    rm -f "$RUN_DIR/ros-supervisor.pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '%s\n' "$$" > "$RUN_DIR/ros-supervisor.pid"

roslaunch --screen drone_sorting camera_delivery.launch > "$RUN_DIR/ros.log" 2>&1 &
LAUNCH_PID=$!

required_nodes=(/unity_endpoint /package_perception /camera_pickup /motion_controller)
ready=false
for ((attempt=0; attempt<60; attempt++)); do
    kill -0 "$LAUNCH_PID" 2>/dev/null || { tail -40 "$RUN_DIR/ros.log"; exit 1; }
    nodes="$(rosnode list 2>/dev/null || true)"
    ready=true
    for node in "${required_nodes[@]}"; do
        if ! [[ "$nodes" == *"$node"* ]]; then ready=false; break; fi
    done
    if [[ "$ready" == true ]]; then
        for node in "${required_nodes[@]}"; do
            rosnode ping -c 1 "$node" >/dev/null 2>&1 || { ready=false; break; }
        done
    fi
    [[ "$ready" == false ]] || break
    sleep 1
done
[[ "$ready" == true ]] || { echo '[FAIL] ROS nodes did not become ready.'; exit 1; }
# Ensure the endpoint is listening, rather than relying only on ROS master registration.
python3 - <<'PY'
import socket
with socket.create_connection(('127.0.0.1', 10000), timeout=5):
    pass
PY

python3 -u "$PROJECT_ROOT/ROS1/src/drone_sorting/scripts/delivery_demo_monitor.py" \
    --camera-guidance > "$RUN_DIR/monitor.log" 2>&1 &
MONITOR_PID=$!
printf 'ready\n' > "$RUN_DIR/ros-ready"

health_tick=0
while true; do
    if [[ -n "$MONITOR_PID" ]] && ! kill -0 "$MONITOR_PID" 2>/dev/null; then
        monitor_status=0
        wait "$MONITOR_PID" || monitor_status=$?
        MONITOR_PID=''
        if ((monitor_status == 0)); then
            echo '[COMPLETE] Monitor finished. ROS remains available; stop Play and use Ctrl+C to end this run.'
        else
            # Guidance's FAILED state commands a hold. Preserve connections for
            # inspection rather than turn the original failure into socket errors.
            echo '[FAIL] Mission monitor reported failure. ROS remains available; stop Play and use Ctrl+C to end this run.'
        fi
    fi
    if ((health_tick % 5 == 0)); then
        for node in "${required_nodes[@]}"; do
            rosnode ping -c 1 "$node" >/dev/null 2>&1 || {
                echo "[FAIL] Required ROS node stopped responding: $node"; exit 1;
            }
        done
    fi
    health_tick=$((health_tick + 1))
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo '[FAIL] ROS launch exited during the demo.'
        exit 1
    fi
    sleep 1
done
