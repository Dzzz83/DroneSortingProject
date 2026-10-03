#!/usr/bin/env bash

set -euo pipefail

if ! command -v distrobox >/dev/null 2>&1; then
    echo "ERROR: Run this script from the CachyOS host, not inside the container."
    exit 1
fi

distrobox enter --name ros-noetic -- bash -s <<'ROS_TEST'
set -eo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/drone_sorting_ros1_ws/devel/setup.bash"

STATE_TIMEOUT=240
FINAL_TIMEOUT=30

if ! rostopic list >/dev/null 2>&1; then
    echo "ERROR: ROS is not running."
    echo "Start the demo first."
    exit 1
fi

get_state() {
    rostopic echo -n 1 /drone/mission_state 2>/dev/null \
        | sed -n 's/data: "\(.*\)"/\1/p'
}

get_action() {
    rostopic echo -n 1 /drone/package_action 2>/dev/null \
        | sed -n 's/data: "\(.*\)"/\1/p'
}

wait_for_state() {
    expected="$1"
    timeout="$2"
    start_time=$(date +%s)

    echo "Waiting for $expected..."

    while true; do
        current_state=$(get_state || true)

        if [[ "$current_state" == "$expected" ]]; then
            echo "PASS: reached $expected"
            return
        fi

        elapsed=$(( $(date +%s) - start_time ))

        if (( elapsed >= timeout )); then
            echo "FAIL: timed out waiting for $expected"
            echo "Current state: ${current_state:-UNKNOWN}"
            exit 1
        fi

        sleep 1
    done
}

verify_action() {
    expected="$1"
    actual=$(get_action || true)

    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: expected package action $expected"
        echo "Actual: ${actual:-NONE}"
        exit 1
    fi

    echo "PASS: package action = $expected"
}

send_confirmation() {
    confirmation="$1"

    rostopic pub -1 \
        /drone/package_action_status \
        std_msgs/String \
        "data: $confirmation" \
        >/dev/null

    echo "Sent $confirmation"
}

echo "=============================================="
echo "       FULL MISSION AUTOMATED TEST"
echo "=============================================="
echo

initial_state=$(get_state || true)

echo "Initial state: ${initial_state:-UNKNOWN}"
echo

case "$initial_state" in
    GO_TO_PICKUP|WAIT_FOR_PICKUP)
        ;;
    *)
        echo "ERROR: Full mission test must start before pickup is completed."
        echo "Current state: ${initial_state:-UNKNOWN}"
        echo "Restart the demo and run the test again."
        exit 1
        ;;
esac

# --------------------------------------------------
# Pickup
# --------------------------------------------------

wait_for_state "WAIT_FOR_PICKUP" "$STATE_TIMEOUT"
verify_action "PICKUP"

send_confirmation "PICKUP_DONE"

echo

# --------------------------------------------------
# Drop-off
# --------------------------------------------------

wait_for_state "WAIT_FOR_DROP" "$STATE_TIMEOUT"
verify_action "DROP"

send_confirmation "DROP_DONE"

echo

# --------------------------------------------------
# Completion
# --------------------------------------------------

wait_for_state "COMPLETE" "$FINAL_TIMEOUT"

echo
echo "=============================================="
echo "              TEST PASSED"
echo "=============================================="
echo
echo "Mission sequence verified:"
echo
echo "GO_TO_PICKUP"
echo "  -> WAIT_FOR_PICKUP"
echo "  -> PICKUP_DONE"
echo "  -> GO_TO_DROPOFF"
echo "  -> WAIT_FOR_DROP"
echo "  -> DROP_DONE"
echo "  -> COMPLETE"
echo
ROS_TEST