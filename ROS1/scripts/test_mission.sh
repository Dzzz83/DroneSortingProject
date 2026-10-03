#!/usr/bin/env bash

set -e

ACTION="${1:-}"

run_ros() {
    distrobox enter --name ros-noetic -- bash -lc "
        source /opt/ros/noetic/setup.bash
        source \"\$HOME/drone_sorting_ros1_ws/devel/setup.bash\"
        $1
    "
}

check_ros() {
    if ! run_ros "rostopic list >/dev/null 2>&1"; then
        echo "ROS is not running."
        echo "Start it with:"
        echo "  ./ROS1/scripts/start_demo.sh"
        exit 1
    fi
}

case "$ACTION" in
    pickup)
        check_ros
        echo "Sending PICKUP_DONE..."
        run_ros \
            "rostopic pub -1 /drone/package_action_status std_msgs/String 'data: PICKUP_DONE'"
        ;;

    drop)
        check_ros
        echo "Sending DROP_DONE..."
        run_ros \
            "rostopic pub -1 /drone/package_action_status std_msgs/String 'data: DROP_DONE'"
        ;;

    action)
        check_ros
        echo "Current package action:"
        run_ros "rostopic echo -n 1 /drone/package_action"
        ;;

    target)
        check_ros
        echo "Current mission target:"
        run_ros "rostopic echo -n 1 /drone/target_position"
        ;;

    state)
        check_ros
        echo "Current mission state:"
        run_ros "rostopic echo -n 1 /drone/mission_state"
        ;;

    status)
        check_ros

        run_ros '
            STATE=$(rostopic echo -n 1 /drone/mission_state \
                | sed -n "s/data: \"\(.*\)\"/\1/p")

            TARGET=$(rostopic echo -n 1 /drone/target_position \
                | grep -E "^(x|y|z):")

            echo "================ MISSION STATUS ================"
            echo
            echo "State:"
            echo "  $STATE"
            echo
            echo "Target:"
            echo "$TARGET" | sed "s/^/  /"
            echo
            echo "Package action:"

            case "$STATE" in
                WAIT_FOR_PICKUP)
                    echo "  PICKUP"
                    ;;
                WAIT_FOR_DROP)
                    echo "  DROP"
                    ;;
                *)
                    echo "  None"
                    ;;
            esac

            echo
            echo "================================================"
        '
        ;;

    *)
        echo "Usage:"
        echo "  ./ROS1/scripts/test_mission.sh status"
        echo "  ./ROS1/scripts/test_mission.sh state"
        echo "  ./ROS1/scripts/test_mission.sh action"
        echo "  ./ROS1/scripts/test_mission.sh target"
        echo "  ./ROS1/scripts/test_mission.sh pickup"
        echo "  ./ROS1/scripts/test_mission.sh drop"
        exit 1
        ;;
esac
