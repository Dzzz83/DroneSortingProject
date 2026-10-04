#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from drone_sorting.interfaces import ros_topics


class DeliveryDemoMonitor:
    """Displays and verifies the end-to-end delivery mission."""

    def __init__(self):
        self.last_state = None
        self.last_action = None
        self.last_status = None

        self.pickup_confirmed = False
        self.drop_confirmed = False
        self.mission_complete = False

        # Keep references to every subscriber for the
        # lifetime of this node.
        self.subscribers = [
            rospy.Subscriber(
                ros_topics.MISSION_STATE,
                String,
                self._on_state,
            ),
            rospy.Subscriber(
                ros_topics.PACKAGE_ACTION,
                String,
                self._on_action,
            ),
            rospy.Subscriber(
                ros_topics.PACKAGE_ACTION_STATUS,
                String,
                self._on_status,
            ),
        ]

    def run(self):
        rospy.loginfo(
            "Delivery demo monitor started"
        )

        rospy.spin()

    def _on_state(self, message):
        state = message.data.strip().upper()

        if not state:
            return

        if state == self.last_state:
            return

        self.last_state = state

        print(
            f"[STATE]   {state}",
            flush=True,
        )

        if state == "COMPLETE":
            self.mission_complete = True
            self._check_result()

    def _on_action(self, message):
        action = message.data.strip().upper()

        if not action:
            # M3 intentionally clears the latched
            # action after receiving confirmation.
            self.last_action = None
            return

        if action == self.last_action:
            return

        self.last_action = action

        if action == "PICKUP":
            print(
                "[ACTION]  PICKUP requested",
                flush=True,
            )

        elif action == "DROP":
            print(
                "[ACTION]  DROP requested",
                flush=True,
            )

        else:
            print(
                f"[ACTION]  Unknown action: {action}",
                flush=True,
            )

    def _on_status(self, message):
        status = message.data.strip().upper()

        if not status:
            return

        if status == self.last_status:
            return

        self.last_status = status

        if status == "PICKUP_DONE":
            self.pickup_confirmed = True

            print(
                "[PICKUP]  confirmed",
                flush=True,
            )

        elif status == "DROP_DONE":
            self.drop_confirmed = True

            print(
                "[DROP]    confirmed",
                flush=True,
            )

        self._check_result()

    def _check_result(self):
        if not self.mission_complete:
            return

        if (
            not self.pickup_confirmed
            or not self.drop_confirmed
        ):
            # COMPLETE and the final status messages
            # may arrive almost simultaneously.
            # Wait for the remaining callback instead
            # of immediately reporting a false failure.
            return

        print(
            "",
            flush=True,
        )
        print(
            "========================================",
            flush=True,
        )
        print(
            "[MISSION] COMPLETE",
            flush=True,
        )
        print(
            "[RESULT]  DELIVERY END-TO-END TEST PASSED",
            flush=True,
        )
        print(
            "========================================",
            flush=True,
        )

        rospy.signal_shutdown(
            "Delivery mission completed successfully"
        )


def main():
    rospy.init_node(
        "delivery_demo_monitor"
    )

    monitor = DeliveryDemoMonitor()
    monitor.run()


if __name__ == "__main__":
    main()