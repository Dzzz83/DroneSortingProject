#!/usr/bin/env python3

import argparse
import json
import sys

import rospy
from std_msgs.msg import String

from drone_sorting.interfaces import ros_topics


class DeliveryDemoMonitor:
    """Displays and verifies the end-to-end delivery mission."""

    def __init__(self, camera_guidance=False):
        self.last_phase = None
        self.failure_reason = None
        self.last_perception = None
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

        if camera_guidance:
            self.subscribers.append(rospy.Subscriber(
                ros_topics.PICKUP_TRIAL_STATUS, String, self._on_guidance))
            self.subscribers.append(rospy.Subscriber(
                ros_topics.PACKAGE_CANDIDATES, String, self._on_perception))

    def _on_perception(self, message):
        try:
            observation = json.loads(message.data)
            state, reason = observation["state"], observation.get("reason")
            if not isinstance(state, str) or (reason is not None and not isinstance(reason, str)):
                raise ValueError("Invalid perception fields")
        except (ValueError, KeyError, TypeError, AttributeError):
            rospy.logwarn("Ignoring malformed package perception status")
            return
        status = (state, reason)
        if status != self.last_perception:
            self.last_perception = status
            print(f"[VISION]  {state}: {reason or 'no rejection reason'}", flush=True)

    def _on_guidance(self, message):
        try:
            status = json.loads(message.data)
            phase = status["phase"]
            reason = status.get("reason")
            if not isinstance(phase, str) or (reason is not None and not isinstance(reason, str)):
                raise ValueError("Invalid guidance fields")
        except (ValueError, KeyError, TypeError, AttributeError):
            rospy.logwarn("Ignoring malformed camera guidance status")
            return
        if phase == self.last_phase:
            return
        self.last_phase = phase
        print(f"[PHASE]   {phase}", flush=True)
        if phase == "FAILED":
            self.failure_reason = reason or "unspecified_guidance_failure"
            print(f"[FAIL]    {self.failure_reason}", flush=True)
            if self.last_perception is not None:
                print(f"[VISION]  Last result: {self.last_perception}", flush=True)
            rospy.signal_shutdown(self.failure_reason)

    def run(self):
        rospy.loginfo(
            "Delivery demo monitor started"
        )

        rospy.spin()
        return 1 if self.failure_reason else 0

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
            "[RESULT]  Pickup and drop acknowledgements confirmed",
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
    parser = argparse.ArgumentParser(description="Observe delivery state, actions, and optional camera phases.")
    parser.add_argument("--camera-guidance", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node(
        "delivery_demo_monitor"
    )

    monitor = DeliveryDemoMonitor(camera_guidance=args.camera_guidance)
    return monitor.run()


if __name__ == "__main__":
    sys.exit(main())
