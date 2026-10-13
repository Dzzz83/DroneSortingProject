#!/usr/bin/env python3
"""Record pickup events without publishing commands or reading scene ground truth."""

import argparse
import json
import os
from pathlib import Path
import threading
import time

import rospy
from std_msgs.msg import Float32, String

from drone_sorting.interfaces import ros_topics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    args.output.mkdir(parents=True, exist_ok=True)
    # Each run has its own history; never append a new trial to old evidence.
    history = (args.output / "ros-events.jsonl").open("x")
    lock = threading.Lock()
    rospy.init_node("camera_pickup_recorder")

    def record(kind, message):
        if kind == "guidance":
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("Expected a guidance object")
            except (ValueError, TypeError) as error:
                rospy.logwarn("Invalid pickup status: %s", error)
                return
        else:
            payload = message.data
        event = {"kind": kind, "received": time.time(), "payload": payload}
        with lock:
            history.write(json.dumps(event, allow_nan=False) + "\n")
            history.flush()
            if kind == "guidance":
                temporary = args.output / "guidance.tmp"
                temporary.write_text(json.dumps(payload, allow_nan=False))
                os.replace(temporary, args.output / "latest-guidance.json")

    subscriptions = [
        (ros_topics.PICKUP_TRIAL_STATUS, String, "guidance"),
        (ros_topics.PACKAGE_ACTION, String, "action"),
        (ros_topics.PACKAGE_ACTION_STATUS, String, "acknowledgement"),
        (ros_topics.COMMAND_SPEED, Float32, "speed"),
    ]
    for topic, message_type, kind in subscriptions:
        rospy.Subscriber(topic, message_type, lambda message, kind=kind: record(kind, message), queue_size=100)
    try:
        rospy.spin()
    finally:
        history.close()


if __name__ == "__main__":
    main()
