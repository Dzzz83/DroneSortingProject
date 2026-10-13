#!/usr/bin/env python3
"""Record read-only perception output for the stationary Unity evaluator."""

import argparse
import json
import os

import rospy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    os.makedirs(args.output, exist_ok=True)
    rospy.init_node("package_perception_recorder")

    def record(message):
        payload = json.loads(message.data)
        with open(os.path.join(args.output, "observations.jsonl"), "a") as history:
            history.write(json.dumps(payload, allow_nan=False) + "\n")
        temporary = os.path.join(args.output, "latest.tmp")
        with open(temporary, "w") as latest:
            json.dump(payload, latest, allow_nan=False)
        os.replace(temporary, os.path.join(args.output, "latest.json"))

    rospy.Subscriber("/drone/package_candidates", String, record, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
