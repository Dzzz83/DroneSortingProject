import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from drone_sorting.interfaces import ros_topics

from drone_sorting.planning.obstacle_avoidance import (
    ObstacleAvoidancePlanner,
)


class MotionController:
    def __init__(self):
        self.speed = 2.0
        self.update_rate = 20.0

        # Allows obstacle avoidance to be disabled
        # for delivery demonstrations.
        self.enable_obstacle_avoidance = rospy.get_param(
            "~enable_obstacle_avoidance",
            True,
        )

        self.current_position = None
        self.target_position = None
        self.command_position = None
        self.obstacle_distances = None

        self.avoidance_planner = ObstacleAvoidancePlanner()

        # Control runs at 20 Hz.
        # Terminal status is printed only twice per second.
        self.status_log_interval = 0.5
        self.last_status_log_time = 0.0

        self.command_publisher = rospy.Publisher(
            ros_topics.COMMAND_POSITION,
            Point,
            queue_size=10,
        )

        rospy.Subscriber(
            ros_topics.CURRENT_POSITION,
            Point,
            self._current_position_callback,
        )

        rospy.Subscriber(
            ros_topics.TARGET_POSITION,
            Point,
            self._target_position_callback,
        )

        rospy.Subscriber(
            ros_topics.OBSTACLE_DISTANCES,
            Float32MultiArray,
            self._obstacle_callback,
        )

        rospy.loginfo(
            "Motion Controller started "
            "(obstacle avoidance: %s)",
            (
                "ENABLED"
                if self.enable_obstacle_avoidance
                else "DISABLED"
            ),
        )

    def run(self):
        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self._update_command()
            rate.sleep()

    def _current_position_callback(
        self,
        message,
    ):
        self.current_position = Point(
            x=message.x,
            y=message.y,
            z=message.z,
        )

        # Synchronize ROS command position
        # with the actual Unity position.
        self.command_position = Point(
            x=message.x,
            y=message.y,
            z=message.z,
        )

    def _target_position_callback(
        self,
        message,
    ):
        self.target_position = Point(
            x=message.x,
            y=message.y,
            z=message.z,
        )

        # New mission target -> reset the
        # stateful avoidance planner.
        self.avoidance_planner.reset()

        rospy.loginfo(
            "Received target -> "
            "x=%.2f y=%.2f z=%.2f",
            message.x,
            message.y,
            message.z,
        )

    def _obstacle_callback(
        self,
        message,
    ):
        if len(message.data) < 7:
            return

        self.obstacle_distances = tuple(
            message.data[:7]
        )

    def _update_command(self):
        if self.command_position is None:
            return

        if self.target_position is None:
            return

        # Difference between current commanded
        # position and mission target.
        dx = (
            self.target_position.x
            - self.command_position.x
        )

        dy = (
            self.target_position.y
            - self.command_position.y
        )

        dz = (
            self.target_position.z
            - self.command_position.z
        )

        distance = math.sqrt(
            dx * dx
            + dy * dy
            + dz * dz
        )

        # Already effectively at the target.
        if distance <= 0.01:
            self.command_publisher.publish(
                self.command_position
            )
            return

        # Normalized direction toward the
        # mission target.
        target_direction = (
            dx / distance,
            dy / distance,
            dz / distance,
        )

        # Use the avoidance planner when enabled.
        # Otherwise, move directly toward the target.
        if self.enable_obstacle_avoidance:
            decision = (
                self.avoidance_planner.choose_direction(
                    self.obstacle_distances,
                    self.command_position,
                )
            )
        else:
            decision = (
                self.avoidance_planner.FORWARD
            )

        movement = self._movement_direction(
            decision,
            target_direction,
        )

        # STOP or invalid movement.
        if movement is None:
            self.command_publisher.publish(
                self.command_position
            )

            self._log_status(decision)
            return

        # 2 m/s / 20 Hz =
        # approximately 0.1 m per update.
        step = (
            self.speed
            / self.update_rate
        )

        move_x, move_y, move_z = movement

        self.command_position.x += (
            move_x * step
        )

        self.command_position.y += (
            move_y * step
        )

        self.command_position.z += (
            move_z * step
        )

        # Send calculated position to Unity.
        self.command_publisher.publish(
            self.command_position
        )

        self._log_status(decision)

    def _movement_direction(
        self,
        decision,
        target_direction,
    ):
        # Move normally toward the full XYZ target.
        if (
            decision
            == self.avoidance_planner.FORWARD
        ):
            return target_direction

        # Move vertically upward.
        if (
            decision
            == self.avoidance_planner.UP
        ):
            return (
                0.0,
                0.0,
                1.0,
            )

        # Move vertically downward.
        if (
            decision
            == self.avoidance_planner.DOWN
        ):
            return (
                0.0,
                0.0,
                -1.0,
            )

        # Move horizontally toward the target
        # while preserving current altitude.
        if (
            decision
            == self.avoidance_planner.FORWARD_LEVEL
        ):
            return self._horizontal_forward(
                target_direction
            )

        horizontal_forward = (
            self._horizontal_forward(
                target_direction
            )
        )

        if horizontal_forward is None:
            return None

        (
            forward_x,
            forward_y,
            _,
        ) = horizontal_forward

        # Perpendicular movement for sidestepping.
        if (
            decision
            == self.avoidance_planner.LEFT
        ):
            return (
                -forward_y,
                forward_x,
                0.0,
            )

        if (
            decision
            == self.avoidance_planner.RIGHT
        ):
            return (
                forward_y,
                -forward_x,
                0.0,
            )

        # STOP or unknown command.
        return None

    @staticmethod
    def _horizontal_forward(
        target_direction,
    ):
        x = target_direction[0]
        y = target_direction[1]

        length = math.sqrt(
            x * x
            + y * y
        )

        if length < 0.001:
            return None

        return (
            x / length,
            y / length,
            0.0,
        )

    @staticmethod
    def _format_sensor(value):
        # Unity uses -1 to mean that
        # no obstacle was detected.
        if value < 0.0:
            return "CLEAR"

        return f"{value:.2f}"

    def _log_status(
        self,
        decision,
    ):
        now = rospy.get_time()

        if (
            now
            - self.last_status_log_time
            < self.status_log_interval
        ):
            return

        self.last_status_log_time = now

        if self.current_position is None:
            return

        if self.target_position is None:
            return

        if self.command_position is None:
            return

        if self.obstacle_distances is None:
            sensor_values = [
                "WAITING",
                "WAITING",
                "WAITING",
                "WAITING",
                "WAITING",
                "WAITING",
                "WAITING",
            ]
        else:
            sensor_values = [
                self._format_sensor(value)
                for value
                in self.obstacle_distances
            ]

        (
            front,
            upper_front,
            lower_front,
            left,
            right,
            up,
            down,
        ) = sensor_values

        if self.enable_obstacle_avoidance:
            planner_mode = getattr(
                self.avoidance_planner,
                "mode",
                "UNKNOWN",
            )
        else:
            planner_mode = "DISABLED"

        action = str(decision).upper()

        status = (
            "\n"
            "==================== DRONE CONTROL ====================\n"
            "\n"
            "MISSION\n"
            "Target        : "
            f"x={self.target_position.x:6.2f}  "
            f"y={self.target_position.y:6.2f}  "
            f"z={self.target_position.z:6.2f}\n"
            "\n"
            "UNITY -> ROS\n"
            "Current       : "
            f"x={self.current_position.x:6.2f}  "
            f"y={self.current_position.y:6.2f}  "
            f"z={self.current_position.z:6.2f}\n"
            "\n"
            "Sensors (m)\n"
            f"  Front       : {front}\n"
            f"  Upper Front : {upper_front}\n"
            f"  Lower Front : {lower_front}\n"
            f"  Left        : {left}\n"
            f"  Right       : {right}\n"
            f"  Up          : {up}\n"
            f"  Down        : {down}\n"
            "\n"
            "ROS DECISION\n"
            f"Planner mode  : {planner_mode}\n"
            f"Action        : {action}\n"
            "\n"
            "ROS -> UNITY\n"
            "Command       : "
            f"x={self.command_position.x:6.2f}  "
            f"y={self.command_position.y:6.2f}  "
            f"z={self.command_position.z:6.2f}\n"
            "\n"
            "=======================================================\n"
        )

        rospy.loginfo(status)