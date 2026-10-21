import math

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import Float32, Float32MultiArray, String
from drone_sorting.interfaces import ros_topics


class MotionController:
    def __init__(self):
        self.speed = float(rospy.get_param("~speed", 2.0))
        if not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError("Motion speed must be finite and positive")
        self.maximum_speed = self.speed
        self.enable_dynamic_speed = rospy.get_param("~enable_dynamic_speed", False)
        self.update_rate = 20.0
        self.position_timeout = float(rospy.get_param("~position_timeout", 0.0))
        self.target_timeout = float(rospy.get_param("~target_timeout", 0.0))
        self.speed_timeout = float(rospy.get_param("~speed_timeout", 0.0))

        self.position_received = None
        self.target_received = None
        self.speed_received = None
        if self.enable_dynamic_speed:
            self.speed = 0.0

        self.enable_obstacle_avoidance = rospy.get_param(
            "~enable_obstacle_avoidance",
            True,
        )

        self.current_position = None
        self.target_position = None
        self.command_position = None
        self.obstacle_distances = None

        # Saved Unity dropoff pad position & phase tracking
        self.dropoff_pad_position = None
        self.is_dropoff_phase = False
        self.current_phase = ""

        # Safety distance & 4m minimum gap parameters
        self.safe_distance = 2.5  # Safety buffer (m) away from walls/shelves
        self.min_gap_width = 4.0  # Minimum required gap width (m) to pass through
        self.max_divergence_dist = 2.5  # Max distance drone can stray away from target before ditching wall

        # State memory for wall-following
        self.bypass_direction = None  # "LEFT" or "RIGHT"
        self.gap_start_position = None  # Point where current opening started
        self.closest_target_dist = float("inf")  # Closest distance recorded to target during wall-following
        self.current_yaw = 0.0  # Heading angle in radians

        self.status_log_interval = 0.5
        self.last_status_log_time = 0.0

        # Position publisher (Point) and Pose publisher with heading (PoseStamped)
        self.command_publisher = rospy.Publisher(
            ros_topics.COMMAND_POSITION,
            Point,
            queue_size=1 if self.enable_dynamic_speed else 10,
        )
        self.pose_publisher = rospy.Publisher(
            "/drone/command_pose",
            PoseStamped,
            queue_size=1 if self.enable_dynamic_speed else 10,
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
            "/dropoff_pad_pose",
            PoseStamped,
            self._dropoff_pad_callback,
        )

        rospy.Subscriber(
            "/delivery_demo/state",
            String,
            self._state_callback,
        )

        rospy.Subscriber(
            "/delivery_demo/phase",
            String,
            self._phase_callback,
        )

        rospy.Subscriber(
            ros_topics.OBSTACLE_DISTANCES,
            Float32MultiArray,
            self._obstacle_callback,
        )
        if self.enable_dynamic_speed:
            rospy.Subscriber(ros_topics.COMMAND_SPEED, Float32, self._speed_callback)

        rospy.loginfo(
            "Motion Controller started (obstacle avoidance: %s)",
            ("ENABLED" if self.enable_obstacle_avoidance else "DISABLED"),
        )

    def run(self):
        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self._update_command()
            rate.sleep()

    def _state_callback(self, msg):
        state_str = str(msg.data).upper()
        if "GO_TO_DROPOFF" in state_str or "DROPOFF" in state_str:
            self.is_dropoff_phase = True
        elif any(k in state_str for k in ("IDLE", "GO_TO_PICKUP", "WAIT_FOR_PICKUP")):
            self.is_dropoff_phase = False

    def _phase_callback(self, msg):
        self.current_phase = str(msg.data).upper()
        if any(k in self.current_phase for k in ("CARRY", "DROPOFF", "DESCENT_TO_DROPOFF")):
            self.is_dropoff_phase = True
        elif any(k in self.current_phase for k in ("OBSERVATION", "APPROACH", "DESCEND", "WAIT", "LIFT")):
            self.is_dropoff_phase = False

    def _current_position_callback(self, message):
        if not all(math.isfinite(value) for value in (message.x, message.y, message.z)):
            return
        self.position_received = rospy.get_time()
        self.current_position = Point(x=message.x, y=message.y, z=message.z)
        self.command_position = Point(x=message.x, y=message.y, z=message.z)

    def _target_position_callback(self, message):
        if not all(math.isfinite(value) for value in (message.x, message.y, message.z)):
            return

        self.target_received = rospy.get_time()
        incoming = Point(x=message.x, y=message.y, z=message.z)

        # Reset bypass memory on new target goal
        if self.target_position is None or (
            abs(self.target_position.x - incoming.x) > 0.1 or abs(self.target_position.y - incoming.y) > 0.1
        ):
            self.bypass_direction = None
            self.gap_start_position = None
            self.closest_target_dist = float("inf")

        if self.is_dropoff_phase and self.dropoff_pad_position is not None:
            self.target_position = Point(
                x=self.dropoff_pad_position.x,
                y=self.dropoff_pad_position.y,
                z=incoming.z,
            )
        else:
            self.target_position = incoming

    def _dropoff_pad_callback(self, message):
        pos = message.pose.position
        if not all(math.isfinite(value) for value in (pos.x, pos.y, pos.z)):
            return

        if self.dropoff_pad_position is None:
            rospy.loginfo(
                "Saved dropoff pad position -> x=%.2f, y=%.2f, z=%.2f",
                pos.x, pos.y, pos.z,
            )

        self.dropoff_pad_position = Point(x=pos.x, y=pos.y, z=pos.z)

    def _obstacle_callback(self, message):
        if len(message.data) < 7:
            return
        self.obstacle_distances = tuple(message.data)

    def _speed_callback(self, message):
        if not math.isfinite(message.data) or not 0 <= message.data <= self.maximum_speed:
            self.speed = 0.0
            self.speed_received = None
            return
        self.speed = message.data
        self.speed_received = rospy.get_time()

    def _parse_lateral_sensors(self):
        """Extracts 8 lateral directions or maps legacy 7-sensor array."""
        if not self.obstacle_distances:
            return {dir_key: -1.0 for dir_key in ("F", "FR", "R", "RR", "B", "RL", "L", "FL")}

        data = self.obstacle_distances

        if len(data) >= 8:
            return {
                "F": data[0],
                "FR": data[1],
                "R": data[2],
                "RR": data[3],
                "B": data[4],
                "RL": data[5],
                "L": data[6],
                "FL": data[7],
            }
        else:
            return {
                "F": data[0],
                "FR": -1.0,
                "R": data[4],
                "RR": -1.0,
                "B": -1.0,
                "RL": -1.0,
                "L": data[3],
                "FL": -1.0,
            }

    def _choose_avoidance_action(self):
        """Wall-following avoidance with RIGHT priority and Target-Distance progress check."""
        if not self.obstacle_distances:
            self.bypass_direction = None
            self.gap_start_position = None
            return "FORWARD"

        sensors = self._parse_lateral_sensors()
        front = sensors["F"]
        left = sensors["L"]
        right = sensors["R"]

        def is_blocked(dist, threshold):
            return 0.0 <= dist <= threshold

        # Current 2D distance to target
        dx_t = self.target_position.x - self.command_position.x
        dy_t = self.target_position.y - self.command_position.y
        curr_target_dist = math.sqrt(dx_t * dx_t + dy_t * dy_t)

        # 1. Active Wall-Following Mode
        if self.bypass_direction is not None:
            # Update closest distance record
            if curr_target_dist < self.closest_target_dist:
                self.closest_target_dist = curr_target_dist

            # DITCH WALL CHECK: If moving along wall takes drone further away from target
            if curr_target_dist > self.closest_target_dist + self.max_divergence_dist:
                rospy.loginfo("Ditching wall-following: Heading further away from target!")
                self.bypass_direction = None
                self.gap_start_position = None
                self.closest_target_dist = float("inf")
                return "FORWARD"

            # Check if front is clear
            if not (is_blocked(front, self.safe_distance) or is_blocked(sensors["FL"], self.safe_distance) or is_blocked(sensors["FR"], self.safe_distance)):
                if self.gap_start_position is None:
                    self.gap_start_position = Point(
                        x=self.command_position.x,
                        y=self.command_position.y,
                        z=self.command_position.z,
                    )
                    return self.bypass_direction

                dx_g = self.command_position.x - self.gap_start_position.x
                dy_g = self.command_position.y - self.gap_start_position.y
                gap_traveled = math.sqrt(dx_g * dx_g + dy_g * dy_g)

                if gap_traveled < self.min_gap_width:
                    return self.bypass_direction
                else:
                    # Verified 4m+ gap -> clear wall state and proceed straight
                    self.bypass_direction = None
                    self.gap_start_position = None
                    self.closest_target_dist = float("inf")
                    return "FORWARD"
            else:
                self.gap_start_position = None
                return self.bypass_direction

        # 2. Obstacle Encountered -> DEFAULT TO RIGHT
        if is_blocked(front, self.safe_distance) or is_blocked(sensors["FL"], self.safe_distance) or is_blocked(sensors["FR"], self.safe_distance):
            right_blocked = is_blocked(right, self.safe_distance) or is_blocked(sensors["FR"], self.safe_distance)

            # Turn RIGHT by default unless right side is blocked
            if not right_blocked:
                self.bypass_direction = "RIGHT"
            else:
                self.bypass_direction = "LEFT"

            self.closest_target_dist = curr_target_dist
            self.gap_start_position = None
            return self.bypass_direction

        return "FORWARD"

    def _update_command(self):
        if self.command_position is None or self.target_position is None:
            return

        now = rospy.get_time()
        if (self.position_received is None or now - self.position_received > 2.0 or
                self.target_received is None or now - self.target_received > 2.0):
            return

        dx = self.target_position.x - self.command_position.x
        dy = self.target_position.y - self.command_position.y
        dz = self.target_position.z - self.command_position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= 0.01:
            self._publish_commands(self.command_position, self.current_yaw)
            return

        target_direction = (dx / distance, dy / distance, dz / distance)

        # Bypass avoidance during vertical ground phases
        is_vertical_phase = any(
            p in self.current_phase for p in ("DESCEND", "LIFT", "WAIT", "APPROACH", "DESCENT_TO_DROPOFF")
        )

        if self.enable_obstacle_avoidance and not is_vertical_phase:
            decision = self._choose_avoidance_action()
        else:
            decision = "FORWARD"

        movement = self._movement_direction(decision, target_direction)

        if movement is None:
            self._publish_commands(self.command_position, self.current_yaw)
            self._log_status(decision)
            return

        step = self.speed / self.update_rate
        if decision == "FORWARD":
            step = min(step, distance)

        move_x, move_y, move_z = movement
        self.command_position.x += move_x * step
        self.command_position.y += move_y * step
        self.command_position.z += move_z * step

        # Update yaw orientation to face active movement direction
        if math.sqrt(move_x * move_x + move_y * move_y) > 0.001:
            self.current_yaw = math.atan2(move_y, move_x)

        self._publish_commands(self.command_position, self.current_yaw)
        self._log_status(decision)

    def _movement_direction(self, decision, target_direction):
        if decision == "FORWARD":
            return target_direction

        tx, ty = target_direction[0], target_direction[1]
        length = math.sqrt(tx * tx + ty * ty)
        if length < 0.001:
            return target_direction

        tx, ty = tx / length, ty / length

        # Base tangent vector
        if decision == "RIGHT":
            base_x, base_y = ty, -tx
        else: # LEFT
            base_x, base_y = -ty, tx

        # Active Potential Field Repulsion Overlay (Prevents Wall Penetration)
        sensors = self._parse_lateral_sensors()
        repulsion_x, repulsion_y = 0.0, 0.0

        # Sensor directional vectors relative to target vector
        sensor_directions = {
            "F": (-tx, -ty),
            "FR": (-0.707 * tx - 0.707 * ty, -0.707 * ty + 0.707 * tx),
            "R": (-ty, tx),
            "L": (ty, -tx),
            "FL": (-0.707 * tx + 0.707 * ty, -0.707 * ty - 0.707 * tx),
        }

        for key, (dir_x, dir_y) in sensor_directions.items():
            dist = sensors[key]
            if 0.0 <= dist < self.safe_distance:
                push = (self.safe_distance - dist) / self.safe_distance
                repulsion_x += dir_x * push * 1.5
                repulsion_y += dir_y * push * 1.5

        move_x = base_x + repulsion_x
        move_y = base_y + repulsion_y

        norm = math.sqrt(move_x * move_x + move_y * move_y)
        if norm > 0.001:
            return (move_x / norm, move_y / norm, 0.0)

        return target_direction

    def _publish_commands(self, point_msg, yaw_rad):
        """Publishes position point and orientation pose stamped with yaw quaternion."""
        self.command_publisher.publish(point_msg)

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position = point_msg
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        self.pose_publisher.publish(pose_msg)

    def _log_status(self, decision):
        now = rospy.get_time()
        if now - self.last_status_log_time < self.status_log_interval:
            return
        self.last_status_log_time = now

        if self.current_position is None or self.target_position is None or self.command_position is None:
            return

        yaw_deg = math.degrees(self.current_yaw)
        status = (
            f"\n--- DRONE MOTION ---"
            f"\nTarget : x={self.target_position.x:.2f}, y={self.target_position.y:.2f}, z={self.target_position.z:.2f}"
            f"\nPhase  : {self.current_phase if self.current_phase else 'N/A'}"
            f"\nAction : {decision} (Latched: {self.bypass_direction})"
            f"\nHdng   : {yaw_deg:.1f}°"
            f"\nPos    : x={self.command_position.x:.2f}, y={self.command_position.y:.2f}, z={self.command_position.z:.2f}\n"
        )
        rospy.loginfo(status)