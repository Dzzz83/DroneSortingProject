"""Apply mission position targets at 20 Hz with warehouse obstacle avoidance."""

import json
import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Float32MultiArray, String
from drone_sorting.interfaces import ros_topics
from drone_sorting.control.approved_trajectory import ApprovedTrajectoryStream, InsufficientTrajectoryTime


class MotionController:
    def __init__(self):
        self.speed = float(rospy.get_param("~speed", 2.0))
        if not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError("Motion speed must be finite and positive")
        self.maximum_speed = self.speed
        self.enable_dynamic_speed = rospy.get_param("~enable_dynamic_speed", False)
        self.update_rate = float(rospy.get_param("~update_rate", 20.0))
        if not math.isfinite(self.update_rate) or not 1 <= self.update_rate <= 100:
            raise ValueError("update_rate must be between 1 and 100 Hz")
        # Bound outstanding movement while waiting for Unity's slower position feedback.
        self.maximum_command_lead = float(rospy.get_param("~maximum_command_lead", 0.25))
        if not math.isfinite(self.maximum_command_lead) or self.maximum_command_lead <= 0:
            raise ValueError("maximum_command_lead must be finite and positive")
        self.position_timeout = float(rospy.get_param("~position_timeout", 0.0))
        self.target_timeout = float(rospy.get_param("~target_timeout", 0.0))
        self.speed_timeout = float(rospy.get_param("~speed_timeout", 0.0))

        for name in ("position_timeout", "target_timeout", "speed_timeout"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + " must be finite and nonnegative")

        self.position_received = None
        self.position_progress_at = None
        self.target_received = None
        self.speed_received = None
        if self.enable_dynamic_speed:
            self.speed = 0.0

        self.enable_obstacle_avoidance = rospy.get_param(
            "~enable_obstacle_avoidance",
            True,
        )
        mode = rospy.get_param("~avoidance_mode", None)
        # Preserve historical behavior unless a mode is explicitly selected.
        if mode is None:
            mode = "heuristic" if self.enable_obstacle_avoidance else "none"
        if mode not in ("none", "heuristic", "depth"):
            raise ValueError("avoidance_mode must be none, heuristic or depth")
        self.avoidance_mode = mode
        if mode == "depth" and (self.position_timeout <= 0 or self.target_timeout <= 0
                or (self.enable_dynamic_speed and self.speed_timeout <= 0)):
            raise ValueError("Depth mode requires nonzero position, target and active speed timeouts")
        self.depth_navigation = None
        self.depth_reset_seen = None
        self.use_approved_trajectories = (mode == "depth" and bool(
            rospy.get_param("~depth_smooth_trajectory_enabled", False)))
        self.trajectory_stream = ApprovedTrajectoryStream() if self.use_approved_trajectories else None
        self.trajectory_publisher = (rospy.Publisher(
            ros_topics.APPROVED_TRAJECTORY, String, queue_size=20)
            if self.use_approved_trajectories else None)

        self.trajectory_status = None
        self.trajectory_status_received = None
        self.trajectory_stop_sequence = None
        self.trajectory_needs_sync = True
        self.trajectory_first_move = None
        if self.use_approved_trajectories:
            rospy.Subscriber(ros_topics.TRAJECTORY_STATUS, String, self._trajectory_status_callback)

        self.current_position = None
        self.target_position = None
        self.command_position = None
        self.obstacle_distances = None

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

        # Absolute position commands in ROS warehouse coordinates.
        self.command_publisher = rospy.Publisher(
            ros_topics.COMMAND_POSITION,
            Point,
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

        rospy.Subscriber(ros_topics.PICKUP_TRIAL_STATUS, String, self._phase_callback)

        rospy.Subscriber(
            ros_topics.OBSTACLE_DISTANCES,
            Float32MultiArray,
            self._obstacle_callback,
        )
        if self.enable_dynamic_speed:
            rospy.Subscriber(ros_topics.COMMAND_SPEED, Float32, self._speed_callback)
        if self.avoidance_mode == "depth":
            from drone_sorting.control.depth_navigation import DepthNavigation
            self.depth_navigation = DepthNavigation()

        rospy.loginfo("Motion Controller started (avoidance mode: %s)", self.avoidance_mode)

    def run(self):
        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self._update_command()
            rate.sleep()

    def _phase_callback(self, message):
        try:
            phase = json.loads(message.data)["phase"]
            if not isinstance(phase, str):
                return
        except (ValueError, KeyError, TypeError):
            return
        self.current_phase = phase

    def _current_position_callback(self, message):
        if not all(math.isfinite(value) for value in (message.x, message.y, message.z)):
            return
        received_at = rospy.get_time()
        if (self.current_position is None or math.dist(
                (message.x, message.y, message.z),
                (self.current_position.x, self.current_position.y, self.current_position.z)) > .002):
            self.position_progress_at = received_at
        self.position_received = received_at
        self.current_position = Point(x=message.x, y=message.y, z=message.z)
        # Feedback reports an earlier applied command. Replacing the command here
        # would replay old positions and move the drone backward on every update.
        if self.command_position is None:
            self.command_position = Point(x=message.x, y=message.y, z=message.z)

    def _trajectory_status_callback(self, message):
        try:
            status = json.loads(message.data)
            for key in ("reset_sequence", "sequence"):
                if type(status[key]) is not int or status[key] < 0:
                    return
            if type(status["stopped"]) is not bool:
                return
            position = status["position"]
            if not all(type(position[k]) in (int, float) and math.isfinite(position[k])
                       for k in ("x", "y", "z")):
                return
            stamp = status["stamp_ms"] / 1000.0
            if not 0 <= rospy.get_time() - stamp <= .5:
                return
        except (ValueError, KeyError, TypeError, OverflowError):
            return
        self.trajectory_status = status
        self.trajectory_status_received = rospy.get_time()

    def _trajectory_ready(self, now):
        """Restart only from Unity's acknowledged, stationary applied position."""
        if self.trajectory_stream is None:
            return True
        if self.depth_navigation.reset_sequence is None:
            return False
        if self.trajectory_needs_sync and self.trajectory_stop_sequence is None:
            self._stop_trajectory(force=True)
        status = self.trajectory_status
        if (status is None or status["reset_sequence"] != self.depth_navigation.reset_sequence
                or self._feedback_expired(self.trajectory_status_received, .5, now)
                or not 0 <= now - status["stamp_ms"] / 1000.0 <= .5):
            self._stop_trajectory()
            return False
        if self.trajectory_stop_sequence is not None:
            if status["stopped"] and status["sequence"] == self.trajectory_stop_sequence:
                position = status["position"]
                self.command_position = Point(position["x"], position["y"], position["z"])
                self.trajectory_stop_sequence = None
                self.trajectory_needs_sync = False
                self.trajectory_first_move = None
            return False
        if (self.trajectory_stream.in_motion and status["stopped"]
                and self.trajectory_first_move is not None
                and status["sequence"] >= self.trajectory_first_move):
            self._stop_trajectory()
            return False
        return True

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

        self.target_position = incoming

    def _obstacle_callback(self, message):
        if len(message.data) < 7:
            return
        self.obstacle_distances = tuple(message.data[:7])

    def _speed_callback(self, message):
        if not math.isfinite(message.data) or not 0 <= message.data <= self.maximum_speed:
            self.speed = 0.0
            self.speed_received = None
            return
        self.speed = message.data
        self.speed_received = rospy.get_time()

    def _parse_lateral_sensors(self):
        # Unity publishes front, upper-front, lower-front, left, right, up, down.
        sensors = {name: -1.0 for name in ("F", "L", "R")}
        if self.obstacle_distances:
            sensors.update(F=self.obstacle_distances[0], L=self.obstacle_distances[3],
                           R=self.obstacle_distances[4])
        return sensors

    def _choose_avoidance_action(self):
        """Wall-following avoidance with RIGHT priority and Target-Distance progress check."""
        if not self.obstacle_distances:
            self.bypass_direction = None
            self.gap_start_position = None
            return "FORWARD"

        sensors = self._parse_lateral_sensors()
        front = sensors["F"]
        right = sensors["R"]

        def is_blocked(dist, threshold):
            return 0.0 <= dist <= threshold

        # Current 2D distance to target
        dx_t = self.target_position.x - self.command_position.x
        dy_t = self.target_position.y - self.command_position.y
        curr_target_dist = math.sqrt(dx_t * dx_t + dy_t * dy_t)

        # Continue a bypass until a usable opening or excessive divergence.
        if self.bypass_direction is not None:
            # Update closest distance record
            if curr_target_dist < self.closest_target_dist:
                self.closest_target_dist = curr_target_dist

            # Leave the wall when following it stops making progress.
            if curr_target_dist > self.closest_target_dist + self.max_divergence_dist:
                rospy.loginfo("Ditching wall-following: Heading further away from target!")
                self.bypass_direction = None
                self.gap_start_position = None
                self.closest_target_dist = float("inf")
                return "FORWARD"

            # Check if front is clear
            if not (is_blocked(front, self.safe_distance)):
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

                self.bypass_direction = None
                self.gap_start_position = None
                self.closest_target_dist = float("inf")
                return "FORWARD"

            self.gap_start_position = None
            return self.bypass_direction

        # Prefer the right side when starting a bypass.
        if is_blocked(front, self.safe_distance):
            right_blocked = is_blocked(right, self.safe_distance)

            # Turn RIGHT by default unless right side is blocked
            if not right_blocked:
                self.bypass_direction = "RIGHT"
            else:
                self.bypass_direction = "LEFT"

            self.closest_target_dist = curr_target_dist
            self.gap_start_position = None
            return self.bypass_direction

        return "FORWARD"

    @staticmethod
    def _feedback_expired(received, timeout, now):
        return timeout > 0 and (received is None or not 0 <= now - received <= timeout)

    def _within_feedback_limit(self, point):
        observed = self.current_position
        lead = math.dist((point.x, point.y, point.z), (observed.x, observed.y, observed.z))
        return lead <= self.maximum_command_lead + 1e-9

    def _update_command(self):
        # Physical trial reset invalidates every old path and command.
        if self.depth_navigation is not None:
            sequence = self.depth_navigation.reset_sequence
            if sequence is not None and sequence != self.depth_reset_seen:
                if self.depth_reset_seen is not None:
                    self.command_position = None
                    self.current_position = None
                    self.position_received = None
                    self.position_progress_at = None
                    self.target_position = None
                    self.target_received = None
                self.depth_reset_seen = sequence
                if self.trajectory_stream is not None:
                    self.trajectory_stream.reset(sequence)
                    self.trajectory_needs_sync = True
                    self.trajectory_stop_sequence = None
                    self.trajectory_first_move = None
                    self.trajectory_status = None
        now = rospy.get_time()
        if not self._trajectory_ready(now):
            return
        if self.command_position is None or self.target_position is None or self.current_position is None:
            return
        if (self._feedback_expired(self.position_received, self.position_timeout, now)
                or self._feedback_expired(self.target_received, self.target_timeout, now)):
            self._stop_trajectory()
            return
        if self.enable_dynamic_speed and (self.speed_received is None
                or self._feedback_expired(self.speed_received, self.speed_timeout, now)):
            self._stop_trajectory()
            return
        # Unity holds its last applied position. Do not publish lagging feedback
        # as a hold command, which could move the drone backward.
        if self.speed == 0:
            self._stop_trajectory()
            return
        if not self._within_feedback_limit(self.command_position):
            if (self.trajectory_stream is not None and self.position_progress_at is not None
                    and now - self.position_progress_at > .25):
                self._stop_trajectory()
            self._log_status("WAIT_FOR_FEEDBACK")
            return

        if self.avoidance_mode == "depth":
            self._update_depth_command(now)
            return

        dx = self.target_position.x - self.command_position.x
        dy = self.target_position.y - self.command_position.y
        dz = self.target_position.z - self.command_position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= 0.01:
            self._publish_commands(self.command_position)
            return

        target_direction = (dx / distance, dy / distance, dz / distance)

        # Bypass avoidance during vertical ground phases
        is_vertical_phase = any(
            p in self.current_phase for p in ("DESCEND", "LIFT", "WAIT", "APPROACH", "REFINE", "LOWER")
        )

        if self.avoidance_mode == "heuristic" and not is_vertical_phase:
            decision = self._choose_avoidance_action()
        else:
            decision = "FORWARD"

        movement = self._movement_direction(decision, target_direction)

        if movement is None:
            self._publish_commands(self.command_position)
            self._log_status(decision)
            return

        step = self.speed / self.update_rate
        if decision == "FORWARD":
            step = min(step, distance)

        move_x, move_y, move_z = movement
        next_position = Point(self.command_position.x + move_x * step,
                              self.command_position.y + move_y * step,
                              self.command_position.z + move_z * step)
        if not self._within_feedback_limit(next_position):
            self._log_status("WAIT_FOR_FEEDBACK")
            return
        self.command_position = next_position

        # Update yaw orientation to face active movement direction
        if math.sqrt(move_x * move_x + move_y * move_y) > 0.001:
            self.current_yaw = math.atan2(move_y, move_x)

        self._publish_commands(self.command_position)
        self._log_status(decision)

    def _update_depth_command(self, now):
        """Advance only along a fresh, epoch-matched approved path."""
        command = self.command_position
        observed = self.current_position
        target = self.target_position
        result = self.depth_navigation.tick(
            (command.x, command.y, command.z),
            (observed.x, observed.y, observed.z),
            (target.x, target.y, target.z), now, self.speed/self.update_rate)
        if result.state != "MOVE":
            # The command frontier can reach a waypoint before Unity applies its
            # queued segments. Let those existing approvals finish; stopping here
            # cancels the last part of every local leg and forces a restart.
            if result.reason != "local_goal_reached":
                self._stop_trajectory()
            self._log_status("DEPTH_HOLD: " + result.reason)
            return  # Unity stops pending segments, not just new publication.
        approved = Point(*result.next_position)
        if not self._within_feedback_limit(approved):
            if (self.position_progress_at is not None and now - self.position_progress_at > .25):
                self._stop_trajectory()
            self._log_status("WAIT_FOR_FEEDBACK")
            return
        if self.trajectory_stream is not None:
            # Reserve outstanding travel, this segment, and local TCP/frame delay.
            queued_distance = math.dist((observed.x, observed.y, observed.z),
                                        (command.x, command.y, command.z))
            segment_distance = math.dist(result.next_position, (command.x, command.y, command.z))
            execution_window = (queued_distance + segment_distance) / self.speed + .05
            try:
                status = self.depth_navigation.follower.status
                packet = self.trajectory_stream.move(
                    (command.x, command.y, command.z),
                    (approved.x, approved.y, approved.z),
                    self.depth_navigation.reset_sequence, self.speed, now, status,
                    minimum_ttl=execution_window)
            except InsufficientTrajectoryTime:
                self._log_status("WAIT_FOR_APPROVAL_WINDOW")
                return  # Existing segments retain their original expiry; never extend it.
            except ValueError:
                self._stop_trajectory()
                return
            if self.trajectory_first_move is None:
                self.trajectory_first_move = self.trajectory_stream.sequence
            self.trajectory_publisher.publish(String(data=packet))
        else:
            self._publish_commands(approved)
        self.command_position = approved
        self._log_status("DEPTH_APPROVED")

    def _stop_trajectory(self, force=False):
        """Cancel pending motion, then wait for the applied stop acknowledgement.

        Reanchoring from ordinary feedback here would race the remaining Unity
        queue. Only _trajectory_ready may anchor, after the stop is acknowledged.
        """
        if self.trajectory_stream is None or self.trajectory_stop_sequence is not None:
            return
        sequence = self.depth_navigation.reset_sequence
        if sequence is None:
            return
        packet = self.trajectory_stream.stop(sequence, force=force)
        if packet is not None:
            self.trajectory_stop_sequence = self.trajectory_stream.sequence
            self.trajectory_needs_sync = True
            self.trajectory_publisher.publish(String(data=packet))

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

        # Bias lateral movement away from measured obstacles.
        sensors = self._parse_lateral_sensors()
        repulsion_x, repulsion_y = 0.0, 0.0

        # Sensor directional vectors relative to target vector
        sensor_directions = {
            "F": (-tx, -ty),
            "R": (-ty, tx),
            "L": (ty, -tx),
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

    def _publish_commands(self, point_msg):
        """Publish the absolute ROS position consumed by Unity."""
        self.command_publisher.publish(point_msg)

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