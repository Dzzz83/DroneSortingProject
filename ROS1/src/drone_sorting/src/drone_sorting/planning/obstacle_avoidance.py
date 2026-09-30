import math


class ObstacleAvoidancePlanner:
    """
    Stateful static obstacle avoidance.

    Unity supplies sensor measurements.
    ROS makes every avoidance decision.
    """

    FORWARD = "forward"
    FORWARD_LEVEL = "forward_level"

    UP = "up"
    DOWN = "down"
    LEFT = "left"
    RIGHT = "right"

    STOP = "stop"

    NORMAL = "normal"
    CLIMB = "climb"
    DESCEND = "descend"
    PASS = "pass"
    SIDESTEP_LEFT = "sidestep_left"
    SIDESTEP_RIGHT = "sidestep_right"

    def __init__(
        self,
        front_threshold=3.0,
        front_release_distance=3.5,
        vertical_clearance=0.5,
        side_clearance=0.8,
        pass_distance=5.0,
        sidestep_distance=1.2,
    ):
        self.front_threshold = front_threshold
        self.front_release_distance = front_release_distance

        self.vertical_clearance = vertical_clearance
        self.side_clearance = side_clearance

        self.pass_distance = pass_distance
        self.sidestep_distance = sidestep_distance

        self.mode = self.NORMAL

        self.pass_start_position = None
        self.sidestep_start_position = None

    def reset(self):
        self.mode = self.NORMAL
        self.pass_start_position = None
        self.sidestep_start_position = None

    def choose_direction(
        self,
        obstacle_distances,
        position,
    ):
        if obstacle_distances is None:
            return self.FORWARD

        (
            front,
            upper_front,
            lower_front,
            left,
            right,
            up,
            down,
        ) = obstacle_distances

        # --------------------------------------------------
        # NORMAL
        # --------------------------------------------------

        if self.mode == self.NORMAL:

            if not self._front_blocked(front):
                return self.FORWARD

            return self._begin_avoidance(
                upper_front,
                lower_front,
                left,
                right,
                up,
                down,
                position,
            )

        # --------------------------------------------------
        # CLIMB
        # --------------------------------------------------

        if self.mode == self.CLIMB:

            if self._front_released(front):
                return self._begin_pass(position)

            if self._is_clear(
                up,
                self.vertical_clearance,
            ):
                return self.UP

            return self._begin_side_avoidance(
                left,
                right,
                position,
            )

        # --------------------------------------------------
        # DESCEND
        # --------------------------------------------------

        if self.mode == self.DESCEND:

            if self._front_released(front):
                return self._begin_pass(position)

            if self._is_clear(
                down,
                self.vertical_clearance,
            ):
                return self.DOWN

            return self._begin_side_avoidance(
                left,
                right,
                position,
            )

        # --------------------------------------------------
        # PASS OBSTACLE
        # --------------------------------------------------

        if self.mode == self.PASS:

            if self._travelled(
                self.pass_start_position,
                position,
            ) >= self.pass_distance:

                self.mode = self.NORMAL
                self.pass_start_position = None

                return self.FORWARD

            return self.FORWARD_LEVEL

        # --------------------------------------------------
        # SIDE STEP
        # --------------------------------------------------

        if self.mode == self.SIDESTEP_LEFT:

            if self._travelled(
                self.sidestep_start_position,
                position,
            ) >= self.sidestep_distance:

                self.mode = self.NORMAL
                self.sidestep_start_position = None

                return self.FORWARD

            return self.LEFT

        if self.mode == self.SIDESTEP_RIGHT:

            if self._travelled(
                self.sidestep_start_position,
                position,
            ) >= self.sidestep_distance:

                self.mode = self.NORMAL
                self.sidestep_start_position = None

                return self.FORWARD

            return self.RIGHT

        return self.STOP

    def _begin_avoidance(
        self,
        upper_front,
        lower_front,
        left,
        right,
        up,
        down,
        position,
    ):
        can_move_up = self._is_clear(
            up,
            self.vertical_clearance,
        )

        can_move_down = self._is_clear(
            down,
            self.vertical_clearance,
        )

        upper_path_clear = self._path_clear(
            upper_front
        )

        lower_path_clear = self._path_clear(
            lower_front
        )

        # Prefer the direction whose forward probe
        # already shows a clear route.

        if upper_path_clear and can_move_up:
            self.mode = self.CLIMB
            return self.UP

        if lower_path_clear and can_move_down:
            self.mode = self.DESCEND
            return self.DOWN

        # Even if the offset probe still sees the obstacle,
        # continue climbing/descending if there is room.
        # This lets the drone move far enough to actually clear it.

        if can_move_up:
            self.mode = self.CLIMB
            return self.UP

        if can_move_down:
            self.mode = self.DESCEND
            return self.DOWN

        return self._begin_side_avoidance(
            left,
            right,
            position,
        )

    def _begin_pass(self, position):
        self.mode = self.PASS

        self.pass_start_position = (
            position.x,
            position.y,
        )

        return self.FORWARD_LEVEL

    def _begin_side_avoidance(
        self,
        left,
        right,
        position,
    ):
        left_space = self._available_space(left)
        right_space = self._available_space(right)

        self.sidestep_start_position = (
            position.x,
            position.y,
        )

        if (
            left_space >= self.side_clearance
            and left_space >= right_space
        ):
            self.mode = self.SIDESTEP_LEFT
            return self.LEFT

        if right_space >= self.side_clearance:
            self.mode = self.SIDESTEP_RIGHT
            return self.RIGHT

        self.sidestep_start_position = None

        return self.STOP

    def _front_blocked(self, distance):
        return (
            distance >= 0.0
            and distance < self.front_threshold
        )

    def _front_released(self, distance):
        return (
            distance < 0.0
            or distance > self.front_release_distance
        )

    def _path_clear(self, distance):
        return (
            distance < 0.0
            or distance > self.front_threshold
        )

    @staticmethod
    def _is_clear(distance, clearance):
        return (
            distance < 0.0
            or distance > clearance
        )

    @staticmethod
    def _available_space(distance):
        if distance < 0.0:
            return float("inf")

        return distance

    @staticmethod
    def _travelled(start_position, position):
        if start_position is None:
            return 0.0

        start_x, start_y = start_position

        return math.sqrt(
            (position.x - start_x) ** 2
            + (position.y - start_y) ** 2
        )