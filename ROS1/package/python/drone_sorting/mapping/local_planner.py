"""Conservative fixed-altitude A* against LocalDepthMap's timestamped evidence.

A route is justified only when every voxel intersecting the entire vehicle
bounding envelope is explicitly free. Unknown and occupied are both non-traversable.
This planner does not publish ROS commands or access Unity geometry.
"""
from dataclasses import dataclass
import heapq
import itertools
import math


@dataclass(frozen=True)
class VehicleEnvelope:
    """Axis-aligned extents from drone root in ROS warehouse metres.

    Horizontal half-widths are in ROS X/Y; below/above refer to ROS Z.
    Caller must obtain these from the actual drone and carried package geometry.
    """
    half_x: float
    half_y: float
    below: float
    above: float

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in vars(self).values()):
            raise ValueError("Vehicle envelope extents must be positive and finite")


@dataclass(frozen=True)
class PlanResult:
    state: str
    reason: str
    path: tuple = ()
    expanded: int = 0


class LocalPlanner:
    """4-neighbor, fixed-height local search; never assumes unseen space is clear."""

    def __init__(self, local_map, envelope, max_expansions=4000):
        if not isinstance(envelope, VehicleEnvelope):
            raise TypeError("Explicit VehicleEnvelope is required")
        if not isinstance(max_expansions, int) or max_expansions < 1:
            raise ValueError("max_expansions must be a positive integer")
        self.map = local_map
        self.envelope = envelope
        self.max_expansions = max_expansions

    def plan(self, start, goal, now):
        if (len(start) != 3 or len(goal) != 3
                or not all(math.isfinite(value) for value in (*start, *goal, now))):
            return PlanResult("HOLD", "invalid_position_or_goal")
        resolution = self.map.settings.resolution
        if abs(goal[2] - start[2]) > resolution / 2:
            return PlanResult("HOLD", "altitude_change_not_supported")
        if self.map.bounds is None:
            return PlanResult("HOLD", "empty_map")

        # snapshot(now) applies independent free/occupied acquisition ages.
        evidence = {cell: state for cell, state, _age in self.map.snapshot(now)}
        if not evidence:
            return PlanResult("HOLD", "no_current_map_evidence")
        below, above = self.envelope.below, self.envelope.above
        hx, hy = self.envelope.half_x, self.envelope.half_y
        epsilon = resolution * 1e-8
        checked = {}

        def volume_free(position):
            if position in checked:
                return checked[position]
            x, y, z = position
            limits = ((x-hx, x+hx), (y-hy, y+hy), (z-below, z+above))
            ranges = [range(math.floor((lo+epsilon)/resolution),
                            math.floor((hi-epsilon)/resolution)+1)
                      for lo, hi in limits]
            result = all(evidence.get(cell) == "free" for cell in itertools.product(*ranges))
            checked[position] = result
            return result

        def sweep_free(a, b):
            length = math.dist(a, b)
            samples = max(1, math.ceil(length / (resolution * 0.5)))
            return all(volume_free(tuple(a[d] + (b[d]-a[d]) * i / samples
                                         for d in range(3)))
                       for i in range(samples+1))

        start = tuple(start)
        goal = tuple(goal)
        if not volume_free(start):
            return PlanResult("HOLD", "start_envelope_not_observed_free")
        if not volume_free(goal):
            return PlanResult("HOLD", "goal_envelope_not_observed_free")

        origin = self.map.cell_at(start)[:2]
        destination = self.map.cell_at(goal)[:2]
        altitude = start[2]

        def center(cell):
            return ((cell[0]+0.5)*resolution, (cell[1]+0.5)*resolution, altitude)

        if not sweep_free(start, center(origin)) or not sweep_free(center(destination), goal):
            return PlanResult("HOLD", "endpoint_connector_not_observed_free")

        if origin == destination:
            if not sweep_free(start, goal):
                return PlanResult("HOLD", "direct_segment_not_observed_free")
            return PlanResult("READY", "observed_route", (start, goal), 0)

        frontier = [(0, 0, origin)]
        gscore = {origin: 0}
        previous = {}
        visited = set()
        expanded = 0
        while frontier:
            _priority, cost, current = heapq.heappop(frontier)
            if current in visited or cost != gscore[current]:
                continue
            visited.add(current)
            expanded += 1
            if current == destination:
                chain = [current]
                while chain[-1] != origin:
                    chain.append(previous[chain[-1]])
                chain.reverse()
                waypoints = (start, *(center(cell) for cell in chain), goal)
                # Include actual start/goal, not just voxel centres. Keep adjacent
                # points for easy inspection; don't simplify across untested cells.
                return PlanResult("READY", "observed_route", waypoints, expanded)
            if expanded >= self.max_expansions:
                return PlanResult("HOLD", "search_limit", (), expanded)
            for dx, dy in ((1,0), (0,1), (-1,0), (0,-1)):
                neighbor = (current[0]+dx, current[1]+dy)
                if neighbor in visited:
                    continue
                if not sweep_free(center(current), center(neighbor)):
                    continue
                score = cost + 1
                if score < gscore.get(neighbor, math.inf):
                    gscore[neighbor] = score
                    previous[neighbor] = current
                    heuristic = abs(neighbor[0]-destination[0])+abs(neighbor[1]-destination[1])
                    heapq.heappush(frontier, (score+heuristic, score, neighbor))
        return PlanResult("HOLD", "no_observed_clear_route", (), expanded)
