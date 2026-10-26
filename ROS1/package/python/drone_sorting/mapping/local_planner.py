"""Conservative fixed-altitude A* against LocalDepthMap's timestamped evidence.

A route is justified only when every voxel intersecting the entire vehicle
bounding envelope is explicitly free. Unknown and occupied are both non-traversable.
This planner does not publish ROS commands or access Unity geometry.
"""
from dataclasses import dataclass
import heapq
import itertools
import math

from drone_sorting.mapping.clearance import ObservedClearance


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

    def __init__(self, local_map, envelope, max_expansions=4000, allow_vertical=False):
        if not isinstance(envelope, VehicleEnvelope):
            raise TypeError("Explicit VehicleEnvelope is required")
        if not isinstance(max_expansions, int) or max_expansions < 1:
            raise ValueError("max_expansions must be a positive integer")
        self.map = local_map
        self.envelope = envelope
        self.max_expansions = max_expansions
        self.allow_vertical = allow_vertical
        self.previous_path = ()

    def envelope_cells(self, position):
        """Voxels with positive-volume overlap; face-only contact is excluded.

        Use the same boundary tolerance for planning and diagnostics.
        """
        resolution = self.map.settings.resolution
        epsilon = resolution * 1e-8
        x, y, z = position
        e = self.envelope
        limits = ((x-e.half_x, x+e.half_x), (y-e.half_y, y+e.half_y),
                  (z-e.below, z+e.above))
        ranges = [range(math.floor((lo+epsilon)/resolution),
                        math.floor((hi-epsilon)/resolution)+1)
                  for lo, hi in limits]
        return itertools.product(*ranges)

    def clearance_diagnostics(self, position, now, camera_position=None, camera_forward=None,
                              evidence=None):
        """Read-only summary against current authoritative evidence, never approval.

        Expired counts cover only retained records: integration prunes history,
        so an absent cell cannot be distinguished from a previously expired one.
        Direction counts overlap and use voxel centres relative to the actual
        acquisition camera, not a requested heading or Unity geometry.
        """
        if evidence is None:
            evidence = {cell: state for cell, state, _age in self.map.snapshot(now)}
        counts = dict(free=0, occupied=0, unknown=0)
        outside = expired = total = 0
        samples = {"occupied": [], "unknown": []}
        directions = {name: dict(occupied=0, unknown=0) for name in
                      ("front", "behind", "left", "right", "above", "below")}
        resolution = self.map.settings.resolution
        for cell in self.envelope_cells(position):
            total += 1
            state = evidence.get(cell, "unknown")
            counts[state] += 1
            in_bounds = (self.map.bounds is not None and
                         all(lo <= value <= hi for value, (lo, hi)
                             in zip(cell, self.map.bounds)))
            outside += not in_bounds
            times = self.map.cells.get(cell)
            retained_expired = (state == "unknown" and times is not None and
                                now-max(times) > self.map.settings.maximum_age)
            expired += retained_expired
            if state == "free":
                continue
            center = tuple((index+.5)*resolution for index in cell)
            if len(samples[state]) < 6:
                samples[state].append({"cell": cell, "center": center,
                                       "in_bounds": in_bounds,
                                       "retained_expired": retained_expired})
            if camera_position is not None and camera_forward is not None:
                dx, dy, dz = (a-b for a, b in zip(center, camera_position))
                fx, fy = camera_forward[:2]
                projections = (("front", "behind", dx*fx+dy*fy),
                               ("left", "right", -dx*fy+dy*fx),
                               ("above", "below", dz))
                for positive, negative, value in projections:
                    if abs(value) > 1e-9:
                        directions[positive if value > 0 else negative][state] += 1
        return {"position": position, "envelope": vars(self.envelope),
                "resolution": resolution, "map_bounds_cells": self.map.bounds,
                "total": total, **counts, "outside_bounds": outside,
                "retained_expired_unknown": expired,
                "camera_position": camera_position, "camera_forward": camera_forward,
                "failed_by_camera_direction": (directions if camera_position is not None
                                               and camera_forward is not None else None),
                "failed_samples": samples}

    def plan(self, start, goal, now, evidence=None, clearance=None):
        if (len(start) != 3 or len(goal) != 3
                or not all(math.isfinite(value) for value in (*start, *goal, now))):
            return PlanResult("HOLD", "invalid_position_or_goal")
        resolution = self.map.settings.resolution
        if not self.allow_vertical and abs(goal[2] - start[2]) > resolution / 2:
            return PlanResult("HOLD", "altitude_change_not_supported")
        if self.map.bounds is None:
            return PlanResult("HOLD", "empty_map")

        # snapshot(now) applies independent free/occupied acquisition ages.
        if evidence is None:
            evidence = {cell: state for cell, state, _age in self.map.snapshot(now)}
        if not evidence:
            return PlanResult("HOLD", "no_current_map_evidence")
        clearance = clearance or ObservedClearance(self.map, self.envelope, evidence)
        volume_free = clearance.volume_free
        sweep_free = clearance.sweep_free

        start = tuple(start)
        goal = tuple(goal)
        if not volume_free(start):
            return PlanResult("HOLD", "start_envelope_not_observed_free")
        if not volume_free(goal):
            return PlanResult("HOLD", "goal_envelope_not_observed_free")

        # Preserve the remaining turns while following a route. Rebuilding the
        # initial connector from delayed feedback changes the commanded line.
        # This is geometry reuse, not approval reuse: every remaining sweep is
        # checked against this cycle's evidence and the current envelope.
        if self.previous_path and self.previous_path[-1] == goal:
            for index in range(len(self.previous_path)-1):
                a, b = self.previous_path[index:index+2]
                delta = tuple(b[d]-a[d] for d in range(3))
                length_sq = sum(v*v for v in delta)
                if length_sq < 1e-15:
                    continue
                progress = sum((start[d]-a[d])*delta[d] for d in range(3))/length_sq
                projected = tuple(a[d]+max(0., min(1., progress))*delta[d] for d in range(3))
                if math.dist(start, projected) > 1e-5 or progress >= 1.-1e-8:
                    continue
                remaining = (start, *self.previous_path[index+1:])
                if all(sweep_free(a, b) for a, b in zip(remaining, remaining[1:])):
                    self.previous_path = remaining
                    return PlanResult("READY", "observed_route", remaining, 0)
        self.previous_path = ()

        dimensions = 3 if self.allow_vertical else 2
        origin = self.map.cell_at(start)[:dimensions]
        destination = self.map.cell_at(goal)[:dimensions]
        altitude = start[2]

        def center(cell):
            coordinates = tuple((value+.5)*resolution for value in cell)
            return coordinates if self.allow_vertical else (*coordinates, altitude)

        if not sweep_free(start, center(origin)) or not sweep_free(center(destination), goal):
            return PlanResult("HOLD", "endpoint_connector_not_observed_free")

        if origin == destination:
            if not sweep_free(start, goal):
                return PlanResult("HOLD", "direct_segment_not_observed_free")
            self.previous_path = (start, goal)
            return PlanResult("READY", "observed_route", self.previous_path, 0)

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
                middle = [center(cell) for cell in chain]
                # Cell centres are search nodes, not compulsory stops. Avoid
                # stepping backward to the origin centre or past the actual
                # destination, only when the replacement sweep is also clear.
                if len(middle) > 1 and sweep_free(start, middle[1]):
                    middle.pop(0)
                before_last = middle[-2] if len(middle) > 1 else start
                if middle and sweep_free(before_last, goal):
                    middle.pop()
                self.previous_path = (start, *middle, goal)
                return PlanResult("READY", "observed_route", self.previous_path, expanded)
            if expanded >= self.max_expansions:
                return PlanResult("HOLD", "search_limit", (), expanded)
            offsets = ((1,0), (0,1), (-1,0), (0,-1)) if dimensions == 2 else (
                (1,0,0), (0,1,0), (-1,0,0), (0,-1,0), (0,0,1), (0,0,-1))
            for offset in offsets:
                neighbor = tuple(a+b for a, b in zip(current, offset))
                if neighbor in visited:
                    continue
                if not sweep_free(center(current), center(neighbor)):
                    continue
                score = cost + 1
                if score < gscore.get(neighbor, math.inf):
                    gscore[neighbor] = score
                    previous[neighbor] = current
                    heuristic = sum(abs(a-b) for a, b in zip(neighbor, destination))
                    heapq.heappush(frontier, (score+heuristic, score, neighbor))
        return PlanResult("HOLD", "no_observed_clear_route", (), expanded)
