"""Sparse 3-D evidence map in ROS warehouse metres: X/Y horizontal, Z up.

Only finite depth rays contribute free evidence. NaN, infinity and unsampled
regions remain unknown. A free voxel records a sampled ray, not certified vehicle
clearance; occupied evidence wins until it expires. This module issues no commands.
"""
from dataclasses import dataclass
import itertools
import math

import numpy as np

from drone_sorting.perception.camera_geometry import transform_point


@dataclass(frozen=True)
class DepthObservation:
    stamp: float
    depth: np.ndarray
    intrinsics: tuple
    position: tuple
    quaternion: tuple
    drone_position: tuple

    def __post_init__(self):
        values = (self.stamp, *self.intrinsics, *self.position, *self.quaternion, *self.drone_position)
        if (len(self.intrinsics) != 9 or len(self.position) != 3 or len(self.quaternion) != 4
                or len(self.drone_position) != 3 or not all(math.isfinite(value) for value in values)):
            raise ValueError("Invalid depth observation metadata")
        k = self.intrinsics
        if (k[0] <= 0 or k[4] <= 0 or any(abs(k[index]) > 1e-9 for index in (1,3,6,7))
                or abs(k[8]-1) > 1e-9):
            raise ValueError("Expected calibrated undistorted pinhole intrinsics")
        transform_point((0,0,0), self.position, self.quaternion)
        depth = np.array(self.depth, dtype=np.float32, copy=True)
        if depth.ndim != 2 or not depth.size:
            raise ValueError("Expected a nonempty two-dimensional depth image")
        finite = depth[np.isfinite(depth)]
        if np.isneginf(depth).any() or (finite < .2).any() or (finite > 15).any():
            raise ValueError("Depth violates the 0.2–15 m forward sensor contract")
        depth.setflags(write=False)
        object.__setattr__(self, "depth", depth)
        for name in ("intrinsics", "position", "quaternion", "drone_position"):
            object.__setattr__(self, name, tuple(getattr(self, name)))

    def surface_points(self, stride):
        rows, columns = np.mgrid[0:self.depth.shape[0]:stride, 0:self.depth.shape[1]:stride]
        depth = self.depth[::stride, ::stride]
        good = np.isfinite(depth)
        z = depth[good].astype(float)
        k = self.intrinsics
        optical = np.column_stack(((columns[good]-k[2])*z/k[0], (rows[good]-k[5])*z/k[4], z))
        # Reuse the established quaternion convention instead of swapping components.
        axes = [transform_point(axis, (0,0,0), self.quaternion)
                for axis in ((1,0,0), (0,1,0), (0,0,1))]
        rotation = np.column_stack(axes)
        return optical @ rotation.T + self.position, z


@dataclass(frozen=True)
class MapSettings:
    resolution: float = .25
    radius: float = 8.
    vertical_radius: float = 3.
    maximum_age: float = 2.
    surface_margin: float = .1
    pixel_stride: int = 8

    def __post_init__(self):
        dimensions = (self.resolution, self.radius, self.vertical_radius, self.maximum_age, self.surface_margin)
        if not all(math.isfinite(value) and value > 0 for value in dimensions):
            raise ValueError("Map dimensions, age and uncertainty margin must be finite and positive")
        if (not isinstance(self.pixel_stride, int) or self.pixel_stride < 1
                or min(self.radius, self.vertical_radius) < self.resolution):
            raise ValueError("Invalid map sampling or window")


class LocalDepthMap:
    def __init__(self, settings=None):
        self.settings = settings or MapSettings()
        # Each cell retains independent free/occupied acquisition times.
        self.cells = {}
        self.bounds = None
        self.last_stamp = None

    def reset(self):
        """Drop all evidence on a new trial or acquisition-clock reset."""
        self.cells.clear()
        self.bounds = None
        self.last_stamp = None

    def cell_at(self, point):
        return tuple(math.floor(value/self.settings.resolution) for value in point)

    def _inside(self, cell):
        return all(low <= value <= high for value, (low, high) in zip(cell, self.bounds))

    def _ray_cells(self, start, end):
        """Voxel traversal, stepping tied axes together at an edge/corner."""
        resolution = self.settings.resolution
        cell = list(self.cell_at(start))
        final = self.cell_at(end)
        direction = [b-a for a,b in zip(start,end)]
        step = [1 if value > 0 else -1 if value < 0 else 0 for value in direction]
        crossing, increment = [], []
        for axis in range(3):
            if step[axis] == 0:
                crossing.append(math.inf); increment.append(math.inf)
            else:
                boundary = (cell[axis] + (step[axis] > 0))*resolution
                crossing.append((boundary-start[axis])/direction[axis])
                increment.append(resolution/abs(direction[axis]))
        maximum_steps = 1 + sum(abs(a-b) for a,b in zip(cell,final))
        for _ in range(maximum_steps):
            exit_time = min(crossing)
            if exit_time > 0:
                yield tuple(cell)
            if tuple(cell) == final or exit_time >= 1:
                return
            for axis in range(3):
                if abs(crossing[axis]-exit_time) < 1e-12:
                    cell[axis] += step[axis]
                    crossing[axis] += increment[axis]

    def _mark(self, cell, stamp, occupied):
        if not self._inside(cell):
            return
        times = self.cells.setdefault(cell, [-math.inf, -math.inf])
        times[1 if occupied else 0] = stamp

    def integrate(self, observation, now):
        s = self.settings
        if not math.isfinite(now) or not 0 <= now-observation.stamp <= s.maximum_age:
            raise ValueError("Stale or future observation cannot update the map")
        if self.last_stamp is not None and observation.stamp <= self.last_stamp:
            raise ValueError("Acquisitions must increase; receipt time cannot refresh a replay")
        radii = (s.radius, s.radius, s.vertical_radius)
        self.bounds = tuple((math.floor((value-radius)/s.resolution), math.floor((value+radius)/s.resolution))
                            for value,radius in zip(observation.drone_position,radii))
        self.cells = {cell:times for cell,times in self.cells.items()
                      if self._inside(cell) and 0 <= now-max(times) <= s.maximum_age}
        points, depths = observation.surface_points(s.pixel_stride)
        camera = np.asarray(observation.position)
        for point, optical_depth in zip(points, depths):
            # Stop short of the uncertain surface. Never clear behind a return.
            if optical_depth > .2 + s.surface_margin:
                fraction = (optical_depth-s.surface_margin)/optical_depth
                endpoint = camera + (point-camera)*fraction
                for cell in self._ray_cells(camera, endpoint):
                    self._mark(cell, observation.stamp, occupied=False)
            lower = self.cell_at(point-s.surface_margin)
            upper = self.cell_at(point+s.surface_margin)
            for cell in itertools.product(*(range(a,b+1) for a,b in zip(lower,upper))):
                self._mark(cell, observation.stamp, occupied=True)
        self.last_stamp = observation.stamp

    def state_at(self, point, now):
        times = self.cells.get(self.cell_at(point))
        if times is None:
            return "unknown"
        free, occupied = times
        if 0 <= now-occupied <= self.settings.maximum_age:
            return "occupied"
        if 0 <= now-free <= self.settings.maximum_age:
            return "free"
        return "unknown"

    def snapshot(self, now):
        records = []
        for cell, (free, occupied) in sorted(self.cells.items()):
            point = tuple((value+.5)*self.settings.resolution for value in cell)
            state = self.state_at(point, now)
            if state != "unknown":
                stamp = occupied if state == "occupied" else free
                records.append((cell, state, now-stamp))
        return records
