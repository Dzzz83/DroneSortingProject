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
        points, depths, _hits = self.sampled_rays(stride)
        return points, depths

    def sampled_rays(self, stride, include_clear_range=False):
        """Optional +inf is an explicit no-hit measurement over the 15 m range.

        NaN remains missing/invalid. A no-hit ray has no occupied endpoint.
        The live adapter must verify the sensor's range declaration first.
        """
        rows, columns = np.mgrid[0:self.depth.shape[0]:stride, 0:self.depth.shape[1]:stride]
        depth = self.depth[::stride, ::stride]
        good = np.isfinite(depth)
        if include_clear_range:
            good |= np.isposinf(depth)
        hits = np.isfinite(depth[good])
        z = np.where(hits, depth[good], 15.).astype(float)
        k = self.intrinsics
        optical = np.column_stack(((columns[good]-k[2])*z/k[0], (rows[good]-k[5])*z/k[4], z))
        # Reuse the established quaternion convention instead of swapping components.
        axes = [transform_point(axis, (0,0,0), self.quaternion)
                for axis in ((1,0,0), (0,1,0), (0,0,1))]
        rotation = np.column_stack(axes)
        return optical @ rotation.T + self.position, z, hits


@dataclass(frozen=True)
class MapSettings:
    resolution: float = .25
    radius: float = 8.
    vertical_radius: float = 3.
    maximum_age: float = 2.
    surface_margin: float = .1
    pixel_stride: int = 8
    clear_range_evidence: bool = False

    def __post_init__(self):
        dimensions = (self.resolution, self.radius, self.vertical_radius, self.maximum_age, self.surface_margin)
        if not all(math.isfinite(value) and value > 0 for value in dimensions):
            raise ValueError("Map dimensions, age and uncertainty margin must be finite and positive")
        if (not isinstance(self.pixel_stride, int) or self.pixel_stride < 1
                or min(self.radius, self.vertical_radius) < self.resolution):
            raise ValueError("Invalid map sampling or window")
        if not isinstance(self.clear_range_evidence, bool):
            raise ValueError("clear_range_evidence must be boolean")


class LocalDepthMap:
    def __init__(self, settings=None):
        self.settings = settings or MapSettings()
        # Each cell retains independent free/occupied acquisition times.
        self.cells = {}
        self.bounds = None
        self.last_stamp = None
        self.last_prune = None

    def reset(self):
        """Drop all evidence on a new trial or acquisition-clock reset."""
        self.cells.clear()
        self.bounds = None
        self.last_stamp = None
        self.last_prune = None

    def cell_at(self, point):
        return tuple(math.floor(value/self.settings.resolution) for value in point)

    def _inside(self, cell):
        x, y, z = cell
        (x0, x1), (y0, y1), (z0, z1) = self.bounds
        return x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1

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

    def _free_ray_cells(self, start, endpoints):
        """Batch the same DDA traversal as _ray_cells, without per-ray Python loops."""
        if not len(endpoints):
            return []
        resolution = self.settings.resolution
        direction = endpoints - np.asarray(start)
        step = np.sign(direction).astype(np.int64)
        current = np.tile(self.cell_at(start), (len(endpoints), 1))
        final = np.floor(endpoints/resolution).astype(np.int64)
        boundary = (current + (step > 0))*resolution
        with np.errstate(divide='ignore', invalid='ignore'):
            crossing = np.where(step != 0, (boundary-start)/direction, np.inf)
            increment = np.where(step != 0, resolution/np.abs(direction), np.inf)
        active = np.ones(len(endpoints), dtype=bool)
        lower, upper = np.asarray(self.bounds).T
        cells = []
        maximum_steps = 1 + int(np.max(np.abs(current-final).sum(axis=1)))
        for _ in range(maximum_steps):
            exit_time = crossing.min(axis=1)
            inside = np.all((current >= lower) & (current <= upper), axis=1)
            visible = active & inside & (exit_time > 0)
            if visible.any():
                cells.append(current[visible].copy())
            active &= ~np.all(current == final, axis=1) & (exit_time < 1)
            if not active.any():
                break
            with np.errstate(invalid='ignore'):
                tied = active[:, None] & (np.abs(crossing-exit_time[:, None]) < 1e-12)
            current += tied*step
            crossing = np.where(tied, crossing+increment, crossing)
        if not cells:
            return []
        # A one-dimensional cell key avoids sorting three-column records.
        shape = upper-lower+1
        offsets = np.concatenate(cells)-lower
        keys = np.ravel_multi_index(offsets.T, shape)
        unique = np.column_stack(np.unravel_index(np.unique(keys), shape))+lower
        return list(map(tuple, unique.tolist()))

    def integrate(self, observation, now):
        s = self.settings
        if not math.isfinite(now) or not 0 <= now-observation.stamp <= s.maximum_age:
            raise ValueError("Stale or future observation cannot update the map")
        if self.last_stamp is not None and observation.stamp <= self.last_stamp:
            raise ValueError("Acquisitions must increase; receipt time cannot refresh a replay")
        radii = (s.radius, s.radius, s.vertical_radius)
        bounds = tuple((math.floor((value-radius)/s.resolution), math.floor((value+radius)/s.resolution))
                       for value,radius in zip(observation.drone_position,radii))
        moved_window = bounds != self.bounds
        self.bounds = bounds
        # Queries always apply evidence ages. While stationary, pruning storage
        # need not repeat on every image; a changed window still prunes at once.
        if moved_window or self.last_prune is None or now-self.last_prune >= .5:
            # Inline the fixed three-axis window test in this large storage pass.
            # Avoid a Python generator and function call for every retained voxel.
            (x0, x1), (y0, y1), (z0, z1) = bounds
            maximum_age = s.maximum_age
            self.cells = {cell: times for cell, times in self.cells.items()
                          if x0 <= cell[0] <= x1 and y0 <= cell[1] <= y1
                          and z0 <= cell[2] <= z1
                          and 0 <= now - (times[0] if times[0] > times[1] else times[1]) <= maximum_age}
            self.last_prune = now
        points, depths, hits = observation.sampled_rays(s.pixel_stride, s.clear_range_evidence)
        camera = np.asarray(observation.position)
        valid_free = depths > .2 + s.surface_margin
        fractions = (depths[valid_free]-s.surface_margin)/depths[valid_free]
        endpoints = camera + (points[valid_free]-camera)*fractions[:, None]
        for cell in self._free_ray_cells(camera, endpoints):
            # Batch traversal already clipped these cells to the current bounds.
            times = self.cells.setdefault(cell, [-math.inf, -math.inf])
            times[0] = observation.stamp
        for point, optical_depth, hit in zip(points, depths, hits):
            if not hit:
                continue
            lower = self.cell_at(point-s.surface_margin)
            upper = self.cell_at(point+s.surface_margin)
            for cell in itertools.product(*(range(a,b+1) for a,b in zip(lower,upper))):
                self._mark(cell, observation.stamp, occupied=True)
        self.last_stamp = observation.stamp

    def state_at(self, point, now):
        return self.cell_state(self.cell_at(point), now)

    def cell_state(self, cell, now):
        times = self.cells.get(cell)
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
            state = self.cell_state(cell, now)
            if state != "unknown":
                stamp = occupied if state == "occupied" else free
                records.append((cell, state, now-stamp))
        return records

    def evidence(self, now):
        """Current cell states for planning, without sorting/visualization records.

        Same independent acquisition ages and occupied priority as snapshot().
        The mapping worker owns the map while constructing this dictionary.
        """
        maximum_age = self.settings.maximum_age
        states = {}
        for cell, (free, occupied) in self.cells.items():
            if 0 <= now-occupied <= maximum_age:
                states[cell] = "occupied"
            elif 0 <= now-free <= maximum_age:
                states[cell] = "free"
        return states
