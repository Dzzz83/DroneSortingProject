"""Full-envelope queries against current evidence, including unknown as blocked."""
import math
import itertools
import numpy as np


class ObservedClearance:
    def __init__(self, local_map, envelope, evidence):
        self.resolution = local_map.settings.resolution
        self.envelope = envelope
        self.bounds = local_map.bounds
        self.checked = {}
        if self.bounds is None:
            self.prefix = None
            return
        self.lower = tuple(lo for lo, _hi in self.bounds)
        shape = tuple(hi-lo+1 for lo, hi in self.bounds)
        # A summed-volume table makes each full-envelope check eight lookups.
        # Neither unobserved nor expired cells may contribute free volume.
        free = np.zeros(shape, dtype=np.int32)
        cells = [cell for cell, state in evidence.items() if state == 'free']
        if cells:
            # Flatten integer triples directly: older Noetic NumPy spends much
            # longer inferring a nested Python sequence's shape and types.
            indices = np.fromiter(itertools.chain.from_iterable(cells),dtype=np.int32)
            indices = indices.reshape(-1,3) - self.lower
            indices = indices[np.all((indices >= 0) & (indices < shape), axis=1)]
            free[tuple(indices.T)] = 1
        self.prefix = np.pad(free, ((1,0),)*3).cumsum(0).cumsum(1).cumsum(2)

    def volume_free(self, position):
        if position in self.checked:
            return self.checked[position]
        if self.prefix is None:
            return False
        e = self.envelope
        limits = ((position[0]-e.half_x, position[0]+e.half_x),
                  (position[1]-e.half_y, position[1]+e.half_y),
                  (position[2]-e.below, position[2]+e.above))
        result = self._box_free(limits)
        self.checked[position] = result
        return result

    def _box_free(self, limits):
        """Check an axis-aligned volume with the planner's face-contact tolerance."""
        if self.prefix is None:
            return False
        r = self.resolution
        epsilon = r*1e-8
        low = tuple(math.floor((a+epsilon)/r) for a, _b in limits)
        high = tuple(math.floor((b-epsilon)/r) for _a, b in limits)
        if any(a < lo or b > hi for a, b, (lo, hi) in zip(low, high, self.bounds)):
            return False
        x,y,z = (a-lo for a,lo in zip(low,self.lower))
        X,Y,Z = (b-lo+1 for b,lo in zip(high,self.lower))
        p = self.prefix
        count = (p[X,Y,Z]-p[x,Y,Z]-p[X,y,Z]-p[X,Y,z]
                 +p[x,y,Z]+p[x,Y,z]+p[X,y,z]-p[x,y,z])
        return bool(count == (X-x)*(Y-y)*(Z-z))

    def sweep_free(self, start, end):
        """Certify every voxel overlap along a linear envelope translation.

        Overlap changes only when an envelope face crosses a voxel plane.
        Checking those events and each intervening interval catches brief corner
        intersections that a fixed spatial sampling distance can miss.
        """
        e = self.envelope
        below, above = (e.half_x, e.half_y, e.below), (e.half_x, e.half_y, e.above)
        swept_limits = tuple((min(a, b)-lo, max(a, b)+hi)
                             for a, b, lo, hi in zip(start, end, below, above))
        # Most search edges are axis-aligned. Their swept volume is exactly one
        # box; a clear enclosing box also certifies any diagonal it contains.
        if self._box_free(swept_limits):
            return True
        if sum(a != b for a, b in zip(start, end)) <= 1:
            return False
        if not self.volume_free(start) or not self.volume_free(end):
            return False

        resolution = self.resolution
        epsilon = resolution*1e-8
        events = {0., 1.}
        for a, b, lo, hi in zip(start, end, below, above):
            delta = b-a
            if delta == 0:
                continue
            # Use the same effective face positions as _box_free, so touching
            # a voxel boundary does not introduce a different clearance rule.
            for offset in (-lo+epsilon, hi-epsilon):
                face_start, face_end = a+offset, b+offset
                first = math.floor(min(face_start, face_end)/resolution) + 1
                last = math.ceil(max(face_start, face_end)/resolution)
                for plane in range(first, last):
                    fraction = (plane*resolution-face_start)/delta
                    if 0 < fraction < 1:
                        events.add(fraction)

        def clear_at(fraction):
            return self.volume_free(tuple(a+(b-a)*fraction for a, b in zip(start, end)))

        ordered = sorted(events)
        for left, right in zip(ordered, ordered[1:]):
            if not clear_at((left+right)*.5) or not clear_at(right):
                return False
        return True
