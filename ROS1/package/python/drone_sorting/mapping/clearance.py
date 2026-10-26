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
        r = self.resolution
        e = self.envelope
        limits = ((position[0]-e.half_x, position[0]+e.half_x),
                  (position[1]-e.half_y, position[1]+e.half_y),
                  (position[2]-e.below, position[2]+e.above))
        epsilon = r*1e-8
        low = tuple(math.floor((a+epsilon)/r) for a, _b in limits)
        high = tuple(math.floor((b-epsilon)/r) for _a, b in limits)
        if any(a < lo or b > hi for a, b, (lo, hi) in zip(low, high, self.bounds)):
            self.checked[position] = False
            return False
        x,y,z = (a-lo for a,lo in zip(low,self.lower))
        X,Y,Z = (b-lo+1 for b,lo in zip(high,self.lower))
        p = self.prefix
        count = (p[X,Y,Z]-p[x,Y,Z]-p[X,y,Z]-p[X,Y,z]
                 +p[x,y,Z]+p[x,Y,z]+p[X,y,z]-p[x,y,z])
        result = bool(count == (X-x)*(Y-y)*(Z-z))
        self.checked[position] = result
        return result

    def sweep_free(self, start, end):
        samples = max(1, math.ceil(math.dist(start,end)/(self.resolution*.5)))
        return all(self.volume_free(tuple(start[d]+(end[d]-start[d])*i/samples
                                          for d in range(3)))
                   for i in range(samples+1))
