"""Select reachable local goals using only the mapper's observed free volume."""
import heapq
import math

from drone_sorting.mapping.clearance import ObservedClearance
from drone_sorting.mapping.local_planner import PlanResult


class LocalGoalPlanner:
    def __init__(self, planner, radius=6., maximum_expansions=2500):
        self.planner = planner
        self.radius = radius
        self.maximum_expansions = maximum_expansions
        self.reset()

    def reset(self):
        self.destination = None
        self.goal = None
        self.visited = []
        self.planner.previous_path = ()

    def plan(self, start, destination, now, evidence):
        start, destination = tuple(start), tuple(destination)
        if self.destination != destination:
            self.reset()
            self.destination = destination
        clearance = ObservedClearance(self.planner.map, self.planner.envelope, evidence)
        if not clearance.volume_free(start):
            return PlanResult('HOLD', 'start_envelope_not_observed_free')
        if self.goal is not None and math.dist(start, self.goal) <= .15:
            self.visited.append(self.goal)
            self.visited = self.visited[-80:]
            self.goal = None
        if self.goal is not None:
            result = self.planner.plan(start, self.goal, now, evidence, clearance)
            if result.state == 'READY':
                return result
            self.goal = None
        if math.dist(start, destination) <= self.radius and clearance.volume_free(destination):
            result = self.planner.plan(start, destination, now, evidence, clearance)
            if result.state == 'READY':
                self.goal = destination
                return result

        r = self.planner.map.settings.resolution
        dimensions = 3 if self.planner.allow_vertical else 2
        origin = self.planner.map.cell_at(start)[:dimensions]
        def center(cell):
            values = tuple((c+.5)*r for c in cell)
            return values if dimensions == 3 else (*values,start[2])
        if not clearance.sweep_free(start, center(origin)):
            return PlanResult('HOLD', 'endpoint_connector_not_observed_free')
        offsets = ((1,0),(-1,0),(0,1),(0,-1)) if dimensions == 2 else (
            (1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1))
        frontier = [(0., origin)]
        costs = {origin:0.}
        visited = set()
        candidates = []
        while frontier and len(visited) < self.maximum_expansions:
            cost, cell = heapq.heappop(frontier)
            if cell in visited:
                continue
            visited.add(cell)
            point = center(cell)
            # A meaningful leg prevents selecting an almost stationary goal at
            # an obstacle face. Revisits are allowed but penalised for recovery.
            if math.dist(point,start) >= 1.:
                revisit = sum(max(0.,1.-math.dist(point,p)/1.5) for p in self.visited)
                score = (math.dist(point,destination) + .15*cost
                         + .3*abs(point[2]-destination[2]) + 3.*revisit)
                candidates.append((score, point))
            for offset in offsets:
                neighbor = tuple(a+b for a,b in zip(cell,offset))
                end = center(neighbor)
                if (neighbor in visited or math.dist(end,start) > self.radius
                        or not clearance.sweep_free(point,end)):
                    continue
                next_cost = cost+r
                if next_cost < costs.get(neighbor, math.inf):
                    costs[neighbor] = next_cost
                    heapq.heappush(frontier,(next_cost,neighbor))
        # Search established reachability, but the normal planner still checks
        # and constructs the actual issued polyline to the chosen endpoint.
        for _score, goal in sorted(candidates)[:8]:
            result = self.planner.plan(start, goal, now, evidence, clearance)
            if result.state == 'READY':
                self.goal = goal
                return result
        return PlanResult('HOLD','no_observed_reachable_goal',expanded=len(visited))
