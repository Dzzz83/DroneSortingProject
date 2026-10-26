# Passive depth local planner — first milestone

This is **not autonomous avoidance**. The planner generates only proposed paths,
never movement commands. The existing wall-following heuristic, mission, package
YOLO detector and Unity control handlers are unchanged.

## Algorithm

- Uses the **same `LocalDepthMap` object** maintained by `DepthMappingNode`, not
  `PointCloud2` visualization data.
- A* on a fixed-height, four-connected XY grid at the map's voxel resolution.
- Every candidate cell and traversed segment is accepted only when **all voxels
  intersecting the full configured bounding envelope are currently `free`**.
- `occupied`, `unknown`, stale map/pose, unsupported altitude changes and missing
  goals result in `HOLD` with an explicit reason. No free-ray cell alone certifies
  clearance for the entire vehicle.
- Does not pan/scan the camera or attempt a detour through unobserved space.

## Geometry safety gate

`depth_mapping.yaml` deliberately leaves `planner_envelope` **unset**. The
planner therefore publishes `HOLD: envelope_not_configured` until the vehicle's
**full** ROS-coordinate bounding extents have been established in the Unity
runtime, including drone body, rotors, landing legs, gripper, carried package,
attachment offset and suitable clearance allowance. No flight-safe dimensions
are claimed here.

Set the private ROS parameter only after this geometry review:

```yaml
# ROS X/Y horizontal; Z vertical, all metres.
planner_envelope: [HALF_X, HALF_Y, BELOW_ROOT, ABOVE_ROOT]
planner_max_expansions: 4000
```

These are placeholders, **not** values to copy. The available scene builder
sets the main body collider to 2.6 × 0.3 × 2.3 m (Unity X/Y/Z), adds separate
landing-leg colliders, and the package prefab has a 0.6 m box. The pickup
mount offset is approximately 0.95 m, but the whole assembly and clearance
margin are not independently verified from those values.

## Running the passive prototype

Use the existing `depth_mapping.launch` (mapping + passive planner, **not** the
mission or motion controller). In another sourced Noetic terminal:

```bash
rostopic pub -1 /drone/local_planner/goal geometry_msgs/Point \
  '{x: 34.0, y: 52.0, z: 2.0}'
rostopic echo /drone/local_planner/status
```

ROS topics:

| Topic | Type | Meaning |
|---|---|---|
| `/drone/local_planner/goal` | `geometry_msgs/Point` | Short local goal, warehouse frame |
| `/drone/local_planner/path` | `nav_msgs/Path` | Candidate route (empty on HOLD) |
| `/drone/local_planner/status` | `std_msgs/String` JSON | READY/HOLD, reason, expanded nodes, waypoint count, reset sequence |

Goal and current pose must be at nearly the same height for this initial
fixed-altitude milestone. The current position comes from the **synchronized,
timestamped forward-camera drone pose** already used to build the map. Its age
is checked via mapping status; a separate position subscriber's zero-drift
behavior is **not** asserted.

Map resets invalidate the previous planner goal and return to HOLD. `ACTIVE`
mapping is not proof of a traversable route. Limited 8 m local windows and sparse
observations will frequently produce HOLD, which is intended.

## Focused verification

Run from the repository root in the sourced ROS environment:

```bash
PYTHONPATH="$PWD/ROS1/package/python:$PYTHONPATH" \
  /usr/bin/python3 -m unittest discover -s ROS1/tests -p 'test_local_planner.py' -v
PYTHONPATH="$PWD/ROS1/package/python:$PYTHONPATH" \
  /usr/bin/python3 -m unittest discover -s ROS1/tests -p 'test_live_local_planner.py' -v
```

Checks cover observed direct route, observed bypass, unknown bypass hold,
insufficient envelope clearance, sparse-ray conservatism, unsupported vertical
changes, path-only ROS publication, stale map and reset invalidation.

**Not verified in this handoff:** ROS/Unity end-to-end integration, live path
quality, collision-free flight, planner latency under full sensor load, package
handling during navigation, or repeated A-to-B missions.
