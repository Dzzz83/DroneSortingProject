# Passive depth local planner — first milestone

This is **not autonomous avoidance**. The planner generates only proposed paths,
never movement commands. The existing wall-following heuristic, mission, package
YOLO detector and Unity control handlers are unchanged.

## Algorithm

- Uses the **same `LocalDepthMap` object** maintained by `DepthMappingNode`, not
  `PointCloud2` visualization data.
- By default, A* on a fixed-height, four-connected XY grid at the map's voxel
  resolution. The opt-in navigation-only launch enables six-connected 3D search
  and reachable local-goal selection; see the [README](../README.md#navigation-only-a-to-b-demo).
- Every candidate cell and traversed segment is accepted only when **all voxels
  intersecting the full configured bounding envelope are currently `free`**.
  Segment checks cover each interval between envelope-face crossings of voxel
  planes, including short diagonal corner overlaps. Axis-aligned sweeps use a
  single volume query. Face-only contact follows the same boundary tolerance as
  the stationary-envelope diagnostics.
- `occupied`, `unknown`, stale map/pose, unsupported altitude changes and missing
  goals result in `HOLD` with an explicit reason. No free-ray cell alone certifies
  clearance for the entire vehicle.
- Does not pan/scan the camera or attempt a detour through unobserved space.

## Geometry safety gate

Without a configured `planner_envelope`, the planner publishes
`HOLD: envelope_not_configured`. The current working configuration contains the
measured unloaded extents plus provisional 0.25 m margins; carrying is outside the
navigation-only goal. Establish the vehicle's **full** ROS-coordinate bounding
extents in the Unity runtime, including body, rotors, landing legs and gripper,
with suitable clearance allowance. Any future carrying workflow also needs the
package and attachment offset measured. The provisional margins are not a general
clearance certification.

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
| `/drone/local_planner/goal` | `geometry_msgs/Point` | Short local goal by default; mission destination with `select_local_goals`, warehouse frame |
| `/drone/local_planner/path` | `nav_msgs/Path` | Candidate route (empty on HOLD) |
| `/drone/local_planner/status` | `std_msgs/String` JSON | READY/HOLD, reason, expanded nodes, waypoint count, reset sequence |

With `select_local_goals:=true`, status also carries `request_goal` (the mission
destination) and `goal` (the selected local endpoint). The autonomous follower
requires matching request/reset identity and a path ending at that endpoint.
Only observed, full-envelope reachable cells can supply intermediate goals;
recently completed goals are penalised to discourage cycling. Goal search uses
no course obstacle coordinates or package positions.

Goal and current pose must be at nearly the same height for this initial
fixed-altitude milestone. The current position comes from the **synchronized,
timestamped forward-camera drone pose** already used to build the map. Its age
is checked via mapping status; a separate position subscriber's zero-drift
behavior is **not** asserted.

Map resets invalidate the previous planner goal and return to HOLD. `ACTIVE`
mapping is not proof of a traversable route. Limited 8 m local windows and sparse
observations will frequently produce HOLD, which is intended.

### Diagnosing initial clearance

`/drone/local_planner/status` includes `start_clearance` whenever an envelope
and synchronized drone position are available, including during a stale-map HOLD.
Use the enclosing state, `observation_stamp`, `stamp` and `reset_sequence` to
interpret its age. Restart passive `depth_mapping.launch` with the updated source,
enter Play, then inspect one message:

```bash
rostopic echo -n 1 /drone/local_planner/status
```

The report reads the planner's own `LocalDepthMap`, using exactly the same
envelope/voxel intersection rule as planning. It includes the actual root position,
configured extents, map bounds (inclusive voxel indices), resolution, total/free/
occupied/unknown counts, out-of-bounds count and up to six failed-cell examples
per state. Example centres are ROS warehouse metres. Failed counts by front,
behind, left, right, above and below use voxel centres relative to the last
accepted camera pose; these groups overlap. Camera orientation is measured from
the observation, rather than inferred from the requested scan heading.

`retained_expired_unknown` is a subset of unknown cells whose expired records
are still present. Integration prunes old records, so zero does **not** prove
expiry never occurred. Reset clears the report's pose/camera context along with
the map. Diagnostics grant no movement approval.

The stationary yaw-only sensor has a confirmed vertical blind spot. Unity's
`ConfigureLens(320, 240, 90)` configures **vertical** FOV, and the camera mount
stays 0.5 m below the root. At root `(34, 50, 2)`, voxel `(136, 200, 10)` occupies
X `[34, 34.25]`, Y `[50, 50.25]`, Z `[2.5, 2.75]`. Its minimum vertical separation
from the camera is 1 m while its maximum horizontal separation is about 0.354 m.
No level camera yaw can see this voxel through a 90-degree vertical FOV.
It intersects the provisional unloaded envelope `[1.55, 1.40, 1.26, 0.772]`.

A synthetic four-yaw test with finite 4 m returns in every pixel, current evidence,
and the real sampling settings observes 1,404 of 1,680 envelope voxels; 276 remain
unknown, with no occupied or out-of-bounds envelope voxels. These are synthetic
counts, **not live Unity counts**. The geometric blind spot
persists even with denser sampling and arbitrary yaw coverage. Actual obstacles,
occlusion, missing finite returns and expiry may add further live failures.

A subsequent passive live check at root `(34, 50, 2)`, camera `(34, 50, 1.5)`
looking ROS +Y, reproduced the HOLD with **221 free, 0 occupied and 1,459 unknown**
envelope voxels. All 1,680 cells were within map bounds; acquisition age was about
0.09 s. All 840 behind-camera cells were unknown, as were 920 above-camera and
539 below-camera cells (overlapping groups). No retained expired records were
identified; pruned history cannot be diagnosed. This used a temporary isolated
passive planner's own in-memory map built from live synchronized observations,
not the original running node's map or visualization clouds. No movement or
camera-control outputs were connected. The original node independently reported
the same HOLD and ACTIVE mapping. These results confirm missing clearance
evidence, rather than occupied cells, map bounds or a stale acquisition, in this
check; they do not prove every unknown cell is physically clear.

The next sensing change should test pitched up/down observations alongside full
horizontal coverage, fused through the existing timestamped map while stationary.
Verify complete envelope coverage within the evidence lifetime before movement;
no-hit/invalid observations still cannot clear voxels. Full delivery additionally
requires altitude-capable navigation and a measured carrying envelope. Until those
changes are validated, preserve HOLD rather than bypassing initial clearance.

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
