# Opt-in depth navigation: milestone 2

## Scope and authority

This module is **not enabled by default**. The original heuristic remains the
existing default and is not changed. The new `avoidance_mode` controller parameter
supports `none`, `heuristic`, and `depth`. Depth mode is the only mode that reads
planner paths. Only the existing motion controller publishes
`/drone/command_position`; the mapping node owns Unity forward-camera commands.
The depth navigator publishes local goals and scan requests to the mapping node.

The manual `depth_motion.launch` experiment defaults to fixed-altitude,
short-range following and HOLDs on altitude changes. The opt-in
[navigation-only launch](../README.md#navigation-only-a-to-b-demo) enables
three-dimensional search and observed-space intermediate goals, and has completed
the static A-to-B course. Package pickup, delivery phases and carried-package
geometry are outside that workflow.

## Geometry gate — required before allowing movement

In the saved `TwoWarehouseDelivery` scene, enter Play and select **Drone Delivery >
Report Depth Planner Envelope**. Copy the console output. The audit measures
runtime bounds including active child colliders/renderers (main body, rotors,
landing legs, gripper). Record separately the **carried** envelope in a controlled
fixture before a later package-carrying milestone. No source currently verifies a
safe clearance margin for the full vehicle. This editor report is a measurement,
not an engineering clearance certification.

Review the environment, select an additional documented margin on EACH side,
and edit `ROS1/package/config/depth_mapping.yaml`:

```yaml
# Intentionally unset until the dimensions and margins are verified.
# planner_envelope: [ROS_HALF_X, ROS_HALF_Y, BELOW_ROOT, ABOVE_ROOT]
```

Never copy the placeholder literally. With no valid configured envelope, every
plan remains `HOLD: envelope_not_configured`, and the controller publishes no
motion. **Do not lower margins or classify unknown voxels as free to force a
route.** Existing 0.25 m cells are sparse samples, not full free-volume coverage.

## Focused ROS/Unity experiment (manual target, no packages)

1. Back up or commit any intentional local work first; do not enable local
   `HardcodedWarehouseFlight`. Build the existing sourced Noetic workspace.
2. From a ROS Noetic shell, start the dedicated mapping/controller experiment:

   ```bash
   source /opt/ros/noetic/setup.bash
   cd ROS1/build/linux-camera-workspace
   catkin_make -j2 -l2
   source devel/setup.bash
   roslaunch drone_sorting depth_motion.launch
   ```

3. Open the saved Unity `TwoWarehouseDelivery` scene and press Play. The fixed
   obstacle must be visible to the forward camera. Observe ROS local map and
   planner status before sending a goal.
4. In a second sourced Noetic shell, send a **same-height, nearby** test target
   at 5 Hz (example only — use a point checked against the scene):

   ```bash
   rostopic pub -r 5 /drone/target_position geometry_msgs/Point \
     '{x: 34.0, y: 53.0, z: 2.0}'
   ```

5. Inspect `/drone/local_planner/status`, `/drone/local_map/status` and
   `/drone/course/observations`. Stop target publication and end Play to finish.
   No package inference, pickup or mission node is started.

Manual targets automatically expire after 0.5 seconds. Depth mode is limited to
0.2 m/s in this experimental launch and preserves 0.25 m command lead. If no
new approved plan arrives for 0.35 seconds, if its camera acquisition is older
than 0.5 seconds, if physical course telemetry stops, or if a course contact is reported, the controller stops
publishing new position commands. Unity keeps its last applied kinematic pose.
The module does not claim physical collision prevention: Unity reports swept
contacts but does not block commands.

## Local planning and ROS topics

- Input mission target `/drone/target_position` to motion controller.
- Controller publishes `/drone/local_planner/goal` (short same-height lookahead)
  and `/drone/local_planner/scan_heading` (camera heading request to mapping node).
- Mapping node publishes `/drone/local_planner/path` (`nav_msgs/Path`) and
  `/drone/local_planner/status` (JSON) with goal, observation acquisition time,
  reset sequence and plan stamp.
- Controller permits steps strictly along certified path segments, rejects
  mismatched goal, reset, stale messages and unexpected path joins, and sends
  movement to Unity on `/drone/command_position`.
- If a bypass remains unknown, scanning may add observations but must never
  override the path clearance checks.

The sampling radius is local (the map window is 8 m by default), so the controller
requests a ~3 m intermediate goal instead of the distant delivery point. The
path search may still hold indefinitely in narrow, occluded or sparsely observed
space. That behavior is safer than guessing a route.

### Freshness timing and the ROS queue fix

The follower distinguishes `route_plan_stale` (the 0.35 s plan limit) from
`route_observation_stale` (the ROS navigator's 0.5 s acquisition limit).
`/drone/local_planner/status` includes `timing_ms`: camera-to-synchronized-bundle
age, callback lock wait, pending wait, integration, snapshot, planning, diagnostics,
and plan/observation ages immediately before publication. Stage durations use a
monotonic clock; acquisition and plan stamps retain the ROS/UTC clock.

Camera callbacks now use a separate, short inbox lock, so they can replace pending
frames while the worker updates the map and plans. Planning reads current evidence
directly without constructing sorted visualization records. The plan stamp marks
the map check **after** integration; the camera acquisition stamp is unchanged.
The publisher checks acquisition freshness again after planning and builds both
messages before publishing path and approval together. Slow work cannot refresh
an expired observation or publish a READY route from it.

A live stationary comparison measured median callback lock wait falling from
197 ms to approximately zero and snapshot time from 120 ms to 20 ms. Median
observation age at approval receipt fell from 674 ms to 204 ms, with a 280 ms
95th percentile in the corrected 10 s sample (80 READY messages). Freshness limits
were unchanged. During a subsequent 18 s movement test, sampled controller logs
contained no freshness rejections and Unity reported no contacts. The drone still
oscillated and reported `command_not_on_approved_path`; the short target was **not
reached**. The separate path-following issue was subsequently reproduced and
corrected as described below.

Pausing only the mapper while Unity feedback and target publication remained live
produced `route_plan_stale`, with no commands after 0.45 s. The mapper was resumed
and the test controller stopped. Regression tests also check expiry during slow
integration/planning, acquisition stamps, occupied/free precedence and resets.

### Path continuity and arrival fix

A subsequent 18 s ROS/Unity recording reproduced 151 backward Y commands among
312 movement commands. Each replacement path began at the camera's observed pose,
then returned to the current grid-cell centre before proceeding forward. In 18
replacement paths, the follower also encountered a nearby early segment before a
later segment containing the command exactly, incorrectly rejecting the join.

The planner now removes initial/final grid-centre stops when a replacement sweep
passes the same full-envelope clearance check. While following a route it retains
the remaining turns, trims completed segments using the observed pose, and checks
all remaining sweeps against current evidence on every approval. Changed goals,
off-route poses or failed clearance trigger a new search. Cached geometry grants
no permission to move through newly occupied, unknown or expired evidence.
The follower checks later exact segment matches before rejecting a join, without
allowing sideways motion onto a nearby route.

A separate regression reproduced final arrival being blocked by clearing approval
every tick inside the intermediate-goal 0.15 m handoff radius. Only intermediate
goals now hand off early; the final goal retains approval to its endpoint. New
local goals publish immediately, and a final target inside lookahead is kept
exactly rather than reconstructed through floating-point interpolation.

With the same 0.2 m/s speed, six-direction sensing, opt-in declared clear-range
evidence and six-second static-course evidence lifetime, the corrected first
flight reached ROS `(34, 51, 3)` in approximately six seconds. Two additional
flights, `(34, 51, 3)` to `(34, 50, 3)` and back, used a 0.5 m local lookahead to
exercise intermediate handoffs and reached their endpoints in approximately seven
seconds each. Both finished with zero reported position error, no backward Y
commands and no contacts (73 and 72 course observations respectively). One
handoff briefly rejected a path from lagging feedback, then resumed by the next
sampled controller log 0.5 s later. Holding for an unapproved join remains intended.
Pausing the mapper again produced stale-plan holds and no commands after 0.45 s;
the 0.35 s plan and 0.5 s acquisition limits remain unchanged.

Focused regressions cover the recorded segment-selection failure, backward
connectors, current-evidence revalidation, delayed float32 feedback through a turn,
final arrival and immediate intermediate-goal publication. These checks establish
nearby target following, not full-course navigation or obstacle-detour feasibility.
The complete sourced ROS Noetic suite passed all 186 tests after the fix.

### Navigation-only integration

The navigation-only command and parameters are documented in the
[README](../README.md#navigation-only-a-to-b-demo). It adds a destination coordinator
and opts into observed-space goal selection; the original manual-target and
heuristic workflows remain separate. A summed-volume table checks full-envelope
clearance with the same voxel-boundary rule as the diagnostic enumeration.

The first forklift test stopped on a reported contact even though the rendered
depth map approved the route. The course's forklift collision proxy is a solid
bounding box with no renderer, extending into apparently empty rendered space.
Forward depth now samples actual Unity physics surfaces on the same calibrated
eight-pixel grid, leaving unsampled pixels NaN and excluding triggers and the
drone's own colliders. It changes neither the obstacles nor the envelope margins.
The `Verify Obstacle Depth` editor fixture passed for a collision-only surface,
optical-axis distance, unsampled NaNs and trigger exclusion.

After this correction, a 1 m/s requested-speed run from the doorway to ROS
`(34, 85, 3)` autonomously cleared the forklift, descended and arrived with zero
reported error in 70.9 seconds. None of 630 course observations reported contact.
The depth and map freshness gates remained active throughout.

Two subsequent complete runs reset the physical scene at A `(34,50,3)` and
arrived at B `(34,119.5,3)` with zero reported final position error:

| Run | Elapsed | Course observations | Reported contacts | Observed forklift detour |
| --- | --- | --- | --- | --- |
| First | 186.6 s | 1,663 | 0 | Left, reaching ROS X=27.625 |
| Repeat | 206.1 s | 2,009 | 0 | Right, reaching ROS X=40.625 |

Both used requested speed 1 m/s, the unchanged unloaded envelope, six-second
static evidence history and unchanged 0.35/0.5 s freshness limits. The repeat
briefly suspended only the mapper: no command arrived more than 0.45 s after
suspension, and normal flight recovered after the mapper resumed. Final position
and contact reports came from Unity, not synthetic acknowledgements. ROS had
exactly one movement publisher (`motion_controller`), one mission-target publisher
(`navigation_demo`) and no package-action publisher. No route or obstacle
coordinates were supplied to goal selection.

These runs verify the saved static A-to-B navigation-only demo. They do not
establish general exploration, dynamic-obstacle handling or carried-package
clearance. The complete sourced ROS suite passed 197 tests after integration.

### Higher-speed and spectator verification

The course spectator now supports an adjustable follow offset, orbit/translation,
free flight from overview/yard, preset selection and F to reset follow. The focused
`Verify Course Spectator Controls` editor fixture passed after compilation, and
manual follow/free controls were confirmed in Unity. Preset roof culling remains
on the spectator camera; sensor cameras and the original warehouse spectator
are unchanged.

Two subsequent full A-to-B runs tested requested speed **2 m/s** with the same
clearance, plan age, acquisition age and 0.25 m command-lead limits:

| Run | First ARRIVED elapsed | Final error | Contact observations |
| --- | --- | --- | --- |
| First | 152.1 s | 0 m | 0 of 1,494 |
| Repeat | 151.4 s | 0 m | 0 of 1,360 |

Both departed from a physical A reset and completed actual sensor-selected
detours, entry into B and sustained final arrival. Holds for fresh observation,
feedback and local-goal handoff remained active. These successful repeated runs
selected 2 m/s as the default; the conditional 1.5 m/s fallback was unnecessary.
The navigation launcher provides `--help` and rejects a second launch while ROS
is already running. Camera compilation, the focused Unity fixture, manual input,
launch XML parsing, shell syntax, help output and duplicate-start refusal were
checked. Python navigation/control code is unchanged from the 197-test result above.

## Verification (non-flight)

```bash
PYTHONPATH="$PWD/ROS1/package/python:$PYTHONPATH" \
  python3 -m unittest discover -s ROS1/tests -p 'test_depth_follower.py' -v
PYTHONPATH="$PWD/ROS1/package/python:$PYTHONPATH" \
  python3 -m unittest discover -s ROS1/tests -p 'test_depth_navigation.py' -v
```

Passing these tests does not establish real Unity/ROS feasibility. Following
installation, run existing ROS regression tests under the sourced Noetic distrobox,
then perform the short live experiment. Measure the real envelope first.

**Remaining limitations:** the unloaded envelope uses provisional clearance
margins; dynamic obstacles, other course layouts and carried-package avoidance
remain unverified. Package handling is outside the current navigation-only goal.

## Opt-in smooth navigation transport (2026-10)

The navigation demo uses `/drone/approved_trajectory` (String JSON). ROS remains
the only movement-command publisher. Each command authorizes one short, ordered
segment, constrained by a 0.25 m outstanding-motion limit and by the original
0.35 s plan / 0.5 s observation freshness. Unity interpolates within approved
segments, never directly across waypoints. Wrong-reset, expired, malformed,
out-of-order, excess-buffer and contact conditions stop execution. The ROS
controller sends explicit STOP on planning holds and waits for a matching
`/drone/trajectory_status` acknowledgement before anchoring a new chain; reset
clears both ends. Unity continues providing
actual applied-position feedback. Legacy `/drone/command_position` is unchanged
for the non-navigation demos. Full contract and limitations are in
`docs/navigation-smoothness.md`.

Baseline from earlier manual runs: 152.1 s and 151.4 s at requested 2 m/s,
zero reported contact. The review run reached B without reported contact in
184.0 s, exposing near-expiry approval churn. After the scheduling fix, a 10 m
confirmation reached its target without local expiry/rejection stops. Full-course
timing at that stage remained unverified. The follow-up below provides the later
full-course result; see `docs/navigation-smoothness.md`.

Follow-up: optimized map processing, route-relative pose validation, queued waypoint
completion, 50 Hz navigation approvals and feedback after movement passed 224 tests.
The next full ROS–Unity run arrived without reported contacts in 112.5 s, versus
184.0 s before these changes. Safety waits remain; continuous 2 m/s and completely
smooth rendering are not established. Measurements: `docs/navigation-smoothness.md`.
