# Opt-in depth navigation: milestone 2

## Scope and authority

This module is **not enabled by default**. The original heuristic remains the
existing default and is not changed. The new `avoidance_mode` controller parameter
supports `none`, `heuristic`, and `depth`. Depth mode is the only mode that reads
planner paths. Only the existing motion controller publishes
`/drone/command_position`; the mapping node owns Unity forward-camera commands.
The depth navigator publishes local goals and scan requests to the mapping node.

Depth mode is a **fixed-altitude, short-range prototype**. It intentionally HOLDs
on changes of altitude. It does not pick packages, choose delivery phases, handle
carried-package geometry or autonomously complete the two-warehouse course.

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
new approved plan arrives for 0.35 seconds, if the camera pose or physical course
telemetry stops, or if a course contact is reported, the controller stops
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

**Current unverified items:** Unity compile; actual vehicle geometry/clearance;
ROS catkin build; live depth planner/controller interaction; scanned bypass
feasibility; absence of physical contacts; full mission and carrying a package.
