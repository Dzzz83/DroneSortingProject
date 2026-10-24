# Unity branch handoff

Branch: `codex/two-warehouse-flight`
Base: `c572044` (`main` at the start of this work)
Verification recorded: 25 October 2026, Unity 6000.6.0f1 on Windows.

## Completed

- Added a separate two-warehouse scene with a static loading yard, parked forklift,
  pallet/cardboard assets, box obstacle, pickup pad, delivery pad and wider doorways.
- Added a local A-to-B flight that starts on Play: pickup with the existing gripper,
  carry around the obstacles, release inside B, confirm settling, then hover.
- Added physical reset and replay controls, attachment/delivery/contact observations,
  and an increasing reset sequence for repeated trials.
- Added three demonstration views: overview (1), drone follow (2), and yard (3).
  Only view 2 actively follows the drone; the other two are fixed views.
- Preserved the original `MainWarehouse` scene, ROS source, package mechanics and
  sensor formats. Fixed shared sensor filtering and same-frame package release.
- Added repeatable Unity editor checks, batch-mode entry points and this handoff.
  No package dependencies were added or changed.

## Run and replay

Open `Unity/` with **Unity 6000.6.0f1**, open
`Assets/Scenes/TwoWarehouseDelivery.unity`, and press **Play**. The local demo
needs no ROS server. In the Game view, use **Reset physical trial** to stop/reset,
**Start A to B flight** to replay, and keys **1**, **2**, **3** to change views.

To use ROS control, stop Play, disable `HardcodedWarehouseFlight` on the drone,
and enter Play again with the ROS endpoint running. The ROS mission still needs
coordinates for this scene. Full coordinates, topics and reset rules are in
[TWO_WAREHOUSE.md](TWO_WAREHOUSE.md).

## Changed files

Paths below are relative to the repository root. Each new Unity asset/script also
includes its `.meta` file so scene references survive a fresh checkout.

| File | Change and purpose |
| --- | --- |
| `README.md` | Links this branch's Unity demo and handoff from the repository entry point. |
| `Unity/Assets/Scenes/TwoWarehouseDelivery.unity` | New saved two-warehouse scene, fixed obstacles, loading openings, pads, markers, collision geometry, cameras and enabled local flight component. |
| `Unity/Assets/Scripts/Warehouse/HardcodedWarehouseFlight.cs` | New smooth waypoint sequence at 3 m/s, cruise height 3 m, pickup hover 8 cm above the settled package, physical pickup/release, settling wait, collision stop, cancellation and replay. Disables ROS movement/action handlers while owning control. |
| `Unity/Assets/Scripts/Warehouse/WarehouseCourse.cs` | New physical observations, swept box contacts, whole-package delivery confirmation, trial reset and HUD. Confirmation requires a previously picked-up package to be released and still inside B for 0.5 simulated seconds. |
| `Unity/Assets/Scripts/Warehouse/WarehouseCourseViews.cs` | New overview, follow and yard views; cutaway views hide roof renderers while retaining their colliders. |
| `Unity/Assets/Scripts/Packages/DeliveryPackage.cs` | Synchronizes transforms before restoring dynamic physics, so movement and release in one frame use the current mount pose. |
| `Unity/Assets/Scripts/ROS/DroneCommandSubscriber.cs` | Rejects non-finite position values, respects disabled local-control interlocks and records swept contacts before applying a ROS position. |
| `Unity/Assets/Scripts/ROS/DroneObstacleSensorPublisher.cs` | Ignores the drone's own descendants rather than every collider sharing its root; grouped sibling obstacles remain detectable. |
| `Unity/Assets/Scripts/ROS/PackageActionBridge.cs` | Adds reset/cancellation of pending actions and the command latch; ignores callbacks while disabled for local flight. |
| `Unity/Assets/Editor/TwoWarehouseCourseBuilder.cs` | New scene builder, saved-scene local-flight installer, presentation setup and carried-envelope clearance check. Refuses to overwrite an existing course when building. |
| `Unity/Assets/Editor/TwoWarehouseCourseVerification.cs` | New editor/batch checks for the real local flight and independent sensor/package/ROS-handler fixtures. The ordinary fixture disables automatic flight for its run. |
| `Unity/Assets/Materials/CourseAsphalt.mat`, `CourseBlue.mat`, `CourseCardboard.mat`, `CourseWall.mat`, `CourseYellow.mat` | New HDRP materials for the yard, pads, walls and box obstacle. All five are under the same `Materials/` directory. |
| `Unity/Assets/Settings/TwoWarehouseVolume.asset` | New scene-specific HDRP presentation profile with explicit motion-blur/depth-of-field overrides for clear view switches. |
| `Unity/TWO_WAREHOUSE.md` | Setup, controls, axes/coordinates, sensor geometry, observation topics, physical reset rules and verification instructions. |
| `Unity/Verification/hardcoded-flight-checks.txt` | Saved eight passing local-flight assertions. |
| `Unity/Verification/course-play-checks.txt` | Saved 26 passing course/sensor/package assertions. |
| `Unity/Verification/course-verification.txt` | Saved clearance result for a 3 x 2 x 2.8 m carried envelope on the fixture route. |
| `Unity/BRANCH_HANDOFF.md` | This file: completed work, file inventory, evidence, untested areas and branch limitations. |

## Tested and evidence

The latest flight and regression runs used an isolated local copy of the project
because the existing editor held the working project open. Unity compiled and
exited successfully for both test runs. The tested flight-enabled scene was
copied back after checking that the source scene had not changed in the meantime.

- **8 flight assertions passed:** enabled saved-scene flight, completed delivery,
  real pickup/carry, released and settled package inside B, zero obstacle contacts,
  blocked ROS position override, replay pickup, and reset while carrying.
  [Saved flight results](Verification/hardcoded-flight-checks.txt).
- **26 course assertions passed:** initial contacts, finite-command guard,
  common-root obstacle sensing, thin-wall crossing observation, pose/reset state,
  payload clearance, real gripper/action-handler pickup, every fixture route
  segment, release versus settled delivery, boundary rejection, reset during
  gripper closure, RGB/depth dimensions and nonempty measurements, and reset
  sequencing. [Saved course results](Verification/course-play-checks.txt).
- The larger carried-envelope bypass sweep had no reported intersections.
  [Saved clearance result](Verification/course-verification.txt).
- Overview, follow and yard captures were visually inspected during development.
  Generated images and full editor logs stay under ignored `Unity/Library/`.
- Whitespace checks pass for hand-written source, documentation and reports.
  Unity-generated scene/material/profile/metadata files retain Unity's serialized
  empty-field spacing. No runtime code changed during this documentation and
  publication step, so the completed Unity runs were not repeated for this push.

These are **34 assertions across two verification scenarios**, not 34 independent
missions or a statistical delivery-success benchmark. The flight fixture uses
4x simulation time to shorten the automated run.

To reproduce in Unity, stop Play and use **Drone Delivery > Verify Two Warehouse
Course**, **Run Two Warehouse Play Checks**, and **Verify Hardcoded A to B Flight**.
For batch mode, set `-projectPath` to `Unity/` and use `-executeMethod
TwoWarehouseCourseVerification.Begin` or `-executeMethod
TwoWarehouseCourseVerification.BeginFlight`. These methods open the scene in
batch mode, run their checks and exit with a success/failure code; omit `-quit`
while those Play checks run. New results are written to `Unity/Library/`.

## Not tested

- Live ROS-TCP transport, the real ROS mission, connection loss/reconnection,
  stale queued network commands, or switching control modes against a live ROS graph.
  Handler checks call Unity callbacks locally; they do not establish TCP integration.
- Heuristic avoidance, YOLO/AI avoidance, planner integration or algorithm comparison.
- Changed layouts, moved/dynamic obstacles, other package placements, sizes, masses,
  drone scales, speed settings or rotated delivery volumes.
- Long-duration runs, large trial batches, statistical success rates, resource/GPU
  profiling, inference latency or measured laptop performance.
- Standalone player builds, other operating systems, other Unity versions or a
  fresh-machine clone/build with no local caches. The original ROS demo was not
  flown end-to-end again after the shared fixes.
- Real flight hardware, aerodynamic behavior, wind, real sensors or hardware safety.

## Current limitations and risks

- This branch completes the **Unity simulation and fixed-route demonstration**.
  It does not complete the document's two avoidance systems or comparison results.
- The route and pickup/drop coordinates are hardcoded for this saved static scene.
  Moving obstacles or the pads can invalidate it. Local flight stops on a reported
  contact or a pickup/release/settling failure; it does not replan or recover.
- Movement is kinematic translation, with no yaw steering, realistic flight forces
  or sensor-driven navigation. It demonstrates delivery mechanics and clearance.
- The default scene runs local flight and disables ROS auto-connect for that mode.
  Stop Play and disable the component before starting ROS operation. ROS must own
  mission reset, discard stale commands and use the new scene coordinates.
- The existing ROS position interface applies absolute positions directly.
  Collision sweeps report crossings but do not block ROS movement. Contact state
  is latched per trial with the latest collider name, not a full collision-event log.
- Delivery containment uses the package's world-axis bounding box and an
  axis-aligned destination volume. Evaluation observations expose simulation
  ground truth and should not be used as the planner's perception input.
- Box collision envelopes, designated support exemptions and the seven existing
  distance probes approximate clearance; they do not prove every possible route safe.
- Reset restores this drone/package pair; it does not restore arbitrary manually
  moved scenery or reset ROS nodes. It is a single-package A-to-B demo.
- Only one camera view follows the drone; there is no orbit/chase-style selector
  beyond the implemented fixed-offset follow view.
- HDRP requires the matching Unity version and suitable graphics support. The new
  scene duplicates warehouse geometry and is relatively large; performance and
  rebaking/lighting quality on other machines have not been characterized.
