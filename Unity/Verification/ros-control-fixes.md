# ROS control fixes — 25 October 2026

Changes are based on Unity branch commit `51acba5`, tested in an isolated Linux
worktree with Unity 6000.6.0f1 and ROS Noetic in Distrobox. The original main
checkout, forklift and course layout were preserved.

## Changed behavior

- The saved scene disables `HardcodedWarehouseFlight`. ROS movement and package
  handlers remain enabled; the drone waits at its saved start without commands.
- Roof renderers remain available to sensors. Only the spectator camera's
  culling mask changes for cutaway views. Roof colliders remain active.
- `run_demo.sh --course` selects TwoWarehouseDelivery and course.yaml together.
  The original command still selects MainWarehouse and demo.yaml. Setup and
  operating instructions are maintained in the repository README.

## Verification

- Full Python suite under sourced ROS Noetic: 116 tests passed, no skips.
- Host Python: 116 tests, 44 ROS-dependent tests skipped, remaining tests passed.
- `bash -n` passed for all three changed shell scripts.
- Unity compiled and passed 38 course assertions, including physical pickup,
  carried clearance, release/settling, ROS-default waiting, and RGB/depth roof
  visibility across all three spectator views. The synthetic roof probe measured
  2.899993 m against an expected 2.9 m.
- Explicit offline local flight passed eight assertions, including delivery,
  replay, reset and zero reported obstacle contacts. Its fixture uses 4x simulation
  time and does not alter the scene's saved ROS default.
- `roslaunch --dump-params` resolved the new course height, delivery destination
  and pickup bounds correctly.
- Final sources compiled after probe cleanup. `DemoLauncher.Prepare` with
  `-demoScene TwoWarehouseDelivery` opened and validated the selected saved scene.
- A live ROS-TCP smoke run completed a known bypass at 1 m/s using absolute ROS
  position commands at 20 Hz. Unity's local flight remained disabled. ROS received
  `PICKUP_DONE` and `DROP_DONE`; the package settled inside B at Unity
  `(34,0.44,119.5)`, released, with no reported obstacle contacts. RGB8 and metric
  32FC1 depth each delivered 243 frames, and seven-distance feedback was received.
  This was an external ROS waypoint fixture, not the autonomous mission or an
  avoidance algorithm.

Saved evidence: [course assertions](ros-course-play-checks.txt),
[optional local-flight assertions](ros-local-flight-checks.txt), and
[live ROS outcome](ros-live-smoke.json). Temporary live probe scripts were removed.

These 46 Unity assertions span two fixture scenarios, not 46 independent missions.
Detailed editor logs, images and reports remain under ignored `Unity/Library/`.

## Limits

Autonomous heuristic/AI navigation, the full detector-guided mission on this
course, loss/reconnection behavior and performance under continuous inference
load are not established by these checks. The course profile is a starting
configuration for avoidance integration.

Git reports 11 inherited LFS texture differences in this worktree. Direct Git
blob-hash comparisons confirmed all 11 files are byte-identical to the branch
commit; no texture edits are part of these fixes.
