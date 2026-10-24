# Two-warehouse Unity handoff

For the complete changed-file inventory, completed work, test evidence and known
limits, see [the branch handoff](BRANCH_HANDOFF.md).

Open this folder (`Unity/`) with Unity **6000.6.0f1**, then open
`Assets/Scenes/TwoWarehouseDelivery.unity`. The original `MainWarehouse` scene
is preserved. Press Play to run the local hardcoded delivery described below.
For ROS control, disable the local flight component before Play; ROS then owns
movement, pickup/drop commands, navigation, mission state and comparison metrics.
The old one-warehouse ROS mission coordinates must be updated by the avoidance team
before they can run a mission on this new layout.

## Hardcoded flight demo

The saved scene now runs a local **A to B delivery automatically when Play starts**.
It ascends, aligns the real package mount, closes the gripper, flies the fixed
bypass at 3 m/s and Unity Y=3 m, lowers the package in B, releases it, waits for
settled delivery, then ascends and hovers. No ROS server is required in this mode.
The HUD shows flight status. Eight automatic-flight checks passed in Unity,
including zero obstacle contacts, replay and reset while carrying. Use **Reset physical trial** to stop and reset, and
**Start A to B flight** to replay. Keys 1, 2 and 3 still select camera views.

The enabled `HardcodedWarehouseFlight` component owns movement and package actions;
ROS command callbacks are blocked while it owns control. For ROS-driven operation,
stop Play, uncheck this component on the drone, and enter Play again. The local
mode is a demonstration of the saved static layout, not an avoidance algorithm.
Select **Drone Delivery > Verify Hardcoded A to B Flight** for the actual smooth
flight, pickup, delivery, restart and reset checks. The ordinary course checks
temporarily disable the automatic flight so their fixtures stay independent.

## Scene and controls

The scene reuses the warehouse structure, shelving, lighting, drone, gripper,
package, camera and ROS components. Warehouse B faces A across a stationary
loading yard. The yard uses the included parked forklift and pallet/cardboard
assets, plus a tall box obstacle. Doorways are 6 m wide and 4.5 m high. The new
scene replaces the original narrow shutters/perimeter collision mesh and opens
an interior passage in each warehouse. Each new visible wall has matching
collision geometry; original retained structure uses mesh colliders.

In the focused Game view: **1** shows the overall cutaway, **2** follows the drone,
and **3** shows the loading yard. Cutaway views hide upper roof renderers while
retaining their colliders. The HUD reports attachment, settled delivery and
whether any obstacle contact has occurred since reset. Its reset button restores
the saved drone/package poses and cancels pending gripper actions.

| Marker | Unity (X, height, Z), metres | ROS (X, Y, height), metres |
| --- | --- | --- |
| DroneStart | (34, 2, 50) | (34, 50, 2) |
| PickupPoint, package centre | (34, 0.45, 55) | (34, 55, 0.45) |
| WarehouseAExit | (34, 3, 67.1) | (34, 67.1, 3) |
| WarehouseBEntrance | (34, 3, 107.4) | (34, 107.4, 3) |
| DeliveryPoint, package centre | (34, 0.45, 119.5) | (34, 119.5, 0.45) |

Pickup/drop commands control the existing package handler. The package centre
marker is not the drone command position: account for its package mount offset
when positioning the drone. Both pads are 5 x 5 m. Their tops are at Unity Y=0.14.
The observation volume covers X=[31.5,36.5], Z=[117,122], Y=[0.1,1.7].

## Observations and reset

Existing position, RGB, metric depth, calibration, pose, transform, seven-distance
sensor and package-action topics keep their names and formats. Distance values
are front, upper-front, lower-front, left, right, up, down at 10 Hz. Probes
use a 0.35 m sphere radius, 0.8 m vertical offsets, a 5 m range, and -1 for no hit.
The existing calibrated RGB-depth camera remains 800 x 600 at 2 Hz, with a
110-degree vertical field of view and a 0.2-15 m metric-depth range. The distance
sensor now ignores the drone's own descendants rather than every object sharing
its root, so grouped warehouse obstacles are still sensed.

`/drone/course/observations` publishes `std_msgs/String` JSON at 10 Hz:

```json
{"reset_sequence":0,"package_attached":false,"delivered":false,"collision_observed":false,"contact":"","package_position":{"x":34,"y":55,"z":0.45}}
```

`package_position` uses ROS axes. `collision_observed` latches until reset and
`contact` identifies the last contacted collider; ROS determines counting and
failure rules. Drone contacts and non-support package contacts are observed.
Package contact with the designated pad/floor supports is permitted for settling.

`delivered` requires this same package to have been picked up, released, completely
inside B's observation volume, and still for at least 0.5 simulated seconds.
Linear speed must be below 0.05 m/s and angular speed below 0.1 rad/s. A release
acknowledgement alone never establishes successful placement. This is simulation
ground truth for evaluation, not perception input for the ROS planner.

To reset, pause ROS movement and package-action publishers, send the exact string
`RESET` on `/drone/course/reset` (`std_msgs/String`), and wait for a larger
`reset_sequence`. Resume with a fresh ROS mission. The reset restores the drone
and package poses, zeros package velocities, clears outcome flags, opens the
gripper, cancels any pending pickup and clears the package bridge's action latch.
ROS must discard stale mission commands; Unity does not reset ROS nodes or metrics.

The existing position subscriber applies absolute ROS positions directly. Box
sweeps report collisions crossed between commands; they do not stop or reroute
the drone. A body/rotor box and separate landing-leg boxes leave the package mount
empty, preserving physical release. The package retains its own collider. Release
synchronizes transforms before enabling dynamic physics, so movement and drop
received in the same frame use the current mount pose.

## Repeatable verification

Stop Play and select **Drone Delivery > Verify Two Warehouse Course** for the
static carried-envelope sweep. **Run Two Warehouse Play Checks** runs an offline
editor fixture through the same ROS position/action handlers, tests pickup,
bypass, settling, boundary rejection, reset during gripper closure, common-root
sensing, non-finite command rejection, thin-wall crossings and RGB-depth capture.
The fixture temporarily disables network connection for that Play session.
It is editor-only and is never an operational local planner.

Reports and 1280 x 720 view captures are written under `Unity/Library/`:
`course-verification.txt`, `course-play-checks.txt`, `course-view-0.png` (overview),
`course-view-1.png` (follow), and `course-view-2.png` (yard).

Verification on 25 October 2026: all 26 Play checks passed, including a physical
release that settled at Unity (34, 0.44, 119.5). All three views were captured
and visually reviewed. No additional packages or dependencies were added.

The clearance fixture uses a 3 x 2 x 2.8 m carried envelope and a route via Unity
positions (34,3,55), (34,3,71), (25,3,71), (25,3,87), (43,3,87), (43,3,103),
(34,3,103), (34,3,119.5). This proves that a spacious bypass exists, not that a
ROS avoidance algorithm can find it. Run each algorithm with the same saved
scene and reset conditions, and retain failed attempts in the ROS comparison.

Live TCP integration, heuristic/AI navigation, algorithm comparisons and laptop
inference/resource measurements require the avoidance team's ROS setup. Those
are not implemented by the Unity changes. No hardware-flight claim is implied.
