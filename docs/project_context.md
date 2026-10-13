# Project context

This is a handoff for continuing the warehouse drone project. Use the
[README](../README.md) for installation and supported runs,
[verification](verification.md) for test evidence, and [PLAN](PLAN.md) for the
next implementation milestone. Inspect current Git status and relevant code
before acting; this document is not a substitute for executable behavior.

## Purpose and current state

Simulate a drone transporting identical packages inside a warehouse, with a
future path to real hardware. ROS owns perception, target selection, mission
states, movement calculations, and obstacle avoidance. Unity owns simulation,
sensor rendering, applying commands, and gripper/package mechanics.

The fixed-coordinate warehouse demo remains available. A separate camera-guided
workflow now completes one delivery from `DroneStart` through observation hover.
It estimates pickup coordinates from RGB-depth observations, attaches and lifts the package,
carries it to the configured zone, and requests release. Baseline and shifted/
rotated package trials passed independent physical checks for attachment and final
settling inside `SortingZoneA`.

The Linux launcher now replaces the previous demo's ROS, YOLO and this project's
Unity editor, opens `MainWarehouse`, and waits for manual Play. The spectator
camera uses a wide aisle view independently of the RGB-depth sensor. Startup,
restart and viewing evidence is in [launcher verification](verification.md#launcher-and-spectator-verification);
operating commands and prerequisites remain in the [README](../README.md#camera-guided-single-package-delivery).

Build/test results and measured outcomes are in [verification](verification.md).
Automatic mission recovery and return-and-repeat are not implemented; warehouse
avoidance remains unverified. Live fault-injection results are recorded in the
full-workflow verification section. See the [current plan](PLAN.md) for the next work.

The sensor implementation was pushed on `camera_drone` as commit `30a980d`.
`main` was left unchanged. These are handoff facts, not instructions to switch
branches or assume the remote has not changed.

## Stack and repository structure

Unity `6000.6.0f1` uses C# and HDRP. ROS 1 Noetic uses Python in an Ubuntu 20.04
environment; the development machine runs ROS in the `ros-noetic` Distrobox
container. Unity ROS-TCP-Connector and ROS-TCP-Endpoint communicate on port 10000.

```text
ROS1/
  scripts/                         Build, sync, run, and mission helpers
  tools/                           Dataset, detector, and physical evaluation tools
  src/drone_sorting/
    launch/                        Mission and sensor-only launch files
    scripts/                       ROS node entry points and monitor
    src/drone_sorting/
      control/                     Movement controller
      interfaces/                  Shared ROS topic constants
      mission/                     Fixed-coordinate and camera-guided missions
      perception/                  RGB-depth validation, localization, and grasp fitting
      planning/                    Static obstacle avoidance
    tests/                         Perception/control tests and wire/live verifiers
Unity/Assets/
  Scenes/                          MainWarehouse, SampleScene, DroneGripperTest
  Prefabs/                         Drone and package prefabs
  Scripts/                         ROS bridges, camera capture, warehouse setup
  DroneDelivery/Runtime/           Gripper, attachment/release, standalone demo
  Editor/                          Scene integration and reusable verifiers
  Shaders/Resources/               Metric-depth shader
docs/                              Context, next plan, and verification
```

`MainWarehouse` is the primary scene; `SampleScene` is a smaller integration
demo. Other standalone/training workflows should not be assumed to use the
warehouse ROS control path.

## Module responsibilities

Paths below are relative to the repository root.

| Module | Responsibility |
| --- | --- |
| `ROS1/scripts/run_camera_delivery_demo.sh` | Coordinates fresh restart, dedicated workspace build, host detector, ROS readiness, scene opening and manual Play. |
| `ROS1/scripts/prepare_camera_demo_ros.sh` and `start_camera_demo_ros.sh` | Build and supervise ROS in native Noetic or Distrobox; keep container cleanup with the ROS environment. |
| `ROS1/tools/camera_demo_processes.py` | Identifies previous project demo processes and stops them using host PIDs and recorded scopes, preserving unrelated services. |
| `ROS1/src/drone_sorting/src/drone_sorting/mission/mission_manager.py` | Existing fixed pickup/drop workflow and action acknowledgement sequence. |
| `ROS1/src/drone_sorting/src/drone_sorting/mission/pickup_guidance.py` | Package locking, grasp refinement, bounded approach/descent, and acknowledgement-gated lift. |
| `ROS1/src/drone_sorting/src/drone_sorting/mission/delivery_guidance.py` | Extends pickup with configured carry/lowering, bounded release waiting, and settling hold. |
| `ROS1/src/drone_sorting/src/drone_sorting/mission/camera_pickup.py` | ROS adapter shared by pickup-only and delivery guidance; publishes targets, speed, actions, mission state, and trial status. |
| `ROS1/src/drone_sorting/src/drone_sorting/control/` | Calculates absolute position commands, normally at 20 Hz. |
| `ROS1/src/drone_sorting/src/drone_sorting/planning/` | Calculates static obstacle-avoidance actions when enabled. |
| `ROS1/src/drone_sorting/src/drone_sorting/perception/camera_geometry.py` | Decodes a depth pixel, deprojects it into camera coordinates, and transforms a point. Independent of ROS and detector libraries. |
| `ROS1/src/drone_sorting/src/drone_sorting/perception/camera_stream_check.py` | Read-only check of synchronized RGB, depth, calibration, and camera pose. Not a detector. |
| `ROS1/src/drone_sorting/src/drone_sorting/perception/package_grasp.py` | Upright 0.6 m square top-face fit; rejects incomplete views and is not general box fitting. Produces a grasp center only when size and coverage checks pass. |
| `Unity/Assets/Scripts/DroneCommandSubscriber.cs` | Applies ROS position commands to the kinematic drone transform. |
| `Unity/Assets/Scripts/DroneStatePublisher.cs` and `DroneObstacleSensorPublisher.cs` | Report position and ordered obstacle distances. |
| `Unity/Assets/Scripts/PackageActionBridge.cs` | Connects ROS pickup/drop requests to physical package handling and reports completion. |
| `Unity/Assets/Scripts/WarehouseRosSetup.cs` and `WarehouseRosRuntimeBootstrap.cs` | Configure the warehouse drone and disable the Unity-only `DeliveryDemo`. |
| `Unity/Assets/Scripts/WarehouseSpectatorCamera.cs` and `Unity/Assets/Editor/WarehouseSpectatorCameraVerification.cs` | Position the spectator independently of sensing; check clearance and render edit-mode previews. |
| `Unity/Assets/Scripts/DroneRgbCamera.cs` | Configures the mounted, on-demand sensor camera without replacing the spectator camera. |
| `Unity/Assets/Scripts/RgbDepthCapture.cs` and `RgbDepthFrame.cs` | Render/read aligned RGB-depth and retain capture pose/time; no package lookup or navigation. |
| `Unity/Assets/Shaders/Resources/DroneMetricDepth.shader` | Converts the HDRP depth buffer to optical-axis metres. |
| `Unity/Assets/Scripts/DroneCameraMessages.cs` and `DroneCameraPublisher.cs` | Format calibration/poses/transforms and publish synchronized observations. |
| `Unity/Assets/Editor/WarehousePackageDatasetPilot.cs` and `VisiblePackageId.shader` | Export editor-only RGB training data and visible package labels; restore scene without saving. |
| `Unity/Assets/Editor/CameraPickupTrial.cs` | Run temporary batch pickup/delivery trials and record physical attachment, motion, release, and zone evidence; ground truth stays in evaluation. |
| `ROS1/tools/record_camera_pickup.py` and `evaluate_camera_pickup.py` | Read-only ROS event capture and offline physical pickup/delivery checks; never publish control commands. |
| `ROS1/tools/` | Dataset checks, YOLO training/inference, and trial recording/evaluation; no ROS movement authority. |

Node entry points remain in `ROS1/src/drone_sorting/scripts/`; reusable Python
logic belongs in the package directories. Topic names must agree between
`ROS1/src/drone_sorting/src/drone_sorting/interfaces/ros_topics.py` and
`Unity/Assets/Scripts/DroneRosTopics.cs`.

## Existing mission and simulation constraints

The state sequence is `IDLE → GO_TO_PICKUP → WAIT_FOR_PICKUP → GO_TO_DROPOFF →
WAIT_FOR_DROP → COMPLETE`. Advance package stages only after Unity returns
`PICKUP_DONE` or `DROP_DONE`. The Unity camera publisher only supplies observations; ROS guidance selects
targets and preserves this public mission sequence.

`warehouse_demo.launch` uses ROS pickup `(34.23, 25.10, 1.39)`, drop-off
`(28, 55, 1.95)`, and arrival tolerance 0.05 m. The drone prefab's package mount
is 0.95 m below its root and its pickup radius is 0.22 m. These explain the
existing launch targets; they are not permission to feed exact package positions
into future perception.

Current movement is kinematic position-command simulation, not a realistic
flight controller, localization system, or dynamics model. Warehouse avoidance
is off by default pending validation; the README records earlier static
avoidance validation in `SampleScene`. LiDAR is not part of the new camera work.

`camera_pickup.launch` estimates the pickup target from RGB-depth observations
and ends after acknowledgement-gated lift. `camera_delivery.launch` reuses it and
adds ascent from `DroneStart`, travel to observation, and hover before visual pickup,
then continues through carry, lowering, release, and settling hold. Pickup-only
starts at the observation waypoint. Both enable dynamic speed and disable avoidance.
`DROP_DONE` confirms release; the offline physical checker separately verifies
final placement. Destination coordinates, heights, speeds, and timeouts are owned
by the [README](../README.md#camera-guided-single-package-delivery) and launch file.

## Agreed visual-search demo

- Three identical 0.6 m box packages start in the 5 × 5 m pickup area.
  Unity centers: `(34.23, 0.44, 25.10)`, `(32.70, 0.44, 23.70)`, and
  `(35.30, 0.44, 26.30)`. They can settle under physics; these initial positions
  are scene/test references only.
- Pickup boundaries are Unity X=31.5–36.5, Z=22.5–27.5, centered at X=34, Z=25.
  The observation waypoint is above that center. Root height 6 m has been tested
  for camera coverage and the configured delivery route; this does not establish
  clearance for other layouts or general warehouse navigation.
- ROS will know the pickup area and destination, but discover package locations
  from observations. No barcode or individual package identity is required.
- The intended mission starts at A, visits observation, finds and delivers one
  package, then returns until repeated valid area observations confirm emptiness.
  Three packages is an evaluation count, not a hardcoded stopping condition.
- Later delivery planning must choose separate safe release positions within
  the destination, rather than pile packages onto one coordinate. Exact release
  slots and search/recovery behavior remain to be designed.

## Sensor settings and contracts

The persistent child is `PackageRgbCamera`. Mount position in drone-local Unity
coordinates is `(0, -0.55, -0.60)`; Euler rotation `(55, 0, 0)` means a fixed
35-degree tilt from straight down toward the gripper. The model's front is -Z.
The mount is below the front sensor/gripper bars and above the landing rails.
This is a virtual camera mount, not a selected physical camera or modeled bracket.

Provisional simulation settings: 800 × 600, 110-degree **vertical** field of view,
0.05–30 m clipping planes, 0.2–15 m valid depth range, and 2 captures/second.
Clipping planes are not sensor operating limits. Realistic field of view,
minimum range, noise, calibration, payload, and clearance need hardware review.

RGB and depth come from the same render, with top-left array origin. Depth is
optical-axis distance in metres, not slanted ray length; invalid readings are
`NaN`. Opaque depth-writing geometry is supported; transparent surfaces without
depth writes are not. GPU readback is synchronous and can stall rendering.
Publishing uses queue size 1 without catch-up bursts and skips capture when
disconnected. Restart Play Mode after changing dimensions or depth limits.

| Topic | Message / meaning |
| --- | --- |
| `/drone/camera/rgb/image_raw` | `sensor_msgs/Image`, packed `rgb8` |
| `/drone/camera/depth/image_raw` | `sensor_msgs/Image`, `32FC1` optical-axis metres |
| `/drone/camera/camera_info` | `sensor_msgs/CameraInfo`, ideal pinhole intrinsics and zero distortion |
| `/drone/camera/pose` | `geometry_msgs/PoseStamped`, optical camera pose in `warehouse` |
| `/drone/current_pose` | `geometry_msgs/PoseStamped`, drone body pose in `warehouse` |
| `/tf` | `tf2_msgs/TFMessage`, `warehouse → drone_base → package_camera_optical` |
| `/drone/package_candidates` | `std_msgs/String` JSON v1, ROS-owned perception states, surface candidates, and accepted grasp centers |
| `/drone/command_speed` | `std_msgs/Float32`, optional metres/second; zero holds, ignored unless dynamic speed is enabled |
| `/drone/pickup_trial_status` | `std_msgs/String` JSON phase, reason, center, target, speed, and UTC stamp for camera pickup/delivery trials |

Messages in an observation share a sequence and UTC acquisition timestamp.
Use ROS wall time (`use_sim_time=false`); synchronize clocks on separate machines.
World positions retain Unity `(x,y,z)` → ROS `(x,z,y)`, with ROS Z vertical.
Body axes are forward, left, up; optical axes are right, down, forward. Rotation
conversion accounts for handedness using basis vectors, not quaternion swaps.
The existing `/drone/current_position` contract is unchanged.

The stream checker tests metadata and exercises geometry at one pixel. If its
center depth is missing, a synthetic 1 m distance is used only to check the math;
it is not a surface estimate. Passing that checker does not prove every pixel's
depth accuracy, spatial alignment, package visibility, or an empty pickup area.

## Continuing work

The detector runs in an isolated host Python 3.11 loopback HTTP worker; ROS Noetic
Python 3.8 owns synchronization, freshness, depth localization, and candidate output.
Only RGB goes to the worker. Operational perception never reads package transforms.
The surface estimate and demo-specific bounds are documented in the README.

Continue with failure recovery and repeated search in the [milestone plan](PLAN.md).
Check [verification](verification.md) for what actually ran and how to reproduce it. Keep changes small, readable,
and aligned with existing modules; explain each step and verify it before moving
on. Do not merge, push, change branches, or overwrite user work without a request.
