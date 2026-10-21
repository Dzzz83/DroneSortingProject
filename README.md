# Drone Sorting Project

Unity + ROS 1 simulation for an autonomous drone that transports lightweight packages between sorting zones in a warehouse.

ROS is the main control system. Unity simulates the warehouse, drone, sensors, package handling, and rendering.

For a detailed handoff, see [project context](docs/project_context.md),
the [next implementation plan](docs/PLAN.md), and [verification results and checks](docs/verification.md).

## Table of Contents

- [Project Overview](#project-overview)
- [Project Structure](#project-structure)
- [Main Components](#main-components)
  - [ROS Components](#ros-components)
  - [Unity Components](#unity-components)
- [Current Data Flow](#current-data-flow)
- [Workflow](#workflow)
- [Installation](#installation)
  - [Requirements](#requirements)
  - [Linux](#linux)
  - [Windows](#windows)
- [Current Status](#current-status)

## Project Overview

The project separates **decision-making** from **simulation**.

**ROS 1** is responsible for:

- Managing mission states and pickup/drop-off targets
- Processing RGB-depth observations, drone position, and obstacle sensor data
- Calculating movement commands
- Deciding obstacle-avoidance actions
- Requesting package pickup and drop

**Unity** is responsible for:

- Simulating the warehouse, drone, obstacles, and package
- Applying movement commands received from ROS
- Reporting the drone's current position and aligned RGB-depth camera observations
- Measuring obstacle distances using Unity physics
- Simulating the gripper and physical package attachment/release
- Rendering the simulation

ROS and Unity communicate through Unity ROS-TCP-Connector and ROS-TCP-Endpoint.

## Project Structure

```text
DroneSortingProject/
├── docs/                          # Context, next plan, and verification
├── ROS1/
│   ├── scripts/
│   │   ├── build_ros.sh
│   │   ├── run_delivery_demo.sh
│   │   ├── run_warehouse_ros_demo.sh
│   │   ├── run_camera_delivery_demo.sh
│   │   ├── prepare_camera_demo_ros.sh
│   │   ├── start_camera_demo_ros.sh
│   │   ├── sync_ros_workspace.sh
│   │   ├── stop_demo.sh
│   │   ├── watch_obstacles.sh
│   │   ├── test_mission.sh
│   │   └── test_full_mission.sh
│   │
│   ├── tools/                     # Restart lifecycle, RGB inference, and physical evaluation
│   └── src/drone_sorting/
│       ├── launch/
│       │   ├── demo.launch
│       │   └── package_perception.launch
│       ├── scripts/
│       │   ├── camera_stream_check_node.py
│       │   ├── package_perception_node.py
│       │   ├── camera_pickup_node.py
│       │   ├── delivery_demo_monitor.py
│       │   ├── mission_manager_node.py
│       │   └── motion_controller_node.py
│       └── src/drone_sorting/
│           ├── control/
│           │   └── motion_controller.py
│           ├── interfaces/
│           │   └── ros_topics.py
│           ├── mission/
│           │   ├── camera_pickup.py
│           │   ├── pickup_guidance.py
│           │   ├── delivery_guidance.py
│           │   ├── mission_config.py
│           │   ├── mission_manager.py
│           │   └── mission_state.py
│           ├── perception/        # Camera validation and package surface localization
│           └── planning/
│               └── obstacle_avoidance.py
│
└── Unity/
    ├── Assets/
    │   ├── DroneDelivery/Runtime/
    │   │   ├── DeliveryDemo.cs
    │   │   ├── DeliveryPackage.cs
    │   │   ├── DeliveryRotor.cs
    │   │   ├── GripperController.cs
    │   │   └── PackageHandling.cs
    │   ├── Editor/
    │   │   ├── CameraPickupTrial.cs
    │   │   ├── DeliveryDemoSceneLauncher.cs
    │   │   ├── WarehouseDroneAutoRunOnce.cs
    │   │   ├── WarehouseSpectatorCameraVerification.cs
    │   │   └── WarehouseDroneIntegrator.cs
    │   ├── Prefabs/
    │   │   ├── Drone.prefab
    │   │   └── Package.prefab
    │   ├── Scenes/
    │   │   ├── SampleScene.unity
    │   │   ├── MainWarehouse.unity
    │   │   └── DroneGripperTest.unity
    │   └── Scripts/
    │       ├── DroneCommandSubscriber.cs
    │       ├── DroneObstacleSensorPublisher.cs
    │       ├── DroneRosTopics.cs
    │       ├── DroneStatePublisher.cs
    │       ├── PackageActionBridge.cs
    │       ├── WarehouseRosRuntimeBootstrap.cs
    │       ├── WarehouseSpectatorCamera.cs
    │       └── WarehouseRosSetup.cs
    ├── Packages/
    └── ProjectSettings/
```

This tree highlights the delivery workflow. The complete sensor module map and
responsibilities are in [project context](docs/project_context.md#module-responsibilities).

## Main Components

### ROS Components

#### Mission Manager

`mission_manager.py` controls the fixed-coordinate delivery demo. The camera
workflow uses `camera_pickup.py` with `PickupGuidance` and, for delivery,
`DeliveryGuidance`. The delivery workflows publish the same public mission sequence:

```text
IDLE
  ↓
GO_TO_PICKUP
  ↓
WAIT_FOR_PICKUP
  ↓
GO_TO_DROPOFF
  ↓
WAIT_FOR_DROP
  ↓
COMPLETE
```

The active mission node selects targets, checks arrival using Unity position
feedback, requests `PICKUP` or `DROP`, and waits for Unity acknowledgements. Camera
guidance estimates the pickup target from RGB-depth observations; delivery uses a
configured destination. Run one mission node at a time.

`mission_config.py` contains default mission values. Scene-specific launch files can override pickup/drop-off coordinates and arrival tolerance.

#### Motion Controller

`motion_controller.py` is the main ROS movement controller. It receives:

- Mission target position
- Current drone position from Unity
- Obstacle distances from Unity
- Command speed when dynamic-speed mode is enabled

It calculates the next commanded position and publishes it to Unity. When
obstacle avoidance is enabled, it uses the Obstacle Avoidance Planner before
producing the next command. Camera launches enable dynamic speed and feedback
timeouts; fixed-coordinate launches retain fixed-speed mode.

#### Obstacle Avoidance Planner

`obstacle_avoidance.py` contains the ROS-side static obstacle-avoidance logic.

Planner modes:

```text
NORMAL
CLIMB
DESCEND
PASS
SIDESTEP_LEFT
SIDESTEP_RIGHT
```

Possible movement decisions:

```text
FORWARD
FORWARD_LEVEL
UP
DOWN
LEFT
RIGHT
STOP
```

Unity only measures obstacle distances. ROS decides how the drone reacts.

#### ROS Launch Files

- `demo.launch` starts camera-guided pickup and the motion controller, including `package_perception.launch` for the endpoint and RGB-depth perception. Its defaults run pickup-only from observation, matching the supplied configuration.
- `run_camera_delivery_demo.sh` invokes `demo.launch enable_delivery:=true start_from_drone_start:=true` for full delivery from `DroneStart`.
- `package_perception.launch` can also run read-only perception separately.

The older fixed-coordinate, camera wrapper, and camera-validation launch files have been removed. Historical verification records refer to those earlier launch names.
Warehouse obstacle avoidance is disabled by default until the warehouse sensor readings are validated.

#### Helper Scripts

| Script | Purpose |
|---|---|
| `sync_ros_workspace.sh` | Copies the repository ROS package into the Catkin workspace |
| `build_ros.sh` | Syncs and builds the ROS workspace |
| `run_delivery_demo.sh` | Retired; exits with the camera runner command |
| `run_warehouse_ros_demo.sh` | Retired; exits with the camera runner command |
| `setup_camera_demo_linux.sh` | Installs pinned detector dependencies and prepares the camera ROS workspace |
| `prepare_camera_demo_unity.sh` | Imports Unity alone with bounded memory and pressure monitoring |
| `run_camera_delivery_demo.sh` | Restarts the camera demo, prepares the workspace, waits for Unity startup to settle, then starts YOLO/ROS for manual Play |
| `prepare_camera_demo_ros.sh` | Links and incrementally builds a dedicated Catkin workspace without replacing the existing package |
| `start_camera_demo_ros.sh` | Starts ROS, checks node/endpoint readiness, monitors the mission, and cleans up its ROS processes |
| `watch_obstacles.sh` | Displays Unity obstacle sensor readings |
| `stop_demo.sh` | Legacy fixed-demo cleanup; use the camera launcher’s Ctrl+C for scoped cleanup |
| `test_mission.sh` / `test_full_mission.sh` | Mission testing helpers |

### Unity Components

#### DroneCommandSubscriber.cs

Subscribes to `/drone/command_position` and applies the position calculated by ROS to the Unity drone.

Unity executes the movement; it does not choose the movement direction.

#### DroneStatePublisher.cs

Publishes the drone's Unity position to `/drone/current_position`.

Coordinate conversion:

```text
Unity (x, y, z)
        ↓
ROS   (x, z, y)
```

Unity `y` is vertical, while ROS `z` is vertical.

#### DroneObstacleSensorPublisher.cs

Uses Unity physics to measure obstacle distances and publishes:

```text
[
  front,
  upper_front,
  lower_front,
  left,
  right,
  up,
  down
]
```

A value of `-1.0` means no obstacle was detected within the configured sensor range.

#### PackageActionBridge.cs

Connects ROS package commands to the Unity package system.

```text
ROS PICKUP / DROP
        ↓
PackageActionBridge
        ↓
PackageHandling
        ↓
PICKUP_DONE / DROP_DONE
        ↓
ROS
```

It automatically installs itself in `SampleScene`. In `MainWarehouse`, it is installed through the warehouse ROS setup.

#### Drone Delivery Package System

The physical package system is under `Unity/Assets/DroneDelivery/Runtime/`:

- `PackageHandling.cs` — detects, picks up, carries, and drops packages
- `GripperController.cs` — controls the gripper jaws
- `DeliveryPackage.cs` — manages package attachment and release
- `DeliveryRotor.cs` — rotor animation
- `DeliveryDemo.cs` — standalone Unity-only package demo

`DeliveryDemo.cs` is disabled during the ROS-controlled warehouse mission so it cannot move the drone independently of ROS.

#### DroneRosTopics.cs

Stores the Unity-side ROS topic names. The matching ROS constants are defined in `interfaces/ros_topics.py`.

#### Warehouse ROS Setup

`WarehouseRosSetup.cs` contains the shared setup for the warehouse drone:

- Adds the ROS command, state, sensor, and package bridge components
- Keeps the drone Rigidbody kinematic for ROS-controlled movement
- Disables `DeliveryDemo`

`WarehouseRosRuntimeBootstrap.cs` applies this setup when `MainWarehouse` enters Play Mode.

The setup also configures the mounted RGB-depth camera, its ROS publisher, and
the independent wide spectator view through `WarehouseSpectatorCamera.cs`.
Camera modules, settings, and message contracts are documented in
[project context](docs/project_context.md#sensor-settings-and-contracts).

#### Unity Editor Tools

- `DeliveryDemoSceneLauncher.cs` opens `SampleScene` or `MainWarehouse` from helper scripts.
- `WarehouseDroneIntegrator.cs` places the M2 drone/package system in the warehouse and applies the shared ROS setup.
- `WarehouseDroneAutoRunOnce.cs` supports the warehouse editor integration workflow.
- `CameraPickupTrial.cs` runs batch pickup/delivery trials and records physical evidence; temporary package poses and ground truth stay in evaluation.
- `WarehouseSpectatorCameraVerification.cs` checks camera clearance and renders preview views without entering Play or saving scene changes.

## Current Data Flow

### Movement and sensing

```text
ROS mission node
      │
      │ /drone/target_position
      ▼
Motion Controller
      ▲
      │ /drone/current_position
      │ /drone/obstacle_distances
      │
    Unity
      ▲
      │ /drone/command_position
      │
Motion Controller
```

ROS chooses the target and calculates the next movement command. Unity applies that command, then sends updated position and sensor measurements back to ROS.

### Package handling

```text
ROS mission node
      │
      │ /drone/package_action
      │ PICKUP / DROP
      ▼
PackageActionBridge
      │
      ▼
PackageHandling / Gripper
      │
      │ /drone/package_action_status
      │ PICKUP_DONE / DROP_DONE
      ▼
ROS mission node
```

### Main ROS Topics

| Topic | Purpose |
|---|---|
| `/drone/target_position` | Current target selected by the active ROS mission node |
| `/drone/current_position` | Current drone position reported by Unity |
| `/drone/obstacle_distances` | Obstacle measurements reported by Unity |
| `/drone/command_position` | Next movement position calculated by ROS |
| `/drone/travel_heading` | Horizontal travel direction used by Unity sensors |
| `/drone/mission_state` | Current delivery mission state |
| `/drone/package_action` | `PICKUP` or `DROP` request from ROS |
| `/drone/package_action_status` | `PICKUP_DONE` or `DROP_DONE` confirmation from Unity |
| `/drone/package_candidates` | RGB-depth perception states, surface candidates, and accepted grasp centers |
| `/drone/command_speed` | Optional dynamic speed in m/s; zero holds |
| `/drone/pickup_trial_status` | Camera pickup/delivery phase, reason, target, speed, and timestamp |

RGB-depth images, calibration, poses, and transforms are published separately;
see [sensor topics and coordinate contracts](docs/project_context.md#sensor-settings-and-contracts).

## Workflow

Choose the fixed-coordinate warehouse launch, camera pickup-only launch, or camera
delivery launch. Each uses one ROS mission node and the same Unity package bridge.
The pickup-only workflow ends at step 7; steps 8–11 apply to delivery launches.

1. The ROS mission node selects a pickup target. Camera guidance first requires
   consistent RGB-depth observations from the observation waypoint. Camera delivery
   first ascends from `DroneStart`, travels there, and holds for one second.
2. Unity reports position, obstacle measurements, and enabled camera observations.
3. Motion Controller calculates the next position and sends it to Unity.
4. Camera guidance approaches, refines alignment, and descends; the fixed-coordinate
   mission moves to its configured pickup target.
5. The mission node requests `PICKUP` when the pickup target is reached.
6. Unity attaches the package and returns `PICKUP_DONE` before lift or delivery begins.
7. Camera pickup-only guidance lifts and holds. Camera delivery enters
   `GO_TO_DROPOFF` after lift, then ascends to carry height; the fixed-coordinate
   mission selects its drop-off target after acknowledgement.
8. ROS controls travel to the destination. Camera delivery lowers vertically there.
9. The mission node requests `DROP` at the release position.
10. Unity releases the package and returns `DROP_DONE`.
11. The mission node publishes `COMPLETE`; camera delivery first holds for settling.
    Independent physical evaluation checks the final package placement.

## Installation

### Requirements

- Unity Editor `6000.6.0f1`
- ROS 1 Noetic
- Ubuntu 20.04 ROS environment
- Git
- Catkin
- Unity ROS-TCP-Connector
- ROS-TCP-Endpoint

ROS-TCP-Endpoint:

```text
https://github.com/Unity-Technologies/ROS-TCP-Endpoint
```

### Linux

#### Linux camera demo setup

For **x86-64 Linux**. Run commands in your host terminal, outside the ROS container.

**Install first:**

- Unity Editor **6000.6.0f1**, activated, with a working graphics driver.
- Python **3.11** with `venv`/`pip`, plus the host's `python3`.
- Git and **Git LFS**.
- ROS Noetic on Ubuntu 20.04, or [prepare a ROS Distrobox container](docs/linux-camera-demo.md#ros-environment).

The host also needs OpenCV's GL/GLib libraries and standard shell utilities;
see [additional prerequisites](docs/linux-camera-demo.md#additional-prerequisites).
Startup was tested on a 16 GiB laptop; lower-memory machines remain unverified.

**1. Get the project**

For a new clone:

```bash
git lfs install
git clone --branch feat/linux-portable-setup https://github.com/Dzzz83/DroneSortingProject.git
cd DroneSortingProject
git lfs pull
```

If you already have the project, save local changes, then run from its root:

```bash
git fetch origin
git switch feat/linux-portable-setup
git pull --ff-only
git lfs install
git lfs pull
```

**2. Set up once**

```bash
bash ROS1/scripts/setup_camera_demo_linux.sh
```

Wait for `[OK] Setup complete`. Setup creates the Python venv and prepares the
ROS workspace automatically. The trained model is included. Rerun setup when
dependencies change; stop the demo first.

**3. Run**

Save Unity edits first: launching closes this project's previous editor.
From the repository root:

```bash
bash ROS1/scripts/run_camera_delivery_demo.sh
```

Wait for **`[READY]`**, then press **Play** in Unity. Leave the terminal open.
The first startup took about **8.5 minutes** in our test; a cached restart took
about **40 seconds**. Keep `Unity/Library` to avoid repeating the full import.

Press **Ctrl+C** in the terminal to stop ROS and YOLO. Unity stays open.
For later runs, use the same launch command.

**Common issues**

- **`git: 'lfs' is not a git command`:** install the Git LFS package, then retry.
- **Python 3.11 missing:** install it or [select its executable](docs/linux-camera-demo.md#python-path).
- **Unity not found:** [set the editor path](docs/linux-camera-demo.md#unity-editor).
- **Missing camera data before Play:** expected. If it continues during Play,
  check Unity's Console and [the run logs](docs/linux-camera-demo.md#logs-and-troubleshooting).
- **Memory failure:** close unused applications and check [memory guidance](docs/linux-camera-demo.md#memory-and-startup).

[GPU and advanced options](docs/linux-camera-demo.md) ·
[Tested behavior and limitations](docs/verification.md#fresh-clone-startup-after-the-memory-failure)

#### Fixed-coordinate warehouse setup

The fixed-coordinate launch files and runners are retired. Use the
[Linux camera demo setup](#linux-camera-demo-setup) and
`bash ROS1/scripts/run_camera_delivery_demo.sh`.

### Windows

The previous fixed-coordinate WSL runner is retired. The supported camera runner
currently targets native Linux with ROS Noetic or Distrobox; a Windows/WSL camera
workflow has not been validated.

## Current Status

### Planned visual-search demo

`MainWarehouse` contains three identical packages in the pickup area. The intended
mission will locate packages from RGB-depth observations, deliver them one at a
time, and return until repeated valid observations confirm the area is empty.
Scene geometry and agreed decisions are in
[project context](docs/project_context.md#agreed-visual-search-demo).

The mounted RGB-depth camera and live ROS sensor pipeline are implemented and
verified. Simulated lens/range settings are provisional, and sensor verification
does not establish flight safety or real-hardware readiness. See the
[sensor contracts](docs/project_context.md#sensor-settings-and-contracts) and
[completed checks](docs/verification.md).

RGB package detection uses an initial YOLO11 nano checkpoint. ROS estimates an
upright package's grasp center from synchronized RGB-depth observations, then
flies from `DroneStart` through observation hover, pickup, lift, carry, release,
and settling hold. Baseline and changed-placement/rotation batch trials passed
independent physical evaluation inside `SortingZoneA`; a manual GUI run also
confirmed final placement. The latest ROS unit suite passed 69 tests. See
[full-workflow evidence](docs/verification.md#full-workflow-from-dronestart).

The Linux camera launcher now restarts the previous demo's ROS, YOLO, and this
project's Unity editor, opens `MainWarehouse`, and waits for manual Play. Fresh
startup and replacement of an active run were tested. The wide spectator view
passed rendered preview checks and a manual Play review. See the
[run instructions](#camera-guided-single-package-delivery) and
[launcher and spectator evidence](docs/verification.md#launcher-and-spectator-verification).

Five live fault trials verified holding without stage advancement under camera
or detector loss, stale mission position feedback, and missing pickup/drop
acknowledgements. Automatic mission recovery and return-and-repeat delivery are
not implemented; warehouse avoidance remains unverified. Pickup-only retains
its observation start, and the
fixed-coordinate warehouse launch remains available. See the
[milestone status and remaining work](docs/PLAN.md).

A small editor-only dataset pilot is available through **Drone Delivery → Export
Package Dataset Pilot**. It exports 16 RGB views, YOLO labels for class `0: package`,
visibility masks, and box previews, using temporary scene changes without saving.
This verifies the capture/label process; it is not a full training or evaluation
dataset. See [pilot reproduction and checks](docs/verification.md#package-dataset-pilot).

**Drone Delivery → Export Package Training Trial** extends this to 200 RGB
images split by package configuration (140 training, 30 validation, 30 test).
Offline training/evaluation tools are under `ROS1/tools/`. The first trained
checkpoint is included in `ROS1/models/`; training outputs and measured reports
remain local artifacts. See
[training trial results and reproduction](docs/verification.md#package-detector-training-trial).

### Stationary package perception

Start the RGB-only worker in the isolated Python 3.11 detector environment. Local
single-class `package` weights are required; startup never downloads a model.
Run Linux setup first to create the ignored detector environment. The checkpoint
is tracked in Git:

```bash
ROS1/build/detector-venv/bin/python ROS1/tools/package_detector_service.py \
  --weights ROS1/models/package_detector.pt \
  --device cpu --confidence 0.25 --image-size 640
```

In a built/sourced ROS workspace containing this package and ROS-TCP-Endpoint, run
`roslaunch drone_sorting package_perception.launch`. It starts the endpoint on
10000 and connects to the loopback worker on 10001 (`detector_port` launch argument).
It starts no mission or movement controller. Start MainWarehouse Play Mode with
the drone positioned for observation; this launch does not fly it there. Do not
start a delivery launch concurrently. Keep Unity focused as required for capture.

`/drone/package_candidates` is `std_msgs/String` containing JSON with schema version
1, acquisition stamp/sequence, `warehouse` frame, state, candidates, and rejection
reasons. States are `DETECTIONS`, `NO_DETECTIONS`, `UNRELIABLE`, and `UNAVAILABLE`.
Candidates contain confidence, image bounding box/pixel, supported sample count,
and a position in ROS `(x,y,z)` metres, equivalent to Unity `(x,z,y)`.
The `position` is an observed **visible surface**, not a package center. A separate
`grasp` result reports whether an upright-box center estimate passed its checks;
camera guidance uses that accepted center for alignment.

Four sensor messages are synchronized exactly; identity, geometry, and freshness
are checked before and after inference. The latest observation replaces pending
work, with a 1.5 second age limit. Missing data or a failed worker never reports
an empty area. `NO_DETECTIONS` alone also does not prove area emptiness.
Depth filtering uses the demo's known pickup bounds (ROS X 31.5–36.5, Y 22.5–27.5)
and surface heights Z 0.2–1.2 m. These private node parameters are `pickup_bounds`
and `surface_height_range`; the assumptions suit this simulation's 0.6 m packages.
Inset depth sampling is not general object segmentation.

See [live trial evidence and reproduction](docs/verification.md#stationary-live-package-perception).

### Camera-guided pickup: baseline verified

`demo.launch` with its default arguments connects RGB-depth package perception, a one-package
guidance node, and the existing motion controller. It expects the robot to begin
at the tested observation point (ROS `(34, 25, 6)`, Unity `(34, 6, 25)`) in a clear,
controlled test area. It moves toward the grasp center derived from an upright
0.6 m square box top, descends, requests one pickup, and lifts after Unity confirms
`PICKUP_DONE`. It ends after lift. With `enable_delivery:=true start_from_drone_start:=true`,
`demo.launch` adds access from `DroneStart` and continues to the destination.

Trial speeds default to 0.5 m/s approach, 0.2 m/s descent, and 0.3 m/s lift,
with 0.03 m arrival tolerance. The guidance node's private parameters configure
these values; positive speeds must stay within the controller's 0.5 m/s cap.
Zero `/drone/command_speed` means hold by sending no further position commands.
The trial controller waits for a speed input and stops sending commands if position,
target, or speed feedback is older than 0.5 seconds. The retired fixed-coordinate launch used fixed speed at 2 m/s.

One baseline run passed with ROS Noetic and batch Unity, using the same trained
checkpoint and an additional RGB inference view (`--context-scale 1.5` on the
detector worker). This view pads the image to make close packages smaller during
inference, then maps detections back to the original sensor pixels. The normal
inference pass is retained; depth/calibration and the 0.25 confidence threshold
are unchanged. The option defaults to 1 (no additional pass). The pickup-only
result is recorded in
[baseline evidence](docs/verification.md#automated-baseline-pickup-and-lift); later
full-delivery results are described below.

An editor-only physical evaluator, read-only ROS recorder, and offline evidence
checker record and judge each trial. They require the selected package to
attach and rise with the drone after `PICKUP_DONE`; `VERIFIED` alone cannot pass.
See [evaluator checks and trial preparation](docs/verification.md#physical-pickup-evaluator)
for recording, batch Unity startup, and evaluation commands.

### Camera-guided single-package delivery

The camera runner enables full delivery in `demo.launch`, starting from `DroneStart` (Unity `(34, 2, 10)`,
ROS `(34, 10, 2)`). It ascends vertically to root height 6 m, travels to the
observation point (Unity `(34, 6, 25)`, ROS `(34, 25, 6)`), and holds within the
arrival tolerance for one continuous second. It ignores package candidates during
access and requires new images acquired after that hover before selecting a package.
ROS remains the sole movement authority. It then reuses camera pickup and continues
after lift: ascend vertically, carry above the configured destination, lower, request release, wait
for `DROP_DONE`, then hold for settling. Baseline and changed-placement/rotation
batch Unity deliveries from `DroneStart` passed independent physical evaluation.
See [full-workflow evidence](docs/verification.md#full-workflow-from-dronestart).
Pickup coordinates come from RGB-depth observations; the observation waypoint,
destination, and travel heights are configured.

For setup and launch commands, follow [Linux camera demo setup](#linux-camera-demo-setup).
[Advanced options](docs/linux-camera-demo.md) cover GPU setup, manual Unity mode,
resource settings, logs, and spectator-camera checks.

On mission failure, the drone holds and ROS stays available until Ctrl+C.
ROS also remains connected after successful completion for inspection.

Use the same detector with `--context-scale 1.5` and read-only recorder as the
pickup trial. Set `DRONE_PICKUP_TRIAL_MISSION=delivery` when starting
`CameraPickupTrial.Run`; the delivery capture limit is 330 s. Delivery trials now
initialize at `DroneStart`.
To reproduce earlier observation-only deliveries, pair launch argument
`start_from_drone_start:=false` with harness environment
`DRONE_PICKUP_TRIAL_START=observation`. Pickup-only trials remain unchanged. Use a
fresh output for every run and confirm previous control publishers have stopped. The
[trial procedure and delivery evidence](docs/verification.md#camera-guided-single-package-delivery)
cover historical batch startup, recording, and evaluation. Use `demo.launch`
with both delivery and start arguments enabled for current full-delivery trials;
its defaults still end after pickup and lift.

Scene-specific parameters are in `demo.launch`. ROS X/Y are horizontal; ROS Z is vertical, and heights refer to drone root:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `observation_x`, `observation_y`, `observation_height` | 34, 25, 6 m | Observation hover target |
| `access_speed`, `access_timeout` | 0.5 m/s, 60 s | Speed and timeout for each access stage |
| `hover_time` | 1 s | Continuous arrival tolerance before visual selection |
| `dropoff_x`, `dropoff_y` | 38, 55 m | Destination center (Unity X/Z) |
| `carry_height` | 6 m | Vertical ascent and horizontal transport height |
| `release_height` | 1.4 m | Root height for release, accounting for the 0.95 m mount offset |
| `carry_speed`, `lowering_speed` | 0.5, 0.2 m/s | Transport and destination descent |
| `travel_timeout`, `drop_timeout` | 120, 5 s | Each transport stage and release acknowledgement wait |
| `settle_time` | 2 s | Hold after acknowledgement before `COMPLETE` |

Delivery uses the existing public mission sequence through `GO_TO_DROPOFF`,
`WAIT_FOR_DROP`, and `COMPLETE`, with ROS as the sole movement authority.
`DROP_DONE` confirms release; `COMPLETE` alone does not prove successful placement.
The physical checker requires continuous attachment during carry, ordered
acknowledgements, final detachment, and the whole package footprint settling
inside the zone. It also checks sampled collider penetration and hold drift.
Avoidance remains disabled. The earlier destination at ROS `(28, 55)` was checked against active
scene colliders, not established as a general navigation solution. The current
destination at ROS `(38, 55)` has not been validated by those earlier runs.

### Working

- ROS ↔ Unity communication
- ROS delivery mission state machine
- ROS-controlled drone movement
- Unity position feedback
- Unity obstacle sensing
- Mounted RGB-depth sensing and synchronized ROS image/calibration/pose publishing
- Sensor-only ROS validation, camera geometry tests, and live camera transform checks
- Offline package dataset export and initial YOLO11 nano fine-tuning/evaluation
- Stationary ROS package candidates from synchronized RGB-depth observations
- Camera-guided one-package delivery from `DroneStart`, with batch physical evaluation and manual GUI placement confirmation
- One-command Linux startup with scoped fresh restart, readiness checks, monitoring, and per-run logs
- Wide warehouse spectator view, with preview and manual Play checks
- Physical gripper and package attachment/release
- Fixed-coordinate `MainWarehouse` delivery with ROS controlling the mission and movement
- Static obstacle avoidance validated in `SampleScene`
- Climb, descend, pass, and side-step avoidance logic

### Next

- Define recovery after missing acknowledgements or observation/feedback loss
- Extend fault validation to other timing, combined failures, and broader layouts
- Implement return-and-repeat search with reliable empty-area confirmation and separate release positions
- Validate obstacle sensor readings in `MainWarehouse`
- Enable and tune static obstacle avoidance in `MainWarehouse`
- Validate broader sensing conditions and dynamic obstacles
