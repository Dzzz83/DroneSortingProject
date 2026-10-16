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
│       │   ├── camera_validation.launch
│       │   ├── package_perception.launch
│       │   ├── camera_pickup.launch
│       │   ├── camera_delivery.launch
│       │   ├── drone_demo.launch
│       │   └── warehouse_demo.launch
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

- `drone_demo.launch` starts ROS-TCP-Endpoint, Mission Manager, and Motion Controller.
- `warehouse_demo.launch` reuses the generic launch file with `MainWarehouse` pickup/drop-off coordinates and warehouse-specific settings.
- `package_perception.launch` starts the endpoint and read-only RGB-depth package perception.
- `camera_pickup.launch` adds camera-guided pickup and lift with slower dynamic-speed control.
- `camera_delivery.launch` adds flight from `DroneStart` to observation, then pickup, carry, release, and settling hold.
- `camera_validation.launch` starts only the endpoint and read-only camera checker, without a mission or motion controller. See [sensor verification](docs/verification.md#repeat-the-live-sensor-only-check).

Warehouse obstacle avoidance is disabled by default until the warehouse sensor readings are validated.

#### Helper Scripts

| Script | Purpose |
|---|---|
| `sync_ros_workspace.sh` | Copies the repository ROS package into the Catkin workspace |
| `build_ros.sh` | Syncs and builds the ROS workspace |
| `run_delivery_demo.sh` | Runs the smaller `SampleScene` integration demo |
| `run_warehouse_ros_demo.sh` | Runs the fixed-coordinate `MainWarehouse` delivery on Linux or WSL2 |
| `setup_camera_demo_linux.sh` | Installs pinned detector dependencies and prepares the camera ROS workspace |
| `prepare_camera_demo_unity.sh` | Imports Unity alone with bounded memory and pressure monitoring |
| `run_camera_delivery_demo.sh` | Restarts the camera demo, prepares ROS/YOLO, opens `MainWarehouse`, and waits for manual Play |
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

ROS Noetic can run directly on Ubuntu 20.04 or inside an Ubuntu 20.04 Distrobox container.

Clone the repository and create the Catkin workspace:

```bash
git clone https://github.com/Dzzz83/DroneSortingProject.git
cd DroneSortingProject

mkdir -p ~/drone_sorting_ros1_ws/src
```

Install ROS-TCP-Endpoint:

```bash
cd ~/drone_sorting_ros1_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

The helper scripts detect the repository location automatically, so the project does not need to be stored at `~/DroneSortingProject`.

Return to the repository root and build the ROS workspace. In the sequence above,
`cd -` returns from the Catkin workspace to the cloned repository:

```bash
cd -
bash ROS1/scripts/build_ros.sh
```

Configure Unity:

```text
Robotics → ROS Settings

Protocol: ROS1
IP:       127.0.0.1
Port:     10000

Edit → Project Settings → Player
Active Input Handling: Both
```

For the RGB-depth/YOLO workflow, use the Linux camera setup below. The trained
model is included in Git; setup installs dependencies without training.

#### Linux camera demo setup

The initial portability target is **x86-64 host Linux**, Unity `6000.6.0f1`,
Python **3.11** with `venv`/`pip`, Git/Git LFS, and either native ROS Noetic or
Distrobox with an Ubuntu 20.04 Noetic container. The host also needs a working
Unity graphics environment and OpenCV's `libGL`/GLib libraries. First-time Unity
import is separate from normal startup; the bounded import helper requires a
systemd user manager, Linux memory pressure reporting, and initially 7 GiB of
available RAM. This is a provisional test budget, not a measured hardware
requirement. Install these prerequisites through your distribution; setup does not install system packages
or Unity. Ubuntu 20.04's default Python 3.8 is for ROS; install Python 3.11
separately on the host for the detector. Set `DETECTOR_SETUP_PYTHON` if its binary
is not named `python3.11`.

For Distrobox, create the environment once (requires Podman or Docker):

```bash
distrobox create --name ros-noetic --image docker.io/library/ros:noetic-ros-base-focal
distrobox enter --name ros-noetic
sudo apt-get update
sudo apt-get install -y build-essential ros-noetic-catkin ros-noetic-message-generation \
  ros-noetic-message-filters ros-noetic-geometry-msgs ros-noetic-sensor-msgs \
  ros-noetic-std-msgs ros-noetic-tf2-msgs
exit
```

Native ROS users install the same ROS/build packages in their Ubuntu 20.04
Noetic environment. ROS Noetic is end-of-life; the existing ROS 1 architecture
is retained here. The container image tag and operating-system packages are not
locked snapshots. See [ROS Noetic status](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160).

From a fresh clone, after installing Git LFS:

```bash
git lfs install
git clone https://github.com/Dzzz83/DroneSortingProject.git
cd DroneSortingProject
git lfs pull
bash ROS1/scripts/setup_camera_demo_linux.sh
bash ROS1/scripts/prepare_camera_demo_unity.sh
bash ROS1/scripts/run_camera_delivery_demo.sh
```

Until the portability branch is merged, check out `feat/linux-portable-setup`
before running setup. Wait for `READY`, then press **Play** in Unity. Setup
fetches a pinned ROS-TCP-Endpoint, creates a Python 3.11 detector environment,
checks the included model, performs CPU inference, and builds a dedicated Catkin
workspace. It supports repeated runs and refuses to replace unrelated workspace
packages or change an existing Endpoint checkout. Stop the launcher before
rerunning setup. Internet access is needed for the first dependency installation.
Generated dependencies stay under ignored `ROS1/build/`.

Prepare Unity once before the first automatic demo launch. The helper starts only
Unity in batch mode, imports assets sequentially, compiles scripts, opens the
warehouse scene, and exits. It never starts ROS or YOLO or closes another editor.
It uses a systemd scope with `MemoryHigh=4G`, `MemoryMax=5G`, and
`MemorySwapMax=512M`. It samples host available RAM and memory pressure each
second, stopping after five consecutive checks below 2 GiB available RAM or at
least 10% full-memory-stall pressure (`avg10`), or after 600 seconds. The scope also has an independent systemd runtime limit. Set
`UNITY_IMPORT_TIMEOUT` from 1 to 600 seconds to shorten this limit. Logs, scope
memory samples, and pressure readings stay under `ROS1/build/unity-import-*/`.
These limits may stop an import before it finishes; they are not a guarantee
against disruption. The controlled real import stopped on memory-stall pressure
before scene readiness; see [measured results](docs/verification.md#controlled-import-result).

On hosts without a systemd user manager, import the project separately through
Unity Hub; the helper's supervision is unavailable there. Cache detection uses
`Library/ArtifactDB`, `Library/SourceAssetDB`, and the compiled project assembly.
Their presence suggests an earlier import but does not guarantee that assets are
current. If the import helper has run, its latest run must also have a `COMPLETE`
marker, written only after successful scene opening; interrupted imports stay
blocked even when all three cache files exist. The normal launcher refuses a
missing/incomplete cache before stopping project processes. `UNITY_MODE=manual` can still test ROS/YOLO without opening
Unity. After an interrupted import, rerun preparation before trying delivery.

CPU inference is the default. For an NVIDIA driver compatible with CUDA 12.4:

```bash
bash ROS1/scripts/setup_camera_demo_linux.sh --gpu
DETECTOR_DEVICE=0 bash ROS1/scripts/run_camera_delivery_demo.sh
```

The CPU and CUDA builds use PyTorch 2.5.1 and torchvision 0.20.1; the remaining
host Python dependencies are pinned in `ROS1/requirements/detector.txt`.
The Unity ROS connector and ROS Endpoint use fixed Git revisions. CPU inference
speed depends on the machine; observations exceeding the existing 1.5 s freshness
limit are rejected, so successful setup alone does not prove delivery performance.
Model provenance and upstream licensing are in [the model notes](ROS1/models/README.md).

The launcher detects standard Unity Hub paths for the required version. For a
custom installation or an editor you want to open yourself:

```bash
UNITY_EDITOR=/path/to/Editor/Unity bash ROS1/scripts/run_camera_delivery_demo.sh
UNITY_MODE=manual bash ROS1/scripts/run_camera_delivery_demo.sh
```

Manual mode preserves your open editor. Open the repository's `Unity` directory,
select `MainWarehouse`, configure ROS at `127.0.0.1:10000`, and press Play after
ROS is ready. Automatic mode closes this project's previous editor; save edits
before launching. Missing dependencies and occupied ports produce diagnostic
errors rather than stopping unrelated services.

The existing fixed-coordinate workflow below does not need YOLO.

Run the fixed-coordinate warehouse mission without obstacle avoidance:

```bash
bash ROS1/scripts/run_warehouse_ros_demo.sh false
```

On the current CachyOS/Distrobox setup, the runner opens `MainWarehouse` in the host Unity Editor automatically. On native Linux, it opens Unity automatically when the configured `UNITY_EDITOR` path exists; otherwise it prints the project path to open manually.

When Unity is ready, press **Play**.

After validating warehouse sensor readings, enable obstacle avoidance with:

```bash
bash ROS1/scripts/run_warehouse_ros_demo.sh true
```

### Windows

Run Unity on Windows and ROS 1 Noetic inside **WSL2 Ubuntu 20.04**.

For the simplest setup, keep the repository on a Windows drive so both Windows Unity and WSL2 use the same checkout. For example:

```text
Windows:
C:\Users\<you>\Documents\DroneSortingProject

WSL2:
 /mnt/c/Users/<you>/Documents/DroneSortingProject
```

Inside WSL2, create the Catkin workspace and install ROS-TCP-Endpoint:

```bash
mkdir -p ~/drone_sorting_ros1_ws/src
cd ~/drone_sorting_ros1_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Then enter the repository from WSL2 and run the same warehouse command used on Linux:

```bash
cd /mnt/c/Users/<you>/Documents/DroneSortingProject
bash ROS1/scripts/run_warehouse_ros_demo.sh false
```

The runner detects WSL2 automatically. It builds and launches ROS inside WSL2, but does **not** try to launch the Linux Unity Editor. Instead, it prints the Windows Unity project path and connection instructions.

In Windows Unity Hub:

1. Open the repository's `Unity` folder.
2. Open `Assets/Scenes/MainWarehouse.unity`.
3. Set `Active Input Handling` to `Both`.
4. Configure ROS Settings:

```text
Protocol: ROS1
Port:     10000
```

Try `127.0.0.1` first. If it does not connect, use the WSL2 IP printed by the runner.

Then press **Play**.

Obstacle avoidance is enabled with the same command:

```bash
bash ROS1/scripts/run_warehouse_ros_demo.sh true
```

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

`camera_pickup.launch` connects RGB-depth package perception, a one-package
guidance node, and the existing motion controller. It expects the robot to begin
at the tested observation point (ROS `(34, 25, 6)`, Unity `(34, 6, 25)`) in a clear,
controlled test area. It moves toward the grasp center derived from an upright
0.6 m square box top, descends, requests one pickup, and lifts after Unity confirms
`PICKUP_DONE`. It ends after lift; `camera_delivery.launch` adds access from `DroneStart` and continues to the final
zone. The existing warehouse demo remains a separate fixed-coordinate workflow.

Trial speeds default to 0.5 m/s approach, 0.2 m/s descent, and 0.3 m/s lift,
with 0.03 m arrival tolerance. The guidance node's private parameters configure
these values; positive speeds must stay within the controller's 0.5 m/s cap.
Zero `/drone/command_speed` means hold by sending no further position commands.
The trial controller waits for a speed input and stops sending commands if position,
target, or speed feedback is older than 0.5 seconds. The fixed-coordinate launch
keeps dynamic speed disabled and defaults to 2 m/s.

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

`camera_delivery.launch` now starts from `DroneStart` (Unity `(34, 2, 10)`,
ROS `(34, 10, 2)`). It ascends vertically to root height 6 m, travels to the
observation point (Unity `(34, 6, 25)`, ROS `(34, 25, 6)`), and holds within the
arrival tolerance for one continuous second. It ignores package candidates during
access and requires new images acquired after that hover before selecting a package.
ROS remains the sole movement authority. It then reuses camera pickup and continues
after lift: ascend vertically, carry above `SortingZoneA`, lower, request release, wait
for `DROP_DONE`, then hold for settling. Baseline and changed-placement/rotation
batch Unity deliveries from `DroneStart` passed independent physical evaluation.
See [full-workflow evidence](docs/verification.md#full-workflow-from-dronestart).
Pickup coordinates come from RGB-depth observations; the observation waypoint,
destination, and travel heights are configured.

```bash
# One-command host Linux startup; wait for READY, then press Play in Unity:
bash ROS1/scripts/run_camera_delivery_demo.sh

# In the sourced ROS environment, with the detector running and Unity at DroneStart:
roslaunch drone_sorting camera_delivery.launch
```

Each invocation first stops this project's previous camera demo ROS/build
processes (including Distrobox), YOLO worker, and Unity editor/import workers.
It then incrementally builds the dedicated linked catkin workspace, starts fresh
YOLO and ROS services, and reopens `MainWarehouse` with Play Mode off. Save Unity
edits before running: a restart closes this project's editor, including an editor
opened manually. The old `--reuse-editor` option is no longer supported.

Keep the terminal open to see mission progress. Ctrl+C stops this run's ROS and
detector while leaving Unity open; the next invocation closes that editor and
starts fresh automatically. Cleanup identifies processes through project paths,
per-run arguments/environment, and recorded systemd scopes. It preserves editors
for other projects and untracked services. If an unrelated service occupies
11311, 10000, or 10001, startup reports the conflict rather than terminating it.
Concurrent restarts are refused until the current startup reaches READY. Logs
from previous runs are retained. Unity recovery scenes left by shutdown are moved
to the new run's `unity-scene-backups/` directory, so the saved scene opens without
a backup-recovery prompt; those backups remain available for recovery.

The terminal reports perception states and rejection reasons. On mission failure,
the drone guidance holds and ROS stays available for inspection until Ctrl+C;
failure does not count as a completed delivery. ROS also stays connected after
successful completion until Ctrl+C, avoiding reconnection errors in Unity.

Warehouse Play Mode uses a wide spectator view in the central aisle. In Unity
coordinates the preferred camera stays at X=34 m and Y=6.5 m, follows 8 m ahead
along Z, and stops at Z=50 m before the far-end overhang. It frames the drone and
ground beneath it, with Cinemachine deocclusion for colliders. This view is
independent of the mounted RGB-depth camera.

With MainWarehouse open and Play stopped, **Drone Delivery → Verify Warehouse
Spectator Clearance** checks 105 route positions and renders six previews into
`ROS1/build/spectator-views/`. It also runs after script compilation and restores
the drone's original pose without saving. Rendered images must be reviewed because
collider checks cannot detect every visual obstruction. Diagnostic requests and
verification results are in [launcher and spectator verification](docs/verification.md#launcher-and-spectator-verification).

This launcher supports host Linux with either native ROS Noetic or Distrobox.
`ROS_ENVIRONMENT=auto` chooses native Noetic when installed, otherwise Distrobox;
select `native` or `distrobox` explicitly when needed. `ROS_CONTAINER` selects the
container. Override paths with `UNITY_EDITOR`, `DETECTOR_PYTHON`,
`DETECTOR_WEIGHTS`, `ROS_TCP_ENDPOINT_SOURCE`, or `CAMERA_DEMO_WORKSPACE`; use the
same overrides during setup and launch. Defaults keep the Endpoint, detector
environment, and linked Catkin workspace under `ROS1/build/`. The helpers are
`prepare_camera_demo_ros.sh` (source links and build), `start_camera_demo_ros.sh`
(ROS readiness, monitoring, and cleanup), and `camera_demo_processes.py`
(project-specific restart ownership, including detector and editor workers).

When a systemd user manager is available, host demo processes run in a scope
capped at 8 GiB RAM and 2 GiB swap by default. Set `CAMERA_DEMO_MEMORY_MAX` and
`CAMERA_DEMO_SWAP_MAX` to adjust these limits. `CAMERA_DEMO_MEMORY_MODE=auto`
uses the scope when available; `required` refuses startup without it; `off`
disables it. Normal startup reports available host RAM and warns below 4 GiB;
this is provisional guidance, not a measured requirement or a refusal threshold.
`CAMERA_DEMO_MIN_AVAILABLE_GIB` optionally enforces an explicit positive minimum
before services start or any editor closes. There is no default 10 GiB minimum.
Swap is not counted as available RAM, and the check does not reserve memory.

Unity imports assets sequentially (`-refreshImportMode InProcess`), uses two job
workers and one asset garbage-collector helper. This reduces concurrency compared
with the previous launcher. `UNITY_STARTUP_TIMEOUT` sets the scene startup timeout
in seconds (default 600). Without a scope, no launcher memory cap is applied.
Builds use two jobs; ROS in Distrobox uses the container's resource group. Each
run retains logs and helper snapshots under
`ROS1/build/camera-demo-<timestamp>-<id>/`. Limits do not guarantee sufficient RAM.
See [Linux portability verification](docs/verification.md#linux-portability-verification)
for the failed fresh import and remaining checks.

Use the same detector with `--context-scale 1.5` and read-only recorder as the
pickup trial. Set `DRONE_PICKUP_TRIAL_MISSION=delivery` when starting
`CameraPickupTrial.Run`; the delivery capture limit is 330 s. Delivery trials now
initialize at `DroneStart`.
To reproduce earlier observation-only deliveries, pair launch argument
`start_from_drone_start:=false` with harness environment
`DRONE_PICKUP_TRIAL_START=observation`. Pickup-only trials remain unchanged. Use a
fresh output for every run and confirm previous control publishers have stopped. The
[trial procedure and delivery evidence](docs/verification.md#camera-guided-single-package-delivery)
cover batch startup, recording, and evaluation. `camera_pickup.launch` continues
to end after lift; `warehouse_demo.launch` retains the fixed-coordinate demo.

Scene-specific parameters are in `camera_pickup.launch`, shared by the delivery
launch. ROS X/Y are horizontal; ROS Z is vertical, and heights refer to drone root:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `observation_x`, `observation_y`, `observation_height` | 34, 25, 6 m | Observation hover target |
| `access_speed`, `access_timeout` | 0.5 m/s, 60 s | Speed and timeout for each access stage |
| `hover_time` | 1 s | Continuous arrival tolerance before visual selection |
| `dropoff_x`, `dropoff_y` | 28, 55 m | Destination center |
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
Avoidance remains disabled; the configured route was checked against active
scene colliders, not established as a general navigation solution.

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
