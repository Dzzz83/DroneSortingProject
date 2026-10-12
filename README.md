# Drone Sorting Project

Unity + ROS 1 simulation for an autonomous drone that transports lightweight packages between sorting zones in a warehouse.

ROS is the main control system. Unity simulates the warehouse, drone, sensors, package handling, and rendering.

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
- Processing drone position and obstacle sensor data
- Calculating movement commands
- Deciding obstacle-avoidance actions
- Requesting package pickup and drop

**Unity** is responsible for:

- Simulating the warehouse, drone, obstacles, and package
- Applying movement commands received from ROS
- Reporting the drone's current position
- Measuring obstacle distances using Unity physics
- Simulating the gripper and physical package attachment/release
- Rendering the simulation

ROS and Unity communicate through Unity ROS-TCP-Connector and ROS-TCP-Endpoint.

## Project Structure

```text
DroneSortingProject/
├── ROS1/
│   ├── scripts/
│   │   ├── build_ros.sh
│   │   ├── run_delivery_demo.sh
│   │   ├── run_warehouse_ros_demo.sh
│   │   ├── sync_ros_workspace.sh
│   │   ├── stop_demo.sh
│   │   ├── watch_obstacles.sh
│   │   ├── test_mission.sh
│   │   └── test_full_mission.sh
│   │
│   └── src/drone_sorting/
│       ├── launch/
│       │   ├── drone_demo.launch
│       │   └── warehouse_demo.launch
│       ├── scripts/
│       │   ├── delivery_demo_monitor.py
│       │   ├── mission_manager_node.py
│       │   └── motion_controller_node.py
│       └── src/drone_sorting/
│           ├── control/
│           │   └── motion_controller.py
│           ├── interfaces/
│           │   └── ros_topics.py
│           ├── mission/
│           │   ├── mission_config.py
│           │   ├── mission_manager.py
│           │   └── mission_state.py
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
    │   │   ├── DeliveryDemoSceneLauncher.cs
    │   │   ├── WarehouseDroneAutoRunOnce.cs
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
    │       └── WarehouseRosSetup.cs
    ├── Packages/
    └── ProjectSettings/
```

## Main Components

### ROS Components

#### Mission Manager

`mission_manager.py` controls the high-level delivery mission:

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

It selects the active target, detects arrival using Unity position feedback, requests `PICKUP` or `DROP`, and waits for Unity to confirm completion.

`mission_config.py` contains default mission values. Scene-specific launch files can override pickup/drop-off coordinates and arrival tolerance.

#### Motion Controller

`motion_controller.py` is the main ROS movement controller. It receives:

- Mission target position
- Current drone position from Unity
- Obstacle distances from Unity

It calculates the next commanded position and publishes it to Unity. When obstacle avoidance is enabled, it uses the Obstacle Avoidance Planner before producing the next movement command.

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

Warehouse obstacle avoidance is disabled by default until the warehouse sensor readings are validated.

#### Helper Scripts

| Script | Purpose |
|---|---|
| `sync_ros_workspace.sh` | Copies the repository ROS package into the Catkin workspace |
| `build_ros.sh` | Syncs and builds the ROS workspace |
| `run_delivery_demo.sh` | Runs the smaller `SampleScene` integration demo |
| `run_warehouse_ros_demo.sh` | Runs the complete `MainWarehouse` mission on Linux or WSL2 |
| `watch_obstacles.sh` | Displays Unity obstacle sensor readings |
| `stop_demo.sh` | Stops the running ROS demo |
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

#### Unity Editor Tools

- `DeliveryDemoSceneLauncher.cs` opens `SampleScene` or `MainWarehouse` from helper scripts.
- `WarehouseDroneIntegrator.cs` places the M2 drone/package system in the warehouse and applies the shared ROS setup.
- `WarehouseDroneAutoRunOnce.cs` supports the warehouse editor integration workflow.

## Current Data Flow

### Movement and sensing

```text
Mission Manager
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
Mission Manager
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
Mission Manager
```

### Main ROS Topics

| Topic | Purpose |
|---|---|
| `/drone/target_position` | Current pickup or drop-off target selected by Mission Manager |
| `/drone/current_position` | Current drone position reported by Unity |
| `/drone/obstacle_distances` | Obstacle measurements reported by Unity |
| `/drone/command_position` | Next movement position calculated by ROS |
| `/drone/travel_heading` | Horizontal travel direction used by Unity sensors |
| `/drone/mission_state` | Current delivery mission state |
| `/drone/package_action` | `PICKUP` or `DROP` request from ROS |
| `/drone/package_action_status` | `PICKUP_DONE` or `DROP_DONE` confirmation from Unity |

## Workflow

1. Mission Manager starts `GO_TO_PICKUP` and publishes the pickup target.
2. Unity reports the drone position and obstacle measurements.
3. Motion Controller calculates the next position and sends it to Unity.
4. The ROS ↔ Unity control loop continues until the pickup target is reached.
5. Mission Manager requests `PICKUP`.
6. Unity performs the physical pickup and returns `PICKUP_DONE`.
7. Mission Manager switches to `GO_TO_DROPOFF` and publishes the drop-off target.
8. ROS controls the drone to the drop-off point.
9. Mission Manager requests `DROP`.
10. Unity releases the package and returns `DROP_DONE`.
11. Mission Manager changes the mission state to `COMPLETE`.

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

Build the ROS workspace:

```bash
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

Run the warehouse mission without obstacle avoidance:

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

`MainWarehouse` contains three identical delivery-package prefab instances in
the existing 5 × 5 m pickup area, centered at Unity X=34, Z=25. Package centers
start at Unity (34.23, 0.44, 25.10), (32.70, 0.44, 23.70), and
(35.30, 0.44, 26.30). These are scene-placement facts, not inputs to the planned
ROS perception system.

The planned observation waypoint is directly above the pickup-area center:
Unity (34, observation_height, 25), or ROS (34, 25, observation_height).
Its height will be selected after checking camera coverage and warehouse clearance.
ROS will know the area boundaries (Unity X=31.5–36.5, Z=22.5–27.5) and drop zone,
but will discover package locations from RGB-depth observations.

The planned mission observes the area, selects an accessible package, approaches
and picks it up, delivers it to the drop zone, and returns to observe again.
Completion requires repeated valid observations covering the pickup area with no
remaining packages; a missed detection or camera failure must not count as empty.
The three-package count is for demo evaluation, not a mission stopping condition.
ROS will select separate release positions within the destination zone to avoid
dropping subsequent packages onto previously delivered packages.

`MainWarehouse` now has a persistent `PackageRgbCamera` child on the delivery
drone. `DroneRgbCamera` configures its local position `(0, -0.55, -0.60)` and
rotation `(55, 0, 0)` degrees: 35 degrees from straight down toward the gripper.
The provisional simulated lens is 800 × 600 with a 110-degree **vertical** field
of view and 0.05–30 m clipping planes; these are not real sensor specifications
or a validated depth range. The camera renders on demand, not to the spectator
display. `WarehouseRosSetup` reuses this child rather than adding duplicates.

To verify the installation, stop Play Mode and save existing edits, then use
**Drone Delivery → Install and Verify Warehouse RGB Camera**. This checks the
mount, duplicate prevention, and frustum coverage, renders observation/approach/
open/closed-jaw views to the system temporary directory under
`drone-permanent-rgb-verification/`, restores the drone and jaws, and saves the
camera installation. The observation check uses drone root height 6 m; it checks
coverage, not flight clearance. Inspect the images separately for occlusion.

The RGB mount and its four rendered views have been verified in Unity.
`RgbDepthCapture` provides on-demand, aligned RGB and floating-point depth from
the same HDRP camera render. Arrays use top-left origin; RGB is packed RGB bytes,
and depth is optical-axis distance in metres with `NaN` for missing/out-of-range
measurements. The provisional capture range is 0.2–15 m. Opaque depth-writing
surfaces are supported; transparent surfaces without depth writes are not.

Use **Drone Delivery → Verify Warehouse RGB Depth Capture** with MainWarehouse
open and Play Mode stopped. It tests known-distance colored probes, off-axis
depth, RGB/depth row alignment, missing depth, range filtering, and repeated
warehouse/gripper capture. Temporary objects and drone/jaw poses are restored;
the scene is not saved. Images and a report go to the temporary directory under
`drone-rgb-depth-verification/`. These checks passed; five warmed observation
captures averaged 64.8 ms each in the editor, not a runtime throughput guarantee.

`DroneCameraPublisher` is configured by the warehouse runtime setup. It starts
with 2 captures per second, queue size 1, and no catch-up bursts; capture pauses
when the ROS connection is unavailable. Unity compilation and message/coordinate
checks passed, including arbitrary drone rotation and TF composition. ROS Noetic
also independently decoded all six saved Unity message types and checked their
metadata, depth values, calibration, and TF translation. The sensor-only live
test passed: ROS received five synchronized 800 × 600 observations at roughly
2 FPS, with valid encodings, layouts, calibration, poses, and acquisition times.
A separate live check confirmed that `warehouse → drone_base →
package_camera_optical` agrees with the stamped camera pose, drone position,
and optical axes.
No mission or motion controller ran during these checks; they establish sensor
communication, not package recognition, navigation, or real-hardware readiness.
Synchronous readback
can stall rendering, so the editor measurement does not justify higher rates.
Restart Play Mode after changing capture dimensions or depth limits.

| Topic | Message / meaning |
| --- | --- |
| `/drone/camera/rgb/image_raw` | `sensor_msgs/Image`, `rgb8`, top-down packed rows |
| `/drone/camera/depth/image_raw` | `sensor_msgs/Image`, `32FC1`, optical-axis metres, invalid `NaN` |
| `/drone/camera/camera_info` | `sensor_msgs/CameraInfo`, ideal pinhole intrinsics, zero distortion |
| `/drone/camera/pose` | `geometry_msgs/PoseStamped`, optical camera pose in `warehouse` |
| `/drone/current_pose` | `geometry_msgs/PoseStamped`, drone body pose in `warehouse` |
| `/tf` | `tf2_msgs/TFMessage`, `warehouse → drone_base → package_camera_optical` |

Each observation's messages share a sequence and UTC acquisition timestamp;
use ROS wall time (`use_sim_time=false`), with synchronized clocks if separate
computers are used. Existing `/drone/current_position` is unchanged. World
positions retain Unity `(x,y,z)` → ROS `(x,z,y)`. Drone body axes are forward,
left, up; optical axes are right, down, forward. Rotations are converted using
these bases, not by swapping quaternion components. The calibration is for the
undistorted simulated lens, not measured real-camera calibration.

After building the ROS package, run `roslaunch drone_sorting camera_validation.launch`
in the ROS environment, then start MainWarehouse Play Mode. This starts only the
TCP endpoint and read-only sensor checker, not mission or motion nodes. Do not
run it alongside another endpoint on port 10000. Use `required_frames:=5` for a
finite checker; stop the launch afterward because the endpoint remains running.
The checker validates synchronized images, calibration, pose, data layouts, and
freshness; it does not prove detection accuracy or flight safety.

Focused Python checks:
`PYTHONPATH=ROS1/src/drone_sorting/src python3 -m unittest discover -s ROS1/src/drone_sorting/tests -v`.
Run in ROS Noetic for all 15 tests; the seven ROS-message contract tests are
skipped without ROS dependencies. Unity message formatting and transform checks
are available through **Drone Delivery → Verify Camera ROS Messages**.
To independently decode that verifier's binary fixtures in ROS Noetic, run
`PYTHONPATH=ROS1/src/drone_sorting/src python3 ROS1/src/drone_sorting/tests/verify_unity_camera_messages.py <fixture_directory>`.
The default fixture directory is the system temporary directory under
`drone-camera-message-verification/`; copy fixtures when ROS runs in a container
with a separate temporary filesystem.
With the sensor-only launch and Unity Play Mode running, check live pose/TF
consistency using
`PYTHONPATH=ROS1/src/drone_sorting/src python3 ROS1/src/drone_sorting/tests/verify_live_camera_transforms.py`.
It waits up to 15 seconds for a synchronized observation and sends no commands.

Camera perception and repeated delivery are not implemented yet. The existing
warehouse launch still performs its single delivery using fixed pickup coordinates.

### Working

- ROS ↔ Unity communication
- ROS delivery mission state machine
- ROS-controlled drone movement
- Unity position feedback
- Unity obstacle sensing
- Physical gripper and package attachment/release
- Automatic pickup and drop-off target switching
- End-to-end `MainWarehouse` delivery with ROS controlling the mission and movement
- Static obstacle avoidance validated in `SampleScene`
- Climb, descend, pass, and side-step avoidance logic

### Next

- Validate obstacle sensor readings in `MainWarehouse`
- Enable and tune static obstacle avoidance in `MainWarehouse`
- Improve sensing where needed
- Add dynamic obstacle avoidance
- Final warehouse and system polish
