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
| `run_warehouse_ros_demo.sh` | Runs the complete `MainWarehouse` mission |
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

Build the project ROS package:

```bash
cd ~/DroneSortingProject
bash ROS1/scripts/build_ros.sh
```

Open the repository's `Unity` folder in Unity Hub and configure:

```text
Robotics → ROS Settings

Protocol: ROS1
IP:       127.0.0.1
Port:     10000
```

For the warehouse asset's legacy camera controller, set:

```text
Edit → Project Settings → Player
Active Input Handling: Both
```

Run the warehouse mission without obstacle avoidance:

```bash
bash ROS1/scripts/run_warehouse_ros_demo.sh false
```

When Unity finishes loading `MainWarehouse`, press **Play**.

After validating warehouse sensor readings, enable obstacle avoidance with:

```bash
bash ROS1/scripts/run_warehouse_ros_demo.sh true
```

### Windows

Run Unity normally on Windows and run ROS 1 Noetic inside **WSL2 Ubuntu 20.04**.

Inside WSL, clone the repository, create `~/drone_sorting_ros1_ws/src`, install ROS-TCP-Endpoint, and build the ROS package as described in the Linux section.

Open the repository's `Unity` folder with Unity Hub on Windows.

Configure Unity ROS Settings:

```text
Protocol: ROS1
Port:     10000
```

Try `127.0.0.1` first. If Unity cannot reach ROS through WSL2, run:

```bash
hostname -I
```

and use the WSL2 IP address in Unity's ROS Settings.

## Current Status

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
