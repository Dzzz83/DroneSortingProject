# Drone Sorting Project

Unity + ROS 1 simulation for an autonomous drone that transports lightweight packages between sorting zones while avoiding obstacles.

ROS acts as the main control system. Unity provides the simulated warehouse environment, drone movement, position feedback, and obstacle sensor data.

## Table of Contents

- [Drone Sorting Project](#drone-sorting-project)
  - [Table of Contents](#table-of-contents)
  - [Project Overview](#project-overview)
  - [Project Structure](#project-structure)
  - [Main Components](#main-components)
    - [ROS Components](#ros-components)
      - [Mission Manager](#mission-manager)
      - [Motion Controller](#motion-controller)
      - [Obstacle Avoidance Planner](#obstacle-avoidance-planner)
      - [ROS Launch File](#ros-launch-file)
      - [Helper Scripts](#helper-scripts)
    - [Unity Components](#unity-components)
      - [DroneRosSubscriber.cs](#dronerossubscribercs)
      - [DroneStatePublisher.cs](#dronestatepublishercs)
      - [DroneObstacleSensorPublisher.cs](#droneobstaclesensorpublishercs)
      - [ObstacleCourseCamera.cs](#obstaclecoursecameracs)
      - [Unity Editor Tools](#unity-editor-tools)
  - [Current Data Flow](#current-data-flow)
  - [Workflow](#workflow)
  - [Installation](#installation)
    - [Requirements](#requirements)
    - [Linux](#linux)
    - [Windows](#windows)
  - [Current Status](#current-status)

## Project Overview

The project separates simulation from decision-making.

**Unity** is responsible for:

- Simulating the warehouse and obstacles
- Moving the drone
- Measuring obstacle distances
- Reporting the drone's current position

**ROS 1** is responsible for:

- Managing mission targets
- Processing position and sensor data
- Deciding how the drone should avoid obstacles
- Calculating the next movement command

Communication between ROS and Unity is handled through the Unity ROS-TCP-Connector and ROS-TCP-Endpoint.

## Project Structure

```text
DroneSortingProject/
├── ROS1/
│   ├── scripts/
│   │   ├── build_ros.sh
│   │   ├── start_demo.sh
│   │   ├── stop_demo.sh
│   │   ├── watch_obstacles.sh
│   │   └── recompile_unity.sh
│   │
│   └── src/drone_sorting/
│       ├── launch/
│       │   └── drone_demo.launch
│       ├── scripts/
│       │   ├── mission_manager.py
│       │   └── motion_controller.py
│       └── src/drone_sorting/
│           ├── mission/
│           ├── control/
│           ├── planning/
│           └── perception/
│
└── Unity/
    ├── Assets/
    │   ├── Editor/
    │   ├── Scenes/
    │   └── Scripts/
    ├── Packages/
    └── ProjectSettings/
```

## Main Components

### ROS Components

#### Mission Manager

The Mission Manager controls the high-level destination of the drone. It selects the current target position for the mission and publishes that target to:

```text
/drone/target_position
```

The Motion Controller uses this position as the destination the drone should move toward.

#### Motion Controller

The Motion Controller is the main ROS control node.

It receives:

- The target position from the Mission Manager
- The current drone position from Unity
- Obstacle distance readings from Unity

It combines this information with the obstacle avoidance planner, calculates the next safe movement position, and publishes it to:

```text
/drone/command_position
```

#### Obstacle Avoidance Planner

The Obstacle Avoidance Planner decides how the drone should react when an obstacle is detected.

It evaluates the front, upper, lower, side, up, and down sensor readings and determines whether the drone should continue forward, climb, descend, or move sideways.

Main planner states include:

```text
NORMAL
CLIMB
DESCEND
PASS
SIDESTEP_LEFT
SIDESTEP_RIGHT
```

Possible movement actions include:

```text
FORWARD
FORWARD_LEVEL
UP
DOWN
LEFT
RIGHT
STOP
```

#### ROS Launch File

`drone_demo.launch` starts the main ROS nodes required for the current simulation, allowing the ROS side of the project to be launched with one command.

#### Helper Scripts

Scripts inside `ROS1/scripts/` simplify common development tasks such as:

- Building the ROS workspace
- Starting the complete demo
- Stopping the running ROS processes
- Monitoring obstacle sensor data
- Recompiling Unity-related components

### Unity Components

#### DroneRosSubscriber.cs

Receives movement commands from:

```text
/drone/command_position
```

It moves the Unity drone toward the position calculated by ROS.

Unity therefore performs the simulated movement, but the movement decision itself comes from ROS.

#### DroneStatePublisher.cs

Continuously reports the drone's current Unity position to ROS through:

```text
/drone/current_position
```

This gives the Motion Controller feedback about where the drone actually is after each movement.

#### DroneObstacleSensorPublisher.cs

Simulates the drone's obstacle sensors using Unity physics.

It measures obstacle distances in several directions and publishes them through:

```text
/drone/obstacle_distances
```

Sensor order:

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

A value of `-1.0` means that no obstacle was detected within the configured sensor range.

#### ObstacleCourseCamera.cs

Controls the camera used to observe the drone and obstacle course during the simulation.

#### Unity Editor Tools

Scripts inside `Unity/Assets/Editor/` help automatically configure parts of the simulation, including:

- Warehouse and obstacle course creation
- Drone state publisher setup
- Obstacle sensor setup

This reduces the amount of manual configuration required inside the Unity Editor.

## Current Data Flow

```text
Mission Manager
      |
      | /drone/target_position
      v
Motion Controller
      ^
      |
      | /drone/current_position
      | /drone/obstacle_distances
      |
    Unity
      |
      v
Obstacle Avoidance Planner
      |
      v
Motion Controller
      |
      | /drone/command_position
      v
Unity Drone
      |
      └──────── updated position and sensor data ────────> ROS
```

Main ROS topics:

| Topic | Purpose |
|---|---|
| `/drone/target_position` | Destination selected by the Mission Manager |
| `/drone/current_position` | Current drone position reported by Unity |
| `/drone/obstacle_distances` | Obstacle sensor readings reported by Unity |
| `/drone/command_position` | Next safe movement position calculated by ROS |
| `/drone/travel_heading` | Current direction of travel |

## Workflow

1. The Mission Manager selects the current destination.
2. Unity reports the drone's current position.
3. Unity measures nearby obstacles.
4. ROS receives the position and obstacle data.
5. The Motion Controller checks whether the direct path is safe.
6. The Obstacle Avoidance Planner selects an appropriate movement action if an obstacle is detected.
7. The Motion Controller calculates the next command position.
8. ROS publishes `/drone/command_position`.
9. Unity moves the drone toward that position.
10. Unity sends updated position and sensor data back to ROS.
11. The control loop repeats until the target is reached.

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

ROS Noetic can run directly on Ubuntu 20.04 or inside an Ubuntu 20.04 container such as Distrobox.

Clone the project:

```bash
git clone https://github.com/Dzzz83/DroneSortingProject.git
cd DroneSortingProject
```

Create a Catkin workspace:

```bash
mkdir -p ~/drone_sorting_ros1_ws/src
```

Copy the ROS package:

```bash
cp -r ROS1/src/drone_sorting ~/drone_sorting_ros1_ws/src/
```

Install ROS-TCP-Endpoint:

```bash
cd ~/drone_sorting_ros1_ws/src

git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Build the workspace:

```bash
cd ~/drone_sorting_ros1_ws

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Open the repository's `Unity` folder in Unity Hub.

Configure:

```text
Robotics → ROS Settings

Protocol: ROS1
IP:       127.0.0.1
Port:     10000
```

Start ROS:

```bash
roslaunch drone_sorting drone_demo.launch
```

Then press **Play** in Unity.

### Windows

Unity runs normally on Windows, while ROS 1 Noetic should run inside **WSL2 with Ubuntu 20.04**.

Inside WSL:

```bash
git clone https://github.com/Dzzz83/DroneSortingProject.git
cd DroneSortingProject

mkdir -p ~/drone_sorting_ros1_ws/src
cp -r ROS1/src/drone_sorting ~/drone_sorting_ros1_ws/src/
```

Install ROS-TCP-Endpoint:

```bash
cd ~/drone_sorting_ros1_ws/src

git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Build the ROS workspace:

```bash
cd ~/drone_sorting_ros1_ws

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Open the repository's `Unity` folder with Unity Hub on Windows.

Start ROS inside WSL:

```bash
roslaunch drone_sorting drone_demo.launch
```

Configure Unity:

```text
Protocol: ROS1
Port:     10000
```

Try:

```text
127.0.0.1
```

first.

If Unity cannot connect to ROS through WSL, get the WSL IP:

```bash
hostname -I
```

and use that IP in Unity's ROS Settings.

Then press **Play**.

## Current Status

Currently working:

- ROS ↔ Unity communication
- Target-based autonomous movement
- Position feedback
- Obstacle sensing
- ROS-based motion control
- Static obstacle avoidance
- Automatic climbing and descending
- Side-step avoidance support
- Repeated obstacle-course traversal

Planned work:

- Improved sensing
- Package attachment
- Package delivery
- Dynamic obstacle avoidance
- Full-system testing