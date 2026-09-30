# Drone Sorting Project

Unity + ROS 1 simulation for an autonomous drone that transports packages between sorting zones while avoiding obstacles.

ROS acts as the **control system**, while Unity provides the **simulation environment, sensors, and drone movement**.

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

### ROS

**Mission Manager**

Chooses the current destination and publishes:

```text
/drone/target_position
```

**Motion Controller**

Receives the target, current drone position, and obstacle sensor data.

It decides the drone's next position and publishes:

```text
/drone/command_position
```

**Obstacle Avoidance Planner**

Processes obstacle distances and decides whether the drone should:

```text
FORWARD
UP
DOWN
LEFT
RIGHT
STOP
```

The planner uses states such as:

```text
NORMAL
CLIMB
DESCEND
PASS
SIDESTEP_LEFT
SIDESTEP_RIGHT
```

### Unity

**DroneRosSubscriber.cs**

Receives `/drone/command_position` from ROS and moves the drone.

**DroneStatePublisher.cs**

Sends the drone's current position to ROS:

```text
/drone/current_position
```

**DroneObstacleSensorPublisher.cs**

Uses Unity sensors to detect nearby obstacles and publishes:

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

`-1.0` means no obstacle was detected within the sensor range.

**ObstacleCourseCamera.cs**

Controls the camera used to view the obstacle course.

**Unity Editor Tools**

Scripts inside `Assets/Editor/` automatically create or configure parts of the simulation such as the warehouse, obstacle sensors, and state publisher.

## Current Data Flow

```text
Mission Manager
      |
      | target_position
      v
Motion Controller
      ^
      |
      | current_position
      | obstacle_distances
      |
    Unity
      |
      v
Obstacle Avoidance Planner
      |
      v
Motion Controller
      |
      | command_position
      v
Unity Drone
      |
      └──────── feedback to ROS
```

Main ROS topics:

| Topic | Purpose |
|---|---|
| `/drone/target_position` | Mission destination |
| `/drone/current_position` | Current drone position |
| `/drone/obstacle_distances` | Unity sensor readings |
| `/drone/command_position` | Movement command from ROS |
| `/drone/travel_heading` | Current travel direction |

## Workflow

1. Mission Manager selects a target position.
2. Unity reports the drone's current position.
3. Unity measures nearby obstacles.
4. ROS receives the position and sensor data.
5. The obstacle avoidance planner decides whether the path is clear.
6. The motion controller calculates the next drone position.
7. ROS publishes `/drone/command_position`.
8. Unity moves the drone.
9. Unity sends updated position and sensor data back to ROS.
10. The process repeats until the target is reached.

## Installation

### Requirements

- Unity Editor `6000.6.0f1`
- ROS 1 Noetic
- Ubuntu 20.04 ROS environment
- Git
- Catkin
- ROS-TCP-Endpoint

ROS-TCP-Endpoint:

```text
https://github.com/Unity-Technologies/ROS-TCP-Endpoint
```

---

## Linux

The project can run on Ubuntu 20.04 directly or from an Ubuntu 20.04 container such as Distrobox.

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

Build:

```bash
cd ~/drone_sorting_ros1_ws

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Open the `Unity` folder using Unity Hub.

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

---

## Windows

ROS 1 Noetic should run inside **WSL2 Ubuntu 20.04**, while Unity runs normally on Windows.

Install WSL2 and Ubuntu 20.04, then install ROS Noetic inside Ubuntu.

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

Build:

```bash
cd ~/drone_sorting_ros1_ws

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Open the repository's `Unity` folder using Unity Hub on Windows.

Start ROS inside WSL:

```bash
roslaunch drone_sorting drone_demo.launch
```

In Unity, configure the ROS connection:

```text
Protocol: ROS1
Port:     10000
```

Try:

```text
127.0.0.1
```

first.

If Unity cannot connect to ROS through WSL, obtain the WSL IP:

```bash
hostname -I
```

and use that IP in Unity's ROS Settings.

Then press **Play**.

## Current Status

Working:

- ROS ↔ Unity communication
- Autonomous target movement
- Position feedback
- Obstacle sensing
- ROS-based motion control
- Static obstacle avoidance
- Automatic climb/descend avoidance
- Repeated obstacle-course traversal

Planned:

- Improved sensing
- Package attachment and delivery
- Dynamic obstacle avoidance
- Final system testing