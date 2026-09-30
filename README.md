# Drone Sorting Simulation

A Unity and ROS 1 simulation for automated drone-based parcel sorting in a logistics environment.

ROS acts as the main control system. Unity provides the simulated environment, drone movement, position feedback, and obstacle sensor data.

## Current Features

- ROS 1 ↔ Unity communication through ROS-TCP-Endpoint
- Automated movement between sorting zones
- Drone position feedback from Unity to ROS
- Simulated obstacle-distance sensors
- ROS-based motion control
- ROS-based static obstacle avoidance
- Vertical avoidance by climbing or descending
- Side-step avoidance support
- Automated demo startup scripts

## System Architecture

```text
Mission Manager
      |
      | /drone/target_position
      v
Motion Controller
      |
      +---- Obstacle Avoidance Planner
      |             ^
      |             |
      |     /drone/obstacle_distances
      |             |
      |           Unity
      |
      | /drone/command_position
      v
Unity Drone
      |
      +---- /drone/current_position ----> ROS
```

ROS decides where the drone should move.

Unity simulates the movement and sends the drone's current position and obstacle sensor readings back to ROS.

## ROS Topics

| Topic | Message Type | Purpose |
|---|---|---|
| `/drone/target_position` | `geometry_msgs/Point` | Current mission destination |
| `/drone/current_position` | `geometry_msgs/Point` | Drone position reported by Unity |
| `/drone/obstacle_distances` | `std_msgs/Float32MultiArray` | Obstacle sensor readings |
| `/drone/command_position` | `geometry_msgs/Point` | Movement command calculated by ROS |
| `/drone/travel_heading` | `geometry_msgs/Vector3` | Current travel direction |

Obstacle sensor order:

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

A value of `-1.0` means that no obstacle was detected within the sensor range.

## Project Structure

```text
DroneSortingProject/
├── ROS1/
│   ├── scripts/
│   │   ├── build_ros.sh
│   │   ├── recompile_unity.sh
│   │   ├── start_demo.sh
│   │   ├── stop_demo.sh
│   │   └── watch_obstacles.sh
│   │
│   └── src/
│       └── drone_sorting/
│           ├── launch/
│           │   └── drone_demo.launch
│           ├── scripts/
│           │   ├── mission_manager.py
│           │   └── motion_controller.py
│           └── src/drone_sorting/
│               ├── control/
│               ├── mission/
│               ├── perception/
│               └── planning/
│
└── Unity/
    ├── Assets/
    │   ├── Editor/
    │   ├── Scenes/
    │   └── Scripts/
    ├── Packages/
    └── ProjectSettings/
```

## Main Unity Scripts

- `DroneRosSubscriber.cs` — receives movement commands from ROS and moves the drone.
- `DroneStatePublisher.cs` — publishes the drone's current position to ROS.
- `DroneObstacleSensorPublisher.cs` — detects nearby obstacles and publishes sensor distances.
- `ObstacleCourseCamera.cs` — controls the simulation camera.

## Main ROS Components

### Mission Manager

Publishes the current destination using:

```text
/drone/target_position
```

### Motion Controller

Receives the drone position, target position, and obstacle readings.

It calculates the next movement command and publishes:

```text
/drone/command_position
```

### Obstacle Avoidance Planner

The planner can use several avoidance states:

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

## Requirements

- Unity Editor 6000.6.0f1
- ROS 1 Noetic
- Ubuntu 20.04 ROS environment
- Catkin
- Unity ROS-TCP-Connector
- ROS-TCP-Endpoint

ROS-TCP-Endpoint should be installed separately from:

```text
https://github.com/Unity-Technologies/ROS-TCP-Endpoint
```

It is not included directly in this repository.

## Current Project Status

Working:

- ROS 1 and Unity communication
- Target-based drone movement
- Unity position feedback
- Unity obstacle sensing
- ROS motion control
- Static obstacle avoidance
- Automated obstacle-course traversal

Future work:

- Improve obstacle-avoidance reliability
- Add additional sensors/camera-based perception
- Package attachment
- Package delivery between sorting zones
- Dynamic obstacle avoidance
- Full-system testing