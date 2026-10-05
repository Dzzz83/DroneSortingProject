# Drone Sorting Project


Unity + ROS 1 simulation for an autonomous drone that transports lightweight packages between sorting zones while avoiding obstacles.


ROS acts as the main control system. Unity provides the simulated warehouse environment, drone movement, position feedback, obstacle sensor data, and physical package handling.


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

      - [DroneCommandSubscriber.cs](#dronecommandsubscribercs)

      - [DroneStatePublisher.cs](#dronestatepublishercs)

      - [DroneObstacleSensorPublisher.cs](#droneobstaclesensorpublishercs)

      - [PackageActionBridge.cs](#packageactionbridgecs)

      - [Drone Delivery Package System](#drone-delivery-package-system)

      - [DroneRosTopics.cs](#dronerostopicscs)

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

- Handling physical package pickup, attachment, transport, and drop-off


**ROS 1** is responsible for:


- Managing the delivery mission and mission states

- Selecting pickup and drop-off targets

- Processing position and sensor data

- Requesting package pickup and drop actions

- Deciding how the drone should avoid obstacles

- Calculating the next movement command


Communication between ROS and Unity is handled through the Unity ROS-TCP-Connector and ROS-TCP-Endpoint.


## Project Structure


```text

DroneSortingProject/

├── ROS1/

│   ├── scripts/

│   │   ├── build_ros.sh

│   │   ├── run_delivery_demo.sh

│   │   ├── run_warehouse_ros_demo.sh

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

│           ├── mission/

│           │   ├── mission_manager.py

│           │   ├── mission_config.py

│           │   └── mission_state.py

│           ├── control/

│           ├── interfaces/

│           │   └── ros_topics.py

│           ├── planning/

│           └── perception/

│

└── Unity/

    ├── Assets/

    │   ├── DroneDelivery/

    │   │   └── Runtime/

    │   │       ├── DeliveryDemo.cs

    │   │       ├── DeliveryPackage.cs

    │   │       ├── DeliveryRotor.cs

    │   │       ├── GripperController.cs

    │   │       └── PackageHandling.cs

    │   ├── Editor/

    │   │   └── DeliveryDemoSceneLauncher.cs

    │   ├── Prefabs/

    │   │   ├── Drone.prefab

    │   │   └── Package.prefab

    │   ├── Scenes/

    │   │   ├── SampleScene.unity

    │   │   ├── MainWarehouse.unity

    │   │   └── DroneGripperTest.unity

    │   └── Scripts/

    │       ├── DroneCommandSubscriber.cs

    │       ├── DroneStatePublisher.cs

    │       ├── DroneObstacleSensorPublisher.cs

    │       ├── DroneRosTopics.cs

    │       ├── PackageActionBridge.cs

    │       └── WarehouseRosRuntimeBootstrap.cs

    ├── Packages/

    └── ProjectSettings/

```


## Main Components


### ROS Components


#### Mission Manager


The Mission Manager controls the high-level package delivery mission. It decides which stage of the mission is currently active and selects the corresponding destination.


The current mission sequence is:


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


Default pickup and drop-off positions are defined in `mission_config.py`, while launch parameters can override them for a specific scene. `warehouse_demo.launch` supplies the coordinates used by `MainWarehouse.unity`. The available mission states are defined in `mission_state.py`.


The Mission Manager publishes the current destination through:


```text

/drone/target_position

```


The Motion Controller uses this position as the destination the drone should move toward.


When the pickup or drop-off position is reached, the Mission Manager publishes a package command through:


```text

/drone/package_action

```


Possible commands are:


```text

PICKUP

DROP

```


The mission then waits for confirmation through:


```text

/drone/package_action_status

```


Expected confirmations are:


```text

PICKUP_DONE

DROP_DONE

```


The current mission state is also published through:


```text

/drone/mission_state

```


This allows the mission progress to be monitored and tested independently.


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


`drone_demo.launch` starts ROS-TCP-Endpoint, the Mission Manager, and the Motion Controller and accepts configurable pickup/drop-off coordinates. `warehouse_demo.launch` reuses it with the `MainWarehouse` mission coordinates. Warehouse obstacle avoidance is disabled by default for the first integration test and can be enabled explicitly after sensor validation.


#### Helper Scripts


Scripts inside `ROS1/scripts/` simplify common development and testing tasks such as:


- Building the ROS workspace

- Starting the complete demo

- Stopping the running ROS processes

- Monitoring obstacle sensor data

- Inspecting the current mission state and target

- Manually sending pickup and drop confirmations

- Running the end-to-end delivery demo and monitoring its result

- Automatically testing the ROS mission sequence


`test_mission.sh` provides manual mission inspection and testing commands.


`test_full_mission.sh` automatically verifies the ROS mission sequence from pickup to mission completion.

`run_delivery_demo.sh` keeps the smaller `SampleScene.unity` integration test. `run_warehouse_ros_demo.sh` syncs the current repository ROS package into the Catkin workspace, opens `MainWarehouse.unity`, starts the warehouse mission, and runs `delivery_demo_monitor.py`. Pass `false` for stage 1 without avoidance or `true` after warehouse sensor readings have been validated.


### Unity Components


#### DroneCommandSubscriber.cs


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


This gives both the Motion Controller and Mission Manager feedback about where the drone actually is.


The Motion Controller uses the position for navigation, while the Mission Manager uses it to determine when the drone has reached the pickup or drop-off target.


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


#### PackageActionBridge.cs


Bridges ROS package commands to the physical Unity package-handling system.


It subscribes to `/drone/package_action`, calls `PackageHandling` for `PICKUP` or `DROP`, verifies that the requested action actually completed, and then publishes `PICKUP_DONE` or `DROP_DONE` through `/drone/package_action_status`.


#### Drone Delivery Package System


The package-handling implementation is stored under `Unity/Assets/DroneDelivery/Runtime/`.


- `PackageHandling.cs` coordinates pickup, attachment, and drop-off

- `GripperController.cs` opens and closes the gripper jaws

- `DeliveryPackage.cs` represents a package that can be carried

- `DeliveryRotor.cs` handles rotor animation


`DroneGripperTest.unity` and `DeliveryDemo.cs` remain standalone package-system test assets. The warehouse-integrated workflow uses `MainWarehouse.unity`, `PackageActionBridge.cs`, and the ROS command/state/sensor components. `WarehouseRosRuntimeBootstrap.cs` disables any serialized `DeliveryDemo` in `MainWarehouse` so Unity cannot bypass ROS movement control.


#### DroneRosTopics.cs


Stores the shared Unity-side ROS topic constants used by the runtime ROS scripts. The ROS package uses the matching `interfaces/ros_topics.py` module.


#### ObstacleCourseCamera.cs


Controls the camera used to observe the drone and obstacle course during the simulation.


#### Unity Editor Tools


`DeliveryDemoSceneLauncher.cs` can open either `SampleScene.unity` or `MainWarehouse.unity` from the Linux helper scripts. `WarehouseDroneIntegrator.cs` now attaches the ROS integration components to the warehouse drone and intentionally does not install `DeliveryDemo`.


## Current Data Flow


```text

Mission Manager

      |

      | /drone/target_position

      v

Motion Controller <──── /drone/current_position ──── Unity

      ^                                             |

      |                                             |

      └──── /drone/obstacle_distances ──────────────┘

      |

Obstacle Avoidance Planner

      |

      v

Motion Controller

      |

      | /drone/command_position

      v

Unity Drone

      |

      └──── updated position and sensor data ──────> ROS


Package Mission Interface


Mission Manager

      |

      | /drone/package_action

      | PICKUP / DROP

      v

PackageActionBridge
      |
      v
PackageHandling / Gripper

      |

      | /drone/package_action_status

      | PICKUP_DONE / DROP_DONE

      v

Mission Manager

```


Main ROS topics:


| Topic | Purpose |
|---|---|
| `/drone/target_position` | Current pickup or drop-off destination selected by the Mission Manager |
| `/drone/current_position` | Current drone position reported by Unity |
| `/drone/obstacle_distances` | Obstacle sensor readings reported by Unity |
| `/drone/command_position` | Next safe movement position calculated by ROS |
| `/drone/travel_heading` | Current direction of travel |
| `/drone/mission_state` | Current stage of the delivery mission |
| `/drone/package_action` | Pickup or drop command sent by the Mission Manager |
| `/drone/package_action_status` | Confirmation that the requested package action has completed |


## Workflow


1. The Mission Manager starts the mission in `GO_TO_PICKUP`.

2. The pickup position is published through `/drone/target_position`.

3. Unity continuously reports the drone's current position and obstacle sensor readings.

4. The Motion Controller calculates the next movement command. When obstacle avoidance is enabled, it uses the Obstacle Avoidance Planner.

5. ROS publishes `/drone/command_position`, and Unity moves the drone.

6. The control loop continues until the Mission Manager detects that the pickup point has been reached.

7. The mission changes to `WAIT_FOR_PICKUP` and publishes `PICKUP`.

8. `PackageActionBridge` performs the physical pickup through `PackageHandling`. After the package is attached and `PICKUP_DONE` is published, the Mission Manager changes to `GO_TO_DROPOFF`.

9. The drop-off position becomes the new target.

10. The ROS navigation loop continues until the drop-off point is reached. Obstacle avoidance is used when enabled; the current delivery demo disables it so the package workflow can be tested independently.

11. The mission changes to `WAIT_FOR_DROP` and publishes `DROP`.

12. `PackageActionBridge` releases the package and publishes `DROP_DONE`. The Mission Manager then changes to `COMPLETE`.


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


For the current package-delivery workflow, obstacle avoidance can be disabled explicitly:


```bash

roslaunch drone_sorting drone_demo.launch enable_obstacle_avoidance:=false

```


On the current Linux/Distrobox development setup, `run_delivery_demo.sh` runs the smaller SampleScene test. For the warehouse integration, use:

```bash

bash ROS1/scripts/run_warehouse_ros_demo.sh false

```

After the warehouse sensor values have been inspected with `ROS1/scripts/watch_obstacles.sh`, enable M3 obstacle avoidance with:

```bash

bash ROS1/scripts/run_warehouse_ros_demo.sh true

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

- ROS delivery mission state machine

- Automatic pickup and drop-off target switching

- Pickup and drop command interface

- Physical gripper and package attachment/detachment

- ROS package commands integrated with the Unity package system

- End-to-end pickup, carry, drop-off, and mission completion in the smaller integration scene

- MainWarehouse ROS runtime integration path with Unity `DeliveryDemo` disabled

- Warehouse-specific mission coordinates and staged avoidance launch

- Mission-state monitoring

- Automated ROS mission testing and delivery demo verification


Planned work:


- Improved sensing

- Dynamic obstacle avoidance

- Validate the ROS-controlled pickup/drop workflow in `MainWarehouse` at runtime

- Validate warehouse sensor readings, then enable and tune obstacle avoidance

- Dynamic obstacle avoidance

- Final warehouse/environment polish
