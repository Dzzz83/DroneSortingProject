
# Drone Sorting Simulation

A Unity and ROS 1 project demonstrating automated drone movement between two sorting zones.

## Requirements

- Unity Editor 6000.6.0f1
- ROS 1 Noetic (Ubuntu 20.04)
- Git and Catkin build tools

## Project Structure

- `Unity/` — Unity simulation, scenes, and C# scripts.
- `ROS1/src/drone_sorting/` — ROS 1 Mission Manager.

## Setup

### 1. Unity

Open Unity Hub and add the `Unity` folder as an existing project.

Open `Assets/Scenes/SampleScene.unity`.

Configure Robotics → ROS Settings:

- Protocol: ROS1
- IP: 127.0.0.1
- Port: 10000

### 2. ROS 1

Create a Catkin workspace and copy the ROS package:

```bash
mkdir -p ~/drone_sorting_ros1_ws/src
cp -r ROS1/src/drone_sorting ~/drone_sorting_ros1_ws/src/
```

Install the Unity ROS-TCP-Endpoint:

```bash
cd ~/drone_sorting_ros1_ws/src

git clone -b main \
  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Install dependencies and build:

```bash
sudo apt install -y python-is-python3 ros-noetic-tf2-msgs

cd ~/drone_sorting_ros1_ws

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Running the Simulation

Open three terminals with the ROS 1 environment sourced.

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
roslaunch ros_tcp_endpoint endpoint.launch \
  tcp_ip:=127.0.0.1 \
  tcp_port:=10000
```

Terminal 3:

```bash
rosrun drone_sorting mission_manager.py
```

Open Unity and click Play.

The drone moves between Zone A and Zone B.

## Current Status

- ROS 1 and Unity communication: Working.
- Automated movement between sorting zones: Working.
- Obstacle avoidance: Planned.