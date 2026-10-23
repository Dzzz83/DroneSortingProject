# Setup and troubleshooting

Use this guide to prepare another Linux machine, configure custom executable paths,
or diagnose startup problems. Check the [requirements](../README.md#installation)
first. For everyday commands, use [Run the demo](../README.md#run-the-demo);
for drone speed, destination and other mission tuning, use
[Demo settings](../README.md#demo-settings).

Run project commands from the repository root in your host terminal unless a step
explicitly says to run inside the ROS container.

## Additional prerequisites

Install system packages using your distribution's package manager. Setup installs
project dependencies; Unity and system packages must already be installed.

- OpenCV needs `libGL` and GLib libraries on the host.
- The scripts use Bash, `python3`, `flock`, `realpath`, and `sha256sum`.
- Distrobox needs a working Podman or Docker installation.
- A running systemd user manager enables memory isolation.
- First-time dependency installation needs internet access.

## ROS environment

Choose native ROS Noetic on Ubuntu 20.04 or Distrobox. Unity and YOLO run on the
host in both cases. Do not replace ROS's system Python with the detector's
Python 3.11.

For Distrobox, create the container once from your host terminal:

```bash
distrobox create --name ros-noetic --image docker.io/library/ros:noetic-ros-base-focal
distrobox enter --name ros-noetic
```

Inside the container, install the build and ROS packages, then return to the host:

```bash
sudo apt-get update
sudo apt-get install -y build-essential ros-noetic-catkin ros-noetic-message-generation \
  ros-noetic-message-filters ros-noetic-geometry-msgs ros-noetic-sensor-msgs \
  ros-noetic-std-msgs ros-noetic-tf2-msgs
exit
```

Native Noetic users install the same packages in their Ubuntu 20.04 environment
after installing ROS; they do not need the container commands.

The scripts prefer native ROS when `/opt/ros/noetic/setup.bash` exists. Otherwise,
they use container `ros-noetic`. To override that choice, set `ROS_ENVIRONMENT`
to `native` or `distrobox`; set `ROS_CONTAINER` to use another container name.
Use the same values for setup and launch.

ROS Noetic is end-of-life. The image tag and operating-system packages are not
locked snapshots. See [ROS Noetic status](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160).

## Python path

Setup normally finds `python3.11`. If yours has a different name or location:

```bash
DETECTOR_SETUP_PYTHON=/absolute/path/to/python3.11 bash ROS1/scripts/setup_demo.sh
```

Setup creates and uses `ROS1/build/detector-venv` automatically. Manual activation
is unnecessary. It also fetches a pinned Endpoint and builds the linked Catkin
workspace under `ROS1/build/`. It refuses to replace unrelated workspace packages
or use an Endpoint checkout with local changes or a different revision.

## Unity editor

Standard Unity Hub paths for version 6000.6.0f1 are detected. For another location:

```bash
UNITY_EDITOR=/absolute/path/to/Editor/Unity bash ROS1/scripts/run_demo.sh
```

To manage the editor yourself instead of letting the launcher reopen it:

```bash
UNITY_MODE=manual bash ROS1/scripts/run_demo.sh
```

Open the repository's `Unity` directory, select `MainWarehouse`, and configure
**Robotics → ROS Settings** for ROS1 at `127.0.0.1:10000`. Wait for `[READY]` in
the terminal, then press Play. Manual mode does not check editor readiness.

## NVIDIA GPU inference

CPU inference is the default. With an NVIDIA driver compatible with CUDA 12.4:

```bash
bash ROS1/scripts/setup_demo.sh --gpu
DETECTOR_DEVICE=0 bash ROS1/scripts/run_demo.sh
```

Set `DETECTOR_DEVICE=0` on subsequent GPU runs too. The optional CUDA installation
has not been validated by the current live checks. Perception rejects observations
older than `package_perception.maximum_age` in
[demo.yaml](../ROS1/package/config/demo.yaml), so inference speed affects delivery.

PyTorch is pinned to 2.5.1 and torchvision to 0.20.1. Other Python dependencies
are in [detector.txt](../ROS1/requirements/detector.txt). See the
[model notes](../ROS1/models/README.md) for provenance and licensing.

## Memory and startup

Keep `Unity/Library/` to reuse imported assets; a fresh import takes longer than
a cached restart. Close unused applications if startup runs short of memory.

With systemd available, host demo processes share an 8 GiB RAM / 2 GiB swap cap.
ROS inside Distrobox uses its container's resource group, outside the host cap.

| Setting | Purpose |
| --- | --- |
| `CAMERA_DEMO_MEMORY_MAX`, `CAMERA_DEMO_SWAP_MAX` | Override the RAM/swap caps. Check host capacity before increasing them. |
| `CAMERA_DEMO_MEMORY_MODE` | `auto` uses systemd when available; `required` refuses startup without it; `off` disables isolation. |
| `CAMERA_DEMO_MIN_AVAILABLE_GIB` | Optional positive minimum host available RAM checked before startup/restart. No minimum is enforced by default. |
| `UNITY_STARTUP_TIMEOUT` | Seconds to wait for editor startup, default 600. |

The launcher waits for editor startup work to settle before starting ROS/YOLO.
If startup takes longer than expected, inspect `unity.log` for the remaining
background tasks. Later actions or Play Mode can still increase memory use.

Fresh editor preferences disable automatic Search indexing. For an existing
checkout, turn off **Index on Editor startup** under **Edit → Preferences → Search**,
then close Unity before rerunning. Existing preferences are otherwise preserved.

## Logs and troubleshooting

Each run prints its directory under `ROS1/build/camera-demo-<timestamp>-<id>/`.
The next launch removes completed run output. Logs still used by an open Unity
editor in manual mode are retained. Startup preserves Unity recovery scenes in the
current run's `unity-scene-backups/` directory; before that run is later deleted,
they are moved to `Unity/UserSettings/SceneBackups/`.

| Log | Use it for |
| --- | --- |
| `unity.log` | Imports, shader compilation, editor readiness, connection/capture errors |
| `detector.log` | YOLO service startup and errors |
| `monitor.log` | Mission phases and vision results |
| `build.log` | Catkin build failures |
| `ros-logs/` | ROS node logs |

Before Play, `IDLE` and `missing_synchronized_camera_data` are expected. During
Play, look for repeated `DETECTIONS` or `NO_DETECTIONS`. If data remains missing,
open **Window → General → Game**, confirm Play is active, and check Unity's Console.

If a port is unavailable, identify its owner before stopping it. The demo needs
ports `11311`, `10000`, and `10001`; it refuses to terminate unrelated services.

## Other paths

Use these consistently during setup and launch when overriding defaults:

| Variable | Default |
| --- | --- |
| `DETECTOR_PYTHON` | `ROS1/build/detector-venv/bin/python` |
| `DETECTOR_WEIGHTS` | `ROS1/models/package_detector.pt` |
| `ROS_TCP_ENDPOINT_SOURCE` | `ROS1/build/ROS-TCP-Endpoint` |
| `CAMERA_DEMO_WORKSPACE` | `ROS1/build/linux-camera-workspace` |
