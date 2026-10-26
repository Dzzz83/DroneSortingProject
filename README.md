# Drone Sorting Project

One warehouse demo: a drone finds a package with an RGB-depth camera, picks it up,
and delivers it to a sorting zone. **ROS owns the mission and movement; Unity owns
simulation, sensors, package physics, and rendering.**

This branch also includes a ROS-controlled two-warehouse course. See the
[Unity branch handoff](Unity/BRANCH_HANDOFF.md) for completed work, changed files,
test evidence, limitations and how to run it. The ROS instructions below use the
original `MainWarehouse` demo unless `--course` is selected.

## Run the demo

Use a **host terminal**, outside the ROS container, without `sudo`. Run the commands
from the **repository root**: the folder containing `README.md`, `ROS1/` and `Unity/`.

| When | Script to run |
| --- | --- |
| First setup or dependencies changed | `bash ROS1/scripts/setup_demo.sh` |
| Every run or restart | `bash ROS1/scripts/run_demo.sh` |
| Two-warehouse course | `bash ROS1/scripts/run_demo.sh --course` |

### First-time setup

Install the [requirements](#installation) first. The
[Setup and troubleshooting guide](docs/setup-and-troubleshooting.md) covers ROS/Distrobox
installation, custom Unity/Python paths, optional GPU inference and startup problems.

For a new checkout:

```bash
git lfs install
git clone https://github.com/Dzzz83/DroneSortingProject.git
cd DroneSortingProject
git lfs pull
bash ROS1/scripts/setup_demo.sh
```

For an existing checkout, enter its root and run `git lfs pull`, then
`bash ROS1/scripts/setup_demo.sh`.

Setup creates the detector environment, verifies the included model, and builds
the ROS workspace. It does not install Unity or system packages.
Wait for `[OK] Setup complete`.
Rerun setup after dependencies change, with the demo stopped. No training is needed.

### Run or restart

Open a host terminal and enter your checkout (replace `/path/to/DroneSortingProject`
with its actual location). Save Unity edits first: the runner closes this project's
previous demo editor.

```bash
cd /path/to/DroneSortingProject
bash ROS1/scripts/run_demo.sh
```

The runner builds the ROS workspace, opens `MainWarehouse`, and starts ROS and
YOLO. You do not need to run individual nodes or scripts in `scripts/internal/`.
Wait for **`[READY]`**, then press **Play** in Unity. Keep the launcher terminal open
and the **Game** view available. The saved scene is `Unity/Assets/Scenes/MainWarehouse.unity`.

### Stop and retry

Stop Play in Unity, then press **Ctrl+C** in the launcher terminal to stop ROS and
YOLO. Unity stays open. Save any edits, then use the same run command above to
start a fresh attempt.

### Two-warehouse course

Use `bash ROS1/scripts/run_demo.sh --course` after setup. This opens
`TwoWarehouseDelivery` and loads `ROS1/package/config/course.yaml`; wait for
`[READY]`, then press Play. ROS owns movement and package actions. The saved scene
disables `HardcodedWarehouseFlight`; keep it disabled for ROS operation.

The course profile observes the package near ROS `(34, 55)`, delivers to
`(34, 119.5)`, and uses a 3 m drone-root cruising height for the 4.5 m entrances.
Its pickup bounds, release height and travel timeout match the new layout.
Changing these settings does not create a safe route. Autonomous avoidance on
this course still requires validation. The original command and `demo.yaml`
retain their existing behavior. See [course coordinates and controls](Unity/TWO_WAREHOUSE.md).

A live ROS-TCP smoke test completed pickup, a known bypass and settled delivery
inside B with no reported contacts. This checks transport and package mechanics;
the full detector-guided mission and autonomous avoidance remain unvalidated.
[Verification details](Unity/Verification/ros-control-fixes.md).

## Installation

| Requirement | Supported setup |
| --- | --- |
| Host | x86-64 native Linux with a working graphics driver |
| Unity | Activated Editor **6000.6.0f1** |
| ROS | **Noetic**, native on Ubuntu 20.04 or in Distrobox `ros-noetic` |
| Python | Host Python **3.11** with `venv`/`pip`, plus host `python3` |
| Tools | Git + Git LFS, Bash, `flock`, `realpath`, `sha256sum`; OpenCV GL/GLib libraries |

Unity and the detector run on the host. Noetic is end-of-life; Windows/WSL is not
supported by these launch scripts. See [Setup and troubleshooting](docs/setup-and-troubleshooting.md)
for ROS/container installation and executable overrides.

## Workflow

```mermaid
flowchart LR
    A[DroneStart] --> B[Ascend and fly to observation]
    B --> C[Find and align with package]
    C --> D[Pick up and confirm attachment]
    D --> E[Ascend and carry to destination]
    E --> F[Lower, release and settle]
```

Check that the package is physically carried and rests inside the destination.
`COMPLETE` alone does not prove correct placement or collision-free travel.
A mission performs one delivery; repeated cycles and automatic failure recovery
are not implemented.

| Spectator camera control | Action (click the Game view first) |
| --- | --- |
| Right mouse, held | Enter free flight and look around |
| WASD; Q/E | Move; move down/up while holding right mouse |
| Shift | Move faster while holding right mouse |
| Release right mouse / Esc | Release cursor; leave view in place |
| F | Return to drone-follow view |

Free flight can pass through geometry. It does not move the drone or its sensor camera.

## What to change and where

Edit the **source** files below, not copies under `ROS1/build/`. Restart the demo
after ROS configuration changes. Stop Play before editing Unity and save the scene.

| I want to… | Edit here | What to change |
| --- | --- | --- |
| Move the delivery destination | [config/demo.yaml](ROS1/package/config/demo.yaml), `camera_pickup` | `dropoff_x`, `dropoff_y`; `carry_height` and `release_height` refer to the **drone root** |
| Move the visible destination zone | `SortingZoneAVisual` and `SortingZoneA` in the Unity Hierarchy | Move both to match the ROS destination; changing YAML does not move these objects |
| Change the starting position | `DroneStart` in `MainWarehouse`, and the drone transform | `DroneStart` is a scene marker; the mission starts from the drone's reported position |
| Change drone speed | [Demo settings](#demo-settings), `camera_pickup` | Separate speeds for travel, approach, descent, lift and lowering; `motion_controller.speed` sets the ceiling |
| Change the observation point or flight heights | [config/demo.yaml](ROS1/package/config/demo.yaml), `camera_pickup` | `observation_x`, `observation_y`, `observation_height`, `carry_height`, `release_height` |
| Change the package search area | Same YAML, `package_perception` | `pickup_bounds`: `[min X, max X, min Y, max Y]`; `surface_height_range` filters package tops |
| Change obstacle avoidance and feedback limits | Same YAML, `motion_controller` | `enable_obstacle_avoidance`, speed, stale-feedback timeouts and `maximum_command_lead` (metres) |
| Change the warehouse viewing camera | Select **Player → CinemachineCamera**, then **SpectatorCamera** in the Inspector | Follow view position, distance, focus offset, smoothing and free-camera speed; implementation: [SpectatorCamera.cs](Unity/Assets/Scripts/Warehouse/SpectatorCamera.cs) |
| Change the drone's perception camera | [DroneRgbCamera.cs](Unity/Assets/Scripts/Sensors/DroneRgbCamera.cs) | Mount position/rotation and lens/resolution defaults; this camera is created under `PackageRgbCamera` at Play |
| Change pickup/drop mechanics | [PackageHandling.cs](Unity/Assets/Scripts/Packages/PackageHandling.cs), drone/package prefab Inspectors | Detection range, package mount, gripper and attachment/release behavior |
| Change package placement or warehouse geometry | `MainWarehouse` scene; prefabs under `Unity/Assets/Prefabs/` | Edit saved scene objects or prefab assets, preserving required component references |
| Change detector/model behavior | [detector.py](ROS1/tools/detector.py), [model notes](ROS1/models/README.md) | Inference settings; model override and GPU settings are in the advanced guide |

**Coordinates:** metres; Unity `(X, Y, Z)` becomes ROS `(X, Z, Y)`. Unity **Y** and
ROS **Z** are vertical. For example, ROS destination `(12, 55)` lies at Unity
horizontal `(X=12, Z=55)`. Changing flight targets does not generate a new safe route;
check warehouse obstacles and package clearance.

### Demo settings

[ROS1/package/config/demo.yaml](ROS1/package/config/demo.yaml) is the main file for
ROS demo tuning. Edit it directly; ordinary tuning does not require changing
Python code or rerunning setup. Its three sections control:

| YAML section | What you can change |
| --- | --- |
| `camera_pickup` | Mission: destination, observation point, heights, speeds, pickup tolerances and phase timeouts |
| `motion_controller` | Movement: maximum speed, obstacle avoidance, feedback timeouts and command-lead limit |
| `package_perception` | Vision: package search bounds, accepted surface heights and maximum observation age |

All speeds below are **metres/second**, under `camera_pickup`:

| Parameter | Movement stage |
| --- | --- |
| `access_speed` | Ascend and fly to the observation point |
| `approach_speed` | Align above the selected package |
| `descent_speed` | Descend to pick it up |
| `lift_speed` | Lift immediately after attachment |
| `carry_speed` | Ascend to carry height and transport the package |
| `lowering_speed` | Lower at the destination before release |

For example, use **1 m/s** for the long flights by editing these existing values
(keep the rest of the YAML file):

```yaml
camera_pickup:
  access_speed: 1.0
  carry_speed: 1.0

motion_controller:
  speed: 1.0
```

`motion_controller.speed` is the **maximum allowed speed**, not a multiplier.
Every requested mission speed must be at or below it; higher requests are rejected
and stop movement. Keep `enable_dynamic_speed: true` so the mission selects each
stage's speed. Start by changing access/carry speeds while keeping pickup and
lowering slower.

`maximum_command_lead` is a distance limit, not a speed. If the controller logs
`WAIT_FOR_FEEDBACK`, it is waiting for Unity's reported position to catch up;
raising a requested speed alone may not increase actual movement speed.

To apply changes: stop Play, press **Ctrl+C** in the launcher, save Unity edits,
edit the YAML, then run `bash ROS1/scripts/run_demo.sh`. Wait for `[READY]` and
press Play. Verify obstacle avoidance, pickup and final placement at the new speeds.

## Project Structure

| Location | Responsibility |
| --- | --- |
| `ROS1/scripts/{setup_demo,run_demo}.sh` | The two supported user commands |
| `ROS1/scripts/internal/` | Environment selection, workspace build and ROS supervision |
| `ROS1/package/config/{demo,course}.yaml` | Original demo and two-warehouse mission/controller/perception settings |
| `ROS1/package/launch/` | Start ROS nodes and TCP endpoint; load configuration |
| `ROS1/package/nodes/` | Thin ROS entry points; `monitor.py` prints terminal status |
| `ROS1/package/python/drone_sorting/` | Actual mission, control and perception implementation |
| `ROS1/tools/` | Detector service, scoped process cleanup and memory check |
| `ROS1/tests/` | Regression tests; keep these with the source |
| `ROS1/models/`, `requirements/` | Included checkpoint/provenance and detector dependencies |
| `Unity/Assets/Scenes/`, `Prefabs/` | Saved warehouse scene, drone and package |
| `Unity/Assets/Scripts/` | Unity code, grouped by responsibility below |
| `Unity/Assets/Editor/DemoLauncher.cs` | Opens and validates the saved scene; waits for editor readiness |
| `Unity/Assets/UnityWarehouseSceneHDRP/` | Imported warehouse models, materials, lighting and decorative shelf script |
| `Unity/Assets/{Settings,Materials,DroneDelivery,TextMesh Pro,Shaders,Resources}/` | Rendering settings/materials, text shaders, depth shader and ROS connection prefab |
| `Unity/Packages/`, `ProjectSettings/` | Dependency versions and Unity project settings |
| `ROS1/build/`, `Unity/Library/` | Ignored generated dependencies/workspace/logs and Unity import cache |
| `docs/setup-and-troubleshooting.md` | Linux environment setup, executable overrides and troubleshooting |

The Catkin package is separate from `build/` to prevent ROS resource searches
following generated workspace symlinks back into the package.

## Main Components

| ROS module under `ROS1/package/python/drone_sorting/` | Responsibility |
| --- | --- |
| `mission/delivery.py` | ROS transport: read settings, receive feedback, publish targets/actions/state |
| `mission/delivery_guidance.py` | Observation access, carry, lower, release and settling |
| `mission/pickup_guidance.py` | Select/track a package, align, descend, request pickup and lift |
| `control/motion_controller.py` | One controller: absolute position commands at 20 Hz, avoidance, stale-data stops and bounded command lead |
| `perception/package_perception.py` | Synchronize camera inputs, call detector and publish candidates |
| `perception/camera_validation.py`, `camera_geometry.py` | Validate camera data and convert image/depth coordinates |
| `perception/package_localization.py`, `package_grasp.py` | Locate packages in the warehouse and estimate grasp readiness |
| `mapping/depth_map.py` | Local map: occupied, free-ray evidence and unknown space |
| `mapping/live_mapping.py`, `nodes/depth_mapping.py` | Synchronize forward observations, update the map and publish map/status topics |
| `interfaces/ros_topics.py` | ROS topic names; keep aligned with Unity's `DroneRosTopics.cs` |

| Unity scripts under `Unity/Assets/Scripts/` | Responsibility |
| --- | --- |
| `Warehouse/WarehouseBootstrap.cs`, `WarehouseSetup.cs` | On Play, wire ROS/sensors, configure the saved spectator and make the drone kinematic |
| `Warehouse/SpectatorCamera.cs` | Human viewing camera: follow and free flight |
| `ROS/DroneCommandSubscriber.cs`, `DroneStatePublisher.cs` | Apply ROS movement commands and report position |
| `ROS/DroneObstacleSensorPublisher.cs` | Seven distances: front, upper-front, lower-front, left, right, up, down; `-1` means clear within range |
| `Sensors/DroneRgbCamera.cs` | Drone sensor mount and lens |
| `Sensors/ForwardObstacleCamera.cs` | Opt-in forward RGB/depth camera with independent ROS-controlled heading |
| `Sensors/RgbDepthCapture.cs`, `RgbDepthFrame.cs` | Capture synchronized RGB/metric depth and hold frame data |
| `ROS/DroneCameraPublisher.cs`, `DroneCameraMessages.cs` | Publish image, calibration, poses and transforms |
| `ROS/PackageActionBridge.cs`, `DroneRosTopics.cs` | Translate ROS package actions/confirmations; shared topic names |
| `Packages/PackageHandling.cs`, `GripperController.cs` | Detect nearby packages, move jaws and manage pickup/drop |
| `Packages/DeliveryPackage.cs`, `DeliveryRotor.cs` | Attachment/release physics and rotor visuals |

The warehouse is a **saved scene**, not generated by a scene-builder script.
Some components are installed when Play begins, so they appear only at runtime.

## Current Data Flow

```mermaid
flowchart LR
    Sensor[Unity RGB-depth sensor] --> Perception[ROS perception]
    Perception <--> Detector[YOLO detector service]
    Perception --> Mission[ROS mission]
    Mission --> Controller[ROS motion controller]
    Controller --> Drone[Unity drone]
    Drone -->|Position and obstacle feedback| Controller
    Drone -->|Position feedback| Mission
    Mission -->|PICKUP / DROP| Packages[Unity package mechanics]
    Packages -->|PICKUP_DONE / DROP_DONE| Mission
```

| ROS topics | Data / purpose |
| --- | --- |
| `/drone/{current_position,target_position,command_position}` | `geometry_msgs/Point`: feedback → mission target → applied movement |
| `/drone/command_speed`, `/drone/travel_heading` | `Float32` speed and `Vector3` sensor heading |
| `/drone/obstacle_distances` | `Float32MultiArray`, ordered seven-sensor distances |
| `/drone/{package_action,package_action_status}` | `String` pickup/drop requests and physical-operation confirmations |
| `/drone/{mission_state,pickup_trial_status}` | `String` public mission state and detailed guidance JSON |
| `/drone/camera/{rgb/image_raw,depth/image_raw,camera_info,pose}` | RGB8, metric 32FC1 depth, calibration and camera pose |
| `/drone/current_pose`, `/tf`, `/drone/package_candidates` | Drone pose, transforms and perception JSON |
| `/drone/forward_camera/{rgb/image_raw,depth/image_raw,camera_info,pose,drone_pose}` | `Image` RGB8/32FC1 depth, `CameraInfo` calibration and `PoseStamped` camera/drone poses sharing a capture timestamp |
| `/drone/forward_camera/range_max` | `Float32`: maximum optical-axis depth, 15 m |
| `/drone/avoidance/{camera_enabled,camera_heading}` | `Bool` enable request and `Vector3` heading in ROS warehouse axes; only horizontal X/Y are used |
| `/drone/local_map/{occupied,free}` | Separate `sensor_msgs/PointCloud2` voxel centres in ROS `warehouse`; unknown cells are absent |
| `/drone/local_map/status` | `String` JSON: observation age, processing time, cell counts, reset sequence and wait/rejection reason |

Runtime node names `camera_pickup` and `package_perception` match the YAML sections.
They are ROS identities; the readable entry points are `mission.py` and `perception.py`.

## Verification

Stop the demo before tests; launcher tests bind demo ports. In a sourced Noetic shell
(enter `distrobox enter ros-noetic` first if needed), change to this checkout and run:

```bash
source /opt/ros/noetic/setup.bash
PYTHONPATH="$PWD/ROS1/package/python:$PWD/ROS1/tools:$PYTHONPATH" \
  /usr/bin/python3 -m unittest discover -s ROS1/tests
```

Host Python skips ROS-dependent checks. For Unity changes, compile in the editor,
then run ROS and Unity together and inspect physical pickup and placement.

### Depth-based sensor and local map

The forward camera is separate from the package camera: 320 × 240 RGB/depth at a
requested 6 Hz, mounted 0.5 m below the drone root. Calibration uses optical axes
X right, Y down, Z forward; camera/drone poses use ROS `warehouse` coordinates
(X/Y horizontal, Z up). Depth is optical-axis metres in [0.2, 15]. Positive infinity
means rendered no-hit within the declared range; NaN means invalid or too near.
The package camera retains its existing settings and NaN convention.

`WarehouseSetup` installs the forward sensor at runtime. It stays idle until a ROS
consumer requests capture. Consumers should repeat enable/heading requests until
frames arrive after startup or reconnection. Match bundles by acquisition timestamp,
not independent per-topic sequence counters. Drone body renderers are hidden only
during capture; carried-package geometry remains visible as a real occluder. Body
meshes use a per-renderer CPU fallback so HDRP applies that mask reliably; original
material properties are restored when the sensor is disabled.

The local map uses 0.25 m voxels in a window approximately 16 × 16 × 6 m centred
on the drone, retains observations for 2 seconds and samples every eighth pixel in
each direction. Finite returns mark occupied cells with a configurable 0.1 m
surface uncertainty band and contribute free-ray evidence before the surface.
Invalid/no-hit readings, unseen regions and expired evidence remain unknown.
Occupied evidence takes priority until it expires. A free-ray cell does not
certify clearance for the drone/package volume. Call `reset()` for a new trial.
These defaults are map parameters, not validated flight-clearance margins.

Replay existing project forward-camera recordings from the repository root
(`python3` needs NumPy; choose a new output directory for every run):

```bash
PYTHONPATH="$PWD/ROS1/package/python:$PYTHONPATH" \
  python3 ROS1/tools/replay_depth_map.py \
  --input /path/to/forward-recordings --output ROS1/build/depth-map-replay
```

The reader supports the project's live `frame-*.npz` recordings and Unity
`.json`/`.depth.f32` capture pairs. It exports `cells.csv` and `summary.json`;
missing/expired cells are unknown. It does not load Unity obstacle geometry.
The replay tool runs offline. For live observations, use the separate mapping-only
launch below. Replay mode has no planner. The live mapping node also exposes a *passive*
local planner that publishes only proposed paths and hold reasons, not movement
commands. It holds by default until a measured vehicle envelope is configured.

After normal project setup, open a Noetic terminal (`distrobox enter ros-noetic`
when using the container). From the repository root:

```bash
source /opt/ros/noetic/setup.bash
cd ROS1/build/linux-camera-workspace
catkin_make -j2 -l2
source devel/setup.bash
roslaunch drone_sorting depth_mapping.launch
```

Open `Unity/Assets/Scenes/TwoWarehouseDelivery.unity` in Unity and press Play.
Use this launch instead of `run_demo.sh` for mapping-only work: it starts just the
ROS-TCP endpoint and map node, without detector, mission or controller nodes.
Stop Play, then use Ctrl+C in the launch terminal to stop ROS. The node releases
its camera-enable request on shutdown.

Edit `ROS1/package/config/depth_mapping.yaml` for resolution, horizontal/vertical
window radii, history age, uncertainty margin, pixel stride, accepted observation
age, update rate and camera heading (ROS horizontal X/Y). The default map loop is
4 Hz and accepts observations no older than 0.5 s. Camera requests repeat at 1 Hz
through startup/reconnection. Synchronization uses acquisition timestamps, not
per-topic sequence numbers. There is one replaceable pending bundle; newer complete
observations supersede pending work instead of creating a processing backlog.

Inspect `/drone/local_map/status` with `rostopic echo`. If RViz is available, set
Fixed Frame to `warehouse` and add PointCloud2 displays for the occupied/free
topics. Point positions are voxel centres. Clouds are evaluated at their header
stamp; status separately reports the last accepted observation's age. `ACTIVE`
means the input is being mapped, not that a route is safe. Free cells remain
sampled-ray evidence, not clearance for the drone/package volume.

When input stops, status becomes `WAITING` with `stale_observation`. Both clouds
continue publishing and become empty as evidence exceeds the 2 s history age.
A changed course `reset_sequence` clears the map and pending observations; captures
preceding the received reset are rejected. Recovery after an input gap longer
than the accepted observation age starts a fresh history. Course observations
supply only the reset sequence; their package/geometry fields are not map inputs.

A stationary live integration check accepted 27 observations with no rejected
frames, produced occupied/free clouds and emptied both after input stopped.
The separate position subscriber received no samples, so that run did not verify
zero drift. Nineteen focused map/adapter tests and the catkin build passed.
Movement, bypass planning and full-course mapping remain unverified.

## Depth movement prototype

Milestone 2 adds an **opt-in** depth-only controller experiment. The default
heuristic stays unchanged. Before moving, measure and configure the full
drone envelope and read [depth-motion instructions](docs/depth-motion.md).
No live collision-free autonomous bypass has been verified.

## Current Status

The original warehouse demo and a two-warehouse course profile share one motion
controller and two public scripts. The course defaults to ROS control; spectator
cutaways affect only the viewing camera, preserving sensor geometry and colliders.
General route planning, repeated deliveries and automatic recovery are unfinished.
This branch includes the forward depth sensor and a live local-map node.
A conservative fixed-altitude planning prototype is available but is not
connected to movement. Depth-based movement integration remains unfinished.
Two consecutive local ROS/Unity runs completed physical pickup, carry, release and
settling inside the destination zone at a carry speed of **0.5 m/s**. Carry took
about 110 seconds in each run, within the 120-second limit. Those checks cover the
saved route at that speed.

A later [laptop AI feasibility probe](docs/avoidance-feasibility.md) completed one
existing delivery at 1 m/s with GPU package inference and a temporary forward
camera/inference load stage. The package was recorded released and stationary
inside the destination. This qualifies a lightweight perception prototype in the
current scene; it does not validate obstacle avoidance or the proposed larger
course. The probe did not change the saved scene or supported demo configuration.

The controller retains its command trajectory between feedback updates and waits
if commands would lead Unity's reported position by more than `maximum_command_lead`.

| Symptom | Check |
| --- | --- |
| Missing camera data before Play | Expected; press Play. If it persists, check Game view and Unity Console |
| Mission fails or holds | Read terminal guidance reason, `monitor.log`, ROS logs and Unity Console; stop Play and restart |
| Unity/Python missing, occupied ports, slow startup | [Setup and troubleshooting](docs/setup-and-troubleshooting.md) |

Each run prints `ROS1/build/camera-demo-*/`. Logs separate Unity (`unity.log`),
inference (`detector.log`), mission (`monitor.log`), build (`build.log`) and ROS
(`ros-logs/`). The next launch removes completed runs, preserves live logs, and
moves Unity recovery scenes to `Unity/UserSettings/SceneBackups/`. Keep
`Unity/Library/` to avoid full reimports. Save intentional edits before restarting.
