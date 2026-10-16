# Verification

This document records sensor, perception, and physical mission verification, with
reproduction instructions. See the
[project context](project_context.md) for architecture/settings and the
[README](../README.md) for installation and supported mission runs. The
[milestone plan](PLAN.md) separates the completed one-package delivery demo from
failure recovery and repeated delivery.

## Recorded results

The table below records the initial RGB-depth implementation at commit `30a980d`
(October 2026). Later perception and mission results are recorded in their own
sections. The [full-workflow checks](#full-workflow-from-dronestart) passed from
`DroneStart` in baseline and changed-placement/rotation trials. Later GUI runs,
launcher restart tests, spectator checks, and the latest 69-test ROS unit result
are recorded in [launcher and spectator verification](#launcher-and-spectator-verification).
Historical test counts below describe the code at each stage. Recheck affected
behavior when code, dependencies, scene geometry, or sensor settings change.

| Check | Completed result | Boundary of the evidence |
| --- | --- | --- |
| Unity C# compilation | Passed for camera capture, publishing, and editor verifiers. | Obsolete-API warnings existed; this was not a warning-free build. |
| Camera installation/views | One persistent camera; idempotent setup; pickup-area corners and three package bounds fit at 6 m observation height. Approach and open/closed-jaw pickup views passed and were visually reviewed. | Frustum checks alone do not detect occlusion. Coverage does not prove safe flight clearance. |
| RGB-depth capture | Known 2 m depth within the 2 cm test tolerance, off-axis optical depth, RGB/depth row alignment, sky invalidation, min/max filtering, and repeated capture passed. | Controlled probes and selected warehouse views, not accuracy across every pixel or transparent materials. |
| Capture timing | Five warmed editor observation captures averaged 64.8 ms each. | Synchronous GPU readback; not a runtime throughput or real-camera benchmark. |
| Unity message/geometry checks | Metadata, intrinsics, depth serialization including `NaN`, arbitrary drone rotation, optical handedness, and TF composition passed. | Controlled message fixtures, not real localization hardware. |
| Independent ROS decoding | ROS Noetic decoded all six saved Unity message types and checked metadata, depth, calibration, and TF translation. | Fixtures were synthetic 2 × 2 images; they do not establish real stream timing. |
| Python camera tests | All 15 tests passed in ROS Noetic: eight geometry tests and seven sensor-message contract tests. | Missing ROS dependencies cause the seven contract tests to skip outside ROS. |
| Catkin package build | Passed in an isolated temporary workspace using ROS Noetic. | Not an install-space deployment test or a fresh-machine installation. |
| Live sensor stream | Five synchronized 800 × 600 observations passed at roughly the configured 2 FPS. | Only endpoint and checker ran; no mission/motion controller or detector. |
| Live pose/TF check | Camera pose, drone position, and optical axes agreed with the `warehouse → drone_base → package_camera_optical` chain. | The current verifier also checks drone-body rotation; that added assertion was not separately rerun live in the recorded session. |

The README also records earlier fixed-coordinate warehouse delivery and
`SampleScene` static avoidance demonstrations. The new sensor-only tests do not
revalidate those missions. Synthetic acknowledgements from `test_mission.sh` or
`test_full_mission.sh` do not prove physical pickup/drop; mission `COMPLETE` alone
does not prove collision-free travel or correct final placement.

## Package dataset pilot

The editor exporter `Unity/Assets/Editor/WarehousePackageDatasetPilot.cs` creates
16 deterministic RGB views with one to three transportable demo packages, changed
placements/rotations and camera heights, two empty views, partial/full occlusion,
and two image-edge probes. These are editor test poses, not verified flight paths.
The warehouse's decorative boxes are background, not transportable-package labels.

Run **Drone Delivery → Export Package Dataset Pilot** outside Play Mode with all
scene edits saved. It uses the existing mounted camera and exports to a fresh
timestamped directory under `/tmp/drone-package-dataset-pilot/` by default.
`DRONE_DATASET_PILOT_OUTPUT` can select another empty directory outside
`Unity/Assets`. The exporter reloads the scene without saving and restores the
original scene setup, including on failure. Existing output is never overwritten.

Outputs are `images/`, `labels/`, `masks/`, `overlays/`, and `manifest.json`.
Each label row is `0 center_x center_y width height`, normalized to the image size
with a top-left origin. Empty/fully hidden images have empty label files. Boxes
enclose visible package pixels; they are not estimates of hidden full-object
bounds. An editor-only ID shader compares package fragments against the scene's
opaque depth with a 1 mm tolerance. Transparent occluders without depth writes
are unsupported. The manifest/masks use scene ground truth for label checking
only and must never become operational detector inputs.

Use the standalone checker with Python and Pillow installed:

```bash
python3 ROS1/tools/verify_package_dataset_pilot.py /path/to/export
```

It checks file dimensions, label classes/ranges, top-left conversion, mask bounds,
pixel counts, empty/full-occlusion cases, and clipped edge cases. Also inspect
overlays against the RGB images; mask consistency alone cannot establish that
every rendered surface was labeled correctly. `COMPLETE.txt` is written only
after the full export and scene restoration finish. A failed export is incomplete
and should not be used for training.

This pilot has no train/validation/test split, lighting randomization, broad
clutter variation, or real-camera data. Expand varied configurations and split
by configuration/session before assessing a trained detector.

On 12 October 2026, the exporter compiled and ran successfully in Unity
`6000.6.0f1` with GPU rendering. The current local output is
`ROS1/build/package-dataset-pilot/` (ignored by Git): 16 images, 24 package boxes,
and three zero-label views (two empty, one fully occluded). The mask/label checker
passed, and the overlay contact sheet was visually reviewed. Six independent
checker tests passed, including wrong classes, inverted rows, invalid normalized
coordinates, labels on empty images, and incomplete exports:

```bash
python3 -m unittest discover -s ROS1/tools -p 'test_verify_package_dataset_pilot.py' -v
```

Existing obsolete-API and warehouse lighting-data warnings were present in the
Unity log; no compiler or shader errors were reported. Git showed no scene,
prefab, or ProjectSettings changes after export. No ROS mission or training ran.

## Package detector training trial

On 12 October 2026, **Drone Delivery → Export Package Training Trial** compiled
and exported 200 RGB images in Unity `6000.6.0f1`. The original 16-image pilot
menu remains available. The training export uses 100 deterministic package
configurations, each with an observation view at root height 5.6–6.8 m and an
approach view at 2.5–3.5 m. Package centers vary inside the pickup area with
at least 1 m separation, random rotations, zero to three active packages, and
occasional temporary opaque occluders. Camera horizontal placement (Unity X/Z) and yaw
also vary. These are editor references, not flight-clearance validation.

Configurations 0–69 belong to training, 70–84 to validation, and 85–99 to test;
both views of each configuration stay in the same split. The seed is `20261012`.
Counts are 140/30/30 images, 306 visible package boxes in total, and 40 empty
images. The checker passed all masks/labels, split counts, paired poses, unique
configuration signatures, and exact RGB duplicate checks. Label preview samples
from every split were visually reviewed. The same warehouse, lighting, lens,
package appearance, and decorative clutter are shared across splits; this is
held-out placement testing, not independent real-world or warehouse testing.

Training used local COCO-pretrained `yolo11n.pt` (AGPL-3.0/enterprise licensing)
and Ultralytics `8.3.195`, PyTorch `2.5.1+cu124`, Python `3.11.16`, and the RTX
3060 Laptop GPU. The isolated environment remains under
`ROS1/build/yolo-feasibility/venv/`; ROS Noetic's Python environment is unchanged.
The installed dependency lock is
`ROS1/build/package-training-trial/requirements.lock.txt`. Training ran for 50
epochs at image size 640, batch 4, AdamW learning rate 0.001, seed `20261012`,
workers 0, AMP disabled, mosaic probability 0.5, and mosaic disabled for the last
10 epochs. AMP was disabled to avoid its auxiliary model download/check. An
existing local DejaVu font was copied as the plotting font; no weights were
downloaded during training. The best checkpoint was selected using validation,
and no test metrics were used for tuning. Training took 234.3 seconds.

The selected checkpoint was evaluated once on the 30 held-out test images:

| Measure | Result |
| --- | --- |
| At confidence 0.25 and matching IoU ≥ 0.5 | 45 true positives, 0 false positives, 0 missed labels |
| Observation views | 24/24 labeled packages detected across 15 images |
| Approach views | 21/21 labeled packages detected across 15 images |
| Empty views | No detections in all 6 empty images |
| Mean IoU for matched boxes | 0.9617 |
| Standard mAP at IoU 0.5 / IoU 0.5–0.95 | 0.995 / 0.9661 |
| Warmed full prediction call, 20 repetitions | Mean 16.6 ms, including file load/pre/postprocessing |

The fixed-threshold counts and the standard AP confidence sweep are different
measurements. Annotated predictions for all 30 test views were visually reviewed.
Thirteen focused tool tests passed, covering labels, split leakage, duplicate
placements, and detection matching. No scene/prefab/ProjectSettings diffs were
produced by capture. Existing warehouse lighting-data/obsolete-API warnings
were present, with no compiler or shader errors. No ROS mission ran.

Local artifacts are ignored by Git under `ROS1/build/package-training-trial/`:

- `dataset/`: RGB images, YOLO labels, masks, label previews, grouped manifest,
  and `packages.yaml` with the split paths.
- `training/weights/best.pt`: validation-selected single-class package detector.
- `training/`: saved arguments, per-epoch CSV/plots, and `training-report.json`.
- `evaluation/`: AP plots, per-image predictions/counts in
  `evaluation-report.json`, and `test-predictions.png` (green truth, red prediction).
- `REPORT.md`: compact local result/provenance report.

To repeat capture, stop Play Mode, save existing scene edits, and use the menu.
Default output is a fresh directory under `/tmp/drone-package-training-trial/`;
`DRONE_DATASET_TRIAL_OUTPUT` selects an empty alternative outside `Unity/Assets`.
The exporter restores scene setup without saving, and refuses existing output.
To check, train, or evaluate from the repository root in the isolated detector
environment:

```bash
export YOLO_CONFIG_DIR="$PWD/ROS1/build/package-training-trial/config"
export MPLCONFIGDIR="$PWD/ROS1/build/yolo-feasibility/matplotlib"
export WANDB_MODE=disabled
ROS1/build/yolo-feasibility/venv/bin/python ROS1/tools/verify_package_dataset_pilot.py ROS1/build/package-training-trial/dataset
ROS1/build/yolo-feasibility/venv/bin/python ROS1/tools/train_package_detector.py --dataset ROS1/build/package-training-trial/dataset --weights ROS1/build/yolo-feasibility/weights/yolo11n.pt --output ROS1/build/package-training-trial/training-new --epochs 50
ROS1/build/yolo-feasibility/venv/bin/python ROS1/tools/evaluate_package_detector.py --dataset ROS1/build/package-training-trial/dataset --weights ROS1/build/package-training-trial/training-new/weights/best.pt --output ROS1/build/package-training-trial/evaluation-new
ROS1/build/yolo-feasibility/venv/bin/python -m unittest discover -s ROS1/tools -p 'test_*.py' -v
```

The scripts require local weights and fresh run directories. Keep test data
held out for each final evaluation; these commands are reproduction instructions,
not encouragement to tune against the test set. This training trial exported RGB
only and did not test depth localization or ROS/Unity runtime integration. Those
were tested in the later stationary and delivery trials below. Robust empty-zone
confirmation, severe occlusion, and lighting/appearance generalization remain
unverified.

## Repeat the focused Python checks

From the repository root, in the ROS Noetic Python environment:

```bash
PYTHONPATH="ROS1/src/drone_sorting/src:$PYTHONPATH" python3 -m unittest discover -s ROS1/src/drone_sorting/tests -v
```

Source `/opt/ros/noetic/setup.bash` first and preserve its Python path as above.
On 13 October 2026, all 69 tests passed in ROS Noetic with no skips. They cover
camera contracts and geometry, package localization/grasp fitting, pickup and
delivery guidance, motion/feedback controls, recorder output, and monitor failure
reporting, including a nonzero CLI exit status. No Unity session
or ROS master is required for these unit tests. ROS-dependent tests may skip in a
host environment without Noetic; use the ROS environment for the complete suite.
Earlier suite counts in the historical sections reflect their implementation stage.

## Repeat the Unity checks

Open `Unity/Assets/Scenes/MainWarehouse.unity`, stop Play Mode, and let Unity
finish importing/compiling. Use the **Drone Delivery** menu:

- **Install and Verify Warehouse RGB Camera**: save existing scene edits first.
  Checks the mount, duplicate prevention, and view coverage; renders four views,
  restores the drone/jaws, then saves the installation. Requires the three-package
  demo. Inspect the rendered images separately for occlusion.
- **Verify Warehouse RGB Depth Capture**: tests temporary colored probes and
  warehouse/gripper views, restores scene poses, and does not save the scene.
- **Verify Camera ROS Messages**: checks calibration, encoding, rotations, and
  TF composition using a temporary drone, then writes ROS1 binary fixtures.

Check the editor log for compiler errors/exceptions and the success markers
`WAREHOUSE_RGB_CAMERA_VERIFIED`, `WAREHOUSE_RGB_DEPTH_VERIFIED`, and
`DRONE_CAMERA_MESSAGES_VERIFIED`. Review output reports and images; successful
compilation alone does not mean the verifiers ran.

Outputs use the system temporary directory:

| Directory | Contents |
| --- | --- |
| `drone-permanent-rgb-verification/` | Observation, approach, open/closed-jaw RGB previews and report |
| `drone-rgb-depth-verification/` | Probe/warehouse RGB and grayscale depth previews plus report |
| `drone-camera-message-verification/` | Six serialized ROS1 messages and report |

On Linux these usually live under `/tmp`. Files may disappear or be overwritten
by reruns; they are not tracked regression artifacts. Depth PNGs are grayscale
previews, not the metric floating-point data. Preserve an explicitly needed
result before cleanup, and do not assume host/container temporary paths are shared.

## Independently decode saved Unity messages

After the Unity message verifier succeeds, run in ROS Noetic from the repository
root, replacing the final path if necessary:

```bash
PYTHONPATH="ROS1/src/drone_sorting/src:$PYTHONPATH" python3 ROS1/src/drone_sorting/tests/verify_unity_camera_messages.py /tmp/drone-camera-message-verification
```

Copy the fixture directory into the ROS container first if its temporary
filesystem is separate. This verifier reads files only; it starts no mission.

## Repeat the live sensor-only check

1. Build/source the Catkin workspace as described in the README. Inspect helper
   scripts before using them: workspace sync replaces the destination package,
   and mission runners can terminate existing ROS/Unity processes.
2. Ensure no other endpoint or delivery mission is running on port 10000. Set
   ROS `use_sim_time=false`; timestamps use UTC wall time. Separate computers
   require synchronized clocks.
3. In the ROS environment, run:

   ```bash
   roslaunch drone_sorting camera_validation.launch required_frames:=5
   ```

4. Start MainWarehouse Play Mode and keep Unity focused. Background running is
   disabled in the current project settings. This launch starts only the endpoint
   and read-only checker; it does not command the drone to observation height.
5. Expect `CAMERA_STREAM_CHECK_PASSED frames=5`. `required_frames:=0` keeps the
   checker running. A passing checker validates message contracts/freshness and
   exercises center-pixel geometry, not full image quality or package detection.
6. While the endpoint and Unity remain running, optionally check pose/TF consistency
   from the repository root in another ROS terminal:

   ```bash
   PYTHONPATH="ROS1/src/drone_sorting/src:$PYTHONPATH" python3 ROS1/src/drone_sorting/tests/verify_live_camera_transforms.py
   ```

   It waits up to 15 seconds for synchronized poses/transforms and sends no commands.
7. Stop the launch with Ctrl+C and stop Unity Play Mode. The finite checker exits
   after five frames, but the endpoint/master can remain until the launch stops.

## Stationary live package perception

The October 12, 2026 trial ran ROS Noetic, the loopback RGB inference worker,
and Unity MainWarehouse Play Mode together. The model was the trial-trained
YOLO11 nano checkpoint, confidence 0.25, image size 640, GPU 0. Unity supplied
synchronized 800 × 600 RGB/depth, camera calibration, and optical pose at 2 FPS.
ROS produced candidates without package-coordinate inputs. Node/topic inspection
confirmed only perception and logging publications, with no publishers on the
movement or package-action topics.

The agreed 10 cm limit applies to **visible surface** localization, not grasping.
An editor-only evaluator temporarily positioned the drone and packages, then
independently raycast each selected sensor pixel against package colliders.
The evaluator uses sensor resolution, not the editor Game View resolution.
Test transforms/known counts are used only by this evaluator. Play Mode changes
were discarded; no scene or prefab was saved.

| Case | Evaluated frames | Candidates/frame | Maximum 3D surface error |
| --- | ---: | ---: | ---: |
| Three packages at observation height | 37 | 3 | 0.33 cm |
| Changed placements and rotations | 17 | 3 | 0.33 cm |
| Approach height | 17 | 3 | 1.27 cm |
| Partial opaque occlusion | 17 | 3 | 0.33 cm |
| Empty pickup area | 17 | 0, `NO_DETECTIONS` | — |
| Camera publishing disabled | One recorded health result | 0, `UNAVAILABLE` | — |

All 264 surface estimates matched three distinct package colliders in their
respective frames and met the 10 cm limit. Total 3D error also bounds both
horizontal and vertical error. Median ROS processing time was 19.8 ms; maximum
acquisition-to-result age was 97.5 ms for the evaluated valid observations.
The repeated frames are not 88 independent scenes: there are four positive
test views/configurations in one warehouse, with unchanged lighting/package assets.
No false candidate occurred in the 17 empty frames. This population does not
establish general robustness or reliable search-area emptiness.

All 27 camera, localization, and perception tests passed in ROS Noetic. These
include invalid/truncated depth, background floor rejection, area bounds,
duplicate candidates, stale/future frames, detector outage, and response sequence
mismatch. Detector outage and invalid-data behavior were unit-tested; missing
camera behavior was also observed live. Missing local weights rejected worker
startup explicitly without a download. Catkin built in an isolated workspace.
Unity compiled and ran the trial; existing obsolete-API and lighting-data warnings
remain. The first evaluator attempt had an image-dimension mismatch; the table
uses only the corrected rerun.

Local ignored evidence is in `ROS1/build/stationary-perception/`:
`unity-evaluation.json`, `observations.jsonl`, and `evaluation-report.json`.
These reports require the local trained checkpoint/environment and are not
included in a fresh checkout.

To reproduce, start the worker and perception-only launch as documented in the
README. In another ROS terminal from the repository root, run:

```bash
python3 ROS1/tools/record_package_perception.py --output "$PWD/ROS1/build/stationary-perception"
```

Close other Unity editors for this project, save scene edits, then launch a separate
GPU-enabled batch editor (replace its executable path for your installation):

```bash
/path/to/Unity -batchmode -projectPath "$PWD/Unity" \
  -executeMethod StationaryPackagePerceptionTrial.Run \
  -logFile "$PWD/ROS1/build/stationary-perception/unity.log"
python3 ROS1/tools/evaluate_stationary_perception.py ROS1/build/stationary-perception --maximum-error 0.1
```

Do not use `-nographics` or `-quit` for the live trial: rendering is required and
the harness exits after the cases. It refuses interactive-editor invocation,
dirty scenes, and existing Play Mode. Reports are overwritten on rerun; the
recorder appends history. Keep the recorder running until the trial exits, then
stop the recorder, perception launch, and worker with Ctrl+C. No delivery launch
should run alongside this test.

The localization prior accepts pickup-area surfaces between ROS Z 0.2–1.2 m,
rejecting the floor and high occluders. This is designed for the current 0.6 m
demo boxes. It is not segmentation and can accept unrelated geometry inside
those bounds if the detector misidentifies it. Severe occlusion, novel clutter,
changed lighting, broader tracking conditions, and hardware sensing remain
untested. Later delivery trials exercised tracking during approach and descent
in the two controlled layouts.

## Camera-guided pickup work in progress

The upright-box grasp-center fit was added after the stationary surface trial.
It fits a yaw-rotated 0.6 m square to RGB-selected top-face depth, requiring a
mostly visible, near-horizontal top. It reports a center only when the top fits
the size/coverage checks; it is not a general 3D box pose estimator.

A second stationary Unity run compared accepted centers to package transforms in
the test evaluator. Of 264 surface candidates, 230 produced a grasp center and
matched the corresponding package. Maximum measured center error was 0.0062 m
(0.62 cm). Center estimates were available for 3/3 boxes in the observation view,
3/3 after changed placements, 2/3 in the lower approach view, and 2/3 with the
test occluder. Empty and missing-camera cases produced no grasp samples. This is
stationary synthetic-scene evidence only; it did not enable a movement controller.
Evidence is local under `ROS1/build/camera-pickup/grasp/` in
`unity-evaluation.json` and `observations.jsonl`.

The working tree now also contains `PickupGuidance`, a ROS adapter, a separate
`camera_pickup.launch`, and optional dynamic-speed/feedback-watchdog support in the
motion controller. They implement target locking, fresh-frame checks, a bounded
approach and descent, one pickup request, acknowledgement-gated lift, and failure
status. On 12 October 2026, all 39 unit tests passed in ROS Noetic and the current
package built successfully in a disposable isolated catkin workspace. These checks
did not exercise the live guidance/motion control loop. Moving pickup was still
unverified at that stage; the later baseline trial below supplies that evidence.
The normal motion launch
keeps dynamic speed disabled and its configured speed unchanged.

Guidance now resets initial confirmation and alignment-refinement counts after
unavailable, unreliable, stale, or unmatched observations, including silent camera
gaps. It preserves the locked center and loss timer. Malformed candidate messages
use the same reset path. Regression tests cover interrupted confirmation/refinement,
stale frames, missing grasp estimates, unmatched packages, and recovery on the same
package. Unity ran the earlier stationary center evaluation without compiler errors.

The stationary trial was repeated after this fix on 12 October 2026, using the
existing perception-only ROS launch, a newly started detector worker, and a separate
recorder. All six cases passed: 83 positive frames yielded 249 surface estimates
with maximum error 1.27 cm; 215 accepted grasp centers had maximum error 1.18 cm.
Each observation/changed-placement frame supplied three centers; approach and
partial-occlusion frames supplied two. All 17 empty frames had no candidates;
disabled capture reported `UNAVAILABLE`. Median ROS processing time was 81.1 ms,
and maximum evaluated observation age was 383.5 ms. This repeats stationary
synthetic-scene evidence, not moving guidance or physical pickup verification.

No connection failure was observed during active capture. At startup, the earlier
perception launch and recorder were still running, but no detector was listening
on port 10001; the worker was started with the existing local checkpoint. This
does not establish the cause of any earlier connection error. Unity exited normally
without compiler errors, and no scene/prefab/ProjectSettings diffs were produced.
The rerun worker and separate recorder were stopped; the pre-existing ROS launch
and recorder were left running. Evidence is ignored under
`ROS1/build/camera-pickup/confirmation-fix/`: `unity-evaluation.json`,
`observations.jsonl`, `evaluation-report.json`, and `unity.log`.

### Pickup control-loop review

On 12 October 2026, eight focused ROS tests reproduced and verified fixes for
zero speed being ignored, missing speed updates retaining the previous speed,
nonfinite coordinates refreshing feedback timers, invalid timeout configuration,
and startup sleeping while holding the guidance callback lock. Guidance now sends
zero speed while observing, holding after perception loss, waiting for pickup,
or in a terminal phase. The adapter snapshots position under the lock and sleeps
outside it. Pickup acknowledgement remains required before lift.

Dynamic control waits for its first valid speed input, accepts zero as hold,
and stops command publication on invalid speed or expired position/target/speed
feedback. The pickup launch uses a 0.5 m/s cap and 0.5 second timeouts, with a
single pending ROS position command. Fixed-speed mode still defaults to 2 m/s;
tests verified its normal step and a final step without overshoot. All 47 tests
passed in ROS Noetic and the disposable isolated catkin build passed.

Read-only launch expansion contained `/unity_endpoint`, `/package_perception`,
`/camera_pickup`, and `/motion_controller`, without the fixed-coordinate mission
manager. The existing ROS graph had no publishers on target, command-position,
command-speed, or package-action topics. Warehouse setup disables `DeliveryDemo`;
Unity applies the position command and sends `PICKUP_DONE` only after attachment.
No movement launch or Unity trial ran during this review. These are unit/static
checks, not measured transport stopping latency: speed/target topics arrive
separately, and commands already in transit can still be applied by Unity.

### Physical pickup evaluator

The evaluator has three responsibilities with separate evidence files:

- `CameraPickupTrial.cs` initializes baseline or changed package test poses only
  in Play Mode, associates the first `APPROACH` center with exactly one package
  within 10 cm, and keeps that evaluation identity. It records ROS-coordinate
  positions, phase/status age, attachment ownership/parent, and hold state at up
  to 10 Hz. It never sends package ground truth to ROS. Pickup runs are bounded at 150 s;
  delivery runs now use 330 s and add release, velocity, bounds, and penetration evidence.
- `record_camera_pickup.py` subscribes to guidance, action, acknowledgement, and
  speed topics. It writes UTC receipt times to `ros-events.jsonl` and atomically
  replaces `latest-guidance.json` for the harness. It publishes no commands and
  refuses an existing history file.
- `evaluate_camera_pickup.py` requires one pickup request, ordered acknowledgement,
  the same selected/attached package, guidance completion, and sustained package
  and drone rise. Default lift is 1 m with 5 cm tolerance. It checks attachment
  loss, wrong package, premature lift, timeout, stale final status, and hold drift
  after a 0.5 s transport grace. Original samples preserve motion during that grace.

Initial evaluator checks on 12 October 2026 passed all 14 synthetic tests,
including the CLI's
complete/incomplete evidence paths. All 48 ROS Noetic tests passed, including
mocked recorder subscriptions and file output. Unity `6000.6.0f1` compiled the
harness and ran `CameraPickupTrial.VerifySetup` in edit mode, checking package
references and scene-state serialization/restoration. The success marker was
`CAMERA_PICKUP_EVALUATOR_SETUP_VERIFIED packages=3`. Existing Unity warnings remain.
This initial evaluator check did not run Play Mode, moving pickup, or collector/ROS
transport integration; subsequent trials exercised them together. No scene,
prefab, or ProjectSettings changes were saved. The compile/setup log is
ignored under `ROS1/build/camera-pickup/evaluator-verification/unity.log`.

Repeat the checker tests from the repository root:

```bash
python3 -m unittest discover -s ROS1/tools -p 'test_evaluate_camera*.py' -v
/path/to/Unity -batchmode -quit -projectPath "$PWD/Unity" \
  -executeMethod CameraPickupTrial.VerifySetup -logFile /tmp/pickup-evaluator-setup.log
```

To repeat the moving trial, first build/source ROS, stop any conflicting
delivery/perception launch, and ensure no competing control publishers. Use a
fresh shared output directory outside `Unity/Assets`, and synchronized UTC clocks
with ROS wall time. Start `roscore` and the detector worker from the README,
adding `--context-scale 1.5` for the verified baseline configuration.
In a ROS terminal, start the read-only recorder before guidance:

```bash
export PICKUP_TRIAL_OUTPUT="$PWD/ROS1/build/camera-pickup/physical-trial-new"
PYTHONPATH="ROS1/src/drone_sorting/src:$PYTHONPATH" python3 ROS1/tools/record_camera_pickup.py \
  --output "$PICKUP_TRIAL_OUTPUT"
```

In a separate host terminal at the repository root, launch the GPU-enabled harness:

```bash
DRONE_PICKUP_TRIAL_OUTPUT="$PWD/ROS1/build/camera-pickup/physical-trial-new" \
DRONE_PICKUP_TRIAL_SCENARIO=baseline DRONE_PICKUP_LIFT_DISTANCE=1 \
  /path/to/Unity -batchmode -projectPath "$PWD/Unity" \
  -executeMethod CameraPickupTrial.Run \
  -logFile "$PWD/ROS1/build/camera-pickup/physical-trial-new/unity.log"
```

Wait for `CAMERA_PICKUP_TRIAL_READY`, then start `roslaunch drone_sorting
camera_pickup.launch` promptly. Use a fresh launch and output for every trial.
`changed` selects changed placements/rotations; `DRONE_PICKUP_LIFT_DISTANCE` must
match the guidance node's `lift_clearance`. These are evaluation settings, not
ROS package-coordinate inputs. Do not use `-quit` or `-nographics` for `Run`.
The harness exits one second after a terminal phase or at timeout, discards Play
Mode changes, and restores the initial saved scene setup; a fresh editor with no
loaded scene returns to an unsaved empty scene. `COMPLETE.txt` marks completed
capture/restoration, not success. Stop the recorder, launch, and worker, then
verify the actual processes have exited. Container wrappers may stop before their
ROS children; check ownership and stop only the trial processes. Evaluate the files:

```bash
python3 ROS1/tools/evaluate_camera_pickup.py ROS1/build/camera-pickup/physical-trial-new
```

Inspect `evaluation-report.json` alongside `manifest.json`, `unity-samples.jsonl`,
`ros-events.jsonl`, and the Unity log. ROS event times are receipt times; ambiguous
transport ordering or inadequate sampling should fail verification and be
investigated, rather than treated as physical success. Baseline/changed pose setup
and delivery capture are automated; live camera-loss and stale-feedback injection
were unverified at that stage; the full-workflow section records later checks.
The delivery trials below exercised baseline and changed
placements with the detector setting above.

### Automated baseline pickup and lift

On 12 October 2026, ROS and Unity ran together in batch Editor Play Mode. The
baseline started at the observation waypoint with avoidance disabled. Package
coordinates were used only to set up and evaluate the scene; guidance selected
its target from RGB-depth observations.

The first two attempts stopped safely during descent with `lost_locked_package`.
Neither sent a pickup request or attached a package, and measured hold drift was
zero. Fresh images continued arriving: the selected package was visible, but the
detector stopped recognizing it at close range while retaining the neighboring
package. The saved descent image reproduced this detection failure offline.

An optional `--context-scale 1.5` adds a second inference view with gray padding,
then maps detections back to original sensor pixels. It retains the normal view,
trained checkpoint, and 0.25 confidence threshold. The saved image recovered the
selected box at approximately 0.929 confidence. Three coordinate-mapping tests
passed, and an HTTP request to the actual worker confirmed that recovery. The
existing localization logic removes duplicate surface candidates. Default scale
1 preserves the previous single-view behavior.

The third attempt passed the independent physical checker:

- Exactly one `PICKUP` request was followed by Unity's `PICKUP_DONE`.
- Selected `package-1` was the attached package, with matching ownership and parent.
- Package and drone both rose 0.99 m, satisfying the 1 m lift with 5 cm tolerance.
- The run recorded 446 physical samples, including 55 attached samples and 11
  final `VERIFIED` samples spanning one second. Measured hold drift was zero.

Local ignored evidence is under `ROS1/build/camera-pickup/baseline-01/`,
`baseline-02/`, and `baseline-03/`. The third directory contains `run-config.json`,
`manifest.json`, `ros-events.jsonl`, `unity-samples.jsonl`, `COMPLETE.txt`,
`evaluation-report.json`, and Unity/ROS logs. The second also preserves the
close-view diagnostic image and perception observations. Failed attempts remain
available for comparison.

Unity exited normally after the successful capture and restored scene setup.
Initial connection refusals occurred before the endpoint started; after Unity
exited, the ROS position watchdog reported unavailable feedback. That later status
is outside the captured physical trial and does not change its final attached
`VERIFIED` samples. All task-owned Unity, ROS, recorder, and detector processes
were stopped. No scene/prefab/ProjectSettings changes were saved.

This pickup-only run verified one controlled baseline. The subsequent delivery
trials below cover changed placements/rotations and transport. Live injected
observation loss was tested in the later full-workflow trials; hardware
performance remains unverified.

### Camera-guided single-package delivery

`DeliveryGuidance` extends the existing pickup phases rather than duplicating
perception or grasp logic. `camera_delivery.launch` enables delivery through the
shared camera launch; pickup-only and fixed-coordinate demos remain separate.
ROS is still the sole movement authority. Pickup and drop acknowledgements gate
advancement, and perception cannot update the locked grasp after pickup. The
recorded run configurations contain the exact settings; their defaults and
meaning are documented in the
[README](../README.md#camera-guided-single-package-delivery).

The zone visual has Unity bounds centered at `(28, 0.1, 55)`, size `(5, 0.04, 5)`;
its collider is disabled. A downward physics ray found the actual floor at Y=0,
so the expected settled package center is Y=0.3. Ground-truth floor and zone data
are recorded only in the evaluator manifest. ROS release height is configured,
accounting for the 0.95 m mount offset. Unity compiled the evaluator and a
conservative envelope sweep found no active collider hits on vertical ascent,
straight carry, or destination lowering. Route checks do not establish general
warehouse navigation; avoidance remains disabled.

On 12 October 2026, the ROS package built in a disposable isolated Catkin workspace.
All 57 Noetic tests passed, including acknowledgement timing, delivery timeouts,
stale feedback holds, terminal-state preservation, and the existing public mission
sequence. All 21 synthetic pickup/delivery checker tests passed. The checker now
requires attachment throughout carry, matching package/drone offsets, one ordered
release request/acknowledgement, detachment, full horizontal package bounds inside
the zone, floor height, low linear/angular speed, and at least 0.5 s of sustained
final placement. Active collider penetration is sampled during the trial.

The full baseline batch Unity trial passed independently: the same selected
`package-1` attached, lifted 0.99 m, was carried to the zone, and detached and
settled at Unity `(28, 0.30, 55)`. Maximum measured carry-offset drift was below
0.000001 m and hold drift was zero. No sampled collider penetrations were recorded.
One pickup and one release were acknowledged by Unity. Evidence is ignored under
`ROS1/build/camera-delivery/baseline/`, including manifest, ROS events, Unity samples,
logs, configuration, and `evaluation-report.json`.

The first changed-pose attempt was invalid because the prior ROS launch outlived
its stopped container wrapper. The new recorder received its old `COMPLETE` status;
no package was selected or delivered. The physical checker rejected the attempt,
and evidence remains under `ROS1/build/camera-delivery/changed/`. Scoped cleanup
stopped the identified processes, and a ROS graph check confirmed no remaining
control publishers before retry. The harness now also requires the first fresh
guidance status to be `OBSERVE`, rejecting leftover terminal statuses explicitly.
The fresh changed-placement/rotation retry passed, with package yaw rotations of
25, 60, and 95 degrees and horizontal offsets from the baseline. The selected
package lifted 0.975 m (within the configured 5 cm tolerance), stayed attached
through transport, and settled at the same final zone center. Baseline and retry
recorded 1513 and 1434 physical samples over 154.6 and 148.2 s, respectively,
including startup. Each recorded 11 final `COMPLETE` samples over approximately
one second, one `PICKUP`, one `DROP`, and their corresponding acknowledgements.
Neither recorded collider penetration or hold drift. Retry evidence is under
`ROS1/build/camera-delivery/changed-02/`.

Unity compiled and exited normally for both successful trials. Scene setup was
restored, and no scene/prefab/ProjectSettings changes were saved. Task-owned launch,
recorder, detector, and master processes were stopped using identified process
IDs, then checked for leftovers. Logs and run configurations are retained; the
disposable Catkin workspace was removed after verification.

To repeat delivery, use the [physical trial procedure](#physical-pickup-evaluator)
with the detector's `--context-scale 1.5`. Set
`DRONE_PICKUP_TRIAL_MISSION=delivery` on the batch Unity command and start
`roslaunch drone_sorting camera_delivery.launch` after `CAMERA_PICKUP_TRIAL_READY`.
Use scenario `baseline` or `changed`, a fresh output directory, and the same
read-only recorder and `evaluate_camera_pickup.py` command. Confirm no publishers
remain on target, command-position, command-speed, or package-action topics before
starting another Unity run. In a container, stopping its wrapper may not stop ROS;
verify the actual launch/node processes and endpoint port after scoped cleanup.

### Connection diagnosis during the interrupted run

The trained detector logged `PACKAGE_DETECTOR_SERVICE_READY 10001`. The recorder
captured 98 live `DETECTIONS` messages, and `rosnode info /unity_endpoint` showed
active TCPROS connections for RGB, depth, calibration, and camera pose to
`/package_perception`. No connection failure was reproduced during those live
observations. The batch Unity evaluator then disabled its camera publisher in its
final missing-camera case and exited. Subsequent repeated
`UNAVAILABLE: missing_synchronized_camera_data` heartbeats were expected after
Unity stopped; they do not indicate the earlier detector/ROS connection failed.
The trial worker, recorder, and ROS launch were stopped after diagnosis.

For the next chat: inspect current status first. Delivery trial processes were
stopped; identify ownership before stopping newly running processes. Rerun affected
checks after relevant code or dependency changes. Diagnose connection failures while
Unity Play Mode is active by checking `/drone/package_candidates` rate and
`rosnode info /unity_endpoint`. Baseline and changed-placement deliveries passed
with the padded detector view. Later full-workflow and injected-failure results
are recorded below. Avoidance remains disabled in the camera launches.
Never edit `AGENTS.md`.

### Full workflow from DroneStart

On 12 October 2026, camera delivery was extended with vertical ascent from
`DroneStart`, travel to the observation point, and a one-second continuous hover
before enabling package selection. Candidates acquired before observation entry
are ignored. `camera_delivery.launch` enables access by default; pickup-only
remains observation-only. The public mission and acknowledgement sequence is unchanged.

Unity compiled the updated evaluator. Its conservative route envelope found no
active collider intersections on seven segments covering access, approach,
descent, ascent, carry, and lowering. All 62 ROS Noetic unit tests passed with no
skips; all 24 synthetic physical-evaluator tests passed. A disposable catkin
workspace built the package and endpoint successfully.

The baseline ROS/Unity batch trial started at Unity `(34, 2, 10)`, flew to
`(34, 6, 25)`, hovered, and completed camera-guided pickup and delivery. The
independent evaluator passed the access ordering/height/hover checks as well as
physical attachment, lift, carry, release, and settling checks. Final package
center was ROS `(28, 55, 0.300000012)` (Unity `(28, 0.300000012, 55)`), package rise
was 0.989996 m, maximum carry-offset drift was below 0.000001 m, and hold drift
was zero. No sampled collider penetration was recorded. Evidence is under
`ROS1/build/full-workflow/baseline/`.

The first changed-placement attempt was rejected because a prior container ROS
publisher survived host-side process cleanup. Its failure evidence is retained
under `ROS1/build/full-workflow/changed/`. The trial driver was corrected to signal
the container process explicitly before cleaning up the host wrapper; new trials
use fresh output directories and control graphs. The fresh changed-placement and
rotation retry passed all physical checks under `ROS1/build/full-workflow/changed-02/`.
Package rise was 0.989999 m, carry-offset drift was below 0.000001 m, final center
was again ROS `(28, 55, 0.300000012)`, and hold drift was zero. Neither successful
run recorded collider penetration. Both Unity runs compiled and exited normally,
restoring scene setup. The baseline driver initially invoked the evaluator with
an unsupported `--output` flag; evaluating its completed evidence with the documented
positional argument passed. This invocation was corrected for subsequent trials.

Five separate live fault trials also passed. The detector trial terminated the
actual worker during approach; the camera trial disabled Unity's actual
`DroneCameraPublisher`. Position and acknowledgement faults used an evaluation-only
subclass that stopped forwarding position callbacks during access or suppressed
`PICKUP_DONE`/`DROP_DONE` at the mission subscriber. The motion controller, sensing,
package actions, and read-only recorder otherwise continued normally. Unity
acknowledgements were recorded independently even when suppressed at guidance.
No synthetic acknowledgement or package ground truth was fed to guidance.

| Injected failure | Expected terminal reason | Physical result |
| --- | --- | --- |
| Stale mission position feedback during access | `position_feedback_unavailable` | Held during access; no pickup |
| Detector worker loss during approach | `lost_locked_package` | Held before pickup |
| Unity camera publisher disabled during approach | `lost_locked_package` | Held before pickup |
| Pickup acknowledgement suppressed | `phase_timeout` | Package attached; no lift |
| Drop acknowledgement suppressed | `drop_timeout` | Package released; no `COMPLETE` |

Each fault check required the expected reason, no forbidden stage advancement,
expected final attachment state, hold drift within 0.05 m after a 0.5 s grace
interval, no sampled collider penetration, and completed scene restoration. All
five trials measured zero hold drift.
Evidence and `fault-evaluation-report.json` files are under
`ROS1/build/full-workflow/fault-position/`, `fault-camera/` (detector loss),
`fault-sensor/` (camera stream loss), `fault-pickup-ack/`, and `fault-drop-ack/`.
Trial-driver and fault-injection source snapshots are retained alongside the local
evidence. These trials validate those injection timings; they do not establish
recovery behavior or every combination of faults.

For a repeatable camera-stream interruption, set
`DRONE_PICKUP_TRIAL_DISABLE_CAMERA=true` on the batch Unity trial command.
The editor evaluator disables publishing at `APPROACH`, records the injection
stamp in `camera-disabled.txt`, and restores scene state on exit. Operational
launches do not expose or perform this injection.

To repeat, follow the existing physical-trial procedure with
`DRONE_PICKUP_TRIAL_MISSION=delivery` and `camera_delivery.launch`. Delivery now
starts at `DroneStart` and has a 330 s capture limit. For historical observation-only
runs, use `DRONE_PICKUP_TRIAL_START=observation` and launch argument
`start_from_drone_start:=false` together. Keep the detector's verified
`--context-scale 1.5` setting. These checks establish the configured simulation
route, not general obstacle avoidance or hardware flight safety.

## Launcher and spectator verification

On 13 October 2026, the host Linux launcher was exercised with native Unity,
host YOLO and ROS Noetic in Distrobox. The supported command and restart contract
are maintained in the [README](../README.md#camera-guided-single-package-delivery).

| Check | Completed result | Boundary of the evidence |
| --- | --- | --- |
| Shell syntax | All three camera launcher/helper scripts passed `bash -n`. | Syntax only. |
| ROS regression suite | 69 Noetic unit tests passed without skips; the dedicated workspace built successfully. | Unit/build checks do not prove physical placement. |
| Launcher/lifecycle checks | Seven launcher checks and five process-lifecycle checks passed across the host and Noetic environments. They cover prerequisites, unknown options, unrelated port owners, workspace preservation, monitor termination/cleanup, backup preservation, exact project/run ownership and PID reuse. | Noetic helper tests require the container; host isolation/port checks require the host systemd user manager. |
| Fresh restart | Two consecutive launches reached READY. The second stopped the prior ROS/YOLO run and editor, then opened a replacement editor with Play stopped. | Tested on the current Linux/Distrobox machine. |
| Unity recovery backups | A recovery dialog was reproduced after process shutdown. Startup now moves recovery scenes into the new run's `unity-scene-backups/` directory; the preservation test and subsequent startup passed. | Save intentional scene edits before restarting; startup opens the saved scene. |
| Spectator view | C# compiled; 105 sightline/overlap checks passed. Six rendered previews were visually reviewed and repeated without stale drone transforms. The user confirmed the moving view looked good. | Checks cover the configured route and layout, not every camera path or obstruction. |
| GUI physical delivery | An isolated GUI run reached completion; the user confirmed the package was detached and resting inside `SortingZoneA`. Recorded perception observations were below 0.35 s old. | Manual placement confirmation, separate from batch physical evaluation. |
| Later GUI release | The run with the accepted spectator view recorded `DROP_DONE → SETTLE → COMPLETE`. Unity logged successful detachment/release. | The screenshot and acknowledgements alone do not establish settled floor placement in that run. |

The placement-confirmed GUI evidence is under
`ROS1/build/camera-demo-20261013-003646-VtXD62/`, including `monitor.log` and
`perception/observations.jsonl`. The later release-confirmed run is
`ROS1/build/camera-demo-20261013-012011-K0Wb28/`. Fresh-start replacement evidence
is under `ROS1/build/camera-demo-20261013-014348-EPg0q7/` and
`ROS1/build/camera-demo-20261013-014516-scWvB9/`. These are local ignored artifacts,
not files available in a fresh clone.

An earlier GUI attempt exhausted memory/swap, and `systemd-oomd` stopped the
shared application group. Host demo processes now use their own scope capped
at 8 GiB RAM and 2 GiB swap. Distrobox ROS remains in its container resource
group; the host limit does not cover it. A separate observation timeout lacked
recorded perception data, so its exact cause remains unknown. Monitor failure
now preserves ROS connections for inspection rather than introducing secondary
connection-refused errors. One detector import also hit a PyTorch clock assertion;
retrying worked, but the assertion's root cause was not established.

To repeat focused checks without restarting the real project, use the test files
below. Launcher tests create temporary projects. Run these commands on the host;
the two Noetic helper tests skip there when ROS is only installed in Distrobox.

```bash
python3 -m unittest discover -s ROS1/tools -p 'test_camera_demo_processes.py' -v
python3 -m unittest discover -s ROS1/tools -p 'test_camera_demo_launcher.py' -v
```

In a sourced Noetic environment, run the two ROS helper checks separately. The
launcher tests use fake editors and disable systemd isolation; they do not start
the real Unity Editor. Host runs skip the two ROS helper checks.

```bash
python3 -m unittest discover -s ROS1/tools -p 'test_camera_demo_launcher.py' -k existing_workspace_package -v
python3 -m unittest discover -s ROS1/tools -p 'test_camera_demo_launcher.py' -k finished_monitor -v
```

For spectator checks, keep `MainWarehouse` open with Play stopped and choose
**Drone Delivery → Verify Warehouse Spectator Clearance**. The six PNGs in
`ROS1/build/spectator-views/` show start, elevated start, pickup hover, mid-carry,
destination hover and release. The verifier waits for renderer transform updates
between poses and restores the original drone pose without saving. It also runs
after script compilation. The previews do not include a physically carried package.

To compare another editor-only angle without recompiling, write a request in that
output directory, for example:

```json
{"cameraX":34,"cameraHeight":6.5,"forwardOffset":8,"maximumZ":50}
```

Save it as `request.json`; the editor consumes it and replaces the previews.
This does not alter the runtime camera configuration. Inspect images even when
the collider check passes: an earlier office-roof obstruction was visible in
rendered previews but absent from the collider results.

## Linux portability verification

The `feat/linux-portable-setup` changes package the existing checkpoint and pin
host Python dependencies, ROS-TCP-Endpoint, and the Unity ROS connector. Checks on
the x86-64 CachyOS host with the existing Ubuntu 20.04 Noetic Distrobox found:

- A disposable fresh checkout downloaded all nine Git LFS pointer assets from
  GitHub. Git LFS also warned about 150 existing assets tracked as ordinary Git
  binaries despite matching the LFS attributes; these files were present.
- CPU setup installed a new Python 3.11 environment, passed `pip check`, loaded
  the included single-class model and performed inference, and built a new
  dedicated Catkin workspace. Repeating setup succeeded. No existing detector
  environment or Catkin workspace was reused; the host Python 3.11 interpreter
  and existing ROS container were prerequisites.
- Setup refused to replace the existing camera workspace's Endpoint link.
- Fresh launch reached CPU detector readiness and started ROS before opening
  Unity. **Unity startup failed:** at 02:17 on 2026-10-15 the system memory-pressure
  service killed Chromium's process group; at 02:18 the kernel killed Unity after
  its demo scope reached 8 GiB RAM and 2 GiB swap. The pre-launch host had only
  about 3.8 GiB available RAM. Scene readiness, compilation completion and physical
  delivery were not verified for this checkout.
- After that failure, the launcher gained a 10 GiB available-RAM preflight and
  sequential asset imports with two job workers and one GC helper. Unit tests
  exercise memory boundaries using files, not memory allocation, and check the
  editor arguments using a fake editor. The real Unity import was **not rerun**;
  these precautions have not been validated under a full import workload. The
  blanket 10 GiB minimum was subsequently removed as described below.
- The host Python tool suite ran 66 tests successfully, with two ROS-only checks
  skipped. Inside Noetic, all 69 existing ROS tests and all 26 camera launcher,
  Linux configuration, resource, and process tests passed without skips. Shell
  syntax, affected Python parsing, Unity package JSON, the model checksum, and
  diff whitespace checks passed.

For lightweight regression checks (no real Unity launch):

```bash
bash -n ROS1/scripts/camera_demo_linux.sh ROS1/scripts/setup_camera_demo_linux.sh \
  ROS1/scripts/run_camera_delivery_demo.sh
python3 -m unittest discover -s ROS1/tools -p 'test_*.py'
```

### Separate import and cached startup

Inspection of the original checkout found both asset databases and
`Library/ScriptAssemblies/Assembly-CSharp.dll`. The user's fresh checkout had none
of these files. This establishes different cache states, but does not measure
the RAM required for either workload. No cache was deleted or copied between
checkouts.

Normal startup now reports memory, warns below a provisional 4 GiB, and enforces
a hard minimum only when `CAMERA_DEMO_MIN_AVAILABLE_GIB` is explicitly set. A
missing/incomplete cache directs automatic startup to `prepare_camera_demo_unity.sh`
before project processes are stopped. Manual backend-only startup still works
without a cache. Cache files are indicators, not proof that no reimport is needed.

The new import helper runs Unity alone with sequential imports, two job workers,
one GC helper, and a ten-minute maximum. Its provisional experiment budget is
4 GiB memory high, 5 GiB memory max, 512 MiB swap max, and 7 GiB initially available
host RAM. Five consecutive checks with less than 2 GiB available RAM or at least
10% full memory pressure (`avg10`) stop its scope. It records host memory/pressure
and scope memory samples, and cleans up only its own scope on failure or timeout.
These are experiment limits, not measured minimum hardware requirements. The
latest host tool suite ran 76 tests with two ROS-only skips. Shell syntax and
diff whitespace checks passed. The proposed systemd memory and runtime limit
properties were accepted in a `/bin/true` probe; this did not start Unity. The
helper refuses to run alongside an editor for the same checkout.

Tests exercise cached startup at 8.7 GiB, fresh/partial caches, advisory versus
explicit minimums, pressure boundaries, bounded command arguments, import failure,
timeout, and scope cleanup. Editors and systemd are faked for import tests; they
allocate no test memory. The first real import attempt is recorded below;
subsequent delivery has not run with this policy.

### Controlled import result

On 2026-10-16, the approved Unity-only preparation command ran against
`~/Documents/code/test/DroneSortingProject-fresh` with the limits above. It began
with about 10.6 GiB available host RAM and no full-memory-stall pressure. The
monitor stopped it after five consecutive pressure checks exceeded 10% full
`avg10`; systemd recorded scope shutdown after about 93 seconds. No limits were
increased and no second import was attempted.

- Sampled peak scope memory: 4.02 GiB. The journal recorded a 512 MiB swap peak.
- Lowest sampled host available RAM: 6.63 GiB.
- Highest recorded full memory pressure `avg10`: 17.89%.
- The project assembly was emitted; no `error CS` lines appeared in the captured
  log. Scene readiness and complete import were not confirmed.
- Scope shutdown was explicitly requested by the monitor. No project editor
  processes remained afterward; no kernel OOM events appeared in the test window.
  These observations do not guarantee that other applications felt no slowdown.

Logs remain in the fresh checkout under
`ROS1/build/unity-import-AsDIuR/` (`unity.log`, `scope.log`, `memory.log`, and the
recorded scope name). The tested limits did not complete this first import, so
normal demo startup and physical delivery were not attempted. This run measures
a resource-constrained import; it does not establish the machine's minimum RAM
requirement or prove that the 4 GiB soft limit is appropriate.

The stopped import created all three cache files despite never confirming scene
readiness. The cache guard therefore now requires a `COMPLETE` marker in the
latest recorded import run, in addition to the cache files. The preparation
helper writes it only after successful exit and scene confirmation. Previously
working checkouts that predate this helper still use the file indicators. A
regression test covers interrupted imports with apparently complete cache files.
The final lightweight tool suite ran 77 tests, with two ROS-only skips.

Native Noetic on another Linux distribution, a newly created ROS container,
optional CUDA installation, and physical delivery using the new CPU default
remain unverified. A future live check should be scheduled only with sufficient
memory headroom; the scope limit alone did not protect other applications.

## What remains unverified

- Detector/localization performance outside the tested synthetic warehouse cases,
  real package observations, and reliable search-area emptiness confirmation.
- Camera-guided access and delivery in broader layouts and under combinations
  of faults; recovery after failure and real flight dynamics.
- Return-and-repeat delivery, separate release positions, and automatic mission
  recovery are not implemented. Warehouse avoidance still needs validation.
- Physical RGB-depth camera field of view/range, noise, mounting/payload safety,
  real drone localization/control, and real-warehouse performance.

Keep these separate from the completed sensor-communication evidence. Do not
claim autonomous package search or real deployment readiness from these tests.
