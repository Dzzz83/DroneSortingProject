# Laptop AI avoidance feasibility — 24 October 2026

## Purpose and scope

Check whether the existing laptop can run the warehouse simulation, ROS delivery,
an additional forward RGB-depth camera and lightweight obstacle inference together.
This is a resource and data-pipeline test, not a comparison of avoidance algorithms.
No obstacle planner, second warehouse, truck course or trained obstacle model was
introduced. The current mission and its safety timeouts were preserved.

## Decision

The laptop can support a lightweight AI perception prototype in the tested
warehouse scene using GPU inference. The controlled GPU run met the initial
camera-rate, latency and frame-rate targets and completed the existing delivery.
New hardware is not required to begin development. The larger two-warehouse
scene, trained obstacle model and actual avoidance planner still need validation.

## Machine and software

- Ryzen 5 5600H: six cores, twelve threads; approximately 16 GiB RAM.
- RTX 3060 Laptop GPU: 6,144 MiB VRAM. Host driver access was verified.
- Unity 6000.6.0f1, existing HDRP MainWarehouse scene and ROS Noetic in Distrobox.
- Existing package perception: YOLO11n package checkpoint, 640-pixel inference,
  including its existing second padded inference view.
- Additional perception: pretrained general-purpose YOLO11n, batch one.
- Existing mission access/carry speeds: 1 m/s. No movement parameters were changed.

The normal Python environment has PyTorch 2.5.1+cpu. A separate GPU experiment
uses an isolated environment under ignored build output; the normal environment
is retained.

## Method

A temporary editor probe added a runtime-only forward camera and published
synchronised RGB/depth on diagnostic ROS topics. It never issued movement or
package commands and never saved the scene. The existing mission remained the
only movement authority.

After a 15-second warm-up, each run measured three consecutive 30-second stages:
existing camera only, additional camera without obstacle inference, and additional
camera with inference. Both detector processes were loaded before measurement.
The ROS consumer retained only the latest pending camera pair.

End-to-end latency starts immediately before Unity capture and ends after the ROS
consumer receives the inference result. It includes capture, serialisation,
transport, depth validity counting and inference; it excludes a future planner.
Inference timings include image preparation and model prediction. Frame timings
come from editor callbacks observing successive Unity frame counts, not a Unity
Profiler capture or guaranteed display FPS.

The drone and viewing geometry changed during each mission. These short,
sequential stages are indicative load measurements, not controlled algorithm
benchmarks. Percentiles use the sorted observation at floor(0.95 × (N − 1)).

## Controlled GPU result

Both detectors used PyTorch 2.5.1+cu124 on the RTX 3060. The additional camera was
320 × 240 at a requested 6 Hz, with 320-pixel obstacle inference and two PyTorch
CPU threads. The Game view was explicitly opened, background simulation enabled
at runtime, and ROS consumers started after all package-camera publishers had
registered. These were diagnostic startup arrangements, not production changes.

| Measurement | Observed result |
| --- | --- |
| Received obstacle frames | 5.81 Hz; 175 results during the AI stage |
| Capture-to-result latency | Mean 37 ms; 95th percentile 45 ms; maximum 110 ms |
| Inference time | Mean 13 ms; 95th percentile 23 ms |
| Estimated Unity loop rate during AI stage | 81 FPS; maximum observed frame interval 79 ms |
| Position-feedback arrival gap | 95th percentile 236 ms; maximum 266 ms |
| Peak GPU memory across the run | 3,873 MiB of 6,144 MiB |
| Minimum available system RAM across the run | 3.62 GiB |
| Mission result | Pickup and drop confirmed; COMPLETE |

The final Unity state placed `DeliveryPackage_02` at `(12, 0.3, 55)`, released,
with zero reported speed. Its collider bounds were `(11.7, 0, 54.7)` to
`(12.3, 0.6, 55.3)`: resting on the floor within the destination's horizontal
extent. Other delivery packages remained in the pickup area. This verifies the
recorded delivery outcome, not collision-free travel along the route.

Extra inference was active for a 30-second stage during the mission, then stopped
before the mission finished. This is not a full-mission continuous AI-load test.
There was no autonomous obstacle planner in the loop. Stages also view different
geometry, so the slightly higher loop rate in the AI stage than the camera-only
stage is not evidence that inference improves rendering.

An earlier GPU attempt had good inference latency but ran at approximately ten
editor updates per second and failed with missing synchronized package-camera
data. Its logs contain a camera-info subscriber registration error. It is not a
valid presentation-performance run; the controlled rerun used the startup
arrangement described above. The underlying production startup issue has not
been repaired or fully isolated.

## CPU results

| Configuration | First CPU run | Lower-load CPU run |
| --- | --- | --- |
| Added camera | 640 × 480, requested 5 Hz | 320 × 240, requested 6 Hz |
| Obstacle inference input | 416 pixels | 320 pixels |
| Obstacle PyTorch threads | Default | Two |
| Received rate with inference | 4.61 Hz | 4.43 Hz |
| Mean capture-to-result latency | 116 ms | 154 ms |
| 95th-percentile capture-to-result latency | 290 ms | 705 ms |
| Maximum capture-to-result latency | 563 ms | 968 ms |
| Mean inference time | 57 ms | 90 ms |
| Estimated Unity loop rate with inference | 57 FPS | 28 FPS |
| Mission outcome | Failed: position feedback unavailable | Failed: position feedback unavailable |

The lower-load run recorded a maximum position-feedback arrival gap of 1,055 ms.
The mission requires feedback within 500 ms. Its stop is therefore consistent
with the measured feedback interruption. The specific cause of the stalls is not
isolated: capture, rendering, CPU contention, memory compression and editor work
may contribute. Lower image resolution alone did not establish reliability.

GPU memory peaked at approximately 3.9 GiB in both CPU runs. Available system RAM
fell to 2.7 GiB and 2.4 GiB during their respective full run windows, including
startup. Swap is `/dev/zram0`, compressed RAM, not disk-backed swap. A live sample
showed 13.7 GiB of logical compressed data occupying approximately 4.3 GiB of
physical memory. Large swap counters alone must not be called disk thrashing.

## Recognition and safety limitations

Pretrained outputs included unvalidated labels such as suitcase, train and truck
in warehouse views. No labelled evaluation set was available, so accuracy, recall
and reliable truck/box recognition were not established. Metric-depth messages
arrived with many valid pixels, but this test did not independently calibrate
depth accuracy or prove full obstacle coverage.

The failed CPU runs do not establish hardware insufficiency: background memory
load differed between attempts. The successful GPU run started with almost no
zram usage, unlike the CPU trials. CUDA, startup ordering and host load changed
together, so these results do not isolate the benefit of CUDA alone.

These probes do not validate autonomous AI avoidance, minimum clearance or
collision-free travel. Five repeated collision-free deliveries remain a later
acceptance test after the obstacle model and planner exist.

## Recommended next work

Use the small YOLO11n architecture and GPU inference for the first prototype.
Capture forward-camera views of the actual course and train/evaluate obstacle
recognition on Kaggle, keeping evaluation views separate from training. Depth
must detect occupied space independently of object labels.

Implement the ROS planner next and repeat full-mission continuous-load and
collision/clearance tests in the two-warehouse scene. Test the trained checkpoint
again; successful training does not establish runtime or navigation reliability.
GPU utilization reached 96% in the controlled run, so keep the new environment's
rendering cost modest and close unnecessary applications before comparisons.

## Recovery and retained evidence

The first editor exited with signal 11 after shutdown was requested. The next
startup was blocked by a recovery prompt; that attempt was excluded from timing
results. Recovery scenes were preserved under
`Unity/UserSettings/SceneBackups/avoidance-feasibility-1792776053/`.
The tuned CPU and controlled GPU editors subsequently exited normally. No crash
report was submitted. Temporary editor code and one-time worker scripts were
removed after measurement; saved scene, prefab and mission settings were unchanged.

Raw CSVs, logs, summaries and two camera views are retained locally in ignored
`ROS1/build/avoidance-feasibility/` and
`ROS1/build/avoidance-feasibility-tuned/`. The excluded startup attempt is under
`ROS1/build/avoidance-feasibility-startup-blocked/`. GPU evidence is under
`ROS1/build/avoidance-feasibility-gpu/` and
`ROS1/build/avoidance-feasibility-gpu-visible/`; the latter includes final package
state in `final-state.tsv` and computed statistics in `summary.json`.
The isolated GPU environment is retained at `ROS1/build/avoidance-gpu-venv/`.
It reuses the existing detector environment's non-PyTorch dependencies through
a local Python path file; retain that original environment as well.
