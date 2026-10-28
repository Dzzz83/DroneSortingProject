# Navigation-only speed and smoothness integration

**Base:** `feat/depth-avoidance`, `a1b3ea2474ffb1be2fcb1cf01a5ec0b0514c94c8`.
**Scope:** Drone from Warehouse A to B without pickup/YOLO/dropoff.

## User workflow

1. From the repository root: `bash ROS1/scripts/run_navigation.sh`.
2. Open `TwoWarehouseDelivery` in Unity and press Play.
3. Reset using **Reset physical trial**. Stop ROS using Ctrl+C.

## Transport and safety contract

The sole ROS motion controller publishes either old absolute `geometry_msgs/Point`
commands for legacy modes, or ordered `std_msgs/String` approvals on
`/drone/approved_trajectory` for the navigation demo. These do not run together.

JSON payload: `reset_sequence` (Unity physical trial ID), `sequence` (monotonically
increasing within trial), `stop` (boolean). MOVE additionally carries `restart`,
`issued_ms`, `expires_ms` (wall-clock Unix milliseconds), `speed` (m/s), and
`start`/`end` (ROS x, y, z points). Each MOVE covers exactly one certified
planner/follower segment. Explicit STOP invalidates queued movement and forces
ROS to re-anchor a subsequent restart at Unity's actual applied pose. Unity
publishes `/drone/trajectory_status` (`std_msgs/String`) at a nominal 20 Hz:
`reset_sequence`, last processed `sequence`, `stopped`, `stamp_ms`, and actual
`position` in ROS axes. ROS waits for the matching STOP acknowledgement before
anchoring a restart. Initial startup uses the same handshake. Local rejection or
expiry reports `stopped` and triggers a new STOP handshake; delayed ordinary
position feedback cannot supply a restart anchor. ROS retries a pending STOP
every 0.25 s with its original reset identity and sequence until a fresh matching
acknowledgement arrives. Unity ignores duplicate packets but continues reporting
the acknowledged state; delayed retries cannot cancel a newer movement chain.

Unity verifies the epoch, order, expiry, finite coordinates, segment continuity,
maximum 0.255 m queued arc (0.25 m plus float tolerance), and collision latch. It consumes only sequential
segments at actual frame/physics rate, with a physical box sweep before each
move. Old or malformed messages cannot authorize a shortcut. Plan/observation
freshness remains 0.35 / 0.5 s; authorization TTL is capped at 0.2 s.
ROS admits a segment only when its remaining approval time covers the estimated
outstanding travel, the new segment, and 50 ms of local transport/frame delay.
Otherwise it waits without advancing sequence or command position. Previously
queued segments keep their original deadlines. This avoids sending approvals
that expire before Unity can apply them; Unity still independently enforces expiry.
The ROS lead bound remains 0.25 m; the envelope is unchanged. ROS STOP or a
local expiry clears buffered segments, requiring a new explicit restart.

**Clock prerequisite:** ROS and Unity run on synchronized system clocks.
Cross-machine clock skew can reject otherwise valid approvals (fail closed).

The navigation coordinator also publishes latched `false` on
`/drone/camera/enabled` (`std_msgs/Bool`) to pause the unused package RGB/depth
camera. This eliminates its GPU capture/readback cost. Other workflows retain
capture enabled by default; start a fresh Play session when switching workflows.
The component remains subscribed and accepts `true` to resume capture.

The forward navigation camera now uses physics-only depth (sparse every 8th
pixel; NaN unknown, positive-infinity measured no-hit), preserving proxy
obstacles and package occlusion. No RGB is fabricated. Setting
`physicsDepthOnly=false` on `ForwardObstacleCamera` restores the full RGB/HDRP
stream for other camera consumers.

Navigation requests approved segments at 50 Hz (legacy controllers default to
20 Hz). This uses shorter segments at the same requested 2 m/s and preserves the
same distance and freshness limits. Reaching the command frontier's local goal
lets already-approved pending motion finish; it does not cancel that motion.
Invalid or stale approvals still stop immediately.

The pose guard measures distance to the certified polyline, rather than distance
back to the capture's starting point. Normal progress along a fresh route no
longer causes a false stop after 0.35 m; the 0.35 m pose guard, command lead,
polyline restriction and freshness checks remain active.

The saved two-warehouse scene and publisher default request one actual Unity
position update every 0.05 s, after movement in LateUpdate. Position and trajectory
status timers retain fractional frame time without sending catch-up duplicates. Frame drops and ROS TCP latency may lower the
achieved rate; messages are never artificially backfilled.

The spectator view uses frame-rate-independent positional/rotational damping;
key-triggered view switches and manual input remain immediate.

## Verification checklist

Run focused test: `PYTHONPATH=ROS1/package/python python3 -m unittest discover -s ROS1/tests -p 'test_approved_trajectory.py'`.
Then run all Python regressions with the same command minus `-p`.
In Unity, verify the scene compiles, physical ray sensing observes collision
proxies, sensor calibration/stamps remain aligned, and reset/contact/expiry
cancel pending motion. Perform **one** 2 m/s full-course run, if runtime is
available, collecting arrival time/error, contact reports, actual feedback
rate, hold reasons, frame times, and sensing/planning latency.

**Baseline:** earlier human-reported runs: 152.1 s and 151.4 s at requested
2 m/s with no reported contacts. These are not results from this bundle.

## Review results (2026-10-27)

- 220 ROS/Python tests passed in the Noetic container.
- Unity compiled and printed `COURSE_SPECTATOR_CHECKS_PASS`,
  `OBSTACLE_DEPTH_CHECKS_PASS`, and `NAVIGATION_SMOOTHNESS_CHECKS_PASS`.
  Checks include corner following, expiry stop, stale restart rejection, wrong
  reset identity, sequence gaps, bounded queues, physical reset cancellation,
  depth-only message stamps, and manual/damped spectator controls.
- A full live A-to-B run reached `(34,119.5,3)` with zero final reported position
  error and no course-reported contacts. The coordinator reported **184.0 s**;
  first-motion-to-arrival measurement was **182.4 s**. This was slower than the
  earlier roughly 152 s baseline. Actual feedback averaged **16.8 Hz**. Observed
  frame time was 58.7 ms median / 66.2 ms p95; startup frames were included.
- That run exposed near-expiry packet churn: sampled rejected approvals had only
  2–62 ms left, while transport/execution took 25–66 ms. The final scheduling fix
  waits for enough approval time without extending any safety deadline.
- After that fix, a **10 m live confirmation** to `(34,60,3)` arrived with zero
  final error, no reported contacts, and **zero Unity-local rejection/expiry stops**.
  Coordinator elapsed time was **19.3 s**, including initial mapping and sustained
  arrival; first-motion-to-arrival was **17.0 s**. Feedback averaged **16.5 Hz**.
  ROS planning/arrival holds remained active.

At this stage the scheduling-only fix had not been retimed over the full course.
The later follow-up run below establishes faster arrival after additional fixes;
continuous 2 m/s motion remains unverified. Temporary measurement probes were
removed; the supported command is unchanged.

**Limits:** This changes simulated Unity execution, not real drone motor
control. On transport loss/gaps, it deliberately stops rather than guessing a
route. A stale restart is rejected; ROS sends STOP, waits for its matching stationary
acknowledgement, then obtains a fresh approved segment. Unity/ROS profiling is
needed to establish measured improvements.

## Follow-up diagnosis (2026-10-27)

The user's live run confirmed slow mapping: one sampled update spent 121.5 ms
integrating, 57.3 ms building/counting evidence, and 26.7 ms planning. Actual
position feedback in that interval was 13.9 Hz. That update rate consumed much
of the approval window.

The optimized moving-window pruning removes per-voxel generator/function calls;
batch-clipped free rays no longer repeat bounds checks. Evidence counts use a
single counting pass. In a matched four-capture workload with roughly 78,000
stored cells, median integration/evidence/counting time dropped from **148.4 ms
to 83.8 ms**. Both implementations produced identical stored acquisition times,
occupied/free cells, moving bounds and planning evidence. These are CPU-workload
measurements, not a full-demo speed measurement.

All 224 regression tests passed after stopping the previous ROS run. Checks cover
forward progress on a fresh route, off-route pose rejection, expiry, moving-window
boundaries, queued arrival completion and 50 Hz step sizing. Unity compilation was
confirmed by the user.

The follow-up full live A-to-B run completed in **112.5 s** (110.3 s from first
motion), versus 184.0 s in the prior review run: **38.9% less elapsed time**.
Final ROS position was exactly `(34,119.5,3)` with no course-reported contacts.
This was also 25.7–26.0% less time than the earlier 151.4/152.1 s runs, although
routes and runtime load can vary between trials.

The run was faster but was not continuously smooth: unchanged pose reports
accounted for approximately **33.6%** of the first-motion interval. This estimate
uses ROS receipt times, not a Unity frame profiler. Actual position feedback
averaged **10.0 Hz**. There were 146 ROS stop packets and 22 unique stopped
acknowledgements associated with movement packets; these counts do not attribute
every stop to one cause. Sampled controller logs were dominated by waiting for
sufficient remaining approval time, with additional plan/observation expiry holds.
Remaining safety waits and uneven feedback still prevent sustained 2 m/s motion.

The actual traced path was approximately 86.5 m. Median depth integration time
was 103.6 ms (p95 184.3 ms). Planning/snapshot aggregate medians from this monitor
included pre-Play waiting, so they are not used as active-flight performance
measurements. This single run confirms faster arrival and contact-free reported
travel for this layout, not fully smooth rendering or all future runs.

## Recording workflow

Use `bash ROS1/scripts/run_recording_demo.sh` for the same course and ROS launch
outside the Editor. The first run builds a Linux player; `--rebuild` refreshes it.
Rendering is capped at 30 FPS in a 1280×720 window and player background execution
is enabled. This avoids the Editor's documented focus throttling when recorder
controls are activated; it does not move Unity API calls onto unsafe background
threads or relax sensing deadlines. See README for build and run instructions.
### Recording investigation result: not fully fixed

The inspected `~/.local/bin/screen-record` shortcut selects an area with `slurp`
and runs `wf-recorder` without a frame-rate cap or explicit encoder. Its default
encoder was observed using libx264 with 11 CPU worker threads. That personal script
was not modified. NVIDIA H.264 encoding was verified separately and used for the
optimized comparison at 30 FPS.

The standalone build succeeded and its depth, trajectory, spectator-control and
camera-local render-override checks passed. Native Wayland startup failed with
`Surface already has a fifo`; the launcher now uses SDL's X11 backend through
XWayland. Keeping Editor and player open together exhausted GPU memory and caused
fallback-buffer warnings; the tests below ran after closing the Editor and
restarting the player.

Short live trials, same 10 m destination `(34,60,3)`:

| Configuration | Arrival time | Contacts |
| --- | ---: | --- |
| Initial standalone, recording off | 17.8 s | None reported |
| Initial standalone, default wf-recorder | 46.6 s | None reported |
| Initial standalone, NVIDIA H.264 / 30 FPS | 39.0 s | None reported |
| Repeated recording-off control | 25.1 s | None reported |
| Final spectator profile, recording off | 17.9 s | None reported |
| Final spectator profile, NVIDIA H.264 / 30 FPS recording | 32.3 s | None reported |

The final spectator profile disables SSAO, SSGI and volumetric fog only on the
spectator camera, and uses FXAA. Depth sensing, collision geometry and all ROS
safety limits are unchanged. Both final trials reached the exact destination;
position feedback averaged about 20 Hz in each. Unchanged pose intervals accounted
for about 50.7% off versus 67.8% on (ROS-receipt-time estimates).

**Verdict:** background focus handling and the standalone startup path were
addressed, but recording-induced navigation slowdown is **not fixed**. The final
recorded trial was approximately 80% slower than the unrecorded control. Runtime
load varied between resets, so these small trials do not prove a precise causal
contribution for each change. No full-course recorded run was attempted after the
short acceptance comparison failed. Work stopped at the user's request to finish
the current comparison and report the result. Test recordings and probes were
removed, and the test recorder, player and ROS processes were stopped.

## Review fixes verification (2026-10-28)

The ROS Noetic suite passed **237 tests**, including regressions for brief swept
envelope overlaps, stop retry/recovery, blocked heuristic bypasses and missing
ROS NumPy. Sweeps were also checked against an independent segment/box
intersection calculation on 250 seeded segments. Changed Python files parsed,
changed shell scripts passed `bash -n`, and `git diff --check` passed.
Unity 6000.6.0f1 compiled and passed `COURSE_SPECTATOR_CHECKS_PASS`,
`OBSTACLE_DEPTH_CHECKS_PASS` and `NAVIGATION_SMOOTHNESS_CHECKS_PASS`. The executor
checks now include repeated STOP acknowledgements and a delayed duplicate STOP
arriving after a newer movement chain.

A fixed 3D local-goal obstacle fixture with the configured drone envelope took
**143.9 ms median before** and **96.2 ms after** the clearance change, across five
plans per implementation. Both produced READY routes. This checks that the
stronger sweep query is practical on that fixture; it is not a general flight
speed benchmark.

Two real ROS/Unity trials used the existing standalone course player and current
ROS sources, requested 2 m/s, start `(34,50,3)` and destination `(34,119.5,3)`:

| Player mode | Outcome | Contact |
| --- | --- | --- |
| Headless (`-batchmode -nographics`) | FAILED at 64.1 s, ROS `(35.035,109.875,3.125)` | `Bldg_Screw_002`; 9.68 m short of B |
| Rendered, 1280×720 | ARRIVED at 244.9 s; exact destination, zero final error | None reported |

The rendered duration comes from the coordinator's NAVIGATING/ARRIVED log times;
its status `elapsed` continues increasing after arrival. The initial observer's
240 s deadline expired before arrival, so observation resumed on the same flight
to confirm arrival and the course's latched no-contact state. The rendered
trial's sampled planning times had a 44.1 ms median and 257.9 ms maximum.

The headless contact cause was not established. The later rendered success does
not explain it or prove repeatable collision-free travel. Requested 2 m/s was
not sustained, and these runs do not establish a speed improvement. Known
TextMeshPro font-resource exceptions appeared at startup. The headless run also
reported queue drops at startup. Recording was off in both trials; recording
slowdown remains unresolved. Test player, ROS and observer processes were stopped
and disposable probes/logs removed after collecting these results.
