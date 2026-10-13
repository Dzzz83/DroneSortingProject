# Current plan: full camera-guided single-package workflow

## Status

**Completed:** one-package camera-guided delivery in batch Unity, from
`DroneStart` through ascent, travel, observation hover, pickup, lift, carry,
release, and settling
inside `SortingZoneA`. Both baseline and changed-placement/rotation trials passed
independent physical evaluation. Each used one pickup and one release request,
acknowledged by Unity. Five live fault trials verified holding without stage
advancement under camera-stream loss, detector loss, stale mission position
feedback, and suppressed pickup/drop acknowledgements.

**Completed operational setup:** the Linux one-command launcher performs a fresh
restart of the previous demo's ROS, YOLO and this project's Unity editor, opens
`MainWarehouse`, and leaves Play manual. The wide spectator view passed preview
and manual Play checks. Results and reproduction are in
[launcher verification](verification.md#launcher-and-spectator-verification).

Implementation, launch choices, and settings are in the
[README](../README.md#camera-guided-single-package-delivery). Build/test results,
physical measurements, and the invalid trial retained for diagnosis are in
[full-workflow evidence](verification.md#full-workflow-from-dronestart).

## Goal

Deliver one package from `DroneStart` via observation hover using RGB-depth
observations for pickup. Success requires `PICKUP_DONE` before lift, `DROP_DONE`
before completion, and independent verification that the same package settles
inside the destination zone. This milestone is complete and ends after one delivery.

Pickup coordinates come from observations; destination coordinates and travel
heights are configured. Ground truth is confined to evaluation. Return-and-repeat
is not part of the completed demo. Access is verified for the configured route;
broader warehouse navigation remains unverified.

## Remaining work

1. Decide recovery behavior after failure, including a package attached without
   a received pickup acknowledgement or released without a received drop acknowledgement.
   Current guidance holds and requires a fresh mission; the launcher can restart
   the demo, but automatic mission recovery is not implemented.
2. Extend validation to other fault timing, combinations of failures, and broader
   layouts before adding return-and-repeat delivery.
3. Implement repeated search and delivery with reliable empty-area confirmation
   and separate release positions inside the destination zone.

## Preserve these boundaries

- Operational detection must not read package transforms or known package count.
- Require `PICKUP_DONE` before lift and `DROP_DONE` before delivery completion.
- `DROP_DONE` proves release, not successful placement; evaluate physical settling.
- Keep pickup-only and fixed-coordinate delivery launches available.
- Avoidance is disabled in the camera launches. Use the configured, checked layout.
- Sampled collider checks and kinematic simulation do not establish hardware safety.
- Start a fresh mission and output directory for every trial; verify process cleanup
  and an empty control graph before the next Unity run.

## Later milestones

Extend the verified single-delivery workflow with recovery and return-and-repeat
search. Validate warehouse avoidance and real sensor/flight constraints separately.
