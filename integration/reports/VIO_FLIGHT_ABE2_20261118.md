# VIO watchdog failure, run 20261118-flightabe2d67e5306

Evidence: `integration/phase1_runs/20261118-flightabe2d67e5306/evidence/`.
Tests: `tests/test_vio_flight_abe2.py`. Previous run: `VIO_FLIGHT_FBF7_20261118.md`.

## Code that ran

`flight-source/configuration.json`: `run_vio_stream.py` 9e9e1e11 and `vio_quality_gate.py`
cc6c6bf2 are the `53eeddf` versions. `phase1-integration-merge` had `vio_flow_consistency.py`
but not `9f3226a`, which wires it in, so the flow check was not active and the per-frame
diagnostics (gyro, prior, flow_consistency) were not recorded. The noise ground was active
(`--size-m 16 --tile-size-m .02 --pattern noise`).

## Timeline

Host-to-frame mapping by the source's own wall clock (`stream.jsonl` `wall`).

| frame t | host s | event |
|---|---|---|
| 38.5 | 6479.86 | ARM |
| 39.3 | 6481.18 | CLIMB (takeoff command) |
| 40.9-41.7 | | still on the ground (h 0.19 m, GT velocity 0); inliers fall 200 -> 61-145 |
| 41.7 | | **last sample delivered** (published, inliers 70) |
| 41.8 | | withheld: distributed flow disagreement (35 inliers), h 0.20 |
| 41.9 | | withheld: frontend unavailable (inliers), h 0.21 |
| 42.0 | | withheld: distributed flow disagreement (15 inliers), h 0.23 |
| 42.1, 42.2 | | withheld: frontend unavailable, h 0.27, 0.31 |
| 42.3-42.5 | | readiness hysteresis 1/3, 2/3, 3/3 (inliers 15, 43, 92) |
| (42.289 sim) | 6485.61 | **transport watchdog** (odom age 500.6 ms), latched closed, EMERGENCY_LAND |
| 42.6-47.3 | | source publishes again (inliers 111-187); transport is latched |
| 47.3 | 6492.02 | last publication |
| | 6492.03 | transport DONE, process exits |
| 47.4+ | | readiness 1/3 -> 3/3 -> **waiting for downstream DDS discovery**, repeating |

## Answers

1. **Why publication stopped:** it stopped at 41.7, not 47.3. At liftoff (0.19-0.31 m) the
   frontend lost its features; 5 consecutive frames were withheld by the inlier minimum or the
   distributed audit, then readiness needed 3 more frames. 0.9 s without output; watchdog at 500 ms.
2. **gate_ready=false at 56-58 s with ~180 inliers:** the transport had exited at 47.3. Each time
   readiness completed, the subscriber check found no subscriber and reset readiness. After
   startup (t=24.0), every DDS-discovery skip is after the transport reached DONE.
3. **Subscribers:** they disappeared because the transport process exited, not through a discovery
   or accounting fault. No DDS skip occurred while the transport ran.
4. **Resets, backpressure, freshness:** epoch stayed 0 (no estimator reset). One admission skip and
   one stale-at-publication withhold, both after the watchdog. Processing 61-84 ms in the window.
5. **Noise texture in real flight:** airborne (h > 0.3 m) flow vs Gazebo GT has a max error of
   0.37 cm per frame (median 0.08 cm). Run fbf7, with the 2 cm checker, had a max of 12.6 cm and
   4 wrong published values. Inliers at 0.6-0.8 m: 157-173. The aliasing is resolved.
6. **Before the watchdog / transport delivery:** the source stopped at 41.7 before the watchdog;
   the transport received all 205 publications (identical stamps, strictly increasing). The only
   unmapped ones before 41.7 are those before the clock reference (27.2 s).
7. **Hysteresis vs DDS:** no interaction during flight. After transport exit the cycle is expected
   fail-closed behaviour (it never publishes without a subscriber), but it makes logs look like a
   readiness fault.

## Root cause: the drone's own shadow at liftoff (scene)

The sun casts hard shadows. At 0.19 m the drone's shadow covers ~53% of the image (fbf7 probe
frame: 148 of the 200 strongest corners lie on the shadow outline). The pinned frontend keeps
200 corners, so the shadow uses up the budget. The shadow moves with the drone, not the ground,
and during liftoff its image shrinks and shifts, so ground matches are outvoted.

Bench (`SelfShadowBench`): the recorded shadow over the noise ground, recorded liftoff heights,
pinned frontend with the source's scale compensation:

| step | no shadow | recorded shadow |
|---|---|---|
| 0.19 -> 0.20 m | 166 inliers, audit ok | 33 inliers, audit: ambiguous spatial support |
| 0.41 -> 0.48 m | 193, ok | 152, ok |

Matches the flight: 41.8 had 35 inliers with the same audit reason, and recovery with height.
The flight was worse (3 frames with no frontend result). Inliers already fell on the ground once
the takeoff command was given (40.9-41.7, drone stationary), consistent with spinning propeller
shadows. Motor outputs are not in the evidence, so that part is an inference.

Not fixed in VIO: the frontend is pinned, and local illumination normalisation recovered only
40-60 inliers (bench), too little to justify changing the measurement pipeline.

## Required changes

**Person 2 (Gazebo scene).** Stop the drone casting shadows in the flight world, either:

* `iris_runway.sdf`, light `sun`: `<cast_shadows>false</cast_shadows>` (one line), or
* `<cast_shadows>false</cast_shadows>` on every visual of `iris_with_gimbal` /
  `iris_with_standoffs`, including the propellers.

This is a simulation scene choice. A real drone's spinning propellers make a faint blurred disc,
not hard blades, but hard-sunlight takeoff remains unvalidated for the real system.

**Person 4 (integration).**

* Integrate this branch (`9f3226a` wiring). Without it a wrong flow can be published again.
* Save camera frames after the fixture is created. The current probe frames predate it, so
  nobody has seen what the camera renders in flight.
* Transport, after the latch: repeated `bounded odometry buffer overflow` events (6491.7-6492.0)
  while it no longer drains odometry. Harmless after the latch, but noisy evidence.

## Still open

* One withheld frame still costs at least 0.5 s (4-frame readiness), so liftoff must be clean.
* Not flown with the shadow change.
