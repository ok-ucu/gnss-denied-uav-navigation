# VIO source backlog during CLIMB — run 20261117-flight4414cb818f2a

Evidence: `integration/debug_evidence/20261117-flight4414cb818f2a/` (committed `858273e`) only.
Reproduce every number: `python3 integration/analyze_vio_latency.py integration/debug_evidence/20261117-flight4414cb818f2a`.

## Failure

The transport latched `permission expired during conversion; never extend it` for acquisition 37.9 s
(`events.jsonl`, stage `transport_pair`, `gate_blocked=false`). It reached the transport 183 ms old
(`odom_arrival_age`), so its budget `200 ms − age − 17 ms mapping uncertainty` was 0 at the approval
instant (`permission_created`: `deadline_host_ns == approved_host_ns`, `remaining_ns = −6326072`).
Transport hops stayed flat through CLIMB (arrival→pair 3.3 ms, `frame_conversion` 2.0 ms,
`mavros_conversion` 1.3 ms median); the conversions did not cause the expiry.

## A. Baseline (measured, per frame)

Frame age before processing = delivery to the camera callback (`source_age`) + wait in the camera
slot. Host idle time before a frame = start of frame − end of previous frame.

| Window | Delivery p50/p95/max ms (sim) | Slot wait p50/p95/max ms (sim) | Thread idle before frame p50 ms (host) | Processing p50/p95/max ms (host) |
|---|---|---|---|---|
| Before CLIMB (n=98) | 12 / 17 / 42 | −1 / 2 / 3 | 124 | 70 / 84 / 102 |
| CLIMB (n=36) | 13 / 32 / 37 | 0 / 61 / 108 | 119 (7–13 from 37.8 s) | 73 / 131 / 168 |

Frame 37.9 s: delivered after 26 ms, waited 54 ms in the slot, processed for 120 ms, published 175 ms
old (source check), 183 ms old after enqueue. Source→transport delivery was 1.2 ms.

DISARMED test: its logs are not in the committed evidence, so it is not compared here.

## B. Source of the backlog

* Not a FIFO queue: the camera slot is latest-only (`vio_ros_source.RosFrameSource._latest`).
* Not the camera callback: it only converts the image; delivery stayed 12–37 ms.
* Not a slow fixed-rate loop: the consumer polls every 2 ms.
* **The single frame thread saturated.** From 37.8 s processing rose to 111–168 ms while takeoff
  raised the real-time factor from 0.51 to 0.73, so the thread stopped going idle (7–13 ms between
  frames instead of ~124 ms) and each new frame waited for the previous one. The largest removable
  item was IMU propagation: every frame integrated its ~100 IMU samples (1 kHz) on the frame thread,
  23–70 ms per frame, although they had arrived up to ~230 ms earlier. The pinned
  `vio/vio_core.py` `predict()` is 83 % of that cost.
* At real-time factor ≈ 1 the thread has more work than time (frame work 82–117 ms plus IMU
  37–86 ms per 100 ms in 38.4–39.5 s; the earlier run's airborne segment shows the same). Moving work
  into idle time cannot help there; only shedding frames that would be stale can.
* **Contract mismatch.** The source published up to 180 ms old, but the transport could only use
  samples younger than ~175 ms (200 − 17 ms uncertainty − ~8 ms pairing). 37.9 s passed the source
  check at 175 ms and was certain to expire.

## C. Fix

1. **IMU off the frame path** (`vio_imu_preintegration.py`): the same per-sample `ekf.predict`
   calls, in the same order, run between frames in chunks of 10, never past a frame that may still
   be processed; the frame path keeps the remainder, the split at the frame time and every
   fail-closed check. Bit-identical EKF state is tested against the original loop.
2. **Most recent frame when behind** (`vio_admission.py`): a frame whose age at start plus the
   recent processing time would exceed the source publication age is skipped, never two in a row
   (gaps ≤ 2 × camera period). Every processed frame still goes through the unchanged checks.
3. **Coordinated latency contract** (`vio_latency_contract.py`): the source publication limit is
   derived from the transport's unchanged limits:
   `200 ms freshness − 20 ms declared max mapping uncertainty − 15 ms delivery reserve − 8 ms
   pairing reserve = 157 ms` (was 180 ms). Reserves are the recorded maxima (delivery 11/15 ms,
   pairing 7.9/6.3 ms in the two preserved flights). This **tightens** the source; no transport
   gate changes. The transport logs `latency_contract_exceeded` (diagnostics only, rate-limited)
   when a sample exceeds a reserve, so the reserves can be re-derived from the next run. Tests pin
   the contract to the transport's 200 ms and 20 ms in code.
4. **Every dropped frame logged** with acquisition_ns, rate-limited (`odometry_skipped` in the
   source's `skips.jsonl` and stderr): admission skips, frames superseded in the latest-only slot
   (`vio_frame_slot.py`, new), and frames never received by the source.

Acquisition stamps remain the only reference; nothing is restamped.

## G. Before/after latency

"Replay" runs the recorded per-frame costs through a model of the capture loop
(`analyze_vio_latency.replay`). Its "before" column was checked against the recording first:
median absolute error 1.7 ms, p95 9.0 ms over 186 frames. **The "after" columns are replayed, not
flown.** "Over source limit" uses the end-of-processing age; the real check runs 2.5–13.5 ms
earlier, so it over-counts frames the source would block.

**CLIMB** (sim 34.92–38.09 s, CLIMB until the latch)

| Column | acq→publish p50/p95/max ms | acq→conversion p50/p95/max ms | over transport limit 175.2 ms | over source limit 157 ms | largest gap ms |
|---|---|---|---|---|---|
| Measured (recorded run) | 50 / 168 / 189 | 50 / 165 / 183 | 2 | 3 | 100 |
| Replay, before (IMU on frame path) | 49 / 157 / 176 | 49 / 157 / 176 | 1 | 3 | 200 |
| Replay, idle-time IMU only | 36 / 124 / 146 | 35 / 124 / 146 | 0 | 0 | 100 |
| Replay, after (idle-time IMU + admission at 157 ms) | 36 / 124 / 146 | 35 / 124 / 146 | 0 | 0 | 100 |

**flight** (all frames inside /clock coverage)

| Column | acq→publish p50/p95/max ms | acq→conversion p50/p95/max ms | over transport limit 175.2 ms | over source limit 157 ms | largest gap ms |
|---|---|---|---|---|---|
| Measured (recorded run) | 50 / 191 / 276 | 49 / 58 / 183 | 13 | 14 | 1800 |
| Replay, before (IMU on frame path) | 49 / 188 / 247 | 48 / 188 / 246 | 12 | 15 | 1600 |
| Replay, idle-time IMU only | 35 / 167 / 250 | 35 / 166 / 249 | 8 | 11 | 1000 |
| Replay, after (idle-time IMU + admission at 157 ms) | 35 / 135 / 167 | 35 / 135 / 167 | 0 | 2 | 200 |

**Acquisition 37.9 s**

| Column | acq→publish ms | slot wait ms | acq→conversion ms |
|---|---|---|---|
| Measured (recorded run) | 185.4 | 52.5 | 183.0 |
| Replay, before (IMU on frame path) | 176.4 | 33.2 | 175.8 |
| Replay, idle-time IMU only | 146.2 | 4.5 | 145.7 |
| Replay, after (idle-time IMU + admission at 157 ms) | 146.2 | 4.5 | 145.7 |

Replay validation: {"frames": 186, "median_abs_error_ms": 1.7, "p95_abs_error_ms": 9.0}. Frames skipped by admission: 9.

**Earlier run 20261117-flight6f32c08e7aaf, airborne 43.5–51 s (RTF ≈ 0.99), acq→publish ms**

| Column | p50 / p95 / max | over 175.2 ms | over 157 ms | skipped |
|---|---|---|---|---|
| Measured | 209 / 244 / 337 | 52 | 57 | — |
| Replay, before (IMU on frame path) | 199 / 243 / 302 | 47 | 57 | 0 |
| Replay, idle-time IMU only | 204 / 242 / 303 | 52 | 57 | 0 |
| Replay, after (idle-time IMU + admission at 157 ms) | 130 / 161 / 251 | 1 | 4 | 33 |

In the measured CLIMB row the over-limit frames are the last before the latch, so no later frame
closes a gap inside the window. Measured acquisition→conversion covers only frames that reached the
transport before it latched.

## Remaining risk

* No flight has run with this change; the after-numbers are replayed from recorded costs.
* The tighter source limit can still block a frame when processing spikes beyond the recent
  maximum (replay: ≤ 2 frames here, ≤ 4 in the earlier airborne run). That is fail-safe (nothing
  stale is sent), but the unchanged readiness gate then withholds ~3 more frames. Below a
  real-time factor of ~0.8 such a pause can approach the transport's 500 ms watchdog.
* Under sustained overload the source publishes fresh samples at a reduced rate (9 of 194 frames
  skipped here, 33 in the earlier run's airborne segment). If frame work alone exceeds the camera
  period, no scheduling keeps outputs fresh; a test asserts this case is reported, not hidden.
* The real lever for headroom at real-time factor ≈ 1 is CPU: the pinned `vio_core.predict` costs
  0.3–0.86 ms per sample on the flight host (owned by the VIO teammate, hash-pinned), or more CPU
  for the VIO container.
* Evidence that would settle the remaining questions: the source's `flight-source/stages.jsonl`
  (preemption vs compute) and the VIO container's cgroup `cpu.stat`.

---

# Follow-up: run 20261117-flightfea1c93152a2 (VIO availability watchdog, 5 s into CLIMB)

Evidence: `integration/phase1_runs/20261117-flightfea1c93152a2/evidence/` (commit `127e43a`):
`flight-source/skips.jsonl`, `flight-transport/{stages,flight}.jsonl`, `flight_result.json`.
The source's `stream.jsonl` was not committed, so per-frame detail exists only for the frames the
rate-limited skip log reported (first of each reason per second, with suppressed counts).

## What happened

| Sim time | Evidence | Meaning |
|---|---|---|
| 38.5–40.6 s | transport `odom_arrival_age` 33–57 ms every frame; RTF 0.57–0.60 | CLIMB commanded, still on the ground; fresh odometry every frame |
| 40.7–41.2 s | arrival age 87 → 108 → 124 → 142 → 145 → 148 ms; RTF jumps to 0.99 | liftoff removes ground contact; 100 ms of wall time per frame, processing 86–113 ms: the source falls behind (not a queue: nothing superseded) |
| 41.3 s | `skipped: predicted to exceed publication age`, age 53 ms + expected 105 ms > 157 ms | admission skip, as designed |
| 41.4 s | `EKF not ready`, 31 inliers, audit `ambiguous spatial support/flow`, predicted scale 0.636 | the frame was paired with 41.2 s across the skipped one: 0.2 s baseline while the camera height rose ~57 %. Tracking/audit failed, so the flow update was not accepted. Estimator epoch stayed 0: no EKF reset or internal fault |
| 41.5, 41.7 s | `readiness hysteresis 1/3`, `2/3 (0.200 s)` | recovering |
| 41.8 s | published, 600 ms (sim) after 41.2 s | 100 ms after the watchdog (`odom_age_ns 500764476`, `gate_blocked false`) |

Answers to the questions asked:

* **Inliers / "EKF not ready".** Loss of tracking, not an EKF fault: "EKF not ready" is the gate's
  label when the flow velocity was not accepted, and it was not accepted because the spatial audit
  failed on the 0.2 s pair. Later in EMERGENCY_LAND inliers stayed low (18–74) while frames kept
  being skipped or blocked, i.e. pairs kept spanning 0.2–0.3 s.
* **Backlog vs throughput.** Throughput. Nothing queued (one superseded frame in the whole flight);
  once airborne at RTF 0.99 each frame took 86–113 ms of a 100 ms period, so ages rose by
  ~15–20 ms per frame until the admission check skipped one.
* **Why readiness did not return in time.** By construction. At 10 Hz the unchanged
  3-frame/250 ms readiness needs 4 good frames after a block, so one blocked frame means ≥ 0.5 s of
  simulation time without output, which is ≥ 500 ms of wall time at RTF ≤ 1 — the watchdog limit.
  **In flight the system tolerates zero gate blocks.** Neither gate is changed; the blocks must be
  prevented.

## Fix (thresholds and watchdog unchanged)

1. **IMU propagated in 5 ms blocks** (`vio_imu_preintegration.PROPAGATION_STEP_S`). The flight model
   feeds the ArduPilot IMU at 1 kHz; the VIO was developed against 200 Hz
   (`gazebo/worlds/camera_world.sdf`). Each block is one pinned `predict()` with the
   interval-weighted mean specific force (same body-frame Δv), the rotation held at the block
   midpoint, and the interval-weighted mean gyro (per-frame gyro mean unchanged). Every raw sample
   still passes the gap/non-finite checks. Measured: 3.9× less IMU time; after 3 s of 1 rad/s
   rotation and ±1.5 m/s² manoeuvring, |Δv| 9.4e-6 m/s and |Δp| 1.8e-5 m vs per-sample propagation;
   covariance within 0.1 %. A ≤ 200 Hz IMU is propagated exactly as before.
2. **A skipped frame stays the tracking reference** (`vio_admission.keep_as_reference`): the skipped
   image (and its camera depth for scale compensation) becomes the frontend/audit reference, so the
   next pair is one period apart; the gyro window restarts with it. Nothing is estimated or
   published for the skipped frame; IMU gap evidence is kept.

## Replay of recorded costs (not flown)

The failing run cannot be replayed (no `stream.jsonl`); these use the two runs that have one.

| Run / window | Policy | acq→publish p50 / p95 / max ms | frames over 157 ms | skipped | largest fresh gap |
|---|---|---|---|---|---|
| 4414cb818f2a, whole flight | before (`frame`) | 49 / 178 / 247 | 15 | 0 | 1.9 s |
| 4414cb818f2a, whole flight | `idle+admit` (c20de2c/6f9052f) | 35 / 124 / 167 | 2 | 9 | 0.4 s |
| 4414cb818f2a, whole flight | **`idle+admit+5ms` (this fix)** | 35 / 124 / 155 | **0** | 3 | **0.2 s** |
| 6f32c08e7aaf, airborne RTF 0.99 | before (`frame`) | 199 / 243 / 302 | 57 | 0 | 3.3 s |
| 6f32c08e7aaf, airborne RTF 0.99 | `idle+admit` | 130 / 158 / 251 | 4 | 33 | 0.3 s |
| 6f32c08e7aaf, airborne RTF 0.99 | **`idle+admit+5ms`** | 121 / 138 / 266 | **1** | 11 | 0.4 s |

The remaining 6f32 frame is a single recorded processing spike; those recorded costs also predate
the parallel audit, so they overstate today's cost.

## What to check on the Gazebo side

1. **Commit the source's `flight-source/stream.jsonl` (and `stages.jsonl`) with every run.** Without
   it, inliers, contrast, heights and per-stage times are only visible for rate-limited skip rows.
2. **Real-time factor at liftoff.** Every failing run shows RTF jumping from ~0.6 (ground contact) to
   ~1.0 at liftoff, which cuts the per-frame budget from ~165 ms to 100 ms in one step. Check
   `<physics><real_time_factor>` / `max_step_size`, and whether lockstep with ArduPilot is holding
   the sim back on the ground. Capping RTF (e.g. 0.6–0.7) would make the simulation test the logic
   with headroom, but it does not prove real-time capability; please tell us if you use it.
3. **IMU rate.** The VIO consumes the iris `imu_sensor` at 1 kHz. A separate 200 Hz IMU sensor for
   the VIO topic (keeping 1 kHz for ArduPilot) would be equivalent to fix 1 without the averaging.
4. **Camera.** Confirm the gimbal camera really publishes at 10 Hz with exact 0.1 s stamps
   (`ros_camera.jsonl`) and has no motion blur or auto-exposure enabled; check the runway texture
   under the camera at 0.2–1.0 m (inliers at rest are ~180–200; in hover they were 18–74 here, which
   the doubled baselines explain only in part).
5. **Lighting.** The vehicle's own shadow moves with it near the ground; a light directly overhead
   or shadows disabled for the camera would show whether it affects inliers at liftoff.

## Still open

* Not flown. The replay models CPU scheduling, not image tracking.
* Zero tolerance for gate blocks in flight is a property of the current readiness (10 Hz,
  3 frames/250 ms) and watchdog (500 ms) settings. Any genuine tracking glitch will still trip the
  watchdog. Making one block survivable without relaxing either needs a faster camera (20 Hz:
  re-ready in 0.3 s), which needs CPU the source does not have today. This is a design decision for
  the team.

## Follow-up: wrong flow velocities in flight (periodic ground texture)

Evidence: `debug_evidence/20261117-flight6f32c08e7aaf/flight-source/stream.jsonl`, airborne
(h > 0.9 m) fused flow velocities vs Gazebo GT displacement between the two frames.

* Error is a whole ground lattice step, not noise: every frame with error > 1 cm is off by
  (-2.0, +2.0) cm (once twice that), and only when the true displacement exceeds ~1.45 cm,
  half a period. Below that the error is < 0.6 cm.
* The frontend is confident when wrong (sigma_v 0.0012-0.0024 m/s, 70-150 inliers) and the
  distributed audit agrees, because every feature aliases together. The EKF NIS gate passes it
  (measurement floor sigma_vis 0.08 m/s > error 0.14 m/s). 14 of 55 airborne flows were fused wrong.
* Reproduced with the pinned frontend on a tiled synthetic texture: error = exactly one period.
* Consequence: with a ~2 cm ground period and a 10 Hz camera, flow is ambiguous above
  ~0.1 m/s (0.05 m/s on a skipped-frame pair), at any height.

Fix (fail-closed, `vio_flow_consistency.py`): before fusion, compare flow with the IMU-propagated
EKF prior (chi-square 2 dof 99.9 % with the frontend's measured sigma), and when the image has a
periodic texture (autocorrelation peak), accept flow only if the IMU prediction resolves the alias
(|p| > 2 * 3.29 sigma and agreement within 3.29 sigma along p). Replay of 6f32: all aliased
frames rejected, and also all good airborne frames: in this world the IMU (+-1.2 px over 0.1 s)
cannot resolve a 3.7 px period, so VIO is correctly unavailable in flight until the ground
texture changes. Diagnostics added to stream.jsonl: gyro_frd, depth_m, pair_dt_s, flow_px_s,
flow_rot_px_s, prior velocity/covariance, gt_body_q (logging only), flow_consistency.

Gazebo side: replace or rescale the runway ground texture so it has no repeat shorter than the
largest per-frame displacement (e.g. > 0.5 m for 2 m/s at 10 Hz), or a non-repeating texture.
Sun casts shadows (direction -0.5 0.1 -0.9): the drone's shadow sits at a fixed image position
(~0.55 h ground offset) and is large at takeoff; the audit rejects those frames ("ambiguous
spatial support/flow"), so it costs availability at liftoff, not wrong fusion.
