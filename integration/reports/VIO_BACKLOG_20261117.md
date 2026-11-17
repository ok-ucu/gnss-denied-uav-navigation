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
