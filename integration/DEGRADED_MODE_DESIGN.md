# Gate-closed hold for single image rejections: design (rev 3)

Status: **option D, §5.2 message definition and cause list APPROVED. Phase C in progress.**
**Flight use is NOT approved.** This is not a Phase 1 PASS. Nothing is committed.

- Base: `origin/fix/phase1-linux-validated` @ `8e9ede4`, branch `feat/vio-degraded-imu-odometry`,
  worktree `/home/marta/ПОК/gnss-degraded-mode`.
- Rev 3 changes:
  - The FC firmware is now **ArduCopter 4.6.3**; every fact was re-verified.
  - The approved hold and cap rules are applied: the cap is anchored at the first closed-gate
    sample, and each status message carries a sequence number.
  - The exact status message is defined.
  - Camera and measurement plan updated.
  - Options C and (a) dropped.

## 1. Problem

One rejected frame blocks `QualityGate` (`vio_quality_gate.py:86-88`). Reopening needs at least 3
good frames spanning at least 0.25 s (`:89-94`). Our transport latches when no odometry has arrived
for more than 500 ms of host time (`phase1_transport.py:183,559,578`), and the flight contract needs
a pair no older than 500 ms (`:542-543`, `simulation_flight.py:451`).

With the **VIO output rate** f (frames actually processed and published, not the sensor rate), the
output gap after one isolated rejection is `2/f + ceil_to_frame(max(0.25 s, 2/f))`. The test
`tests/test_degraded_mode_timeline.py` checks this on the real gate:

| VIO output rate f | gap | at ratio 1.0 (hardware) vs 500 ms |
|---|---|---|
| 5 Hz | 0.80 s | **trips** |
| 8 Hz | 0.50 s | zero margin |
| 10 Hz | 0.50 s | zero margin |
| 15 Hz | 0.40 s | 0.10 s margin |
| 20 Hz | 0.35 s | 0.15 s margin |
| 30 Hz | 0.33 s | 0.17 s margin |

**Camera:** Raspberry Pi IMX219 wide-angle module, **120° FOV**. **VIO output rate on the Pi 5:
not measured; M2 comes first** (`PI5_VIO.md:174`: "No numerical Pi FPS is promised"). Whether the
hold is needed on hardware at all is therefore open until M2 (§6).

A 120° lens is far from the pinhole model at the edges. K and D must be calibrated at the operating
resolution (`PI5_VIO.md:67`), and a fisheye model may be needed. That affects the inlier and flow
rejection rates measured in M3.
In SITL, every measured run (sim/host median 0.53–0.66) trips at 10 Hz.

## 2. ArduPilot facts: 4.4.4 (SITL firmware) vs 4.6.3 (flight controller)

Full files were downloaded by commit and read verbatim:
- **4.4.4:** commit `9279df6`, `version.h:9` `"ArduCopter V4.4.4"`.
- **4.6.3:** tag `Copter-4.6.3` = `92b0cd7`, `version.h:9` `"ArduCopter V4.6.3"`.
- Local copies are in `/home/marta/.cache/gnss-degraded-evidence/ap444src` and `.../ap463src`
  (not in Git). sha256 for 4.6.3: GCS_Common.cpp `63a1129b`, AP_VisualOdom_MAV.cpp `93151c38`,
  AP_VisualOdom.cpp `426330cb`, AP_NavEKF3_PosVelFusion.cpp `2e9e2316`, AP_NavEKF3_Control.cpp
  `48d24b9f`, AP_NavEKF3.h `0d092471`.

| item | 4.4.4 | 4.6.3 | changed? |
|---|---|---|---|
| posErr formula | `cbrtf(sq(c[0])+sq(c[6])+sq(c[11]))` if `!isnan(c[0])`, GCS_Common.cpp:3543-3544 | identical, GCS_Common.cpp:3934-3935 | no |
| posErr clamp | `constrain(posErr, VISO_POS_M_NSE, 100)`, default 0.2 (MAV.cpp:32) | identical (MAV.cpp:33; AP_VisualOdom.cpp:99) | no |
| Velocity noise | `writeExtNavVelData(vel, get_vel_noise(), …)`; VISION_SPEED_ESTIMATE covariance never read (GCS:3567-3578, MAV.cpp:53) | identical (GCS:3958-3968, MAV.cpp:70); VISO_VEL_M_NSE default 0.1 (AP_VisualOdom.cpp:91) | no |
| Quality field | none | **new:** `VISO_QUAL_MIN` (AP_VisualOdom.cpp:109-115, default 0). A sample is fused only if `quality >= VISO_QUAL_MIN` (MAV.cpp:40-43, 68-71). VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE always pass **quality = 0, "unknown"** (GCS:3939, 3968). Only ODOMETRY forwards `m.quality` (GCS:3902, 3907) | **yes, see below** |
| Quality semantics | n/a | gating only (accept or drop), never weighting: −1 failed, 0 unknown, 1–100 | new |
| reset_counter | a change resets EKF position (Backend.cpp:77-85; PosVelFusion.cpp:546) | identical (Backend.cpp:86; PosVelFusion.cpp:628) | no |
| EKF3 R for ext-nav | pos `sq(constrain(posErr,0.01,10))` :665; vel `sq(constrain(err,0.05,5))` :652 | identical, :747 / :734 | no |
| Innovation gates | GPS gates, default 500 (5σ), :731/:808 | identical, :813/:890 | no |
| **Vision-loss timeout** | position aiding is timed out after `posRetryTimeNoVel_ms 7000` / `posRetryTimeUseVel_ms 10000` with no position pass; `minTestTime_ms = 7000` (Control.cpp:292,338-366; .h:459-461) | identical constants (.h:472-474; Control.cpp:317,377-379). The pass timer is renamed `lastGpsPosPassTime_ms` and is still set by the shared GPS/ext-nav position fusion (PosVelFusion.cpp:818-828) | **no** (rename only) |
| `deadReckonDeclare_ms` (1000) | declared, unused | used only in `setLatLng` (PosVelFusion.cpp:193), an external lat/lng reset our pipeline never calls | no effect |
| `dead_reckoning` flag | depends on `posTimeout`, Control.cpp:713,743 | identical, :767,797 | no |
| VisualOdom health | 300 ms timeout (`AP_VISUALODOM_TIMEOUT_MS`) | identical; used only for the GCS SYS_STATUS bit (GCS.cpp:336) and pre-arm (AP_Arming.cpp:1967) | no; a hold > 300 ms shows "unhealthy" in the GCS, not in flight control |

**What changed for us in 4.6.3**
1. **Quality exists, but not on our messages.** Our pipeline sends VISION_POSITION_ESTIMATE and
   VISION_SPEED_ESTIMATE through MAVROS, which arrive with quality 0. It still holds that **no
   degraded marker can reach EKF3 through our messages.** Option D sends no navigation data during a
   hold, so it is unaffected.
2. **New parameter hazard.** If `VISO_QUAL_MIN ≥ 1`, EKF3 silently drops **all** of our vision.
   This is now covered by a separate preflight check (§2.1), **not part of D**.
3. Everything D relies on is unchanged: EKF3 coasts on its own IMU in AID_ABSOLUTE for up to
   7–10 s before timing out position aiding.

**Which firmware the SITL evidence used**

| evidence | firmware |
|---|---|
| Phase 1 integration runs (this repo) | **4.4.4** pinned (`runtime_candidate.AP_COMMIT`). A committed log reads `ArduCopter V4.4.4 (9279df61)` (`phase1_runs/20261118-flightabe2d67e5306/evidence/current-fusion-1.BIN`) |
| Loss-of-VIO SITL test (`main:sitl-stand/RESULTS.md`, 00000017.BIN, 2026-11-03, EKF failsafe 8.0 s after the last VISP) | **Unknown.** `main:sitl-stand/Dockerfile` clones ArduPilot **master, unpinned**, at image build time (Dockerfile added 2026-10-10). The `.BIN` is not committed, so the version cannot be confirmed. It was not 4.6.3 by design and not the pinned 4.4.4 |

**Gap:** no SITL evidence exists on 4.6.3. The SITL firmware pin (4.4.4) differs from the flight
controller (4.6.3); this should be aligned or justified before any hardware use.

### 2.1 VISO_QUAL_MIN preflight check (separate from D)

`preflight_ekf3.viso_quality_min_error(values, ap_commit)` returns a refusal reason, or `None`:
- **refused** if `VISO_QUAL_MIN` was read back and is > 0;
- **refused** if it was not read back **and** the firmware is not the 4.4.4 commit. ArduPilot
  silently ignores a read of an unknown parameter (GCS_Param.cpp:390-393 in 4.4.4, :391-394 in
  4.6.3), so a missing value is acceptable only on 4.4.4, where the parameter does not exist.

Where it is enforced:
- `check_profile` refuses a profile with `VISO_QUAL_MIN > 0`.
- `phase1_transport.py` requests `VISO_QUAL_MIN` in its read-only parameter sweep in every mode.
  Any readback > 0 latches the route. In simulation mode, the origin/vision step waits for the
  readback on non-4.4.4 firmware and refuses with the reason above.

### 2.2 Loss-of-VIO SITL test on Copter-4.6.3 (point 2)

- **Dockerfile pinned.** `sitl-stand/Dockerfile` now checks out tag `Copter-4.6.3` and **verifies
  commit `92b0cd788ec2…`**. The build fails on any mismatch, so it is no longer unpinned master.
- **Not run.** This analysis machine (WSL) has no working Docker (Docker Desktop's WSL integration
  is disabled), so **no 4.6.3 log exists yet**. The rerun must happen on a Docker host.
- **Constraint conflict.** The original loss-of-VIO test was an **armed SITL flight**: the FS_EKF
  Land action only fires when armed. That conflicts with the standing rule "no ARM / TAKEOFF /
  LAND". Two variants:
  - **DISARMED (allowed now).**
    1. Set `LOG_DISARMED 1`.
    2. Start `send_vision.py` and confirm VISP/VISV in the log.
    3. Stop `send_vision.py`.
    4. Wait 20 s.
    5. Read XKF4 (`SS` / `TS` flags), XKF3 position innovations, NKF variances and any
       "stopped aiding" text.

    This shows the 7/10 s aiding timeout without flight.
  - **Armed SITL flight** (reproduces the 8 s FS_EKF Land result). **Needs your explicit approval**
    because of the ARM rule.
- **Log location** (both variants): `run.sh` mounts `$HOME/sitl-logs` → `ArduCopter/logs`, so the
  `.BIN` lands in `$HOME/sitl-logs` on the Docker host. Keep it with its sha256; do not commit.

## 3. Phase A drift data (simulation only; unchanged)

Source: `analyze_degraded_drift.py` → `reports/degraded_drift_phaseA.json`, 26 IMU+height-only
windows of the real estimator.
- At τ = 0.5 s: median 0.053 m, max 0.286 m.
- Drift is linear and set by the velocity error inherited from the last accepted frame.
- The simulated IMU is probably near-ideal. **Not valid for hardware.**
- Under D, these numbers matter only for the step when vision resumes. EKF3 coasting drift (M4)
  sets the cap.

## 4. Decision record

- **C** (degraded pose + velocity) and **(a)** (pose only): **dropped.** EKF3 would receive
  IMU-derived data it already has, treat it as independent, and give it full or floored weight.
- **D: approved.** No odometry while the gate is closed. A separate status signal lets the
  transport tell "gate closed" from "VIO silent". EKF3 coasts on its own IMU.

## 5. Option D design

### 5.1 Configuration (required; no hardware default)

Producer and transport each take the same three arguments, **all default `None` = feature off**:
- `--gate-closed-hold-max-sim-s`
- `--gate-closed-hold-max-host-s`
- `--gate-closed-hold-budget-evidence <json>`

Startup rules:
- If any is missing, the feature is off.
- The numbers must equal the evidence file values.
- `scope:"simulation"` requires `--simulation-vision-permit`.
- `scope:"hardware"` is **refused** until M4 has produced a measured cap (approved rule).
- The simulation evidence file states 0.5 s sim / 1.2 s host, scope simulation only.

### 5.2 Status message: exact definition (for approval)

- **Topic:** `/integration/vio/gate_status`
- **Type:** `std_msgs/msg/String` carrying one JSON object. This is the same transport mechanism as
  the existing epoch token `/integration/vio/sample_epoch`.
- **QoS:** **reliable**, keep-last depth **100**, the same as the existing epoch-token publisher
  (`run_vio_stream.py:145`). Reliable QoS with a keep-last history can still drop the oldest samples
  if the subscriber stalls, so a sequence gap is handled as in §5.3.1: it **ends the current hold
  only** and never latches by itself.
- **Publisher:** `run_vio_stream.py`, once per processed frame while the producer considers the hold
  eligible. Nothing is published for frames that were not processed (skipped, superseded, exception
  path).

The object must contain **exactly** these keys. A missing key, an extra key or a wrong type is a
malformed message.

| key | type | meaning / rule |
|---|---|---|
| `schema` | int | `1` |
| `session` | str | producer session UUID; must equal the approved producer (same value as in epoch tokens) |
| `epoch` | int | estimator epoch; must be `0` (the route only admits epoch 0) |
| `seq` | int | status sequence number: 0 for the first status of the session, then **+1 per status message** across all holds in the session. Never reset within a session |
| `frame_stamp_ns` | int | acquisition stamp of the processed frame this status describes; strictly increasing |
| `hold_start_ns` | int | acquisition stamp of the **first closed-gate frame** of this hold; constant within a hold; > `last_normal_stamp_ns` |
| `last_normal_stamp_ns` | int | acquisition stamp of the last normally published odometry sample |
| `cause` | str | one value from the closed list below |
| `frames_closed` | int | ≥ 1; count of processed frames since `hold_start_ns`, inclusive |

The message carries **no** pose, velocity, covariance or attitude.

Example:
```json
{"schema":1,"session":"5f0c…","epoch":0,"seq":17,"frame_stamp_ns":51300000000,
 "hold_start_ns":51100000000,"last_normal_stamp_ns":51000000000,"cause":"RECOVERING","frames_closed":3}
```

**Closed cause list (4 values).** Each value maps from the gate's own reason strings:

| `cause` | gate reason(s) on this frame (`vio_quality_gate.py:69-85`) |
|---|---|
| `LOW_INLIERS` | `inliers` (includes frontend unavailable: inliers = 0) |
| `LOW_CONTRAST` | `contrast` |
| `FLOW_REJECTED` | `distributed flow disagreement/unavailable`, `flow inconsistent with IMU prediction / aliased`, or `EKF not ready` **only** when it is caused by a NIS rejection of this frame's flow (`n_resets == 0`, `vel_init`, no `reset_after`) |
| `RECOVERING` | the frame passed every check and the gate is rebuilding readiness (`readiness hysteresis k/3`) after one of the three causes above |

**Every other reason ends the hold** (no status is sent; today's fail-closed path runs):
- IMU alignment, IMU gap, stale camera, clock synchronization, camera gap;
- non-finite timing or state, covariance, timestamp reset or duplicate;
- `stale at publication`;
- any `block(..., reset=True)`, any exception-path error, the camera or IMU wall watchdogs;
- an internal velocity reset, a change of epoch, DDS subscriber loss.

`stale at publication` is deliberately **excluded** (rev 2 allowed it) to keep the list closed and
visual-only.

**Producer-side conditions for sending a status:**
- The gate was `ready` before the hold started, and a normal sample exists in the same epoch. So
  there is no hold at startup or while recovering from a reset.
- Because `assess` stops at the first failure, every non-visual check is re-evaluated with the
  gate's own thresholds.
- The DDS and clock checks pass.
- `frame_stamp − hold_start ≤ max_sim` and host time since the first closed-gate frame ≤ `max_host`.

### 5.3 Transport rules (independent of the producer)

1. **Message validation.** There are two kinds of failure.
   - **Latch** (`block('gate status invalid: …')`): a message that is not from our approved
     producer. That is invalid JSON, a wrong key set or types, `schema != 1`, `session` ≠ the
     approved producer, `epoch != 0`, or `cause` not in the list. This matches how the existing
     epoch token is handled (`phase1_transport.py:205-209`).
   - **End the current hold only** (no latch): `seq != previous_seq + 1` (gap **or** repeat);
     `frame_stamp_ns` not strictly increasing; sim age versus `/clock` > 200 ms; `last_normal_stamp_ns`
     ≠ the last normal odometry the transport received; `hold_start_ns` changed within a hold.

   After a hold ends, its statuses no longer count as liveness, so the unchanged 500 ms watchdog
   decides: it latches exactly as today if normal odometry does not return in time. A single
   dropped status therefore never costs more than today's behaviour. A hung or replaying process
   cannot keep a hold alive: it needs consecutive `seq` **and** a fresh, strictly increasing
   `frame_stamp_ns`.

   **What "latch" (permanently closes the route) means.** `block()` (`phase1_transport.py:160-165`):
   - writes `blocked=true` to the durable lifecycle store (`navigation_lifecycle.py:217-221`);
   - blocks the egress store (`navigation_egress.py:359-362`);
   - sets `simulation.failure`.

   In flight mode the controller then triggers the emergency LAND (`:544`); DISARMED, the transport
   stops with an error (`:555`). Re-opening needs a new run with an explicit new reference
   (`:125`, "A used reference means restart"). No automatic recovery.
2. **Cap anchor.** The anchor is taken from the **first accepted status of a hold**:
   `hold_start_ns` (sim) and that status's callback host time (host). Later statuses never move it.
   It is cleared only when normal odometry is received.
3. **`hold_valid(now)`** requires all of:
   - not blocked;
   - ≥ 1 normal odometry sample before this hold;
   - ≥ 3 normal odometry samples since the previous hold ended, for whatever reason;
   - last accepted status ≤ 500 ms ago (host);
   - `frame_stamp − hold_start ≤ max_sim`;
   - **`ros_clock − hold_start ≤ max_sim + 200 ms`**, measured on the transport's own `/clock`
     with the same 200 ms freshness allowance every sample gets;
   - `now − hold_start_host ≤ max_host`.

   The `/clock` condition was added during implementation. Without it, a producer that stops at
   its own cap leaves the last in-cap status alive for up to 500 ms more (a test with 7 bad frames
   showed the hold bridging the whole outage). With it, the cap is hard and does not depend on the
   producer or on silence.
4. **Watchdog (`:559`):** the 500 ms constant is unchanged. When configured, it fires only if
   `not hold_valid(now)`.
   - Cap exceeded: `block('VIO gate-closed hold exceeded cap')`.
   - Status silent: `block('VIO availability watchdog')`.
   - The latch stays permanent.
5. **Flight contract (`:542-543`):** `vision_valid` is today's condition **or** `hold_valid(now)`.
   So vision is valid during a hold only while the cap is not exceeded. ARM (`:240-243`) is
   unchanged: no ARM during a hold.
6. **Nothing is forwarded to the FCU during a hold.** Normal odometry resumes through the
   unchanged pairing path.
7. **Unconfigured:** no subscription, byte-identical behaviour.
8. **Permit:** `phase1_transport.py` is a bound source, so a new simulation permit is required.

### 5.4 Files

| file | change |
|---|---|
| `integration/vio_gate_hold.py` | new, pure: cause mapping, `nonvisual_ok` (reuses `QualityGate` itself, so no thresholds are duplicated), message parse/validate, budget loading, producer and transport hold state |
| `integration/gate_hold_budget_simulation.json` | new: simulation-only 0.5 s / 1.2 s evidence, citing `reports/degraded_drift_phaseA.json` by sha256 |
| `run_vio_stream.py` | 3 arguments; `block()` ends any hold; after the publication step, either a normal publication or at most one status |
| `phase1_transport.py` | D: 3 arguments, status subscription, `hold.normal` on validated odometry, watchdog and `vision_valid` conditions. §2.1: `VISO_QUAL_MIN` readback |
| `preflight_ekf3.py` | §2.1 only: `viso_quality_min_error`, `check_profile` |
| `sitl-stand/Dockerfile` | §2.2 only: pin `Copter-4.6.3` / `92b0cd7` |
| `tests/test_vio_gate_hold.py` | new, 6 tests |
| `tests/test_degraded_mode_timeline.py` | new in this task; its source pin was updated for the changed watchdog line |

No change to `vio_quality_gate.py`, `navigation_lifecycle.py`, `navigation_egress.py`,
`simulation_vision.py` or `mavros_epoch.py`.

**Implementation note: what "normal" means at the transport.** It is the last odometry received and
validated in `on_odom`, not the last forwarded pair. A pair can complete after the next status
arrives (it waits for a clock bracket), which would otherwise cause false "last normal mismatch"
endings.

**Launchers not changed.** `launch_phase1.py` and `run_phase1_validation.py` do not pass the hold
arguments, so the hold stays off in every existing launch path until that is approved.

### 5.5 Tests (`tests/test_vio_gate_hold.py`, 6 tests)

1. A single bad frame at ratio 0.6 (real gate, both state machines, 2 ms watchdog poll): it latches
   without the hold and not with it. The odometry stream is identical (no extra samples), the
   reopen still needs 3 frames / 250 ms, and the causes are LOW_INLIERS then RECOVERING ×3. It
   also works at ratio 1.0.
2. Caps: 7 bad frames latch on the `/clock` sim cap at 1.802 s, before the reopen at 2.1 s. The
   host cap holds at 1.2 s although statuses keep arriving. Status stamps 0.5 s apart are
   allowed; 0.6 s apart are not.
3. A sequence gap ends the hold only and blocks re-entry. A foreign session, an unknown cause or
   malformed JSON raise (latch).
4. No hold at startup, after a reset, on a failed non-visual check, or on a non-visual cause.
   `nonvisual_ok` catches a non-finite state, an IMU gap and a covariance failure.
5. Budget rules: unconfigured gives off; partial settings, simulation scope without a permit,
   numbers that differ from the evidence, and hardware scope are all refused.
6. `VISO_QUAL_MIN` check (§2.1).

"Unconfigured behaviour unchanged" is shown by the unchanged full regression result (§8).

## 6. Measurements (order approved; all DISARMED, props off, faults only via `--fault-file`)

| order | ID | what | output |
|---|---|---|---|
| 1 | **M2** | Pi 5 + IMX219 hardware VIO pipeline at real time, vehicle hand-carried and static | **VIO output rate** and jitter, processing p50/p95/p99, admission skips, closure duration after an isolated rejection, transport host gap. **This decides whether the hold is needed** (§1 table) |
| 1 (parallel) | **M1** | ≥ 2 h static IMU log: FC IMU, and the VIO IMU if separate | Allan variance (VRW, bias instability), temperature, predicted drift for τ = 0.1–2 s |
| 2 | **M3** | Same pipeline over several surfaces, lighting conditions and heights | Rejections per minute per cause (in the §5.2 cause list), run-length histogram, share of closures inside or outside a cap |
| 3 (only if M2 and M3 show the hold is needed) | **M4** | FC on **4.6.3** with live vision, hand-carried. The transport drops vision for 0.2/0.5/1.0/2.0 s (labelled fault). An independent position reference (motion capture, total station or surveyed markers; GPS stays out of the EKF) | EKF3 coasting error vs reference, reported variance, resumption innovation and test ratio, any "stopped aiding" or failsafe. **This sets the hardware cap** |

**Cap rule:** `max_sim_s = max_host_s = τ*`, the largest gap for which the M4 **p99** horizontal
coasting error ≤ the position-error allowance **and** the resumption test ratio ≤ 0.5. τ* must also
be ≥ the p95 isolated-closure duration from M2. If not, the feature stays disabled on hardware.

**Position-error allowance: 0.2 m, p99, horizontal (approved starting value; may be relaxed after
M4).** That equals the existing VIO acceptance limit `xy_rmse_m ≤ 0.2` and leaves the
`max_spatial_error_m ≤ 0.4` budget (`preflight_ekf3.motion_checks`) for VIO error itself.

## 7. Open items

| item | owner |
|---|---|
| Rerun the loss-of-VIO test on the pinned 4.6.3 image (§2.2) on a Docker host; DISARMED variant now, armed variant only with explicit approval | Solia |
| Pi 5 VIO output rate (M2), calibration of the 120° lens | hardware owner |
| Phase 1 integration SITL is still pinned to patched 4.4.4 (`runtime_candidate.AP_COMMIT`, `AP444_*` patches); moving it to 4.6.3 is a separate change | Solia |
| New simulation permit after the transport change | Solia |
| Port to `main`: C2 `tolerate_single_rejection` is mutually exclusive with this hold | Marta |

## 8. Phase C regression result (WSL, offline only)

Command: `PYTHONPATH=<pydeps>:.:tests python3 -m unittest discover -s tests` in `integration/`, plus
`python3 -m pytest vio`. Logs are in `/home/marta/.cache/gnss-degraded-evidence/` (not in Git).

| suite | baseline 8e9ede4 (clean) | after Phase C |
|---|---|---|
| integration unittest | 559 run: 553 pass, 1 fail, 5 skip | 571 run (+12 new): 565 pass, **same 1 fail**, 5 skip |
| `vio` pytest | 4 pass, 1 fail | 4 pass, **same 1 fail** |

- **The integration failure** is `test_heartbeat_e2e … cleanup_deadline_exceeded`. It is a WSL
  timing failure and also fails on a pristine 8e9ede4 checkout.
- **The `vio` failure** is `QualityMonitor.inflate` called on the class (a test-usage issue).
- No test was modified except the source pin inside the new `test_degraded_mode_timeline.py`.
- Verified **offline only**: no ROS, Gazebo, SITL or Docker on this machine. Live DISARMED
  validation on the Linux host and a new simulation permit are still needed.
