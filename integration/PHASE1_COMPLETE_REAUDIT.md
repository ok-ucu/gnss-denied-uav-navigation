# Phase 1 complete re-audit (after team fixes)

Date: 2026-11-19 · Auditor host: `DESKTOP-7OJSGJR` (WSL2 Ubuntu 24.04, RTX 4050 Laptop) · Audited refs: `main` f2afe82, `test/ref-port-openvins-ardupilot` 790eb6f (HEAD), `origin/phase1-vio-imu-robustness` 6175539, `origin/fix/phase1-disarmed-diagnostics` 583f233, `origin/fix/audit-person2-and-tests` 1ab9273

**Decision: HOLD. Phase 1 is not ready for CLIMB/HOVER/LAND.**

The blocking defect is unchanged: the quality gate's re-readiness rule cannot meet the 500 ms host-time availability watchdog after any visual rejection. No reviewed recovery or coasting mechanism exists on any branch.

## Limits of this audit (read first)

1. **This is not the runtime host.** The brief assumed the HP ZBook with Gazebo, SITL and evidence. This machine has no Gazebo, ROS 2 or ArduPilot SITL, and Docker Desktop's WSL integration is disabled. **Step 9 (DISARMED NVIDIA validation) was not run and no fusion proof was created.**
2. **The claimed latest state is not available.**
   - HEAD `d13e1a8`, `integration/PHASE1_LINUX_SHADOW_FIX_VALIDATION.md` and run `20261119-shadowvalidation123650` exist in no fetched ref, in no commit's history, and nowhere on this filesystem.
   - Every result attributed to them is **NOT VERIFIED**: DISARMED PASS, 338 pairs / 338 VISP / 338 VISV, 313.493 ms / 316.255 ms gaps, 116 ms age, 0 watchdog, 589/1/7 tests.
   - The same applies to the listed fixes that have no code in any ref (Section 4).
3. **What was verified:** everything that has been pushed to `origin` (fetched 2026-11-19), the raw flight evidence committed in Git, and offline test runs on three source trees.
4. **Environment interruptions.** WSL rebooted twice during the audit (11:06:58 and 11:53:19), wiping `/tmp`. The test runs were repeated from durable worktrees in `/home/marta/ПОК/gnss-reaudit-wt/`; all numbers below come from completed runs.

## Step 0: repository synchronisation

| Item | Result |
|---|---|
| Starting state (2026-11-19 11:17:26 UTC) | HEAD 790eb6f on `test/ref-port-openvins-ardupilot`, tracking `origin/test/ref-port-openvins-ardupilot`. Working tree clean except untracked `PROJECT_AUDIT_REPORT.md` (preserved, not touched). No stash. One worktree. |
| `git fetch --all --prune --tags` | Pruned `origin/fix/gazebo-no-shadows` (deleted on the remote; its commit `ad8e949` is still in `main` and in `origin/phase1-no-shadows`). New ref: `origin/phase1-vio-imu-robustness` 6175539 (Olena Kulynyak, 2026-11-19 12:39 +0200). No tags. |
| Fast-forward of tracking branches | **Nothing to update.** All 8 local tracking branches are 0 ahead / 0 behind their upstream. `audit/integrate-fixes` (79af0e1) has no upstream; its content is identical to `main` (via 01ab634). |
| Divergence | None between local and remote. Not merged into `main` (from the remote side): `phase1-vio-imu-robustness` (+1/−42), `fix/phase1-disarmed-diagnostics` (+2/−44), `fix/audit-person2-and-tests` (+1/−42), `test/ref-port-openvins-ardupilot` (+1/−0). `fix/vio-phase1-fbf7` (+2/−16) is content-identical to `main` (stale). |
| Not done (by constraint) | No reset, checkout, clean, stash, merge, rebase, commit or push. Two **detached** inspection worktrees were added: `/home/marta/ПОК/gnss-reaudit-wt/{olena,integrated}`. They can be removed with `git worktree remove`. |

### Branch tips (all refs)

| Ref | Tip | Author | Date | Subject |
|---|---|---|---|---|
| origin/phase1-vio-imu-robustness | 6175539 | Olena Kulynyak | 11-19 12:39 | harden IMU timestamps and improve VIO scene quality |
| test/ref-port-openvins-ardupilot (= origin) | 790eb6f | mor1artea (Marta) | 11-19 09:46 | port open_vins_ardupilot reference checks; fix send_vision, VIO guards, sample_flight |
| main (= origin/main) | f2afe82 | mor1artea | 11-19 00:04 | opt-in single-rejection tolerance in readiness gate (audit C2) |
| origin/fix/phase1-disarmed-diagnostics | 583f233 | s.leshchuk8 | 11-18 22:22 | Test Phase 1 DISARMED failure reporting |
| origin/fix/codex-repository-repair | 17138b1 | s.leshchuk8 | 11-18 21:31 | resolve repository audit findings (in main) |
| origin/fix/audit-person2-and-tests | 1ab9273 | Устим Трухін | 11-18 21:24 | Fix Person 2 audit items (superseded, see B) |
| origin/phase1-no-shadows | ad8e949 | Olena Kulynyak | 11-18 17:37 | disable sun shadows (in main via ece5622) |
| origin/phase1-integration-merge | 348cdd8 | mor1artea | 11-18 13:43 | pin run abe2 liftoff failure (in main) |
| older: fix/vio-phase1-fbf7, gazebo-sim, fix/vio-climb-continuity, person4-phase2-offline-integration, vio-setup, gazebo-sitl-integration, sitl-stand, mavros-integration | — | — | ≤ 11-18 | all contained in `main` or content-identical |

## A. Executive summary

**What works (verified offline from pushed code):**
- The VIO estimator, frontend, quality gate, flow-consistency and audit checks, latency contract and transport predicates.
- The shadow-free world generation and its pre-start check (image and runtime).
- Olena's duplicate-IMU fix, Marta's VIO guards / `send_vision` / dataset fixes, and Solia's DDS-readiness and diagnostics work.
- The team's changes are **compatible**:
  - The fully integrated tree (`790eb6f` + `6175539`) passes **817** tests.
  - Its only failures are the known gate/watchdog class (8) and three environment/invocation artefacts (3).
  - The only merge conflict is one import line.

**What doesn't work, or isn't proven:**
1. **Gate vs. watchdog (P0, unresolved).** At the recorded RTF of 0.656, *any* rejected frame trips the 500 ms host watchdog with the default gate, at any camera rate up to 30 Hz. The C2 opt-in rescues exactly one isolated rejection. Two consecutive rejections always trip.
2. **No runtime evidence after the shadow fix in Git.** The last committed flights (abe2, fbf7, 2026-11-18) predate it, and all ended in `VIO availability watchdog` during CLIMB.
3. **The work is fragmented across four lines.** The newest runtime state (`d13e1a8`) is unpushed.
4. **The fusion proof does not bind the integration-side VIO sources** (provenance gap), and **no fresh proof exists** for the current permit-bound sources.

## B. Team changes

Attribution is by Git author.
- **"Solia":** taken to be `s.leshchuk8` / `s-leshchuk8` (s.leshchuk.pn@ucu.edu.ua). This is an **inference**: every recorded flight was executed from `/home/solomiia/...` (e.g. `phase1_runs/20261118-flightabe2d67e5306/evidence/flight-owned.json`). No commit uses the name "Solia".
- **"Marta":** `mor1artea` (martaharhalis@gmail.com), the account of this checkout.

| Who | Commit(s) | Files | What changed | Problem addressed | Status | Verification |
|---|---|---|---|---|---|---|
| Olena | **6175539** | `vio_imu_preintegration.py`, `run_vio_stream.py`, `vio_quality_gate.py`, +7 runtime/world files (byte-identical to `main`), tests, 2 reports | (1) exact duplicate IMU samples advance only the cursor; conflicts, backward time, NaN and gaps still raise; (2) seed sample validated; preintegrator hash logged; (3) inlier/contrast minima also guard **fusion**, not only publication; (4) shared constants | IMU duplicate → estimator re-init; weak frames fused | **NOT in main** | Offline: duplicate test passes in the integrated tree; `advance()` cost unchanged (73.4 vs 74.3 µs/sample per-sample mode, 21.8 vs 20.1 µs block mode; identical state). Her SHA-256 table matches the Git objects exactly. Linux: **not run** (her report says so too). |
| Olena | ad8e949 (→ main via ece5622) | `Camera.Dockerfile`, `launch_phase1.py`, `run_camera.py`, both Compose files, tests | Sun `cast_shadows` disabled in image world and in the runtime-derived world; Gazebo refuses to start unless the mounted world passes the check | Drone self-shadow collapsing liftoff features (abe2) | In main | Offline tests pass; one test skips without the original pinned upstream SDF. Bench (her report): shadow-free inliers 165–192 vs 34–117 shadowed for abe2 heights. **Live: NOT VERIFIED here.** |
| Solia | 17138b1, 6a1a2cb (in main) | `run_vio_stream.py` (`downstream_status`), `vio/test_vio_core.py`, gazebo/verify tools, heartbeat test helper, run_all_tests.sh, requirements | DDS readiness classified (`awaiting_initial_match` / `below_minimum_after_match`); **QualityMonitor test fixed to the instance API**; test reliability | Audit A1–A9/B1; misleading DDS skips | In main | All pass on HEAD and integrated. |
| Solia | c04c221, 72a18ef, c3ac7eb, 9ed61a0, 858273e (in main) | `vio_flow_consistency.py`, `run_simulated_flight.py`, transport/launcher/validation | Flow-consistency module shipped; non-repeating ground texture; DISARMED ExternalNav ingress validation; VIO-loss watchdog/run-environment checks; permission-expiry evidence | fbf7 aliasing; ingress validation | In main | Covered by the integration suite (pass). |
| Solia | f77f373, 394f15c, 127e43a, 8a03a30, db61d2a (in main) | `phase1_runs/*`, `debug_evidence/*` | Raw flight evidence for runs 6f32, 4414, fea1, fbf7, abe2 | Evidence preservation | In main | Recomputed in Section E. |
| Solia | **4abc659, 583f233** | `launch_phase1.py`, `run_phase1_validation.py`, `tests/test_runtime_candidate.py` | Failing DISARMED subprocesses raise with a bounded 4 KiB log excerpt | Opaque DISARMED failures | **NOT in main** (merges cleanly) | Diagnostic only, with its own tests. It touches **permit-bound** files (fresh proof needed after merge). |
| Marta | f2afe82 (main) | `vio_quality_gate.py`, `run_vio_stream.py`, `run_simulated_flight.py` | Opt-in `--tolerate-single-rejection` (default **off**) | Gate/watchdog (single rejection only) | In main | Tests pass; not enabled in any launcher. |
| Marta | **790eb6f** | `tests/ref_port/*`, `vio/vio_guards.py`, `vio/vio_ekf.py`, `sitl-stand/send_vision.py`, `run_vio_stream.py`, `vio_quality_gate.py`, dataset generator + `sample_flight` frames, `gazebo/regression_test.py` | Reference-ported tests; caller-side guards; new epoch after a forced EKF reset; `send_vision` stale/bootstrap/covariance; dataset direction | See `TEST_PORT_REPORT.md` | **NOT in main** | All targeted tests pass; `send_vision` live-SITL test skipped (no SITL). |
| Ustym | 1ab9273 | gazebo tools, `requirements.txt`, `run_all_tests.sh`, `vio/test_vio_core.py`, `dataset/recorder.py` | Same audit items as Solia's repair | — | **Superseded** by main; conflicts in 4 files | Do not merge: its silent fallback to `FallbackFlowFrontend` contradicts main's `test_missing_production_frontend_does_not_fall_back`. |

**Compatibility (computed with `git merge-tree`, nothing merged):**
- `main` + 6175539: clean.
- `main` + Solia's diagnostics: clean.
- 790eb6f + 6175539: **one conflict**, `integration/run_vio_stream.py` imports only (resolution: union of both import lists).
- Semantics fit together: Olena's pre-fusion inlier/contrast condition wraps the fusion call, and Marta's `needs_new_epoch` check sits inside it.
- The resolved tree was tested (Section D).

### Solia: specific questions from the brief

| Area | Touched by Solia? |
|---|---|
| VIO frontend / tracking | No (frontend pinned; `vio_flow_consistency.py` shipped, authored earlier by Marta) |
| Camera/IMU sync | `isolated_imu.py` small change in c3ac7eb; no sync algorithm change |
| Estimator | No |
| Quality Gate | Tests only (pins the 500 ms single-rejection case) |
| ArduPilot EKF3 | `920c90c` validated parameter candidate (older); no new EKF3 change |
| Gazebo camera/world | No (world shadows are Olena's) |
| Docker Compose | `phase1.compose.yaml` in c3ac7eb (ingress validation) |
| Watchdog / transport | `phase1_transport.py` in 858273e (permission-expiry evidence); watchdog constant unchanged |
| Not in the runtime yet | 4abc659 and 583f233 (diagnostics branch) |

## C. Component status

| Component | Code state (pushed) | Runtime state | Notes |
|---|---|---|---|
| Gazebo world | Shadow-free world derived at launch and checked before Gazebo starts (main = 6175539) | Not verified since the fix | `cast_shadows` off only; texture, physics and camera unchanged |
| Camera | 640×480, fx 205.4696 (hard-coded in `run_vio_stream.py`), 10 Hz sim grid | abe2: 10 Hz frames, max publication age 101 ms | No camera-config change found after 348cdd8 except the shadow flag |
| IMU | Gazebo IMU ~1 kHz, 5 ms propagation blocks; duplicate handling only in 6175539 | abe2: no IMU-gap epochs (`recoveries` 0) | Gaps > 20 ms still start a new epoch (by design) |
| VIO estimator | Pinned `vio_core.py` / `vio_frontend.py` / `vio_ros_*` match pin d33f660 on every line | — | Guards are applied at callers (`vio_guards`, preintegrator) |
| Quality gate | 3 frames + 250 ms readiness; C2 opt-in (off) in main | Recorded outage 0.900 s in abe2 | **Blocking**, see F-B1 |
| Admission / latency contract | Unchanged: 157 ms source age (200 − 20 − 15 − 8 ms) | abe2 max age 101 ms; fbf7 109 ms | 4414/6f32 (older code) had 175–177 ms |
| Transport / watchdog | Unchanged: 500 ms **host** monotonic (`phase1_transport.py:521`), completed-pair check `:508` | Tripped in fbf7 (500.028 ms) and abe2 (500.637 ms) | Time-base mismatch with the sim-time gate |
| EKF3 | `EK3_SRC1_POSXY/VELXY=6`, `POSZ=1`, `VELZ=0`, `YAW=1`, `VISO_TYPE=1`, `GPS_TYPE=0` (abe2 prearm snapshot) | fusion14 report: ExtNav used; raw evidence not committed | Claimed 338 VISP/VISV run: not verifiable |
| Docker / Compose | Mount paths unchanged since 348cdd8; Gazebo start gated by the world check | Not runnable here | Fresh proof needed (permit-bound files changed) |

## D. Tests

The runs used pytest 9.2 with numpy 2.5.4 and OpenCV 5.0.0. pytest, pymavlink and pyproj were installed into a scratch target directory. Raw logs are in `/home/marta/ПОК/gnss-reaudit-wt/results/`.

| Suite | HEAD 790eb6f | Integrated (790eb6f + 6175539) | Olena 6175539 (as pushed) |
|---|---|---|---|
| vio/ | 5 P | 5 P | 4 P / **1 F** (QualityMonitor) |
| integration/tests (pytest from repo root) | 679 P / 3 F / 6 S (+363 subtests) | 685 P / 3 F / 6 S (+389 subtests) | **collection error** (2 modules), see below |
| integration/tests (unittest, her documented convention) | — | — | 529 run: 524 P / 1 F / 1 E / 3 S (her report: 526 P / 0 F / 3 S on macOS) |
| gazebo/ | 7 P | 7 P | 1 P (older test set) |
| dataset/, camera/, calibration/ | 3 / 3 / 3 P | 3 / 3 / 3 P | 3 / 3 / 3 P |
| vio_offline/tests | 22 P | 22 P | absent on this base |
| integration/person1_phase2 | 7 P | 7 P | absent on this base |
| tests/ref_port | 81 P / 9 F / 2 S / 4 xfail | 82 P / 8 F / 2 S / 4 xfail | absent |
| **Total** | **810 P / 12 F / 8 S / 4 xfail** | **817 P / 11 F / 8 S / 4 xfail** | — |

### Every FAIL

| Test | Tree | Traceback (key line) | Root cause | Class | Recommended fix |
|---|---|---|---|---|---|
| `test_heartbeat_e2e::test_shared_posix_clock_has_cross_process_epoch` | HEAD, integrated | `CalledProcessError: ['/usr/bin/python3','-B','-c','from synthetic_heartbeat_e2e import host_now…'] exit 1` | The subprocess imports an `integration/` module with no `cwd`/`PYTHONPATH`; it works only when tests run from `integration/`. **Re-run with `cd integration && python3 -m unittest discover -s tests -p test_heartbeat_e2e.py`: 13/13 OK.** | Test invocation (not code) | Pass `cwd=` / `env PYTHONPATH` in the test's subprocess call (test change, needs approval), or always run integration tests from `integration/` as documented |
| `test_heartbeat_fault_proxy::test_control_separate_process_receives_during_witness_silence` | HEAD, integrated | `ModuleNotFoundError: No module named 'linux_gpu'` in the spawned process | Same as above. **Run from `integration/`: 13/13 OK.** | Test invocation | Same |
| `test_simulated_flight_report::test_plot_construction_without_saving_synthetic_demo_images` | HEAD, integrated | `ImportError: numpy.core.multiarray failed to import` | Ubuntu `python3-matplotlib 3.6.3` was built for numpy 1.x; numpy here is 2.5.4. `requirements.txt` requires matplotlib ≥ 3.9.2. | Environment | Install matplotlib ≥ 3.9.2 in the test environment |
| `test_ref_output_continuity` ×7 (single rejection [default], 3-frame loss [both], measured RTF [default], abe2 replay [both], …) | both | e.g. `one rejected frame … gap of 0.500 s … watchdog trips at 0.5 s`; `abe2 … 0.900 s (41.7 → 42.6)` | Gate re-readiness (3 frames + 250 ms, `vio_quality_gate.py:157`) vs host watchdog (`phase1_transport.py:521`) | **Real defect (P0)**, F-B1 | Safety decision, F-B1 |
| `test_estimator_reset_outage_within_watchdog` ×2 | both | `single IMU glitch … estimator re-init with odometry gap 0.800 s` | Any IMU gap or timestamp reset starts a new epoch: +0.25 s recovery + continuous-IMU window + readiness | Design (fail-closed); F-B5 | Bridge short gaps or bound the re-init outage (safety approval) |
| `test_duplicate_imu_timestamp_dropped_not_fatal` | **HEAD only** | `ImuGapError('IMU gap/reset/nonfinite')` | `main`'s preintegrator treats an exact duplicate as a gap | **Fixed by 6175539** (passes in integrated) | Merge 6175539 |
| `vio/test_vio_core.py::test_quality_drops_when_stale_and_recovers` | **Olena only** | `TypeError: QualityMonitor.inflate() missing 1 required positional argument: 'q'` (`test_vio_core.py:51`) | Test calls the instance method on the class. The production API (`QualityMonitor(...).inflate(q)`) is correct. | **Incorrect test usage, already fixed in main** by Solia's 6a1a2cb; Olena's branch predates it | None beyond merging (do not change `vio_core.py`) |
| `test_heartbeat_e2e::test_fail_closed_scenario_matrix` (scenario `cleanup_deadline_exceeded`) | Olena only (unittest run) | `AssertionError: report['cleanup']['foreign_preserved_through_cleanup'] is False` (`test_heartbeat_e2e.py:124`) | Synthetic heartbeat harness reliability. `main` carries a 9-line fix to `synthetic_heartbeat_e2e.py` (Solia, 6a1a2cb) that her base lacks; the scenario passes on HEAD and integrated (13/13 from `integration/`) | Test harness, **already fixed in main** | None beyond merging |
| `test_simulated_flight_report::test_plot_construction_…` (unittest ERROR) | Olena only (unittest run) | `ImportError: numpy.core.multiarray failed to import` | Same matplotlib/numpy ABI mismatch as on HEAD | Environment | Install matplotlib ≥ 3.9.2 |
| Olena `integration/tests` pytest collection | Olena only | `ModuleNotFoundError: linux_gpu` / `analyze_ekf3_fusion` | Her base lacks main's test-import fixes; her report uses `unittest` with an explicit `PYTHONPATH` | Invocation | None (resolved by merging) |

### Every SKIP

| Skip | Reason |
|---|---|
| `test_ardupilot_log_parser.py:115` | SITL DataFlash fixture not present (needs a runtime `.BIN`) |
| `test_gps_quality_check.py:92` | Exported SITL GPS CSV not present |
| `test_heartbeat_test_owner.py:288` | Mac-specific admission check (Linux host) |
| `test_heartbeat_test_owner.py:279` | BLOCKED by design: needs a reviewed manager backend and an approved delegated cgroup |
| `test_linux_gpu_handoff.py:162` | Original pinned upstream `iris_runway.sdf` not supplied (`NO_SHADOWS_UPSTREAM_SDF`) |
| `test_physical_analysis.py:261` | matplotlib unusable (same ABI issue); plotting NOT TESTED |
| `tests/ref_port/test_ref_sitl_link.py:240,261` | Live SITL tests; need `REF_PORT_SITL=1` and SITL on `udpin:127.0.0.1:14551` |
| 4 × xfail (`tests/ref_port`) | Strict xfails: the pinned `vio_core.py` / `vio_frontend.py` intentionally lack internal guards; the guards are applied by callers |

### Marta's 18 original failures, re-verified against current code

| Finding | Original failing tests | Current status |
|---|---|---|
| Gate vs watchdog | single (default), 3-frame (both), abe2 replay (both) | **Still relevant.** Fails on HEAD and integrated; constants unchanged on every line |
| Estimator re-init after IMU glitch | reset outage (both), duplicate IMU | **Duplicates: fixed in 6175539** (not merged). Gaps/resets: still relevant (design) |
| `send_vision.py` fabricated/stale data and covariance | 3 tests | Fixed in 790eb6f (sitl-stand path only; not the Phase 1 Gazebo/MAVROS path). **Needs live-SITL validation** (bootstrap) |
| EKF/frontend guards (backwards dt, NaN, tiny dt, first velocity, forced-reset latch) | 5 tests | Fixed at callers in 790eb6f (4 strict xfails document the pinned classes); not merged into main |
| Dataset direction | 1 test | Fixed in 790eb6f (generator + frames + regression check); not merged |

## E. Runtime evidence

Recomputed from the **raw committed files** (`publications.jsonl`, `stream.jsonl`, `events.jsonl`, `flight_result.json`). Every run predates the shadow fix and Olena's IMU change.

| Run | Publications | Max publication gap (sim / host) | RTF | Max publication age (sim) | Watchdog (acquisition / host odometry age / state) | Result |
|---|---|---|---|---|---|---|
| 20261118-flightabe2d67e5306 | 205 | 0.900 s / 0.900 s | 0.656 | 101 ms | 41.7 s / **500.637 ms** / CLIMB | FAIL: VIO availability watchdog |
| 20261118-flightfbf7d8854ed1 | 199 | 4.800 s / 4.803 s (includes post-latch) | 0.698 | 109 ms | 42.3 s / **500.028 ms** / CLIMB | FAIL: VIO availability watchdog |
| 20261117-flight4414cb818f2a | 179 | 1.900 s / 2.003 s | 0.547 | 175 ms | — | FAIL: permission expired during conversion |
| 20261117-flight6f32c08e7aaf | 195 | 3.300 s / 3.343 s | 0.739 | 177 ms | event without acquisition field | FAIL: VIO availability watchdog |
| 20261117-flightfea1c93152a2 | (publications not committed) | — | — | — | — | FAIL: VIO availability watchdog |

- **abe2 trigger:** five consecutive visual failures (41.8–42.2 s). The gap had RTF ≈ 1.0 locally, so it would trip at any RTF.
- **Inliers before/after (Olena's bench, not live):** 34–117 shadowed vs 165–192 shadow-free at the recorded heights.
- **Not verifiable here** (claimed for `20261119-shadowvalidation123650`): 338 pairs, 338 VISP/VISV, EKF3 ExternalNav, max ODOMETRY host gap 313.493 ms, max pair gap 316.255 ms, max age 116 ms, watchdog 0. Even if confirmed, a DISARMED hover-free run does not exercise the CLIMB/LAND rejection pattern that tripped the watchdog.
- **Fusion proof:** none created (no runtime). Any proof made before ad8e949 is invalid for `main`. `launch_phase1.py` and `phase1.compose.yaml` are in `simulation_vision.BOUND_SOURCES` (`simulation_vision.py:22`) and both changed.
- **GIL / latency:**
  - No new scheduler, futex or GIL traces exist in any ref. Diagnostics in code are limited to `schedstat`/`thread_time` sampling (`stage_timing.py`, `vio_source_probe.py`, `linux_gpu/sample_camera.py`); `stage_timing.py:103` states that these "are not futex/IO wait".
  - Comparative measurement: Olena's change does not add measurable CPU on the frame thread (above).
  - The historical 100–180 ms stalls remain **unexplained and unremeasured**.

## F. Remaining bugs

| ID | Severity | Root cause | Files | Evidence | Recommended fix | Regression test | Priority |
|---|---|---|---|---|---|---|---|
| B1 | **Critical** | Re-readiness needs ≥ 3 clean frames spanning ≥ 250 ms **sim**, while the watchdog is 500 ms **host**. At RTF 0.656 any rejection gives ≥ 0.762 s host. Raising the camera rate does not fix it (30 Hz single rejection: 0.508 s host). C2 covers exactly one rejection; 2+ always trip. | `vio_quality_gate.py:157`, `phase1_transport.py:521` | abe2 / fbf7 watchdogs; ref_port continuity tests; gap table below | Safety-owner decision between: (a) bounded IMU-propagated output after transient *image* failures with a justified horizon/covariance (Olena's analysis shows no bound exists yet: ≥ 0.232 m/s σ floor at 0.6 s); (b) faster re-readiness for a healthy estimator after transient image failures; (c) evaluate availability on the source's sim clock with a host backstop. **Do not change without approval.** | `tests/ref_port/test_ref_output_continuity.py`, `test_vio_flight_motion.ReadinessAndWatchdog` | P0 |
| B2 | High | Visual rejection bursts at liftoff/landing (self-shadow, scale change, low height) | Scene, `vio_quality_gate.distributed_flow`, frontend | abe2 41.8–42.2 s; Olena's bench | Shadow-free world (in main): **validate live**; measure the rejection rate on climb-height frames | `test_vio_flight_abe2` bench | P1 |
| B3 | High (process) | Work spread across `main`, 790eb6f, 6175539, Solia's diagnostics branch, and the unpushed ZBook `d13e1a8` | — | Step 0 | Push `d13e1a8` and its evidence; merge the reviewed branches (one import conflict); close superseded branches | Full suite on the merged tree (expect 817 P / 8 real F) | P0 |
| B4 | High (safety provenance) | Permit/proof binds pinned VIO plus transport/launcher sources but **not** `integration/run_vio_stream.py`, `vio_quality_gate.py`, `vio_imu_preintegration.py`, `vio_admission.py`, `vio_flow_consistency.py`, `vio_scale_compensation.py`, `vio_sensor_capture.py` | `simulation_vision.py:22-25`, `simulation_flight.py:54` | Code inspection; also flagged in Olena's report | Add the integration VIO sources to the bound set (or a separate hashed list checked by `verify_proof`) | New test: modifying any of them invalidates proof verification | P1 |
| B5 | Medium | IMU gap > 20 ms or camera timestamp reset → new epoch → ≥ 0.8 s outage | `run_vio_stream.py:367,428-430`, `vio_imu_preintegration.py` | ref_port reset-outage test; not seen in abe2 | Merge the duplicate fix (6175539); decide whether short gaps may be bridged (approval) | ref_port reset test, `BlockPropagation` | P2 |
| B6 | Medium | `send_vision.py` bootstrap logic unverified against EKF3 | `sitl-stand/send_vision.py` | Offline tests only | Run `REF_PORT_SITL=1 pytest -m sitl tests/ref_port` on a SITL host | `test_live_send_vision_bootstraps_extnav` | P2 (sitl-stand path only) |
| B7 | Medium | Frame-thread stalls of 100–180 ms unexplained | `run_vio_stream.py`, OpenCV/GIL | Historical traces only | Capture `py-spy dump --native` / `perf sched` during the next DISARMED run; compare with the abe2 stage timings | — | P2 |
| B8 | Low | Two heartbeat tests depend on the working directory; matplotlib ABI mismatch | `integration/tests/test_heartbeat_e2e.py:57`, `test_heartbeat_fault_proxy.py:195`; environment | Section D | Pass `cwd`/`PYTHONPATH` to subprocesses (test change, needs approval); install matplotlib ≥ 3.9.2 | Same tests | P3 |
| B9 | Low | Ustym's superseded branch reintroduces a silent fallback frontend | `gazebo/regression_test.py` (1ab9273) | merge-tree conflicts; main test | Close the branch without merging | `test_missing_production_frontend_does_not_fall_back` | P3 |

**B1 decision data** (real `QualityGate` from the integrated tree; host gap = sim gap / 0.656):

| Camera rate | Gate | 1 rejected frame | 2 | 3 | 5 |
|---|---|---|---|---|---|
| 10 Hz | default | 0.500 s sim → 0.762 s host **TRIP** | 0.915 TRIP | 1.067 TRIP | 1.372 TRIP |
| 10 Hz | C2 | 0.200 → 0.305 ok | 0.915 TRIP | 1.067 TRIP | 1.372 TRIP |
| 20 Hz | default | 0.350 → 0.534 TRIP | 0.610 TRIP | 0.686 TRIP | 0.838 TRIP |
| 20 Hz | C2 | 0.100 → 0.152 ok | 0.610 TRIP | 0.686 TRIP | 0.838 TRIP |
| 30 Hz | default | 0.333 → 0.508 TRIP | 0.559 TRIP | 0.610 TRIP | 0.711 TRIP |
| 30 Hz | C2 | 0.067 → 0.102 ok | 0.559 TRIP | 0.610 TRIP | 0.711 TRIP |

## G. Safety

**Still fail-closed, unchanged on every line (verified by source hash and value):**
- 500 ms host watchdog (`phase1_transport.py:521`) and the completed-pair check (`:508`).
- 200 ms transport freshness and 157 ms source publication age.
- 3-frame / 250 ms readiness.
- Frame/clock/IMU freshness, covariance and finite-state checks.
- Permission expiry; an epoch reset on IMU gaps/timestamp resets.
- No prediction-only (coasting) output anywhere.
- Pinned VIO files are unchanged.

**Changes needing explicit approval before they are relied on:**
- Enabling C2 in a launcher (it loosens readiness).
- Any IMU coasting or prediction-only publication.
- Any watchdog time-base change.
- Bridging IMU gaps instead of re-initialising.
- Adding sources to `BOUND_SOURCES` (changes permit semantics, though in the stricter direction).
- Merging Olena's pre-fusion inlier/contrast check (stricter) and Marta's `needs_new_epoch` (adds a recovery path that still uses the existing continuous-IMU recovery) should get a normal code review.

**New risks for EKF3:** none introduced in the Phase 1 path by the reviewed changes. The `send_vision.py` bootstrap affects only the sitl-stand harness.

## H. Phase 1 readiness

| # | Check | Status | Basis |
|---|---|---|---|
| 1 | Repository synced, nothing diverged | PASS | Step 0 |
| 2 | All team fixes on one reviewed line | FAIL | 4 lines + unpushed `d13e1a8` |
| 3 | Offline regression of the integrated code | PARTIAL | 817 P; 8 real failures, all class B1/B5 |
| 4 | Shadow-free world in code | PASS | main = 6175539, pre-start check |
| 5 | Shadow-free world confirmed at runtime | NOT VERIFIED | Evidence not pushed |
| 6 | Camera/IMU cadence and timestamp ordering | PARTIAL | Historical abe2 OK; current stack not run |
| 7 | VIO estimator and guards (offline) | PASS | vio, ref_port guards, preintegrator tests |
| 8 | DISARMED EKF3 ExternalNav fusion on current sources | FAIL (not done) | Fresh proof required; claimed run not verifiable |
| 9 | Quality gate vs watchdog continuity | FAIL | B1 |
| 10 | IMU robustness | PARTIAL | Duplicate fix unmerged; gaps still reset |
| 11 | Proof/provenance binds the VIO source | FAIL | B4 |
| 12 | Latency/GIL understood | PARTIAL | Freshness within budget in abe2/fbf7; stalls unexplained |
| 13 | CLIMB / HOVER / LAND | FAIL | All 5 recorded flights failed; none since |

**Flights: HOLD.** No ARM, TAKEOFF or LAND was run or authorised by this audit.

## I. Completion estimate

**About 38 %, counted, not measured.** This is a counting heuristic over the 13 checks in Section H:
- **Scoring:** PASS = 1, PARTIAL = 0.5, FAIL / NOT VERIFIED / not done = 0.
- **Score:** 3 PASS + 4 PARTIAL × 0.5 = 5.0 of 13.
- **Caveat:** every check carries equal weight, though the remaining failures are the hardest part. The blocker (B1) needs a safety decision, not just engineering time.
- **What would change it:** if the pushed `d13e1a8` evidence confirms the DISARMED run, checks 5, 6 and 8 could move up (≈ +2.5).

## J. Next actions (priority order)

| # | Task | Who (by Git ownership) | Tests / evidence | Success criterion |
|---|---|---|---|---|
| 1 | Push `d13e1a8` and the raw `20261119-shadowvalidation123650` evidence (logs, CSV, MAVLink, DataFlash) | Whoever ran the ZBook (brief author; likely Olena) | This audit's Section E script on the raw files | Commit on `origin`; claimed metrics reproduce from raw data |
| 2 | Integrate: merge 6175539 and 790eb6f into `main` (resolve the one import conflict as union), merge Solia's diagnostics, close 1ab9273 and the stale branches | Marta + Olena (review), Solia (diagnostics) | Full suite on the merged tree: expect 817 P / 8 F (B1/B5) / 3 environment | Single line; no unexpected failures |
| 3 | Bind integration VIO sources into the permit/proof (B4) | Solia (permit/launcher author) + safety review | New proof-invalidation test | Changing `run_vio_stream.py` or the gate invalidates verification |
| 4 | Safety decision on B1 (options a/b/c), with the gap table as input | Safety owner + Marta (gate) + Olena (transport) | Ported continuity tests must pass under the chosen policy | Single and 3-frame losses stay below the watchdog at measured RTF, with documented error bounds |
| 5 | One DISARMED NVIDIA run on the integrated stack, with a fresh proof from final hashes; collect `py-spy`/`perf sched` traces | Runtime host operator (Solia/Olena) | Camera/IMU cadence, inliers, rejections, publication/pair host gaps, VISP/VISV, EKF3 ExtNav, watchdog = 0, Docker cleanup | All metrics within contract; no stall > 100 ms unexplained |
| 6 | Live SITL check of `send_vision.py` (B6) | sitl-stand owner (Solia) | `REF_PORT_SITL=1 pytest -m sitl tests/ref_port` | EKF3 starts ExtNav; stale stop observed |
| 7 | Test environment: matplotlib ≥ 3.9.2; heartbeat subprocess `cwd` (approval needed for test edits) | Anyone | Those three tests | 0 environment failures |
| 8 | Only after 2–5: an approved CLIMB → HOVER → LAND simulation | Team + safety owner | Flight evidence as in abe2 | No watchdog; completed pairs < 500 ms host throughout |
