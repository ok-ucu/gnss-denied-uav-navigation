# Test port from robocin/open_vins_ardupilot: report

Date: 2026-11-19 · Suite: `tests/ref_port/` (96 tests) · Production code: items 3–5 applied (see Update); items 1–2 open

## TL;DR

Before any fix the ported suite gave **18 failed, 51 passed, 1 skipped** (the live-SITL test); see the Update section for the current result.

The TAKEOFF → HOVER → LAND blocker is reproduced from recorded flight data. In all three recorded phase-1 flights, the transport stopped the flight with `VIO availability watchdog` during CLIMB.

- **Watchdog vs. gate mismatch.** The flight source publishes nothing unless the current frame passes every check. After any rejected frame it needs ≥3 clean frames **and** ≥250 ms before publishing again, so the minimum outage at 10 Hz is **0.5 s of simulation time**. The transport watchdog fires after **0.5 s of host time**, and the flights ran at RTF 0.656, so that watchdog is only **0.328 s of simulation time**. A single bad frame is therefore already fatal by default.
- **The recorded liftoff can't survive the current gate.** Replaying the recorded liftoff (run `abe2`, 5 bad frames) through the current gate gives a **0.9 s** gap in both the default and the C2 opt-in mode.

The reference (OpenVINS) keeps publishing IMU-propagated odometry at IMU rate while individual images fail. That is the main behavioural difference.

## Update: items 3, 4 and 5 applied (approved 2026-11-19)

Ported suite now: **9 failed, 81 passed, 4 xfailed, 2 skipped** (both skips are live-SITL tests). Before the fixes it was 18 failed, 51 passed, 1 skipped.
- All 9 remaining failures are items 1–2 (F1–F5): the gate/watchdog mismatch and the estimator re-init. They are not fixed yet and need safety-owner approval.
- Existing suites are unchanged from the baseline taken before any edit: vio 5, gazebo 7, dataset 3, camera 3, calibration 3, vio_offline 22 (including the pin check), integration 679 passed + the same 3 environment failures as before. `run_all_tests.sh` exits 0.

**Pinned files.** `vio/vio_core.py` and `vio/vio_frontend.py` are hash-pinned (`d33f660`, `vio_offline/vio_source.py`, `integration/linux_gpu/prepare_vio.py`), and `PROJECT_AUDIT_REPORT.md` forbids editing them. So F9–F12 are fixed **at the callers**, not inside the pinned classes.
- The four tests that check the pinned classes directly are now `xfail(strict=True)`. They document that `VioEKF`/`FlowFrontend` themselves remain unguarded, and will flag if anyone re-pins with internal guards.
- New tests cover the caller-side guards.

| # | Status | Change |
|---|---|---|
| F6 | **fixed (adapted)** | `sitl-stand/send_vision.py`: nothing is sent before the first ATTITUDE. Until the first LOCAL_POSITION_NED it sends an explicit origin start fix with a larger covariance (0.25 vs 0.01 m²). The reference's "send nothing" rule does not transfer to this loopback harness: EKF3 publishes no LOCAL_POSITION_NED without ext-nav aiding. **Not verified in SITL** (Docker unavailable here); run `REF_PORT_SITL=1 pytest -m sitl tests/ref_port` (`test_live_send_vision_bootstraps_extnav`). Rebuild the `ardupilot-sitl` image if send_vision runs from it (the Dockerfile copies it in). |
| F7 | **fixed** | Sending stops when LOCAL_POSITION_NED is more than 0.5 s old, and resumes when it returns. |
| F8 | **fixed** | Explicit covariance on both messages. |
| F9 | **fixed at caller** | `vio/vio_guards.FrameAdmission` (new) drops empty or colour images, out-of-order frames and frames < 12.5 ms apart **before** `FlowFrontend.process()`, so the frontend's reference frame is unchanged. Used by `vio/vio_ekf.py`. The phase-1 source is unchanged (10 Hz Gazebo grid). |
| F10, F11 | **fixed at caller** | `vio_guards.safe_predict` refuses dt ≤ 0, non-finite dt, and NaN/inf specific force or attitude. `vio_ekf.py` routes every `predict`/`predict_kinematic` through it (AST-checked). Phase 1 was already guarded by `vio_imu_preintegration.advance`. |
| F12 | **fixed at caller** | `vio_guards.VelocityInitGuard`: the first velocity must be finite, ≤ 15 m/s, and confirmed by the next measurement within 0.5 m/s. `vio_ekf.py` uses it for both flow and simulated-sensor updates (AST-checked). |
| F13 | **fixed** | `integration/vio_quality_gate.needs_new_epoch()`. `run_vio_stream.py` now opens a new estimator epoch (`block(..., reset=True)` → existing continuous-IMU recovery → fresh `VioEKF`) after a forced velocity reset, instead of staying "EKF not ready" until restart. |
| F14 | **fixed** | `dataset/generate_sample_dataset.py:57-63` moves ground content down (+y) for forward flight. `recordings/sample_flight` frames regenerated (149 PNGs; all CSV/JSON identical, `meta.json` kept). `gazebo/regression_test.py` now also checks the mean velocity against `local_position.csv` (≤ 0.3 m/s; skipped if absent), and `gazebo/test_regression.py` asserts +2 m/s north. Measured: +2.03 m/s; the old frames now fail with error 4.03 m/s. |
| F1–F5 | **open** | Awaiting approval (Fix A–D below). |

## Scope and assumptions (stated explicitly)

1. **Repository location.** The requested worktree `/tmp/gnss-phase1-flight-test` (branch `phase1-integration-merge`) no longer exists: `/tmp` was cleared on reboot, and commit `61dbd07` is gone.
   - `phase1-integration-merge` (348cdd8) is fully merged into `main`, so the work was done on `main` (f2afe82) in `/home/marta/ПОК/gnss-denied-uav-navigation`.
   - Only `tests/ref_port/` and this report were added.
2. **Two VIO paths exist, and both are tested.**
   - **sitl-stand path:** `sitl-stand/send_vision.py` / `vio/vio_ekf.py` sends VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE over `udpin:127.0.0.1:14551` at 30 Hz.
   - **Phase-1 flight path:** Gazebo camera → `integration/run_vio_stream.py` (VioEKF + `vio_quality_gate.QualityGate`) → ROS → MAVROS → `phase1_transport.py`.
   - The recorded TAKEOFF failures are all in the **phase-1 path**.
   - `send_vision.py` does not run VIO at all. It echoes SITL's own LOCAL_POSITION_NED back as vision.
3. **`run_euroc.sh` does not exist** in this repo. The OpenVINS baseline runner is `vio_offline/baseline/run_baseline.py`; it was not executed (Docker/ROS are not available in this WSL).
4. **Simulation vs. host time.** Simulation time runs no faster than host time.
   - Measured RTF is 0.656, from `abe2 publications.jsonl`: 22.5 sim-s over 34.3 host-s.
   - The continuity tests use the conservative RTF = 1 bound. One extra test uses the measured RTF.
5. **Emulated decision glue.** `run_vio_stream.main()` needs `rclpy`, which is not installed here. The gate-continuity tests therefore re-create its per-frame decision glue (`run_vio_stream.py:400-417, 462-495, 532-539`) on top of the real `QualityGate`, `recovery_deadline` and `recovery_ready`.
   - The emulation is validated against real data: replaying the abe2 per-frame outcomes reproduces the recorded outage exactly (41.7 → 42.6 s).
6. **sample_flight frame direction.** The dataset's frames are judged against `calibration/camera_params.json` `T_cam_imu` and the team's own axis contract in `gazebo/verify_sync_axes.py:37-38` ("forward motion → pixels move down").
   - Gazebo flight logs could **not** confirm the sign independently: horizontal motion is too small, |corr| < 0.35.
7. **Test environment.** pytest and pymavlink were not installed system-wide. They were installed into a scratch `--target` directory and loaded with `PYTHONPATH`. numpy 2.5.4, OpenCV 5.0.0, Python 3.12.3.

## License

The reference repository is a fork of OpenVINS and is licensed **GPL-3.0** (`LICENSE` in the reference repo), commit `5f7318a1732212ebf64e1b6ddb8e72fc04f13e0a`.
- No reference source was copied. Each ported test reimplements the *behaviour* in Python against our APIs and links the exact reference file and lines in its docstring or assertion message.
- If any reference code is copied into this repository later, that code would bring GPL-3.0 obligations with it.

## Step 1: what the reference actually contains

The reference has **no pytest, gtest or ROS launch-test suite.** Its "tests" are executables, CI build jobs, and runtime guards inside the estimator.

| Reference item | What it verifies | Inputs | Expected / tolerance | Edge case? |
|---|---|---|---|---|
| `ov_msckf/src/test_sim_repeat.cpp` | Two simulator runs give identical IMU and feature measurements | sim config yaml | exact equality (`assert ==`) | determinism |
| `ov_init/src/test_dynamic_init.cpp` | Dynamic initialiser succeeds; state is compared with GT after pos/yaw alignment | simulator | `assert(success1)`; errors printed | init |
| `ov_msckf/src/test_sim_meas.cpp`, `ov_init/src/test_simulation.cpp` | Simulator emits IMU and camera streams | sim config | no assertion (prints) | — |
| `ov_init/src/test_dynamic_mle.cpp` | Ceres MLE batch init vs GT | simulator | printed | — |
| `ov_core/src/test_tracking.cpp`, `test_webcam.cpp` | KLT / descriptor tracking demo on a bag or webcam | rosbag / webcam | none (visual) | — |
| `ov_core/src/test_profile.cpp` | OpenCV/Eigen timing benchmark | random images | `assert(!isnan)` | NaN |
| `.github/workflows/build*.yml` | Builds (ROS-free, ROS1, ROS2) + `roslaunch ov_msckf simulation.launch` | Docker | exit code | — |
| `ov_msckf/scripts/run_ros_*.sh`, `run_sim_*.sh` | Batch dataset / simulation runs for ov_eval | bags | manual evaluation | — |
| `Propagator::propagate_and_clone` (L33-L101) | Refuse same/backwards timestamp; `assert |Δt − Σdt| < 1e-4` | runtime | refuse / 1e-4 s | timestamp jump, zero dt |
| `Propagator::select_imu_readings` (L268-L392) | Empty IMU → no propagation; missing IMU → interpolate; **zero-dt readings removed** | runtime | `|dt| < 1e-12` dropped | empty buffer, duplicate stamps, rate drop |
| `VioManager::feed_measurement_camera` (L310-L358) | No updates before init; **out-of-order image dropped** | runtime | drop | out-of-order |
| `ROS2Visualizer::callback_monocular` (L498-L535) | Drop images within `1/track_frequency`; cv_bridge failure → drop | runtime | drop | tiny dt, bad image |
| `ROS2Visualizer::callback_inertial` + `visualize_odometry` (L269-L326, L438-L490) | Image processed only once IMU is newer than it; **odometry published on every IMU message** (fast propagate) | runtime | continuous output | IMU/camera sync, tracking loss |
| `StaticInitializer::initialize` (L37-L115) | Needs a full IMU window; jerk / variance thresholds | runtime | refuse init | init failure |
| `UpdaterZeroVelocity` | ZUPT with disparity / chi² checks | runtime | — | hover |
| `ov_repub/src/republisher.cpp` (the ArduPilot glue) | Yaw −90° + (x,y)→(y,−x), normalise quaternion, frame ids; **nothing until first odom; latest msg republished at 20 Hz with its original stamp** | `/ov_msckf/odomimu` | — | frames, missing / late messages |

## Step 2: mapping table

| Reference test / guard | Our component | Status | Ported test(s) |
|---|---|---|---|
| propagate_and_clone same / backwards time | `vio/vio_core.py` `VioEKF.predict` | needs adaptation (unit guard) | `test_predict_zero_dt_is_noop`, `test_predict_refuses_backwards_dt` |
| propagate Σdt assert | `integration/vio_imu_preintegration.py` `frame_step/advance` | directly portable | `test_propagated_dt_sums_to_frame_interval` |
| select_imu_readings: zero-dt removal | `vio_imu_preintegration.advance` | needs adaptation | `test_duplicate_imu_timestamp_dropped_not_fatal` |
| select_imu_readings: empty buffer | `frame_step` | directly portable | `test_empty_imu_buffer_fails_closed_without_propagating` |
| select_imu_readings: missing IMU / rate drop | `advance` (gap ≤ 20 ms) | needs adaptation; ours deliberately fails closed | `test_imu_rate_drop_gap_over_20ms_fails_closed` (pins behaviour) |
| out-of-order IMU (backwards guard) | `advance` | directly portable | `test_out_of_order_imu_sample_not_consumed` |
| NaN guard (test_profile) | `VioEKF.predict` | needs adaptation | `test_predict_nonfinite_imu_does_not_poison_state`, `test_nonfinite_quality_input_blocks[*]` |
| callback_inertial waits for IMU ≥ image time | `preintegration_bound`, `idle_step` | directly portable | `test_idle_preintegration_never_passes_next_frame` |
| feed_measurement_camera out-of-order image | `vio/vio_frontend.py` `FlowFrontend.process`; `QualityGate.assess` | directly portable | `test_out_of_order_frame_rejected`, `test_duplicate_timestamp_frame_rejected`, `test_out_of_order_frame_does_not_corrupt_next_estimate`, `test_camera_timestamp_reset_blocks[*]` |
| callback_monocular track_frequency drop | `FlowFrontend` (no min dt) | needs adaptation | `test_tiny_dt_frame_pair_not_amplified` |
| cv_bridge failure → drop | `FlowFrontend` | needs adaptation | `test_empty_image_dropped_without_exception`, `test_flat_image_rejected_low_contrast`, `test_nonpositive_height_rejected` |
| test_tracking (KLT on real stream) | `FlowFrontend` + `vio_scale_compensation` | needs adaptation (synthetic texture, liftoff scale) | `test_forward_flight_sign_and_scale`, `test_yawed_vehicle_velocity_rotated_to_ned`, `test_liftoff_scale_change_tracked_with_compensation` |
| visualize_odometry: continuous output during image failures; republisher 20 Hz | `QualityGate` + `phase1_transport.py:521` watchdog; `QualityMonitor` (sitl-stand) | needs adaptation (the core of the blocker) | `test_single_rejected_frame_*`, `test_single_rejection_at_measured_flight_rtf`, `test_three_frame_tracking_loss_*`, `test_recorded_abe2_liftoff_replayed_through_gate`, `test_mavlink_path_*` |
| no output before init | `QualityGate`, `QualityMonitor` | directly portable | `test_no_output_before_initialisation`, `test_failed_frame_itself_never_published` |
| re-init after failure (VioManager / initializer) | `run_vio_stream` reset epoch, `recovery_ready`, `ekf_ready` | needs adaptation | `test_estimator_reset_outage_within_watchdog`, `test_recovery_requires_continuous_imu_window`, `test_forced_velocity_reset_has_a_recovery_path` |
| StaticInitializer refuses inconsistent init | `VioEKF.vel_init` | needs adaptation | `test_first_velocity_outlier_not_taken_as_truth`, `test_velocity_fusion_converges_from_rest` |
| republisher: nothing before first odom / original stamp | `sitl-stand/send_vision.py` | needs adaptation (MAVLink mocked) | `test_send_vision_sends_nothing_before_first_position`, `test_send_vision_does_not_restamp_stale_position`, `test_send_vision_rate_and_payload`, `test_send_vision_sets_origin_once` |
| covariance published with odometry, position-then-orientation order | `VioEKF.pose_cov/speed_cov`, `vio_ros_out.build_covariances`, `send_vision.py` | directly portable | `test_vio_ekf_covariance_layout_matches_mavlink`, `test_vio_ekf_covariance_finite_and_psd_after_long_run`, `test_twist_covariance_in_body_frame_and_unknown_angular`, `test_send_vision_declares_covariance` |
| republisher frame transform + normalise | `vio_ros_out.rot_to_quat/quat_to_rot`, `rot_body_to_ned`, `vio_geometry` | needs adaptation (we output NED/FRD, not a yaw-shifted ENU) | `test_quaternion_roundtrip_large_attitudes`, `test_frd_ned_euler_convention`, `test_lever_arm_transform_rejects_bad_inputs[*]` |
| test_sim_meas / test_simulation (streams sane) | `recordings/sample_flight` + `dataset/dataset_reader.py`, `vio_offline/recording.py`, `dataset/timesync.py` | needs adaptation (real recording instead of simulator) | `test_streams_ordered_and_rates_match_meta`, `test_events_merge_is_time_ordered`, `test_imu_at_level_flight_reads_minus_g_in_frd`, `test_timesync_has_only_imu_block_and_loaders_cope`, `test_clock_sync_removes_duplicate_and_backwards_stamps` |
| test_sim_repeat (determinism) | replay of sample_flight through FlowFrontend + VioEKF | needs adaptation | `test_replay_is_deterministic` |
| test_dynamic_init (estimate vs GT) | replay of sample_flight | needs adaptation | `test_replay_velocity_matches_ground_truth` |
| live FCU acceptance | SITL + `ext_nav.parm` | requires SITL | `test_live_sitl_ekf_accepts_vision` (`@pytest.mark.sitl`, skipped) |
| test_dynamic_mle, Ceres factors | — | **not applicable** (no batch MLE in our VIO) | — |
| MSCKF / SLAM feature updates, FEJ, online intrinsics/extrinsics calibration | — | **not applicable** (our EKF has no feature states; calibration is fixed from Kalibr) | — |
| UpdaterZeroVelocity (ZUPT) | — | **not applicable** (no ZUPT component; hover is fused as ordinary zero flow). Noted as a gap, not ported | — |
| test_profile, test_webcam | — | **not applicable** (benchmark / live webcam demo) | — |
| CI build workflows, ROS1 Dockerfiles, run_ros_*.sh | `run_all_tests.sh`, `vio_offline/baseline/` | **not applicable** to a Python pipeline; the OpenVINS baseline needs Docker (unavailable here) | — |

## Step 4: results

Command: `PYTHONPATH=<pydeps> python3 -m pytest -p no:cacheprovider tests/ref_port -rfEs -q`

```
18 failed, 51 passed, 1 skipped, 3 warnings in 14.10s
FAILED test_ref_camera_frontend.py::test_tiny_dt_frame_pair_not_amplified
FAILED test_ref_dataset_replay.py::test_replay_velocity_matches_ground_truth
FAILED test_ref_frames_init.py::test_first_velocity_outlier_not_taken_as_truth
FAILED test_ref_frames_init.py::test_forced_velocity_reset_has_a_recovery_path
FAILED test_ref_imu_propagation.py::test_predict_refuses_backwards_dt
FAILED test_ref_imu_propagation.py::test_predict_nonfinite_imu_does_not_poison_state
FAILED test_ref_imu_propagation.py::test_duplicate_imu_timestamp_dropped_not_fatal
FAILED test_ref_output_continuity.py::test_single_rejected_frame_keeps_output_within_watchdog[default_gate]
FAILED test_ref_output_continuity.py::test_three_frame_tracking_loss_keeps_output_within_watchdog[default_gate]
FAILED test_ref_output_continuity.py::test_three_frame_tracking_loss_keeps_output_within_watchdog[C2_tolerate_single_rejection]
FAILED test_ref_output_continuity.py::test_single_rejection_at_measured_flight_rtf[default_gate]
FAILED test_ref_output_continuity.py::test_recorded_abe2_liftoff_replayed_through_gate[default_gate]
FAILED test_ref_output_continuity.py::test_recorded_abe2_liftoff_replayed_through_gate[C2_tolerate_single_rejection]
FAILED test_ref_output_continuity.py::test_estimator_reset_outage_within_watchdog[default_gate]
FAILED test_ref_output_continuity.py::test_estimator_reset_outage_within_watchdog[C2_tolerate_single_rejection]
FAILED test_ref_sitl_link.py::test_send_vision_sends_nothing_before_first_position
FAILED test_ref_sitl_link.py::test_send_vision_does_not_restamp_stale_position
FAILED test_ref_sitl_link.py::test_send_vision_declares_covariance
SKIPPED [1] test_ref_sitl_link.py:195: live SITL test; set REF_PORT_SITL=1 with SITL running on udpin:127.0.0.1:14551
```

The 3 warnings are numpy `RuntimeWarning`s from `gray.std()` on the deliberately empty image.

| File | Edge cases | Normal path | Pass | Fail | Skip |
|---|---|---|---|---|---|
| test_ref_imu_propagation.py | 7 | 3 | 7 | 3 | 0 |
| test_ref_camera_frontend.py | 8 | 2 | 9 | 1 | 0 |
| test_ref_output_continuity.py | 23 | 2 | 17 | 8 | 0 |
| test_ref_sitl_link.py | 5 | 3 | 4 | 3 | 1 |
| test_ref_dataset_replay.py | 4 | 3 | 6 | 1 | 0 |
| test_ref_frames_init.py | 9 | 1 | 8 | 2 | 0 |
| **Total** | **56** | **14** | **51** | **18** | **1** |

Counts include parametrised cases.

## Root-cause analysis (every failure)

Classes:
- **BUG**: real defect.
- **DESIGN**: deliberate behaviour of ours that differs from the reference; it has a real cost, but the code works as its authors intended.
- **LATENT**: real, but every current caller guards against it.
- **DATA**: the test fixture or data is wrong, not production code.

| # | Failing test(s) | Where in our code | Class | Why |
|---|---|---|---|---|
| F1 | `test_single_rejected_frame_*[default]`, `test_single_rejection_at_measured_flight_rtf[default]` | `integration/vio_quality_gate.py:149` (re-readiness rule), `:63` (C2 off by default), `integration/phase1_transport.py:521` (500 ms **host** watchdog), `integration/run_simulated_flight.py:28,103` (flag off by default) | **BUG** (two components with incompatible budgets) | One rejected frame costs 4 frames before publishing resumes: 0.5 s sim, 0.762 s host at RTF 0.656. That exceeds the watchdog in host time, and equals it even at RTF = 1. The reference never stops publishing (IMU-rate odometry). |
| F2 | `test_three_frame_tracking_loss_*[both]` | same as F1, plus `vio_quality_gate.py:122-134` (C2 tolerates exactly one frame) | **BUG / DESIGN** | A liftoff-style 0.3 s loss gives a 0.7 s gap even with C2. Fail-closed "no flow → no output" cannot meet a 0.33 s-sim watchdog. |
| F3 | `test_recorded_abe2_liftoff_replayed_through_gate[both]` | same as F1/F2; trigger: liftoff feature collapse (frontend `None` ×3, audit ×2) at 0.20–0.31 m camera height | **BUG** (the actual blocker) | Recorded outcomes 41.8–42.2 s produce a 0.9 s gap in both modes. The existing `integration/tests/test_vio_flight_abe2.py` attributes the collapse to the drone's own shadow on the ground texture. |
| F4 | `test_estimator_reset_outage_within_watchdog[both]` | `integration/run_vio_stream.py:428-430` and `:367` (`reset=True`), `:400-409` (new epoch), `run_vio_stream.recovery_deadline` (+0.25 s), `vio_sensor_capture.recovery_ready` (0.25 s continuous IMU) | **DESIGN** (high cost) | Any IMU gap, duplicate stamp, or camera timestamp reset starts a new estimator epoch. The minimum outage is 0.8 s, so it always trips the watchdog. Not observed in the three recorded flights (no IMU-gap skips in abe2), but it is deterministic if it happens. |
| F5 | `test_duplicate_imu_timestamp_dropped_not_fatal` | `integration/vio_imu_preintegration.py:86` (`not 0 < gap`) → `ImuGapError` | **DESIGN** (feeds F4) | The reference drops zero-dt readings and continues. Ours treats an exact duplicate delivery as a gap and resets the estimator. |
| F6 | `test_send_vision_sends_nothing_before_first_position` | `sitl-stand/send_vision.py:29` (zeros) + `:44` | **BUG** (sitl-stand path only) | It sends a fabricated (0,0,0, level) fix at 30 Hz before any data. **Caveat:** in this loopback harness that zero fix may be what bootstraps EKF3 (no LOCAL_POSITION_NED exists before ext-nav starts). Verify with the `sitl` test before changing it. |
| F7 | `test_send_vision_does_not_restamp_stale_position` | `sitl-stand/send_vision.py:43-45` | **BUG** (sitl-stand path) | When SITL stops sending LOCAL_POSITION_NED, the frozen position keeps going out with fresh timestamps. ArduPilot cannot detect the loss (FS_EKF never sees stale vision). The reference keeps the original stamp. |
| F8 | `test_send_vision_declares_covariance` | `sitl-stand/send_vision.py:44-45` (no covariance → pymavlink default zeros) | **BUG** (low) | Zero variance claims a perfect fix; the MAVLink spec says to put NaN in element 0 when unknown. ArduPilot clamps the error to `VISO_POS_M_NSE`/`VISO_VEL_M_NSE`, so the flight impact is small (assumption: based on ArduPilot's documented noise-floor behaviour, not verified in SITL here). |
| F9 | `test_tiny_dt_frame_pair_not_amplified` | `vio/vio_frontend.py:41` (only `t <= prev_t` rejected) | **LATENT** | 0.5 px of jitter over 1 ms produces 10.8 m/s with reported σ 0.8 m/s. The Gazebo camera stamps lie on a 0.1 s grid, so this does not occur in phase 1. It can occur on the `--ros-image` / real-camera path of `vio_ekf.py` (no rate limit there). |
| F10 | `test_predict_refuses_backwards_dt` | `vio/vio_core.py:37-53` | **LATENT** | `predict()` integrates backwards silently. All callers guard: `vio_ekf.py:263`, `vio_imu_preintegration.py:86`. |
| F11 | `test_predict_nonfinite_imu_does_not_poison_state` | `vio/vio_core.py:37-53` | **LATENT** | One NaN makes the state NaN for good. Callers guard: `advance` checks finiteness; RAW_IMU fields are integers. |
| F12 | `test_first_velocity_outlier_not_taken_as_truth` | `vio/vio_core.py:125-128` (`vel_init`) | **BUG** in the `vio_ekf.py` path, **guarded** in phase 1 | In the sitl-stand path, the first flow velocity is taken as truth. Correct measurements are then NIS-rejected for 15 frames before the forced reset. In `run_vio_stream` the flow-consistency check against the IMU prior runs first and would reject it. |
| F13 | `test_forced_velocity_reset_has_a_recovery_path` | `integration/vio_quality_gate.py:20` (`ekf_ready` requires `n_resets == 0`); no epoch trigger in `run_vio_stream.py` | **LATENT / DESIGN** | After a forced reset the gate is permanently "EKF not ready" until the process restarts. Recorded flights all have `ekf_internal_resets` = 0. With the consistency check in place the EKF gate is looser than the pre-check, so this is effectively unreachable. Note that it *was* reachable in the abe2 source (53eeddf ran without the consistency check). |
| F14 | `test_replay_velocity_matches_ground_truth` | `dataset/generate_sample_dataset.py:57-58` | **DATA** | Measured speed is exactly 2.00 m/s, but the direction is reversed (−2.0 N). The generator moves texture content up (−y) while flying forward, which contradicts `T_cam_imu`, the frontend mount (`vio_frontend.py:18-20`) and `gazebo/verify_sync_axes.py:37-38` (pixels move down). The generator's own comment says "down". `gazebo/regression_test.py` checks only tracking ratio and inliers, so it never caught this. Production VIO is not at fault. |

## Ranking: likelihood of causing the TAKEOFF → HOVER → LAND blocker

1. **F1 + F2 + F3 — fail-closed gate vs. the 500 ms host watchdog.** Observed in all three recorded runs.
   - The re-readiness rule alone uses the entire watchdog budget, and RTF < 1 shrinks that budget to 0.33 s of simulation time.
   - Any frame rejection during CLIMB or LAND, where scale change, the drone's shadow and low texture are worst, ends the flight.
   - C2 helps only the single-frame case at the measured RTF; the opt-in tests pass for one frame only.
2. **F4 / F5 — estimator re-init on any IMU or timestamp irregularity.** Not the trigger in the recorded runs. It would end any flight deterministically, and the outage (≥0.8 s) is longer than F1's.
3. **F7, then F6 and F8 — `send_vision.py` stale and fabricated data.** Relevant only if the blocker is reproduced with the sitl-stand harness, not the phase-1 path. F7 hides VIO loss from FS_EKF.
4. **F12 — first-flow outlier in `vio_ekf.py`.** sitl-stand camera path only.
5. **F9, F13, F10, F11 — latent;** currently guarded.
6. **F14 — data only;** fix the generator so the sample_flight replay can serve as a GT test.

## Proposed fixes (diff suggestions only — nothing applied)

These are sketches for review, not tested patches. F1–F5 relax a fail-closed policy, so the safety owner must approve them before they are applied, in line with the existing C2 note.

### Fix A (F1–F3) — publish IMU-propagated odometry while coasting, like the reference

Add a time-bounded "coast" state to the gate. During transient image failures, with a healthy estimator, the source publishes the IMU-propagated state. The covariance grows on its own (no flow is fused), and `covariance_ok(..., max_variance=1)` still bounds it. Every non-image failure still blocks immediately.

```diff
--- a/integration/vio_quality_gate.py
+++ b/integration/vio_quality_gate.py
@@ class QualityGate:
-    def __init__(self, tolerate_single_rejection=False):
+    def __init__(self, tolerate_single_rejection=False, coast_s=0.):
         self.ready = False
+        # coast_s > 0 (OPT-IN, safety approval): after readiness, transient IMAGE
+        # failures keep publishing the IMU-propagated state for at most coast_s
+        # simulation seconds since the last clean frame (OpenVINS publishes
+        # fast-propagated odometry on every IMU message). 0 = current behaviour.
+        self.coast_s = float(coast_s)
+        self.coasting = False
+        self.last_clean_t = None
@@ def assess(...):
         failures = [reason for ok, reason in checks if not ok]
         if failures:
+            if (self.coast_s > 0 and (self.ready or self.coasting) and estimator_healthy is True
+                    and all(r in TRANSIENT_IMAGE_REASONS or r == 'EKF not ready' for r in failures)
+                    and self.last_clean_t is not None and t-self.last_clean_t <= self.coast_s+1e-9):
+                self.ready, self.coasting = False, True
+                self.reason = 'coasting on IMU: ' + failures[0]
+                return False
+            self.coasting = False
             if (self.tolerate_single_rejection and self.ready and not self.suspended
@@
+        self.last_clean_t = t
+        if self.coasting:                       # first clean frame after coasting
+            self.coasting, self.ready = False, True
+            self.transitions.append(dict(t=t, wall=wall, ready=True, reason='resumed after IMU coast'))
         if self.suspended:
```

```diff
--- a/integration/run_vio_stream.py
+++ b/integration/run_vio_stream.py
@@ class CheckedPublisher:
-                    if (not gate.ready or abs(stamp-output_node.t) > 1e-8
+                    if (not (gate.ready or gate.coasting) or abs(stamp-output_node.t) > 1e-8
@@
-            if ready:
+            if ready or gate.coasting:     # coasting: IMU-propagated state, flow NOT fused this frame
@@ argparse
+    p.add_argument('--coast-s', type=float, default=0., help='Opt-in, SAFETY OWNER APPROVAL REQUIRED')
-    gate = QualityGate(tolerate_single_rejection=a.tolerate_single_rejection)
+    gate = QualityGate(tolerate_single_rejection=a.tolerate_single_rejection, coast_s=a.coast_s)
```

Choose `coast_s` so that coast + one frame stays inside the watchdog at the worst RTF. With `--coast-s 0.5`, abe2's 41.8–42.2 s outage would have published IMU-propagated odometry continuously. The ported tests would then need a "published while coasting" notion in `run_gate()`.

### Fix B (F1, alternative or complementary) — evaluate the availability watchdog in the source's time base

The source budgets are in simulation seconds, but `phase1_transport.py:521` uses host monotonic time. One option is to add a simulation-time watchdog and keep a looser host-time backstop. This must be verified first: `clock[0]` is logged as `ros_clock_ns`, so the units need confirming.

```diff
--- a/integration/phase1_transport.py
+++ b/integration/phase1_transport.py
@@
-                elif last_odom[0] is not None and time.monotonic_ns()-last_odom[0]>500_000_000:
+                elif last_odom[0] is not None and (
+                        (clock[0] is not None and last_odom_acquisition[0] is not None
+                         and clock[0]-last_odom_acquisition[0] > 500_000_000)          # sim-time availability
+                        or time.monotonic_ns()-last_odom[0] > HOST_BACKSTOP_NS):        # e.g. 1_000_000_000
```

This loosens a host-time safety limit, so it needs safety approval. Fix A is closer to the reference.

### Fix C (F5 → reduces F4) — drop exact duplicate IMU deliveries at ingestion

```diff
--- a/integration/vio_sensor_capture.py
+++ b/integration/vio_sensor_capture.py
@@ def imu_cb(msg):
         row['callback_build_s']=time.monotonic()-start
+        last=buffers['imu'].latest() if len(buffers['imu'].snapshot()) else None   # or keep a local 'previous'
+        if last is not None and row['t']==last['t'] and row['f']==last['f'] and row['gyro']==last['gyro']:
+            imu_duplicates[0]+=1      # OpenVINS select_imu_readings removes zero-dt readings
+            return
         buffers['imu'].append(row)
```

Use a cheap local `previous` variable instead of `snapshot()`, and apply the same change in `isolated_imu.py`. A duplicate stamp with *different* values stays fail-closed.

### Fix D (F4) — do not re-initialise on a single recoverable IMU glitch

Optional, and needs safety approval. For `ImuGapError` caused by a gap of 20–50 ms, bridge it with one `predict()` step over the gap, inflate P instead of starting a new epoch, and keep `reset=True` only for backwards time, non-finite data or long gaps. This mirrors the reference's interpolation in `select_imu_readings`. (No diff: this changes `advance()` semantics, and the existing `integration/tests/test_vio_*` pin the current behaviour.)

### Fix E (F6–F8) — `send_vision.py`

Verify with `REF_PORT_SITL=1 pytest -m sitl` before and after the change, because of the bootstrap caveat in F6.

```diff
--- a/sitl-stand/send_vision.py
+++ b/sitl-stand/send_vision.py
@@
 x = y = z = vx = vy = vz = 0.0
 roll = pitch = yaw = 0.0
 last = 0.0
+t_pos = None                     # host time of the last LOCAL_POSITION_NED
+STALE_S = 0.5
+POS_COV = [0.04 if i in (0, 6, 11) else (0.001 if i in (15, 18, 20) else 0.0) for i in range(21)]
+VEL_COV = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
@@
         if msg.get_type() == "LOCAL_POSITION_NED":
             x, y, z, vx, vy, vz = msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
+            t_pos = time.time()
@@
-    if now - last >= 1.0 / RATE_HZ:
+    if now - last >= 1.0 / RATE_HZ and t_pos is not None and now - t_pos <= STALE_S:
         usec = int((now - t0) * 1e6)
-        master.mav.vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw)
-        master.mav.vision_speed_estimate_send(usec, vx, vy, vz)
+        master.mav.vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw, POS_COV)
+        master.mav.vision_speed_estimate_send(usec, vx, vy, vz, VEL_COV)
```

If SITL shows that EKF3 never publishes LOCAL_POSITION_NED without a first vision fix, keep an explicit bootstrap: send the origin with a large covariance (e.g. 100 m²) until the first LOCAL_POSITION_NED arrives, instead of a silent zero fix.

### Fix F (F12) — do not take the first flow velocity blindly (`vio_ekf.py` path)

```diff
--- a/vio/vio_core.py
+++ b/vio/vio_core.py
@@ def update_velocity_xy(self, v_xy, sigma_scale=1.0):
-        if not self.vel_init:
+        if not self.vel_init:
+            if not np.all(np.isfinite(v_xy)) or np.linalg.norm(v_xy) > self.max_init_speed:   # e.g. 15 m/s
+                return False, float('inf')
+            if self._init_candidate is None or np.linalg.norm(v_xy-self._init_candidate) > 3*sigma:
+                self._init_candidate = v_xy          # require two consistent measurements
+                return False, 0.0
             self._reset_velocity(v_xy, sigma, idx)
```

`vio/` is pinned by hash in `run_vio_stream.py:167-170` and `vio_offline/vio_source.py`, so changing it requires re-pinning (or applying the guard in `vio_ekf.py` instead).

### Fix G (F9–F11) — defensive guards (latent)

```diff
--- a/vio/vio_core.py
+++ b/vio/vio_core.py
     def predict(self, f_body, R, dt):
+        if not (dt > 0 and np.isfinite(dt) and np.all(np.isfinite(f_body)) and np.all(np.isfinite(R))):
+            raise ValueError('VioEKF.predict: non-positive dt or non-finite input')
--- a/vio/vio_frontend.py
+++ b/vio/vio_frontend.py
-        if prev is None or t <= prev_t or height <= 0.0:
+        if prev is None or t - prev_t < self.min_dt or height <= 0.0:      # min_dt e.g. 0.5/fps
             return None
```

These carry the same pinning note. The frontend guard keeps the current semantics of replacing `prev` on rejection.

### Fix H (F13) — open a new epoch after a forced EKF velocity reset

```diff
--- a/integration/run_vio_stream.py
+++ b/integration/run_vio_stream.py
@@
                     accepted, nis = ekf.update_velocity_xy(velocity, result.sigma_scale)
                     row.update(nis=float(nis))
+                    if ekf.n_resets and state['reset_after'] is None:
+                        block('EKF forced velocity reset', t, reset=True)   # explicit new epoch, as for IMU gaps
```

### Fix I (F14) — test data: make sample_flight move ground content the right way

```diff
--- a/dataset/generate_sample_dataset.py
+++ b/dataset/generate_sample_dataset.py
-        y0 = 200 + (shift_y % (tex_h - h - 400))
+        y0 = 200 + (tex_h - h - 400) - (shift_y % (tex_h - h - 400))   # forward flight: content moves +y (down)
```

Then regenerate `recordings/sample_flight`, which changes tracked data. Separately, add a direction assertion to `gazebo/regression_test.py`.

## How to run

```bash
pip install -r requirements.txt          # pytest, pymavlink (numpy, opencv already required)
python3 -m pytest tests/ref_port -v -rA  # 70 tests, ~15 s, no ROS / SITL needed
REF_PORT_SITL=1 python3 -m pytest tests/ref_port -m sitl   # with SITL (sitl-stand/run.sh extnav) running
```

The `abe2` replay tests read `integration/phase1_runs/20261118-flightabe2d67e5306/evidence` and skip if it is absent.
