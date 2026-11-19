# Phase 1 Linux shadow-fix validation

**Status: PARTIAL**

## Source and scope

Linux HP ZBook Studio G7, NVIDIA Quadro T2000 Max-Q, driver 595.99.02, native Linux amd64. Worktree `/tmp/gnss-phase1-flight-test`, branch `phase1-integration-merge`, HEAD `d13e1a88bab001fa855cfb78d1471d43d0469d9d` (two commits ahead of tracking branch). Initially clean, with no staged or unstaged diff. This report is the only repository change. No commit or push.

The runtime used the existing guarded integrated launcher with NVIDIA, isolated IMU, DISARMED vision ingress and internal subnet `10.249.242.48/28`. One execution, run `20261119-shadowvalidation123650`. Launcher exit 0. The unchanged constraints include 500 ms host watchdog, 250 ms readiness, 157 ms source publication age, 25 ms fallback budget, existing quality and motion-audit thresholds, and `ARMING_CHECK=1`.

## Effective scene and Compose

| Artifact | SHA-256 |
|---|---|
| Integrated resolved Compose | `a3033c8ec899a198dcbdd731ebc1e6821bc0b4ccd8cdb16bdc6dbb432165a5d1` |
| Camera/source Compose path, NVIDIA camera-only resolved configuration | `be22f9a4a915c8fb3a68e831e929c48e15b80d4b131472b5e62d4ceeaf3f17c6` |
| Source-only wrapper resolved Compose | `03267a28744c4f3588c664a448df4421e6eafe6196ed474c8e4fb5f9f12be109` |
| Generated world, both paths, also pinned image default `iris_runway.sdf` | `bf3dce911f0d02f5bd4a6f04710b62473c6784892917f1b6fb20257b14274df5` |
| Generated `iris_with_gimbal/model.sdf`, both paths | `3f23a8e48b77cb5d4d82f1308ceb2a7e007b56935847128c5d0bc6d536602ab0` |
| Image `gimbal_small_3d/model.sdf`, actual nested 10 Hz camera | `48163f06c17c2855b24fb67b7cfd2ab7433ea2e9b39e019b97893271d12c5199` |
| Standalone sample `gazebo/worlds/camera_world.sdf` | `d7c730b4fffe5cc6a421c6f8e05626b7b2449097bea9f61ece2320b38022d41e` |

Both Phase 1 Compose paths bind their own run root at `/run/integration`, resolve the run's models first, verify the installed directional sun has exactly one `cast_shadows=false`, then execute Gazebo with `/run/integration/models/world.sdf`. The legacy standalone sample still specifies `true` and is outside both Phase 1 mount/launch paths. The alternate camera-only path was prepared under `integration/linux_gpu_runs/20261119-shadowconfig123650`. The source-only wrapper calls the same camera preparer and uses its Compose file. Its separate NVIDIA EGL override and resolved configuration were inspected under `integration/linux_gpu_runs/20261119-shadowsource123650`. Both paths resolve to corrected scene bytes. Live loading was observed on the integrated path; the alternate path was inspected without starting it.

Integrated Docker inspect, process arguments, in-container SHA-256 and Gazebo startup logs agree on the world path and image `sha256:1653eb3241db282628fab9a528328aede57d556c0f16bdaa3e0d7ef3c49faed9`. Gazebo initialized `iris_runway` and attached `iris_with_gimbal::gimbal::pitch_link::camera`. Renderer identity and Ogre identify NVIDIA. The pinned FCU and transport images were `sha256:8195bb92e1c8b997f046799ed862d154adb56e5733b2a2cdf859fe968dd987d0` and `sha256:eeba41819880d63018915092d383c5fc6d0a327ec7d230ebe349669975685aaf`, respectively. The world declares 1 ms physics and includes the run-local gimbal model, which includes the pinned image's nested camera model. The sensor is 640 × 480 at 10 Hz simulation, horizontal FOV 2.0 rad, with a nominal SDF pose rotation of −1.57/−1.57/0 rad.

## Fresh DISARMED measurements

- Camera: 391 native and 391 ROS callbacks, strict order, 10 Hz simulation and about 6.00 Hz host. ROS arrival p50/p95/max gap 164.530/186.666/251.356 ms host. No captured frame supersession or overflow.
- IMU: 39,107 native rows and 38,150 accepted isolated rows (different capture windows), 1 ms simulation cadence and about 599 Hz host. Isolated receipt p50/p95/max gaps 1.582/2.133/7.757 ms host. Clocks: 38,147 rows, 1 ms simulation cadence. Zero duplicate/backward timestamps in the captured IMU and clock streams. No IPC fault or overflow, worker stopped with zero queued unconsumed batches. Latest IMU age during the publication interval max 1 ms simulation.
- Tracking: all 380 frames that produced an inlier count had 154 inliers, including all 339 publications. The one frontend rejection at 24.6 s was during startup, before a comparable frame was available. Nine sensor/clock warmup skips and three readiness waits preceded publication. Independent audit recorded 380 agreements and one startup frame without a comparison. No five-frame frontend rejection sequence. This was stationary DISARMED evidence. Historical CLIMB replay counts 34–117 versus 165–192 are a separate dataset and no CLIMB measurements were taken.
- Quality Gate: warmup at 23.7 s, startup inliers rejection at 24.6 s, ready at 25.0 s. It stayed ready through publication at 58.8 s. Starting at 58.9 s, 38 later frames waited for downstream DDS discovery during the post-ingress observation tail. No active quality fault, recovery, missing publication, or watchdog expiry. The transport closed for `owner shutdown`.
- Estimator: no internal reset. Across all odometry, x/y position variance 0.01030–0.03450 m², z 0.03603–0.05141 m², x/y velocity variance 0.004332–0.004403 (m/s)² and z 0.05201–0.13446 (m/s)². Minimum symmetric covariance eigenvalues were positive for all captured pose and twist matrices. Angular-velocity variance 100 (rad/s)² is the configured uncalibrated compatibility value. Gazebo GT Z was used by the companion estimator; no horizontal GT input. DataFlash retained the configured FCU vertical source.

| Stream | Count | Host p50 / p95 / p99 / max, ms | Sim p50 / p95 / p99 / max, ms |
|---|---:|---:|---:|
| ROS Odometry publications | 339 | 164.862 / 194.716 / 214.218 / 313.493 | 100 / 100 / 100 / 100 |
| Completed MAVLink pose/velocity pairs | 338 | 164.824 / 196.670 / 213.590 / 316.255 | 100 / 100 / 100 / 100 |

Odometry host time is publication monotonic time. Pair host time is the end of the second socket write, and simulation time is the corresponding original acquisition stamp from the mapping evidence. Pair FCU-time max gap was also 100 ms. The initial odometry at 25.0 s was the origin diagnostic reference; the 338 pairs cover 25.1–58.8 s. All 339 publications reached the DDS transport callback. DDS publish-to-callback p50/p95/max was 1.251/1.886/2.527 ms. The largest transport odometry receipt gap was 314.344 ms host. There were 676 FCU vision writes, zero partial/duplicate groups, zero control command writes and zero armed samples across 38 heartbeats.

Source publication age max 116 ms sim versus 157 ms, yielding 41 ms headroom. Pair clock age at the final wire check max 118 ms sim. Minimum completed-pair deadline headroom was 49.857 ms host. Maximum mapping uncertainty 18.001 ms, below its 20 ms bound. Clock mapping verified with 302 checks, epochs remained at 0, source and mapped timestamp order remained strictly increasing. Active gap margins against the 500 ms host watchdog were 186.507 ms for odometry and 183.745 ms for pairs. Source processing p50/p95/p99/max 34.464/43.918/50.160/189.570 ms host.

## DataFlash fusion and isolation

Fresh `evidence/flight_fusion_proof.json` was generated offline from this run's closed `00000001.BIN` and successfully re-verified by independent re-decode. All 12 recorded VIO source hashes match the mounted integration files or candidate VIO export. No protected production file or physics hash was altered during validation.

- Proof kind `LIVE_CAMERA_DISARMED_FUSION`, non-synthetic, SHA-256 `1c5e992e55632e1467021507f250c67a02b0fd355efe120d06c036558e389b6a`.
- DataFlash SHA-256 `c0eb54e7bf11fd3f110a912126dc37ab65b6d26c4ac3d4b78b5a3ad7343966d0`, firmware SHA-256 `d8c8aa1126156c7c124fddb0dc0908704e0b9779a5b9fc5e40d24fa39b5f0d36`, decoder source SHA-256 `baa882772441b09aa034cff659e7046b14828e8f440e04632eca7287eee90d54`, runtime candidate SHA-256 `b61130f975c0e6cdf67f2fd56d3e7d6fd025a7094bb379b247e907745900fc3f`. Full evidence manifest is embedded in the proof and the source manifest is in `evidence/vio/configuration.json`.
- 338 VISP and 338 VISV match the exact wire timestamps. Both EKF3 lanes reported using ExternalNav from FCU 25.157028/25.159028 s. EKF3 source-set/status validation used XKFS and XKF4, with no relevant faults, timeout or innovation rejection during fusion. The wire/FCU reception window ends at 58.821626 s FCU. `post_stream_dropout=true` refers to stopped-aiding messages at 73.854311 s FCU, 15.033 s after the last reception, outside the active fusion window.
- All four owned container IDs verified exited. `existing_services_unchanged=true` and the two pre-existing `vazon` container IDs and running states matched before/after. Execution and global forwarding remained false.

## IMU timestamp regression

`IsolatedImuReceiver` rejects a backwards acquisition stamp or backwards receipt time. An equal IMU stamp with identical payload is discarded while the receipt-order watermark advances and the retained measurement keeps its original timestamp and callback time. A conflicting same-stamp payload, nonfinite sensor value, IPC sequence/session mismatch, and genuine reset still latch a fault. The focused `test_isolated_imu.py` suite reproduced identical retransmission across packets, conflicting duplicates and backwards stamps. It also exercised preserved repeated clock stamps, which follow a different policy from IMU samples. The fresh runtime found no unnecessary reset, and no receiver patch was justified.

## Regression results

Targeted offline tests: **69 passed, 1 skipped**, plus 86 passing subtests. The skip was the optional pristine upstream SDF fixture. Initial in-sandbox full runs hit socket permission errors in synthetic heartbeat/UDP tests. The complete host offline suite finished with **589 passed, 1 failed, 7 skipped** and 302 passing subtests in 507.13 s. The targeted and full logs are `targeted-tests-final.txt` and `full-tests-host.txt` in the supplemental evidence directory. Standalone `vio/test_vio_core.py::test_quality_drops_when_stale_and_recovers` reproduces an existing `TypeError`: the test calls the instance method `QualityMonitor.inflate` on the class. The test and method were last modified together at `c5d31e25c884d29f82b6269135e1ff7053a4997d`, before this validation. No changes were made to that test.

## Remaining limits and evidence

The overall validation status is PARTIAL because the full regression suite has one reproducible failure. The integrated DISARMED run and its fresh fusion proof passed.

Stable corrected-scene DISARMED tracking and sustained EKF3 fusion are shown for one active ingress window. Flight-specific tracking and transient recovery remain unvalidated. The historical 10 Hz timing blocker still applies: at the observed 0.5993× simulation/host, a 0.5 s simulation recovery path would take about 0.83 s host against the 0.50 s watchdog. There was also a 189.570 ms processing outlier at 58.5 s sim. Its source age was 116 ms sim, within the freshness limit. Flight readiness remains HOLD.

- Integrated runtime, proof, effective Compose and BIN: `/tmp/gnss-phase1-flight-test/integration/phase1_runs/20261119-shadowvalidation123650`.
- Source-only wrapper configuration: `/tmp/gnss-phase1-flight-test/integration/linux_gpu_runs/20261119-shadowsource123650/artifacts/source-resolved-compose.yaml`.
- Prepared alternate Compose: `/tmp/gnss-phase1-flight-test/integration/linux_gpu_runs/20261119-shadowconfig123650/artifacts/resolved-compose.yaml`.
- Container inspection, loaded scene, supplemental metrics script, output and test logs: `/home/solomiia/visual/shadow-runtime.n4e8TV`.
