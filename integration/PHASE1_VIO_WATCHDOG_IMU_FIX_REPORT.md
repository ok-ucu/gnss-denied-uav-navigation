# Phase 1 VIO availability and IMU timestamps

**Decision: NOT READY. Phase 1 remains HOLD.** The IMU fix is verified offline.
The recorded visual rejection pattern still exceeds the existing watchdog.

## A. Baseline

The isolated worktree `/Users/ok/Projects/gnss-phase1-vio-imu` is on
`phase1-vio-imu-robustness`, based on
`348cdd8a6350daf68e1c22203b2735b8d1276727`. A full fetch disconnected and a
subsequent read-only remote ref query verified this SHA as the current tip of
`phase1-integration-merge`. The original local branch at `b97e641` has
untracked Stage 4 and patch files. It was preserved. The separate no-shadows
branch at `ad8e949` is outside the target history.

Inspected the [abe2 report](reports/VIO_FLIGHT_ABE2_20261118.md),
[fbf7 report](reports/VIO_FLIGHT_FBF7_20261118.md), and
[backlog report](reports/VIO_BACKLOG_20261117.md). The tracked run directories
for `abe2d67e5306` and `fbf7d8854ed1` include source and transport logs.
The tracked subset for `fea1c93152a2` includes source skips and transport phase
and timing logs, with no source publication stream in that subset. Raw replay
of the full sensor-to-FCU chain was unavailable on macOS.

The branch gate requires at least three good frames spanning 250 ms. At 10 Hz,
that means four good frames after rejection. A single rejected frame produced a
0.500 s simulation publication gap in the gate test. At fixed RTF 0.656 this
corresponds to 0.762195 host seconds. The five consecutive unavailable/rejected
frames at 41.8–42.2 s in abe2 were followed by accepted corrections at 42.3,
42.4 and 42.5 s, with output resuming at 42.6 s. The recorded transport watchdog
fired in CLIMB at 500.637484 ms host age of the 41.7 s acquisition. The later DDS
discovery skips occurred after transport exited. The current source publication
age budget is 157 ms simulation time. Pairing has a 200 ms freshness envelope.
The 500 ms host-time availability watchdog and all of those budgets are intact.

The unmodified preintegrator returned a `bad` sample at absolute index 8 for
an exact repeat of the .008 s sample in both step modes. `frame_step()` turns
that into `ImuGapError`; the source recovery path schedules a new epoch.
The replay confirms the trigger. The historical live restart duration and
attribution of a specific recorded restart to a harmless duplicate remain
unverified.

| Source | Target baseline SHA-256 | Updated SHA-256 |
|---|---|---|
| run_vio_stream.py | `9a6205092fcbec1e925a2165095adee934755d26b5a449df6675c5347a0308ce` | `e9058f815d06e8345c96f7e8456dc31c074aaa0bad58016a90e6f5a521b76b15` |
| vio_imu_preintegration.py | `afd5342b7fa8d3a3332ba9fd8b439c9f86cb8a15347d35ccb770e6bb0bd3eed0` | `0741450cca7c6cfc0776a9f718cfa93c69bf4f183a6fcade1b41f4e61b2aee10` |
| vio_quality_gate.py | `a2aed2d90225f1c57e7230bf4d753eb3e59bab71cffcbdf9d41a344f2070f068c` | `c850c65c9b598fd64979e462201843e4cedc4a43e99d43ec7561a019b2e96cb8` |
| phase1_transport.py | `f1ede764b271dfd05d87b035220679b6b953560c7b1cb0b5e3984cf547ce5877` | unchanged |
| navigation_lifecycle.py | `ce429cce55329e298944f36722e43a60e0d2785811f55bf30512670ca6a9d0af` | unchanged |
| vio_latency_contract.py | `fb23eeeb42207b4896259737592bf2c6ab543a3aff35ba3dafb356171b0f5365` | unchanged |

The abe2 and fbf7 configurations record source SHA-256
`9e9e1e11bac02741bc05e2fbbc5764cbd36d206fd20d2b0544089d54867d4955` and gate SHA-256
`cc6c6bf24a879e7fd1ddc0a7aa3f834975700d8524f22ac0f4c42834048c0b4d`.
They predate the target branch's flow consistency wiring.

## B. Code changes

| File | Change and safety effect |
|---|---|
| `vio_imu_preintegration.py` | Advance the raw cursor for same-stamp samples only when frame and the finite force, gyro and orientation payload match. Leave held sample, filter time, gap statistics and EKF updates alone for duplicates. Conflicts, backward time, nonfinite data and gaps remain errors. Raw tick limits still apply. |
| `run_vio_stream.py` | Validate the seed payload and start at the first record on its stamp so the common preintegrator sees repeat or conflict. Record the preintegrator hash in runtime configuration. Check the existing inlier and contrast limits before visual correction, in addition to the existing audit and flow consistency checks. Publication remains subject to the full readiness contract. |
| `vio_quality_gate.py` | Share the existing inlier and contrast constants with the source. Readiness and thresholds remain intact. |
| `tests/test_vio_flight_motion.py` | Exercise exact duplicates and corrupt samples in both step modes using the pinned EKF. Check raw cursor movement, pending accounting, timestamps, prediction count, state and covariance. Exercise idle and camera-frame boundaries. |
| `tests/test_vio_flight_abe2.py` | Replay per-frame gate decisions for the recorded burst and verify the resulting gap against the watchdog. Use recorded frame timing and outcomes. The replay supplies nominal 1 ms IMU spacing because the stream rows do not contain the per-frame maximum gap. |
| `PHASE1_VIO_WATCHDOG_IMU_FIX_REPORT.md` | Record local results, limits and remaining validation. |

The pinned EKF already propagates position, velocity, accelerometer bias and
their covariance using `F P Fᵀ + Q`. That does not establish an admissible
prediction-only Odometry stream. Attitude is supplied separately, with a fixed
orientation covariance argument at the current publisher call. The wire
contract currently treats the epoch token's ready state as one form of
availability. A defensible prediction-only horizon and covariance/error bound
for the affected climb maneuver were unavailable. The recorded gate logs do
not establish those bounds. No prediction-only Odometry path was added.

## C. Tests

Run from the new worktree using the existing unittest convention and a
NumPy/OpenCV environment with the pinned VIO export.

```bash
p=/Users/ok/Projects/gnss-denied-gazebo-integration/integration/source_scope_audit/commit-20261114/test-env/bin/python
export VIO_SOURCE_DIR=/Users/ok/Projects/gnss-phase1-no-shadows/integration/linux_gpu_runs/offline-no-shadows
export PYTHONPATH="$PWD/integration:$PWD/integration/tests:$VIO_SOURCE_DIR"
"$p" -B -m unittest -v test_vio_flight_motion.BlockPropagation \
  test_vio_flight_motion.ReadinessAndWatchdog test_vio_flight_abe2.RecordedLiftoffFailure \
  test_vio_quality_gate
"$p" -B -m unittest -v test_vio_flight_motion test_vio_flight_abe2 \
  test_vio_ground_fixture test_vio_quality_gate test_vio_sensor_capture \
  test_vio_backlog test_vio_source_probe test_vio_latency_contract \
  test_phase1_pipeline test_navigation_preflight test_fcu_receipt_guard \
  test_reset_safety test_simulated_flight_report test_simulation_flight \
  test_simulation_vision test_runtime_candidate
git diff --check
```

| Test | Expected behavior | Observed behavior | PASS/FAIL | Evidence |
|---|---|---|---|---|
| IMU repeat, frame and idle paths | No reset-triggering exception or duplicate integration | Both modes consumed all raw rows. State, covariance and prediction counts matched clean controls in the tested fixed schedules. | PASS | `BlockPropagation` |
| IMU conflict, backward, gap, NaN/Inf | Reject before advancing past corruption | `ImuGapError` and unchanged cursor for each injected fault in both modes. | PASS | `BlockPropagation` |
| One rejection and recorded five-frame burst | Fail closed and expose the availability limit | Replay retained the recorded excessive gap. | PASS for regression, availability blocker persists | `ReadinessAndWatchdog`, `RecordedLiftoffFailure` |
| Sensor freshness, covariance, clock, watchdog, authorization and candidate regressions | Fail closed under invalid evidence or faults | Relevant regression suite passed with one optional plot check skipped. The sensor/clock tests are offline fixtures. | PASS | `/tmp/phase1-vio-imu-final.log` |

Focused suite reported 28 passed, zero failures and zero skips. Relevant suite
reported 219 passed, zero failures and one Matplotlib rendering skip. The
suites overlap. Changed Python files parsed successfully and Git whitespace
validation passed. Full Linux GPU/SITL validation was not run. There is no
configured GPU/SITL execution target or fresh native candidate in this worktree.

## D. Before and after measurements

| Metric | Historical or baseline | Updated result |
|---|---|---|
| Max publication gap, abe2 | 0.900000 s acquisition, 0.900468 s host publication | Gate replay of the same burst: 0.900000 s acquisition and 0.900050 s between recorded frame-entry times, identical for both gate revisions. Updated live publication gap: NOT MEASURED. |
| Watchdog failures | One in abe2 at 500.637484 ms of host age. One in fbf7 at 500.027738 ms. | The replay still crosses the watchdog budget. Updated live failures: NOT MEASURED. |
| Unnecessary reset trigger on an exact repeat | One bad sample in each baseline synthetic mode, which would raise at the frame boundary | Zero `ImuGapError` for identical repeats and zero reset requests in both tested modes. Updated live restart count: NOT MEASURED. |
| Visual observations withheld in abe2 burst | Five consecutive unavailable/rejected frames | Five on fixed-input replay. Updated live rejection count: NOT MEASURED. |
| Recovery after first good frame | 42.3 → 42.6 s, 0.300 s simulation and 0.298810 s between recorded frame entries | Same gate replay. Updated live recovery: NOT MEASURED. |
| Acquisition age at gap endpoints | 87 ms and 80 ms simulation at the 41.7 and 42.6 s publications | Updated acquisition age and runtime latency: NOT MEASURED. |
| Prediction-only covariance and output | No prediction-only publication | No prediction-only publication. Live covariance progression during visual rejection: NOT MEASURED. |
| Healthy state and covariance | Clean synthetic IMU control | Exact equality with duplicate input in tested schedules. Live accuracy and processing latency: NOT MEASURED. |

## E. Safety assessment

The existing audit, consistency and EKF acceptance checks still run. The
existing feature minima now guard the correction as well as publication.
Frame/IMU freshness, covariance and readiness checks still block invalid
output. Actual camera or IMU loss gets no substitute publication. A hardware
or live simulator sensor-loss test remains pending. Source epochs, watchdogs,
clock age, transport TTL, EKF3 settings, physics, flight plan, default
capture-only routing and permissions are unchanged. Historical evidence and
pinned external VIO are unchanged.

The modified integration VIO sources are recorded in new source configuration
logs. The existing permit hashes a separate transport/launcher source list and
the proof verifier checks the pinned external VIO hashes. These particular
integration-source edits may therefore leave old proof verification mechanically
possible. An old proof cannot establish the behavior of the patched source.
Collect a fresh DISARMED proof with the final reviewed source and exact physics
image, especially if the no-shadows integration is included.

## F. Final status

- **Duplicate IMU handling — FIXED BUT NOT FULLY VERIFIED.** The defect is
  reproduced and corrected offline in both propagation modes. The frame and
  idle tests prove cursor advancement and continuity. Native DISARMED proof of
  unchanged timing and epoch behavior is pending.
- **Visual rejection / watchdog — STILL BLOCKED.** Feature rejection now
  precedes correction for the existing inlier and contrast limits. The replay
  still exceeds 500 ms host time. A valid bounded prediction-only output would
  require reviewed state/frame uncertainty, a horizon justified by recorded
  sensor data and acceptance by the transport/FCU safety contract.

Minimum next validation is a reviewed DISARMED Linux run on the final
image/source stack. Record camera and IMU availability, acquisition times,
source/transport timestamps, actual publication gaps, covariance, epochs and
raw DataFlash. Verify the no-shadows scene if it is part of that stack. Keep
HOLD until the evidence demonstrates continuity and the separate approval is
in place.
