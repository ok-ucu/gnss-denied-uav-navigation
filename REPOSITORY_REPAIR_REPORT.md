# Repository repair and Phase 1 readiness

Reviewed 2026-11-18. Initial checkout was clean on `main`, commit
`3e40aa75a1ba067b9365671b1aed4919f32a8714`, matching local `origin/main`.
Branch comparison uses the fetched remote-tracking refs available at inspection.
The supplied audit was read in full and used as evidence. Its embedded proposals
and prior actions were evaluated against the user's repair request.

## Baseline and verification

- The targeted pre-edit baseline reproduced seven failing assertions
  (A2 and B1), with 35 passing tests and one dependency skip.
- The existing temporary Python 3.14 environment had pytest, NumPy and OpenCV.
  The first Phase 2 Person 1 collection failed for missing pymavlink and pyproj.
  Installing the test dependencies into a separate temporary directory restored
  collection. Validation used NumPy 2.5.5, OpenCV 5.1.0, matplotlib 3.11.2
  and pyproj 3.8.0. Person 1 Phase 2 passed 7 tests and VIO offline passed 22 tests.
- Pyflakes ran across tracked Python sources. A1's shell-wrapper names are gone.
  All 210 Python sources parsed. Source scope, shell syntax and diff checks passed.
  Remaining warnings include unused imports and locals, intentional optional
  imports and ROS subscription handles. Historical evidence copies, pinned sources
  and unrelated runtime code were preserved.
- Person 2 finished all seven stages with local fixture UDP enabled. In the
  initial sandbox run, the UDP fixture was blocked with `EPERM`. All 15 pytest
  cases passed in the loopback run. The final pymavlink precheck and dataset
  mock cleanup were verified separately with the dependency import and two
  non-network dataset cases. The mock
  rate check remains explicitly labeled. The production frontend tracked 149
  of 150 sample frames with mean 197.6 inliers. The checked-in sample timestamps
  gave a 3.333334 ms maximum nearest-IMU distance, against a 5 ms default.
- Integration discovery ran 614 cases with local fixture networking and plotting
  enabled. It recorded 611 passes, two native-environment skips and one
  launcher-induced failure. The filtered stdin launcher provided a
  `<stdin>` path to Python multiprocessing spawn. The isolated IMU module
  then passed all eight cases under normal module-based discovery, including
  the failing case. Across these runs, 612 distinct integration cases passed
  and two were skipped. No known offline test failure remains in the exercised
  scope. Reproduce the spawn rerun from `integration/` with
  `python -B -m unittest discover -s tests -p test_isolated_imu.py`.
- The integration run excludes one test that invokes the simulated-flight
  entry point even though the test stubs its subprocess and expects early
  refusal. No runtime image was rebuilt or deployed. No live simulator, FCU,
  hardware, calibration, or aircraft commands were run.

## Audit checklist

| ID | Status | Finding, action, and verification | Approval |
| --- | --- | --- | --- |
| A1 | FIXED BUT NOT RUNTIME-VALIDATED | Removed shell heredoc wrapper from `gazebo/record_gt.py`. Timestamp arithmetic now stays in integer nanoseconds. Static checks and a stubbed callback test verify exact epoch-scale timestamps. Actual recorder operation requires ROS. | Runtime validation only |
| A2 | FIXED AND TESTED | `vio/test_vio_core.py` uses the instance API with a configured inflation maximum. The pinned filter is untouched. | No |
| A3 | FIXED AND TESTED | `integration/tests/test_heartbeat_test.py` resolves the test parent import and passes both directories to the child via a subprocess-only environment. Eight tests passed from each working directory. | No |
| A4 | FIXED AND TESTED | `gazebo/verify_sync_axes.py` reads EuRoC camera and IMU CSVs, requires monotonic integer nanoseconds and full camera interval coverage, reports measured nearest-IMU gap and fails on bad data. Tests exercise exact epoch-scale arithmetic, offset, missing and duplicate evidence. A shared timestamp domain must be established by capture provenance. Clock bias can alias with the IMU sampling period. | No |
| A5 | FIXED AND TESTED | `run_all_tests.sh` uses one configurable interpreter, checks dependencies, isolates generated PNG output and propagates failures. The live-rate label is replaced by MOCK. | No |
| A6 | INVESTIGATED — NO DEFECT | `vio/test_vio_frontend.py` is an experimental camera-gap variant. The file remains in place with an explanatory docstring. Production regression coverage uses the actual frontend under B2. | Relocation or pinned algorithm change |
| A7 | FIXED AND TESTED | Root `requirements.txt` declares matplotlib for plots and pyproj for GPS conversion, with major-version bounds. Plot and GPS tests use an isolated Python environment. | No |
| A8 | INVESTIGATED — NO DEFECT | `integration/test_ekf_covariance.py` is a documented experiment and a public import/CLI target used by `run_live_ekf.py`, `verify_sitl.py`, existing tests and the SITL guide. Kept the entry point. | Migration if requested |
| A9 | FIXED BUT NOT RUNTIME-VALIDATED | Removed the unused frame timer, invalid recorder wrapper, unused mock RNG draw in `dataset/recorder.py` and unused replay state/imports in `vio_offline/replay_vio.py`. Re-ran the mock dataset tests and offline VIO tests. Remaining lint findings were not mass-edited. In particular, subscription handles and evidence copies need lifecycle and provenance review before any removal. | Context-dependent |
| B1 | FIXED AND TESTED | CSV loader and timestamp APIs return exact integer nanoseconds. Direct comparison retains a seconds default and opts into ns for loaded samples. Gap limits and CSV offsets are seconds. Updated stale expected values and explicit gap allowances in `integration/tests/test_offline.py` and added invalid and ambiguous timestamp cases. Production analysis unchanged. | No |
| B2 | FIXED AND TESTED | `gazebo/regression_test.py` defaults to repository `FlowFrontend` and makes the simplified frontend opt-in. Missing production imports fail. Tests exercise frontend selection and the sample fixture. Velocity remains descriptive. | No |
| B3 | FIXED AND TESTED | `integration/synthetic_heartbeat_e2e.py` gives the synthetic foreign helper a separate bounded lifetime after activation. Primary failure and foreign preservation assertions are retained. Fixture creator teardown still reaps all helpers. Two focused repeats verified that the deadline case still fails, the foreign helper survives primary cleanup, and teardown reaps all helpers. No production cleanup rules changed. | No |
| B4 | REQUIRES HUMAN APPROVAL | `origin/phase1-integration-merge` at `348cdd8` carries the flight-line texture, evidence and tests missing from `main`. Review the flight-line commits with owners before integrating, then review the no-shadows commit as its own change. | Merge |
| B5 | FIXED BUT NOT RUNTIME-VALIDATED | `integration/run_vio_stream.py` now adds downstream DDS status and subscriber counts to withheld-frame diagnostics. A missing subscriber count cannot distinguish startup, planned exit, or a disconnect by itself. Transport latch/queue behavior and logging remain unchanged. | Runtime proof |
| B6 | INVESTIGATED — NO DEFECT | `origin/fix/gazebo-no-shadows` at `b55648d` is unsuitable for this flight line. Its only changed file is the separate ArduPilot Gazebo Dockerfile. Prefer review of the camera-image branch for Phase 1. | Branch changes |
| C1 | REQUIRES HUMAN APPROVAL | `origin/phase1-no-shadows` at `ad8e949` is one commit beyond the flight line. It patches the pinned plugin world in `integration/linux_gpu/Camera.Dockerfile` and guards the generated runtime world before Gazebo starts. Offline snapshot verification ran 8 abe2 tests and 23 handoff tests with one skip. New image identity and bound source hashes require a new candidate and fresh DISARMED proof. | Build, runtime validation and flight |
| C2 | REQUIRES HUMAN APPROVAL | Added an offline 10 Hz recovery regression to `integration/tests/test_vio_quality_gate.py`. A rejected frame followed by four valid frames spans 500 ms from the last publication at RTF 1. Clock/processing jitter or RTF below 1 consumes any margin. Faster acquisition can reduce the recovery interval only if image and IMU processing keep up. Readiness-state changes require review of fault persistence. | Any safety-contract change |
| C3 | REQUIRES HUMAN APPROVAL | Reviewed recorded-flight and bench evidence on the flight line. At the lowest tested transition the bench retained 166 inliers without the shadow and 33 with it. The bench supports the shadow mechanism, while propeller-shadow attribution remains inferential. Consider spatially distributing the corner budget and replaying against labeled shadows and moving ground before proposing a pinned algorithm change. | Pinned algorithm changes |
| D1 | BLOCKED BY ENVIRONMENT | Fresh GPU-rendered no-shadows verification requires the approved image, Linux NVIDIA environment and live ground evidence. | Yes |
| D2 | BLOCKED BY ENVIRONMENT | Mock rate coverage passed. Actual camera/IMU rates need the live Gazebo streams and configured scene. | Runtime |
| D3 | INVESTIGATED — NO DEFECT | The ns/s boundary is explicit in code and exercised by both analysis suites. Upstream changes to that contract remain an owner decision. | Contract change only |
| D4 | BLOCKED BY ENVIRONMENT | Owner-host repetition is still pending. Local test verification applies to this machine. | Owner-host access |
| D5 | BLOCKED BY ENVIRONMENT | Both paths import the same estimator and frontend classes. Replay supports a camera-aligned IMU transform, alternate gyro aggregation and vertical source modes. Numerical parity requires a common recording and matched initialization, sampling, calibration and filter settings. | Matched runtime recording |
| D6 | BLOCKED BY ENVIRONMENT | Host plot test setup does not establish the dependency ABI inside the runtime image. | Runtime image inspection |
| D7 | BLOCKED BY ENVIRONMENT | Pi, camera and physical FCU timing, extrinsics and throughput remain unverified. | Approved hardware session |
| D8 | BLOCKED BY ENVIRONMENT | Synthetic local tests do not establish native cgroup, pidfd or live heartbeat-loss behavior. | Approved Linux native validation |

Documentation changes in `gazebo/README.md` and `gazebo/DEMO_GUIDE.md` describe
the measured sample-data check. New tests are in `gazebo/test_verify_sync_axes.py`
`gazebo/test_record_gt.py`, and the existing regression and integration test files
listed above.

## Phase 1 disposition

**HOLD.** The checked-in `integration/PHASE1_FUSION14_RESULT.md` reports 368
VISP and 368 VISV receipts, 736 vision sends and both EKF3 cores announcing
ExternalNav usage. It reports zero armed samples or control commands. The
original fusion14 directory and raw BIN were absent in this checkout and
reachable objects inspected. The user-supplied transport figures (15.72 ms p50,
31.28 ms p95, 35.19 ms p99 and 40.00 ms maximum) remain unrecomputed segment
observations. The reported 121.5 ms IPC maximum measures another segment.
Paired acquisition-to-FCU timing, clock uncertainty, healthy sustained fusion,
and controlled loss-of-vision behavior remain unverified.

For abe2, the recorded line places the last good sample at simulation 41.7 s,
five withheld frames immediately after, and watchdog closure at approximately
42.289 s. Subscriber discovery messages after transport exit provide separate
shutdown evidence. The current checkout lacks the flight-line no-shadows patch.
Do not use the older shadow branch as a substitute. Keep review of the noise
ground change, flight evidence and shadow change separate. No branch was merged.

At fixed 10 Hz, the first rejected frame is 100 ms after the last publication.
The next four accepted frames are at +200, +300, +400 and +500 ms. The
250 ms span first clears on that fourth accepted frame. At RTF 0.8 the
500 ms simulation gap alone is about 625 ms wall time. At 20 Hz the same
250 ms span still needs six accepted frames, giving 350 ms simulation time
from the last publication after one rejected frame, before processing or IPC
overhead. Higher cadence therefore needs fresh load and clock measurements.

## Human-approved validation

1. Review exact commit `ad8e949` and its parent `348cdd8`. Read the branch-bound
   procedure using
   `git show origin/phase1-no-shadows:integration/linux_gpu/NO_SHADOWS_PREPARATION.md`.
2. On an approved NVIDIA host follow sections 1–2 for source verification, image
   rebuild, verification of both world inputs and an evidence-bound candidate.
3. With separate DISARMED authority, follow section 3 for the closed live-camera
   baseline and section 4 for the raw post-fixture ground camera capture. Verify
   the actual hardware renderer and inspect the frames for a silhouette. Record
   ground feature counts and compare with the audit’s proposed 150-inlier
   diagnostic target. Verify mapped samples, source continuity and DataFlash aiding. Stop on rejected
   clocks, unhealthy innovations, continuity gaps or failed cleanup.
4. Recover fusion14 raw files to independently match camera, publication, wire
   and DataFlash records before drawing latency conclusions. Review the
   watchdog margin and ground evidence before any separately approved simulation
   flight. Native runtime admission and flight readiness remain HOLD.


## Follow-up on `fix/codex-repository-repair` at `6a1a2cb`

The second audit attachment still names `main` at `3e40aa7` as its baseline.
The portable runner, timestamp evidence checks, frontend selection, heartbeat
fixes, trajectory tests and dependency updates are already in `6a1a2cb`.
The recorder and timestamp regression files and this report were present as
untracked files at the start of the follow-up and were preserved.

B5 now records whether DDS subscriber counts have reached the configured
minimum during a publication check, and distinguishes initial discovery from
a later drop. The diagnostic history does not participate in readiness or
publication decisions. The reason for a drop still needs process evidence.
B2 now has a regression that requires an import failure when the production
frontend is unavailable, preserving the explicit-only fallback contract.

Follow-up results: 56 targeted pytest cases passed (plus 15 subtests), offline
VIO passed 22, source-probe unittest passed 18, heartbeat supervisor unittest
passed eight, and source-scope and diff checks passed. Person 2 completed all
seven stages with local UDP enabled, including 16 pytest cases. These are
offline tests.
Phase 1 remains HOLD. The pinned VIO, transport interlocks and EKF settings
were not edited, and the flight-line branches were not merged.
