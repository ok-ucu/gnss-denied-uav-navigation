# Phase 1 quality gate and watchdog

**PARTIAL. Phase 1 remains HOLD.** This worktree now carries the render-only
repair for the observed visual trigger. The historical watchdog failure is
still reproducible from the recorded inputs. A new Linux run is needed to
measure the effect of the repaired scene.

## 1. Failure chain and reproducible baseline

Worktree `phase1-vio-imu-robustness`, base
`348cdd8a6350daf68e1c22203b2735b8d1276727`. The preceding uncommitted IMU and
visual pre-screen work is preserved. No base, commit or evidence was rewritten.

1. `vio_imu_preintegration.py` propagates the pinned EKF in idle and frame
   paths. The frame path integrates to the image stamp and supplies a causal
   attitude. `run_vio_stream.py` then computes the visual correction candidate.
   The inlier/contrast, distributed audit, consistency and EKF innovation
   checks prevent an invalid visual correction or output. Existing camera
   height/vertical aiding remains unchanged.
2. `QualityGate.assess()` requires three good frames spanning at least 250 ms.
   At 10 Hz the fourth frame opens publication. Five recorded visual failures
   at 41.8–42.2 s followed the 41.7 s publication. Corrections succeeded at
   42.3, 42.4 and 42.5 s while readiness was false. The next publication was
   42.6 s. The first accepted correction was 0.600 s from the prior correction,
   and recovery hysteresis added 0.300 s. The stream logs show advancing frame
   processing and no epoch change in that interval. There is no evidence that
   estimator inactivity or transport queueing created this publication gap.
3. The publisher uses the acquisition stamp and emits an exact-stamp epoch
   token. `phase1_transport.py` matches that token to Odometry, checks its
   clock/watermark and handles pose and speed as a pair. `navigation_lifecycle.py`
   binds the authorization to the source session, clock and epoch and applies
   acquisition/permission freshness. The simulation route additionally checks
   host receipt age. The controller checks completed valid pairs separately.
   Reset, invalid clock or revoked permission closes the route. New samples do
   not reopen a latched run.
4. The 500 ms watchdog uses host monotonic time. The recorded trigger was
   500.637484 ms after arrival of the 41.7 s acquisition, in CLIMB. Every
   recorded source publication reached transport. Later DDS discovery loss
   followed transport exit. The fbf7 logs contain a separate watchdog, and the
   fea1 logs include a climb watchdog and source-side processing overruns.

Before applying the scene repair, the direct recorded-data assertion
`assert second["publish_monotonic_s"] - first["publish_monotonic_s"] <= .5`
for publications at 41.7 and 42.6 failed with 0.900467697 s. The existing
recorded tests preserve the rejection and downstream exit chronology.

## 2. Implemented repair

The alternative selected here removes the identified self-shadow from the
camera scene. The pinned image world has its directional sun's shadow flag
disabled during the build. The actual runtime launcher derives a fresh SDF
from the manifest-verified historical world and changes the same single flag.
Both Compose physics commands validate the mounted world before starting
Gazebo. Camera and vehicle settings, dynamics, ground texture, and quality and
watchdog policies are preserved. No runtime estimate or MAVLink state was
fabricated. No IMU-predicted output is published.

## 3. Prediction mathematics and horizon

The pinned nine-state EKF uses `[position, velocity, accelerometer bias]`.
For a propagation step h and body-to-NED R, it uses
`a = R(f - b) + g`, `p' = p + v h + a h²/2`, `v' = v + a h`.
Its covariance update is `F P Fᵀ + Q` with cross-covariance via
`F[p,v]=hI`, `F[p,b]=-h²R/2`, `F[v,b]=-hR` and diagonal Q blocks
`10⁻⁶ h I`, `0.3² h I`, and `0.01² h I`. These are pinned model
coefficients. They do not establish calibrated flight-sensor error bounds.

The direct velocity process-noise term alone contributes at least
`0.09 T` to each velocity variance under this prediction-only model. Each
direct velocity-noise injection is carried into future velocities unchanged,
and the remaining PSD covariance contributions cannot subtract it. At
T=0.600 s that gives an optimistic standard-deviation floor of **0.232379 m/s**
before inherited state, accelerometer bias or attitude uncertainty. An
allowable standard deviation V would imply the necessary bound
`T <= V²/0.09`, which is insufficient to authorize operation. Starting at
the recorded 41.7 s covariance and predicting 120 steps of 5 ms at fixed
identity attitude and nominal gravity gives the following model-only values.

| Quantity | At 41.7 s | After 0.600 s model propagation |
|---|---:|---:|
| Horizontal position sigma per axis | 0.151558 m | 0.178136 m |
| Vertical position sigma | 0.213199 m | 0.377207 m |
| Horizontal velocity sigma per axis | 0.065842 m/s | 0.245929 m/s |
| Vertical velocity sigma | 0.305551 m/s | 0.394438 m/s |

An independent discrete covariance summation matched the pinned EKF for
stationary, constant-velocity and accelerating test inputs with accelerometer
bias. This experiment does not measure the actual climb prediction error. The
nine-state filter has no attitude-error or gyro-bias state. For small attitude
error ε, even gravity compensation alone contributes acceleration error of
order g ε, velocity error g ε T and position error g ε T²/2. No aligned raw
IMU quaternion/gyro/force trace for the dropout, calibrated attitude/gyro-bias
error distribution, or FCU-approved predicted-state error limit was available.
The fixed attitude covariance supplied by the current publisher cannot fill
that gap. The numerical lower bound therefore cannot justify a live coast
horizon or a second publication mode.

## 4. Files

| File | Change in this task |
|---|---|
| `linux_gpu/Camera.Dockerfile` | Verify and change only the sun shadow flag in the image world. |
| `launch_phase1.py`, `linux_gpu/run_camera.py` | Derive the runtime world under the new run directory and preserve historical bytes. |
| `phase1.compose.yaml`, `linux_gpu/camera.compose.yaml` | Refuse Gazebo startup unless the actual mounted runtime world passes the shadow check. |
| `tests/test_linux_gpu_handoff.py`, `tests/test_source_runtime.py` | Exercise image edit, runtime generation and fail-closed startup in both paths. |
| `tests/test_vio_flight_abe2.py` | Test the recorded climb-height pairs without shadow using the existing bench. |
| `tests/test_vio_flight_motion.py` | Test pinned IMU state and covariance against an independent reference. |
| `PHASE1_GATE_WATCHDOG_FIX_REPORT.md` | Document implementation, quantitative limits and validation. |

The seven image/runtime files were carried over from the workspace's existing
`ad8e949` no-shadows change. The prior report and IMU production changes
remain intact.

## 5. Before and after evidence

| Item | Measured or recorded result |
|---|---|
| Historical gap | 0.900 s simulation and 0.900467697 s host publication. Frame-entry interval 0.900049983 s. |
| Visual rejection and recovery | Five rejected/unavailable frames from 41.8 through 42.2 s. Corrected output resumes at 42.6 s, 0.300 s after the first accepted recovery correction. |
| IMU freshness and epochs | Recorded per-frame IMU alignment age is 0 at the sampled dropout frames and the epoch is 0. The raw IMU sequence and physical covariance calibration are unavailable for independent replay. |
| Shadow/no-shadow bench | With actual recorded body heights at 41.8–42.3 s, shadowed inliers were 117, 34, 41, 47, 85 and 85. Shadow-free inliers were 172, 176, 174, 174, 192 and 165. All these paired spatial audits passed. The separate canonical 0.19→0.20 m shadow case still fails the spatial audit. The bench uses synthetic noise images and a recorded shadow mask. |
| Updated host/simulation publication gaps, outputs, restarts, live covariance and watchdog | NOT MEASURED. The scene intervention was tested offline, and there were zero prediction-only outputs by design. |

The production transport predicates were evaluated using synthetic host
timestamps separate from simulation timestamps. With 25 ms added scheduler
delay, a 0.100 s simulation gap yielded host gaps of 125, 177.439, 225 and
425 ms at RTF 1.0, 0.656, 0.5 and 0.25 respectively. All stayed below the
watchdog. A 0.900 s simulation gap plus the same delay yielded 925,
1396.951, 1825 and 3625 ms. All tripped it. The actual predicate accepts
exactly 500 ms and trips at 500 ms + 1 ns. The independent flight predicate
rejected a missing pair and a completed pair older than 500 ms. These
predicate tests do not exercise DDS scheduling or EKF3.

## 6. Test commands

Run from this worktree. Set the two input paths to the pinned VIO export and
the pinned upstream camera world respectively.

```bash
p=/Users/ok/Projects/gnss-denied-gazebo-integration/integration/source_scope_audit/commit-20261114/test-env/bin/python
export VIO_SOURCE_DIR=/Users/ok/Projects/gnss-phase1-no-shadows/integration/linux_gpu_runs/offline-no-shadows
export NO_SHADOWS_UPSTREAM_SDF=/Users/ok/Projects/gnss-denied-gazebo-integration/integration/ardupilot_gazebo/source/worlds/iris_runway.sdf
export PYTHONPATH="$PWD/integration:$PWD/integration/tests:$VIO_SOURCE_DIR:/tmp/phase1-no-shadows-py39"
"$p" -B -m unittest -v test_vio_flight_motion.BlockPropagation test_vio_flight_abe2 test_linux_gpu_handoff test_source_runtime
"$p" -B -m unittest discover -s integration/tests -v
"$p" -B -m pytest -p no:cacheprovider integration/tests/test_dataflash_time_adapter.py
PYTHONPATH=/tmp/phase1-gate-watchdog-root-deps:/tmp/phase1-no-shadows-py39 \
  "$p" -B -m pytest -p no:cacheprovider camera calibration dataset gazebo vio -q
git diff --check
```

Focused integration checks passed 66 tests with zero failures and zero skips.
Full integration discovery finished with 526 passed, zero failed and three
skipped (two Linux-native checks and a Matplotlib plot check). Five separate
DataFlash pytest cases passed. Logs are at /tmp/phase1-gate-watchdog-{focused,full,pytest,root}.log.
Both Compose configurations, changed Python syntax and Git whitespace checks
passed. The root Python suite finished with 13 passed, one skipped for missing
pymavlink, and one failure. The root VIO test calls the instance method
`QualityMonitor.inflate` on the class, causing a `TypeError`. The same failure
reproduced in the separate clean no-shadows worktree, and the root `vio/` tree
is identical to the base. The initial root collection required PyYAML, which
was installed into the temporary dependency directory used above.

## 7. Safety and remaining blockers

The original 500 ms watchdog, completed-pair requirement, acquisition clock,
quality checks, covariance checks and permission expiry remain in effect.
Camera or IMU loss, estimator reset, stale samples and invalid clocks still
close the existing path. The previous IMU duplicate regressions ran in both
propagation modes. The isolated IPC receiver has a separate strict timestamp
check before preintegration. A duplicate stopped by that earlier check is
outside the demonstrated preintegrator fix.

A **fresh DISARMED proof is required** because the launch and Compose sources
are permit-bound and the camera image changes. Linux validation needs the
final NVIDIA image, running-scene check, raw downward frames on the noise
fixture, verified native candidate, and approved DISARMED sensor and DataFlash
capture. Measure actual publication and pair gaps before reconsidering flight.
No Linux GPU run, ARM or TAKEOFF was performed for this task.
