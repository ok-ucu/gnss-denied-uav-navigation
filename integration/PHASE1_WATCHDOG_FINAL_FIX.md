# Phase 1 watchdog timing decision

**PARTIAL. Flight readiness HOLD.** The current timing contract cannot guarantee uninterrupted recovery after the specified faults. This change adds regression coverage. Production policy and the previous local launcher patch are preserved.

## Root cause

The 500 ms host availability watchdog protects the time since the last valid navigation update. The producer uses simulation acquisition time to qualify readiness. It requires at least three ready frames spanning at least 250 ms simulation. At 10 Hz that ordinarily takes four clean frames over 300 ms. After one quality rejection, the last good estimate and the first reacquired estimate are therefore 500 ms simulation apart. At the measured 0.6142× progression that is 814 ms host before processing and DDS, exceeding the limit by at least 314 ms in the nominal timing model. At RTF 1.0 it leaves zero slack, even though the exact 500 ms boundary itself is inclusive in simulation forwarding.

With an unchanged readiness duration, RTF 0.50 converts 250 ms to 500 ms host. Even infinitely frequent valid frames leave zero time for fault detection, reacquisition entry and delivery. RTF 0.25 consumes 1,000 ms host for readiness alone. At 10 Hz, one missing frame without a reset already consumes 800 ms host at RTF 0.25. No scheduling or timestamp-only change can guarantee recovery for the entire requested domain under this contract.

## Active boundaries audited

| Observation | Current production check |
|---|---|
| No valid navigation | 500 ms host since last odometry at transport and simulation egress. The flight controller also checks completed-pair age. |
| Camera or IMU stops advancing | Source checks last frame/unique IMU receipt at 500 ms host. Source alignment requires IMU age ≤5 ms and coverage gaps ≤20 ms simulation. Camera interframe gap >300 ms resets frontend continuity. |
| ROS clock/applied physics stalls | Navigation independently checks ROS progression and applied-anchor liveness at 200 ms host. Clock duplicates do not refresh ROS progress. Backwards/invalid observations latch closed. Source separately checks clock simulation age and callback receipt ≤500 ms host. |
| Timing of an estimate | Source publication freshness 157 ms simulation; lifecycle accepts ROS age −5 to 200 ms, applied mapping envelope ≤20 ms, then assigns a host wire deadline from remaining age margin. |
| Estimator and image quality | Inliers, contrast, audit, estimator readiness/epoch, and covariance validated before publishing. 250 ms simulation readiness applies after rejection. |
| DDS/transport | Insufficient DDS subscribers withhold delivery without declaring estimator failure. Transport checks clock, durable reference, matching timestamp/covariance and each wire half's final deadline. A partial pair cannot be completed after expiry. |

## Deterministic before/after matrix

The before matrix is the previously recorded 28 cases. The new regression invokes the same production QualityGate with deterministic timestamps and asserts the same 28 outcomes. No production timing source changed. All entries are **before = after**. Simulation gaps are **100 / 200 / 800 / 1,000 / 500 / 700 / 900 ms** for the columns in order. Host cells are ideal elapsed ms to the first reacquired slot assuming fresh inputs, valid covariance and zero delivery latency. An interval beyond 500 ms predicts hard-stop; E marks the 500 ms equality with no slack. These are injected clock ratios, not measured runtime RTFs.

| RTF | Normal | 1 absent | 3 absent | 5 absent | 1 quality rejection | 3 quality rejections | 5 quality rejections |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.00 | 100 | 200 | 800 | 1,000 | 500 E | 700 | 900 |
| 0.65 | 153.846 | 307.692 | 1,230.769 | 1,538.462 | 769.231 | 1,076.923 | 1,384.615 |
| 0.50 | 200 | 400 | 1,600 | 2,000 | 1,000 | 1,400 | 1,800 |
| 0.25 | 400 | 800 | 3,200 | 4,000 | 2,000 | 2,800 | 3,600 |

For three and five absences, the modeled next arrival is itself rejected by the camera gap rule. The source host sensor watchdog and actual frontend reinitialization may act earlier or extend recovery, so a post-hard-stop readiness slot is not navigation reauthorization.

| Additional deterministic fault | Result on existing code |
|---|---|
| Camera/IMU stall | Actual source tick calls its respective block callback at 500.001 ms host. IMU stall requests estimator reset. |
| Gazebo applied-clock stall with advancing ROS clock | Production lifecycle latches closed on stale applied anchors, even though ROS clock receipt is fresh. |
| /clock pause/reset | A duplicate does not renew host receipt; the 200 ms host timeout closes the gate. A backwards ROS timestamp closes immediately. |
| Late/incomplete pair | The first half remains incomplete. The late second half is rejected and its egress authorization latches closed. |
| Wrong wire timestamp, invalid covariance, expired DDS deadline | Zero socket writes in the fake-socket route tests; lifecycle blocked. |
| Host cutoff | At 500 ms exactly, a matched packet may forward while other permissions are fresh. At 500 ms + 1 ns it is rejected with zero writes. |
| Stale source, absent DDS subscriber, estimator failure | Stale source clears readiness. Missing discovery withholds delivery. Estimator failure blocks quality. |

## Policy for review

Keep existing 500 ms hard-stop, independent sensor and clock progress checks, and durable fail-closed authorization. For a simulation-only DISARMED study, bind an RTF observation to advancing ROS simulation time and applied FCU anchors in the same epoch, measured against host monotonic time. Cross-check rate on bounded observation windows; reset or disagreement yields unavailable. Use the rate to predict a clean-stream margin and classify quality, sensor, clock or delivery failure from the corresponding evidence, without granting forwarding on the basis of RTF alone. Apply a minimum demonstrated RTF for each supported scenario.

The admission inequality is `required simulation gap / lower-bound RTF + measured host scheduling/DDS margin < 500 ms`. At nominal 10 Hz, a clean stream needs RTF above 0.20 even at zero overhead; an isolated missing frame without reset needs above 0.40; one quality rejection needs above 1.00. These are mathematical lower bounds. A supported runtime envelope additionally needs measured worst-case margins and DISARMED loss tests. Options within the current contract are better simulator throughput, reduced measured scheduling/DDS tails, or higher genuine synchronized estimate cadence where the readiness lower bound allows it. Supporting recovery below the demonstrated envelope requires a separately approved simulation-only recovery contract with an independently demonstrated host bound; no new bound is claimed here.

## Changes and evidence

- HEAD `8e9ede457c1e93cb01a645fa6c4197e58e09029e`, branch `fix/phase1-linux-validated`. Added `integration/tests/test_watchdog_timing_contract.py` and this report. Existing tests and the prior uncommitted `integration/launch_phase1.py` patch were preserved.
- Targeted regressions: **156 passed, 66 subtests passed**. The 40 new cases include all 28 timing combinations and 12 additional boundary and fault checks. No real sockets or simulator were used.
- Historical NVIDIA baseline `20261119-pinneddisarmed182141`: measured RTF 0.6142, camera 10 Hz sim, IMU 1 kHz sim, 348 odometry and 346 complete pose/velocity pairs. Host max odometry/pair gaps 311.943/312.472 ms, source freshness max 113 ms simulation, zero estimator reset and no active watchdog. DataFlash proof re-decoded and source/candidate hashes checked on this checkout. Zero flight commands in that run.
- There is no runtime after measurement, new candidate, or fresh proof. The requested runtime rerun and full regression pass were conditional on a production fix. The old proof remains evidence for its exact historical run and unchanged bound production sources. It does not certify recovery or permit flight. Runtime tests for varying RTF, sensor loss, clock reset and representative motion remain open.

| Baseline artifact | SHA-256 |
|---|---|
| Patched launcher | `dbabbcba07b200060567da2a57929d40a213cb1eba8c699549cea990893e0993` |
| Candidate manifest | `b61130f975c0e6cdf67f2fd56d3e7d6fd025a7094bb379b247e907745900fc3f` |
| Pinned physics image | `1653eb3241db282628fab9a528328aede57d556c0f16bdaa3e0d7ef3c49faed9` |
| Loaded shadow-free world | `bf3dce911f0d02f5bd4a6f04710b62473c6784892917f1b6fb20257b14274df5` |
| DISARMED fusion proof | `ff4295e8f3160d6221a88f7c9190fe9712febe4fd0a802d108d2783b704910bf` |

Evidence is in `integration/phase1_runs/20261119-pinneddisarmed182141` and the checkout's parent directory. The source hash list is in the run's `artifacts/simulation_vision_permit.json` and `../pinned-runtime-sha256.txt`. No commit, push, ARM, TAKEOFF or LAND was issued.
