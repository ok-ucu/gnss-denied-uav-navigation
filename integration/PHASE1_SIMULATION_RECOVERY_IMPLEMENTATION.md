# Phase 1 simulation recovery prototype

- Simulation contract implementation: **PARTIAL**
- DISARMED safety validation of the prototype: **PARTIAL** (offline only; the literal simulation budget failed a latency check)
- Phase 1 flight readiness: **HOLD**

## Code and scope

Branch `fix/phase1-linux-validated`, HEAD `8e9ede457c1e93cb01a645fa6c4197e58e09029e`. Existing local launcher patch, prior report and tests preserved. Added `integration/simulation_recovery.py`, `integration/tests/test_simulation_recovery_prototype.py` and this report. Existing production entry points, controller and tests were left as they were.

The prototype is default-off, has no socket or flight API, and is disconnected from the runtime. Explicit enablement requires a DISARMED indication plus run, epoch and source identity. Within the prototype these are caller-supplied fixture inputs. A matching identity alone does not authenticate a rejection. The production wiring would require authenticated producer rejection evidence, independent live sensor and owned-process health witnesses, timestamp provenance, and an authoritative shared interpretation of a completed pair. The current acquisition and wire interfaces do not provide that end-to-end decision. None of the existing ingress/transport watchdogs consumes this prototype state.

## Implemented state calculations

- NORMAL uses 500 ms host from last complete pair. It can begin recovery only on one current quality-only rejection matching the acquisition stamp of the latest camera frame and previous complete pair interval. The observation identity and healthy-estimator indication must match. The last NORMAL interval must be at least 2 s host.
- RECOVERING uses fixed last-complete-pair host + 1,150 ms and acquisition stamp + 510 ms simulation. It can return to NORMAL only on a new complete ready/fresh/covariance-valid pair. The adapter must supply trustworthy readiness, freshness, covariance, health and completion inputs. The prototype cannot certify EKF state or publish.
- BLOCKED latches on rejected eligibility, a second rejection, unavailable sensors, regressions/disagreement in time witnesses, missed rate checks, invalid pair or expiry. Incoming data cannot reopen it.
- Camera progress 280 ms host and 90–110 ms simulation acquisition cadence. IMU unique stamp 100 ms host. Gazebo owned-process witness 50 ms host. ROS/applied progress 200 ms host per witness. Repeated timestamps do not refresh these independent bounds. These use injected observations in the tests.
- Dual-source RTF uses three nonoverlapping 2 s host windows, ≥10 advancing observations per source per window, worst-case endpoint uncertainty, lower bound ≥0.50 and cross-source bound difference ≤0.05. Recovery reevaluates within 50 ms host using a rolling 500 ms check. It cannot move the fixed deadlines.
- Existing 157 ms source freshness, 250 ms readiness, 20 ms mapping uncertainty, 5 ms IMU alignment and 20 ms IMU coverage gap remain production checks. The prototype does not replace them.

## Deterministic validation

Full fixture matrix with nanosecond injection and transition timestamps: `../simulation-recovery-matrix.csv` (relative to repository root). PASS means the fixture reached the expected outcome; BLOCKED is the expected safe outcome for a negative test. These tests do not measure live RTF or end-to-end performance.

| RTF | Normal | Single eligible quality rejection at zero delivery latency | Other tested faults |
|---|---|---|---|
| 1.00 | NORMAL | returns NORMAL | BLOCKED |
| 0.65 | NORMAL | returns NORMAL, ~615 ms host from rejection and ~769 ms from last pair | BLOCKED |
| 0.60 | NORMAL | returns NORMAL, ~667 ms host from rejection and ~833 ms from last pair | BLOCKED |
| 0.50 | NORMAL | returns NORMAL, 800 ms host from rejection and 1,000 ms from last pair with zero clock uncertainty | BLOCKED |
| 0.45 | NORMAL | BLOCKED for RTF floor | BLOCKED |
| 0.25 | BLOCKED by proposed camera deadline | BLOCKED | BLOCKED |

Faults cover double rejection, 1/3/5 missing frames, camera/IMU/Gazebo/applied stalls, ROS pause/reset, estimator indication, invalid covariance, freshness/late or incomplete pair, repeated rejection inside the cooldown, RTF disagreement, wrong source and lost rejection evidence. The new suite directly asserts the last scenario and the simulation deadline failure. Existing production boundary regressions use fake sockets only.

### Budget failure using baseline latency

The ideal recovery pair is acquired 500 ms simulation after the previous pair stamp. A literal 510 ms completion bound leaves only 10 ms simulation for its processing and delivery. In the historical DISARMED baseline, the age of the source estimate against the common source clock at final wire check was **17 ms minimum, 37 ms p95 and 117 ms maximum**. A 500 ms reacquisition gap with those ages implies **517 / 537 / 617 ms simulation** from last pair acquisition to final wire check. These are projected from observed baseline delivery ages, not measured recovery times. A deterministic 17 ms injection latches at the simulation deadline. The 1,150 ms host cap does not remedy this earlier failure. At exact nominal RTF 0.50 any positive uncertainty can also prevent proving the ≥0.50 lower bound.

## Evidence and remaining work

- Targeted regression run: **150 passed, 40 subtests passed**, including 66 prototype cases. See `../simulation-recovery-regressions-final.txt`.
- Baseline `integration/phase1_runs/20261119-pinneddisarmed182141` remains the prior DISARMED proof with 346 complete pairs, EKF3 ExternalNav and no flight commands. Measured RTF 0.6142. Odometry p95/max 182.069/311.943 ms host, pairs 182.643/312.472 ms host, both max 100 ms simulation. See `../pinned-metrics.json` for source age and full distributions.
- Historical fusion proof SHA-256 `ff4295e8f3160d6221a88f7c9190fe9712febe4fd0a802d108d2783b704910bf`. Historical candidate SHA-256 `b61130f975c0e6cdf67f2fd56d3e7d6fd025a7094bb379b247e907745900fc3f`. Pinned physics image `sha256:1653eb3241db282628fab9a528328aede57d556c0f16bdaa3e0d7ef3c49faed9`; loaded world SHA-256 `bf3dce911f0d02f5bd4a6f04710b62473c6784892917f1b6fb20257b14274df5`. Prototype hashes are recorded in `../simulation-recovery-prototype.sha256`.
- No new candidate, runtime, fault-injection capture, raw streams, fusion proof or full-suite result. Establish with measured recovery evidence that completion can fit 510 ms simulation, or separately approve an explicit distinction between the acquisition deadline and completed-pair delivery deadline. Retain both an independent host cap and fresh wire permission in either design. Then add source-bound runtime evidence and audit consistency across source, lifecycle and egress before any DISARMED integration.
- No automatic commit, push, merge, ARM, TAKEOFF or LAND.

## Corrected acquisition/delivery contract

**Contract correctness: PASS in the offline model. DISARMED runtime validation: PARTIAL (historical baseline only). Flight readiness: HOLD.**

The corrected opt-in offline model is `AcquisitionDeliveryRecovery` in `integration/simulation_recovery.py`. The earlier `SimulationRecovery` class is retained as the literal-budget baseline to preserve its regression tests and provide a reproducible comparison. No production supervisor consumes either model.

- The simulation anchor is the original VIO acquisition stamp carried by the last complete valid wire pair. The 510 ms constraint applies to the acquisition stamp of a newly accepted Quality Gate estimate, not to the clock at socket completion. A camera receipt or rejected frame is insufficient. The source verdict must be linked to that acquisition, the run/source/epoch tuple and a fresh source clock snapshot. A verdict can arrive after the acquisition window closes only if it proves an eligible stamp and still meets the existing 157 ms freshness bound. When no eligible proof exists and even the latest eligible stamp has expired, recovery closes.
- The host anchor is the completion time of the last complete valid wire pair. Its absolute 1,150 ms cutoff never moves on camera or accepted-estimate events. Only a pair with both wire acquisition stamps matching the accepted recovery estimate, and independently validated delivery, returns the model to NORMAL. An invalid, unrelated or incomplete pair latches closed.
- The authenticated runtime caller must still enforce current mapping ≤20 ms, original DDS/wire deadlines and covariance, sequence, timestamps and estimator checks. The model takes bounded source age and explicit validity inputs. It cannot attest those inputs on its own.

The tests anchor the prior pair to acquisition **4.000 s simulation** and the recovery frame to **4.500 s simulation**, so the limit is **4.510 s simulation**. The paired host clocks are independent.

| Fixture RTF | Ideal last-pair to next completion | With 35 ms verdict delay plus 5 ms delivery | Corrected outcome |
|---|---:|---:|---|
| 0.65 | ~774.231 ms host | ~809.231 ms host | Complete |
| 0.60 | ~838.333 ms host | ~873.333 ms host | Complete |
| 0.50 | 1,005 ms host | 1,040 ms host | Complete with exact fixture clocks |

The old completion-by-simulation-clock deadline would reject the delayed cases (35 ms verdict delay alone uses 17.5–22.75 ms simulation after acquisition). The corrected model accepts the same timely stamp when both delivery budgets remain satisfied. A 4.600 s acquisition at RTF 0.65 is rejected even when the offered completion precedes the host cap. A timely 4.500 s acquisition cannot authorize delivery after the host cap. Source age above 157 ms, missing acceptance, partial and mismatched pairs, and all exercised independent faults latch closed. Full fixture transition timestamps and attempted event times are in `../corrected-deadline-matrix.csv`. Exact RTF 0.50 is an ideal fixture with zero observation uncertainty; live lower-bound proof is still required.

Regression result: **171 passed, 40 subtests passed**. The final 21 new deadline cases were rerun after tightening the stale-age assertion and all passed. Existing tests were left intact. See `../corrected-deadline-regressions.txt`, `../corrected-deadline-final.txt`, and `../corrected-deadline.sha256`.

### Runtime integration blocker

The current source publishes only ready epoch tokens through `/integration/vio/sample_epoch`. Rejected frames are logged as diagnostics, with no live source-bound rejection record consumed by transport. The DISARMED branch counts complete pairs but only records `flight.last_pair` for an active flight controller. Runtime integration needs an authenticated, ordered producer evidence channel for accepted/rejected estimates and independent camera/IMU/clock health, and a single authoritative completion witness from the verified two-half egress. Using diagnostic logs, a simulated source boolean, or mere odometry receipt to open an exception would let the watchdog wait without sufficient evidence. No transport timeout or flight path was altered.

No new runtime candidate, source-stack attestation, NVIDIA run, full regression run or fusion proof was generated for the corrected model. There are no real after-recovery durations or live pair gaps. For the historical run, see the earlier evidence above. No ARM, TAKEOFF, LAND, commit or push was issued.
