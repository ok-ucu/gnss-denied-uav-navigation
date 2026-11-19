# Phase 1 recovery runtime integration

**PARTIAL. Flight readiness HOLD.**

Base `8e9ede457c1e93cb01a645fa6c4197e58e09029e`, branch `fix/phase1-linux-validated`. The existing launcher authorization patch, offline recovery model, new tests and Linux reports were reviewed and committed. Generated runtime artifacts, candidates, logs and DataFlash remain outside Git.

## New runtime evidence

- `run_vio_stream.py` adds an audit record to the evaluated frame's existing diagnostic row for recognized visual quality rejection. It carries acquisition stamp, host observation time, VIO session, estimator epoch, reason and any injection label. Navigation availability never reads that log. A trusted live, ordered source-health channel is still needed for recovery authorization.
- `phase1_transport.py` and `simulation_vision.py` use the already validated socket completion for each successfully forwarded DISARMED half. The local pair witness is emitted once, at the later completed enqueue time, with both raw half completion times and the original acquisition/FCU stamps. Failed, unvalidated, partial and duplicate writes cannot create a new completion witness. This identifies local egress only. DataFlash remains necessary for FCU receipt and fusion.
- No watchdog consumes either record. The 500 ms host availability cutoff, Quality Gate, source and wire freshness, flight behavior and FCU failsafes are unchanged.

Focused regression testing: **186 passed, 47 subtests passed**. Tests include both new evidence helpers, the existing simulation transport fixtures and the offline recovery cases. Full suite: **718 passed, 1 failed, 7 skipped, 302 subtests passed**. The existing `vio/test_vio_core.py:51` failure calls the instance method `QualityMonitor.inflate` on the class, and reproduces the previously recorded TypeError. No existing tests were edited.

## Runtime admission and proof

No new Phase 1 runtime was started. The existing validated-Linux admission pins base HEAD `8e9ede4` and allows only the previously reviewed launcher change. A new run for these source changes needs a separately reviewed exact-source admission and runtime candidate. A current proof cannot be inferred from the prior run. The new source hashes were recorded in `../recovery-runtime-source-sha256.txt`; no new permit or runtime candidate was issued.

Prior baseline `20261119-pinneddisarmed182141`, measured RTF 0.6142. The historical DISARMED evidence has 348 odometry publications and 346 complete pairs, with host p95/max gaps 182.069/311.943 ms and 182.643/312.472 ms respectively. No active watchdog fired. The old DataFlash proof SHA-256 is `ff4295e8f3160d6221a88f7c9190fe9712febe4fd0a802d108d2783b704910bf`. Those metrics apply to the prior source stack.

Remaining work: deliver authenticated, ordered producer rejection and sensor/progress witnesses to the owned DISARMED transport, consume the actual current pair completion in the recovery decision, and verify the resulting watchdog behavior under real faults. Validate source-bound egress and fresh EKF3 evidence on the revised runtime before considering any flight request.
