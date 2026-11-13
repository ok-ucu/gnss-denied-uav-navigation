# Phase 1 integration source package

**HOLD — `execution_allowed=false`, `forwarding_enabled=false`.**
This directory contains the real camera/IMU VIO integration, coordinate and
covariance conversion, clock/reset/egress guards, patched MAVROS overlay source,
capture-only Compose configuration, analysis and offline tests. This source
package does not prove live-camera EKF3 fusion or GNSS-denied flight.

[Source allowlist](SOURCE_FILES.txt) · [Review/staging plan](SOURCE_COMMIT.md) ·
[External dependencies and bootstrap limits](SOURCE_DEPENDENCIES.md).

```text
Gazebo camera + IMU + simulation /clock
  -> existing teammate FlowFrontend/VioEKF (external vio/ dependency)
  -> quality-gated Odometry NED/FRD
  -> frame conversion ENU/FLU + pose/speed covariance
  -> explicit time mapping + durable epoch guard
  -> patched MAVROS -> final wire-level capture interlock
  -X-> FCU vision input (forwarding remains disabled)
```

Ground-truth Z is an explicitly disclosed test aid for optical-flow depth;
horizontal GT is not an estimator input. Correct mounting, preserved acquisition
timestamps and 21/9-element wire covariance do not establish flight readiness.
Timing, continuity, covariance calibration, reference and live fusion gates
remain open. Historical evidence remains locally under the ignored run folders.
[PHASE1_STATUS.md](PHASE1_STATUS.md) and the [preserved evidence summary](reports/RENDERING_20261111.md) ship in Git; raw historical evidence remains local and is not required by the Linux camera-only workflow.

## Linux GPU handoff

Current validation instructions: [VIO source OFF/ON/OFF](VIO_SOURCE_PROBE.md),
[amd64 build/provenance and isolated TIMESYNC](AMD64_TRANSPORT_READINESS.md),
and [reviewed teammate evidence](reports/VIO_ISOLATION_20261113.md).
The current source package has168offline tests; native runtime results remain
separate from those tests. The selected compact replay JSONL is an intentional
test fixture with recorded provenance, not a runtime capture dump.

Follow these gates in order; stop at the first FAIL or missing evidence:

|Gate|Existing command/procedure|Required PASS/evidence|
|---|---|---|
|Linux GPU access + renderer|On the Linux host: `uname -sm`; driver/device inventory; `docker exec gnss-sim-gazebo cat /tmp/render_gl_identity.txt` plus current Ogre log|Actual context corroborated, no llvmpipe/software/WARP; host GPU alone is insufficient|
|Native builds/provenance|Run the build/export blocks in [amd64 readiness](AMD64_TRANSPORT_READINESS.md)|Pinned AP commit, actual parser5/5, ELF/image architecture, patch/context/base/binary hashes; candidate stays HOLD|
|Isolated overlay|Run `linux_gpu/check_timesync.sh` in the documented NEW network-none container|Actual patched-plugin load, signed offset/jitter/drift/reset replay, unchanged stamps, valid21/9covariance, no packet loss/duplicates|
|VIO continuity|Run `run_vio_source_probe.sh` using the [source procedure](VIO_SOURCE_PROBE.md)|Independent subscriber; full-window/tail gaps<=500ms; publication age<=180ms sim; healthy ordered IMU/IPC, no recovery/loss|
|Native clock admission|Separate review of applied-clock trace/cross-check and firmware identity|Not yet verified for amd64; candidate cannot self-approve or execute|
|Capture normal|Only after admission, run `run_latency_matrix.py --cases normal --normal-duration 60 --isolated-imu` with its documented run arguments|Both wire channels for each eligible sample, valid permission at final boundary,200ms clock/500ms availability, zero FCU vision packets|

Latest environment check on14November: local Darwin arm64; the configured Linux
SSH endpoint timed out. Linux validation is **BLOCKED BY ENVIRONMENT**, not a renderer
or runtime FAIL. No remote test result is inferred. Full amd64 capture startup
also remains blocked by native clock/runtime admission; do not bypass it with
manual Compose startup or relabel source/replay PASS as flight readiness.

Start with [LINUX_GPU_HANDOFF.md](LINUX_GPU_HANDOFF.md). It includes measured
software-rendering evidence, native-architecture build contexts, camera-only
DISARMED launcher, actual GL-device verification and precise remaining full-stack
provisioning limits. Preparation/Compose/closure checks do not start simulations.
New Linux image builds and hardware GPU execution remain NOT TESTED here.

## Checks that do not start a simulator

```bash
python3 -B integration/audit_source_scope.py
PYTHONPATH=integration VIO_SOURCE_DIR="$PWD/vio" \
  python3 -B -m unittest discover -s integration/tests -v
git diff --check
git diff --cached --name-only
```

Full tests need NumPy and OpenCV plus the existing teammate `vio/` checkout.
ROS/Gazebo imports are generally lazy, but using the already installed ROS Jazzy
runtime is the verified full-suite environment. Missing dependencies are not
treated as a PASS. No package is installed by the audit script.

The pre-handoff full offline suite ran **113/113 PASS** in the ARM64 worker;
the handoff validation result is recorded in [the source scope](SOURCE_COMMIT.md).
The earlier source-only bundle review ran 108/108 before the timing regressions
were added; that historical result is not relabelled as a new 113-test run.
Exact local logs are under ignored `source_scope_audit/` or `phase1_runs/`.
Neither unit tests nor Compose resolution is a new live-navigation verification.

The current Compose transport uses one CPU and a run-scoped persistent Linux
volume, `gnss-p1-guard-<run-id>`, for synchronous guard writes and final reads.
This removes measured macOS bind-mount I/O from those operations; it does not
remove durable checks, extend permissions or provide a real-time bound. Never
delete/recreate that volume to clear a latch. Evidence snapshots are diagnostic
copies and must not be restored as authorization. Standalone transport keeps its
legacy run-state path unless `PHASE1_GUARD_STATE_DIR` is explicitly configured.

The historical timing2 capture-only fragment admitted 21 pose/speed pairs, with all 42 packets
timely at the wire check, then closed on the 200 ms clock watchdog. Only 21/88
source Odometry samples reached both wire channels; this is **not** continuous
end-to-end readiness. No vision packet reached FCU. Local details and unresolved
gates are in `PHASE1_STATUS.md` and ignored `phase1_runs/20261109-timing2/`.
The subsequent clock-stall follow-up corrected common-watermark liveness
accounting, but bounded 60-second normal runs still failed on upstream JSON/physics
pauses and expiry. Scheduling hypotheses were reverted; no continuous PASS or
new FCU forwarding is claimed. Details remain in the local status and ignored
`phase1_runs/20261109-clock5/REPORT.md`.

## Runtime entry points — prerequisites required, not executed by this audit

[launch_phase1.py](launch_phase1.py) refuses to overwrite an old run or reuse
container names. Its default is preparation only. A separately authorized
capture run uses [phase1.compose.yaml](phase1.compose.yaml), the actual patched
binary, gimbal model, real VIO and patched MAVROS. It never provides an FCU vision
forward switch or ARM/TAKEOFF command. [run_latency_matrix.py](run_latency_matrix.py)
adds labelled receive/quality faults and process/clock lifecycle checks within
that owned stack. It uses explicit new **capture** references, not FCU approval.

After the prerequisites in [SOURCE_DEPENDENCIES.md](SOURCE_DEPENDENCIES.md) are
supplied, preparation syntax is:

```bash
python3 -B integration/launch_phase1.py --run-id 20261110-review1
```

This command is documented, not run as part of the source-scope audit. It checks
installed image/artifact IDs and creates new local configuration; it is not a
fresh-machine dependency installer. No flight or forwarding approval follows
from preparing files.
