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
The current source package has198offline tests; native runtime results remain
separate from those tests. The selected compact replay JSONL is an intentional
test fixture with recorded provenance, not a runtime capture dump.

Follow these gates in order; stop at the first FAIL or missing evidence:

|Gate|Existing command/procedure|Required PASS/evidence|
|---|---|---|
|Linux GPU access + renderer|Use the separate guarded NVIDIA startup in [the handoff](LINUX_GPU_HANDOFF.md); it records the NEW runtime's context and Ogre log|Actual context corroborated, no llvmpipe/software/WARP; host GPU alone is insufficient|
|Native builds/provenance|Run the build/export blocks in [amd64 readiness](AMD64_TRANSPORT_READINESS.md)|Pinned AP commit, actual parser5/5, ELF/image architecture, patch/context/base/binary hashes; candidate stays HOLD|
|Isolated overlay|Run `linux_gpu/check_timesync.sh` in the documented NEW network-none container|Actual patched-plugin load, signed offset/jitter/drift/reset replay, unchanged stamps, valid21/9covariance, no packet loss/duplicates|
|VIO continuity|After startup PASS, `run_source_runtime.py --execute --probe` invokes the [source procedure](VIO_SOURCE_PROBE.md)|Independent subscriber; full-window/tail gaps<=500ms; publication age<=180ms sim; healthy ordered IMU/IPC, no recovery/loss|
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
# Requires the separate pinned Git object; use a NEW export directory.
python3 -B integration/linux_gpu/prepare_vio.py --output integration/linux_gpu_runs/offline-vio-01
PYTHONPATH=integration VIO_SOURCE_DIR="$PWD/integration/linux_gpu_runs/offline-vio-01" \
  python3 -B -m unittest discover -s integration/tests -v
git diff --check
git diff --cached --name-only
```

Full tests need NumPy and OpenCV plus the four hash-verified VIO exports.
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
binary, gimbal model, real VIO and patched MAVROS. Its default capture mode has
no FCU vision route. The separate opt-in DISARMED simulation route below never
provides ARM/TAKEOFF commands. [run_latency_matrix.py](run_latency_matrix.py)
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

## Opt-in DISARMED live-camera vision ingress (Linux NVIDIA only)

This is an implemented, **not live-verified**, simulation-only route. Default
capture and production forwarding remain disabled. It does not authorize flight.
Use one new matched-physics Gazebo/SITL stack, never a real FCU or a historical
run. The source still uses Gazebo GT **height only**; horizontal GT is not input.

First complete the native build/exports and isolated overlay replay in
[AMD64_TRANSPORT_READINESS.md](AMD64_TRANSPORT_READINESS.md). These produce
`firmware/`, `mavros/`, `patch_install/`, and the verified HOLD `candidate.json`
under one `integration/linux_gpu_runs/<build-id>/`. Do not change context hashes
or replace missing MAVROS2.16.0 packages with another version.

From the repository root, after receiving this source commit, choose fresh IDs:

```bash
set -euo pipefail
: "${P1_BUILD:?set to the existing native export directory below integration/linux_gpu_runs}"
test -f "$P1_BUILD/candidate.json"
test -r /usr/share/glvnd/egl_vendor.d/10_nvidia.json
python3 -B integration/linux_gpu/prepare_vio.py \
  --output integration/linux_gpu_runs/vio-fusion01
python3 -B integration/launch_phase1.py \
  --run-id 20261116-camerafusion01 \
  --runtime-candidate "$P1_BUILD/candidate.json" \
  --vio-dir integration/linux_gpu_runs/vio-fusion01 \
  --gpu-mode nvidia --isolated-imu \
  --simulation-disarmed-vision \
  --simulation-origin=-35.363262,149.165237,584 --execute
```

The explicit simulation flag is approval only for this bounded DISARMED test.
Without it a native candidate still cannot execute. `runtime_admitted` remains
false for production: a simulation permit only makes that exact source-verified
binary eligible for **fresh live** applied-clock/cross-check proof. It never
sets `mapping.verified` or supplies missing anchors. Unsupported environments
fail closed; WSL/D3D12 is not admitted by this NVIDIA-specific mode.

The launcher verifies the actual NVIDIA renderer and gimbal, binds immutable
Docker IDs/internal network/proc namespace/build identity into a one-run permit,
then runs **one 60s normal ingress window** without the fault matrix. The source
runs for 65s so that the interlock closes at the declared observation end before
source export/teardown; all traces are retained and the monotonic end boundary
is saved in `evidence/simulation_observation.json`. No outage within the window
is excluded. The transport
requires sustained expected-source DISARMED MAVLink2, FCU ExternalNav/GPS parameter
readback, explicit EKF-origin readback, actual mapped acquisition timestamps,
covariance/payload equality and the existing durable/final wire permission.
Only VPE/VSE may use the added route. MAVROS still peers with UDP14890/14891,
never directly with FCU TCP5760. Origin initialization is explicit simulation
reference setup, not a GPS measurement. No mode/arming/takeoff command is sent.

Artifacts are under `integration/phase1_runs/20261116-camerafusion01/`:
`artifacts/simulation_vision_permit.json`, `evidence/simulation-owned-initial.json`,
`evidence/transport_1/{wire,params,fcuwitness,mapping}.jsonl`,
`evidence/transport_1/simulation_result.json`, VIO/observer traces and
`state/fcu/logs/*.BIN`. A complete local socket write is **not** FCU reception or
fusion PASS. Inspect DataFlash VISP/VISV (remote timestamps/ignore/reset), MSG
ExternalNav aiding, XKFS source selection and XKF innovations/status/variance.
The historical `analyze_ekf3_fusion.py` expects a synthetic bench directory layout;
do not apply its static-origin RMSE verdict to this camera run.

STOP on any preflight/latch/heartbeat/clock/permission/source failure, partial or
late send, or incomplete cleanup. No TTL/watchdog is relaxed and no stale data
is replayed. No automatic reference reopening after reset/restart. The launcher
stops the exact initially recorded container IDs and preserves logs/state/volumes.
If startup failed before IDs were established, cleanup is explicitly INCOMPLETE;
inspect that run before further action. All Linux runtime/fusion outcomes remain
NOT TESTED by offline regressions. Phase 1 remains HOLD.

## Separate FIRST simulated flight scenario — OFFLINE IMPLEMENTATION ONLY

[run_simulated_flight.py](run_simulated_flight.py) is a separate opt-in entrypoint;
the DISARMED launcher and validation scenario are unchanged. Do not execute it
without separate flight-test approval. Global production flags remain false.
Only its new owned Gazebo/SITL stack can receive fixed simulation commands.
MAVROS still reaches FCU through the transport boundary, never directly.

Prerequisites: a successful **closed live-camera DISARMED fusion run of this
source revision**, matching native build/candidate and pinned VIO export; Linux
NVIDIA renderer/mount verification; `pymavlink` on the Linux host for BIN decoding.
Static synthetic fusion, capture-only success or a handwritten PASS report cannot
substitute. The host re-decodes the raw BIN; transport verifies the approved host
attestation, evidence hashes and semantic checks. This assumes a trusted host
operator; it is not cryptographic attestation against a malicious administrator.

Commands from the repository root (not executed on Mac):

```bash
set -euo pipefail
: "${P1_BASELINE:?closed DISARMED run under integration/phase1_runs}"
: "${P1_BUILD:?matching native exports under integration/linux_gpu_runs}"
: "${P1_VIO:?existing pinned export under integration/linux_gpu_runs}"
python3 -c 'import pymavlink; print(pymavlink.__version__)'
python3 -B integration/simulation_flight.py \
  --run "$P1_BASELINE" --bin "$P1_BASELINE/state/fcu/logs/00000001.BIN"
flight_args=(--run-id 20261117-flight01 \
  --runtime-candidate "$P1_BUILD/candidate.json" --vio-dir "$P1_VIO" \
  --fusion-proof "$P1_BASELINE/evidence/flight_fusion_proof.json" \
  --origin=-35.363262,149.165237,584)
# Offline admission only; no Docker/runtime/ARM:
python3 -B integration/run_simulated_flight.py "${flight_args[@]}"
# ONLY AFTER SEPARATE EXPLICIT APPROVAL:
python3 -B integration/run_simulated_flight.py "${flight_args[@]}" \
  --execute-simulation-flight
```

Use the actual BIN filename if the completed baseline generated another one.
Proof output is new `evidence/flight_fusion_proof.json`; old evidence is not
overwritten. Changed code/evidence/builds require renewed verification, not edits
to a PASS flag. Missing dependencies/proofs fail before Docker or FCU connection.

Before ARM, the new run also needs a read-only **current-boot DataFlash witness**
of ExternalNav aiding, source set0 and healthy innovations, at least5s VPE/VSE,
and a fresh applied-clock comparison. Immutable BIN snapshots/rejected attempts
are retained. Buffered DataFlash (>500ms FCU age) holds ARM closed; Linux must
verify whether its logger/decoder meets this bound. Live origin/HOME, all arming
checks (`ARMING_CHECK=1`), EKF status/innovation ratios, acquisition timestamps,
covariance and DISARMED heartbeat are independently checked. GPS/EKF/arming
parameters are re-read after GUIDED and before ARM. Native ARM ACK and armed
heartbeat are required before TAKEOFF.

Fixed scenario: target1m, hard altitude1.5m, horizontal drift0.4m, hover5s;
preflight45s, command5s, climb15s, LAND30s within an immutable105s controller
deadline. Existing 180ms publication, 200ms clock and 500ms availability limits
remain. A 4m visual-only checker with0.02m tiles extends texture coverage; its
flight performance is NOT VERIFIED. GT height remains a declared test aid;
horizontal GT/SITL position is never fed into VIO. FCU position telemetry is
used only for flight-envelope monitoring.
The source has a bounded110s capture/export window (within its existing buffers);
after confirmed LAND/disarmed the host allows that export to finish.

VIO/clock/gate loss closes vision and requests LAND without replay or automatic
reinitialization. No force-arm, airborne disarm, arming-check override or flight
termination command is available. LAND requires ACK, LAND heartbeat, on-ground
state and disarmed confirmation; failure/transport loss leads to owned simulation
STOP. Parent timeout/crash is not an independently verified recovery guarantee;
no Stage4/systemd mechanism is selected. Separate approval must cover the Linux
administrator's exact-ID emergency STOP procedure.

Evidence: `flight-host.jsonl`, `flight-owned.json`, `current-fusion-*.BIN`,
`flight-transport/{flight.jsonl,flight_result.json,wire.jsonl,fcu_tcp.bin}`,
`flight-source/`, `flight-observer/` and final FCU BIN, under the new run's
`evidence/` or `state/fcu/logs/`. Controller telemetry PASS is not Phase1
completion: inspect post-flight source selection, innovations, VIO accuracy/
covariance and cleanup. No simulator was armed/flown by this implementation.
Phase1 remains HOLD / NO GO.
