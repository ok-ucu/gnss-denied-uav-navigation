# One Linux GPU source-continuity experiment

**HOLD; execution_allowed=false; forwarding_enabled=false.** This is a static,
DISARMED source-only diagnostic. No MAVROS, ExternalNav, navigation-source
changes, movement, world pause/resume, simulator restart or image rebuild.
It does not establish full Phase1 acceptance or capture-to-FCU latency.

The earlier manual sequence in [the review](reports/VIO_ISOLATION_20261113.md)
is superseded by `run_vio_source_probe.sh`. That sequence lacked native
clock/IMU/resource witnesses outside ON and an executable full-window verdict.

## Required existing environment

- A separately started **matched physics + FCU** runtime is required. The old
  `gnss-sim-gazebo`/`gnss-sim-fcu`, partition `gnss-sim` names were a ZIP-only
  prerequisite, not a repository startup procedure. Do not rename containers.
  The supported integration-owned startup below reuses `camera.compose.yaml`
  with NEW `gnss-gpu-physics-<run-id>` / `gnss-gpu-fcu-<run-id>` containers,
  partition/network `gnss-gpu-<run-id>`, 3 CPU physics, shared loopback JSON9002
  and passive TCP5760, no host-published ports. State/evidence is unique per run.
  Only new FCU SERIAL0/LOG_DISARMED/SYSID are set; no navigation-source change.
- ROS2Jazzy, ros_gz_bridge, gz-sim8/transport13/msgs10, NumPy, OpenCV, `setsid`,
  `ps` and Python3 in the Gazebo container; its existing `/opt/integration`
  bind mount must expose this worktree. No pip/apt installation is performed.
- Transfer the current integration files, especially `isolated_imu.py`,
  `vio_sensor_capture.py`, `run_vio_stream.py`, `vio_source_probe.py` and
  `run_vio_source_probe.sh`. Also required: existing `observe_frame_bridge.py`,
  `stage_timing.py`, `vio_ground_fixture.py`, runner imports,
  `linux_gpu/{verified_disarmed,prepare_vio,run_source_runtime}.py`,
  `mavros_live_runs/20261108/mavlink_v20.py` and
  `isolated_blockers/20261109_a/code/mount_math.py`. They exist locally; the
  protocol/mount helpers are tracked, not hidden raw-log dependencies.
- Pinned VIO object `d33f66008fd7b6de332c96dd324b8ddbd320c194` must be in the
  Linux repository. `prepare_vio.py` exports four files and verifies their
  exact hashes; it never writes `vio/`. If the object is absent, stop and obtain
  that dependency explicitly. Do not substitute unverified VIO sources.
- Camera mount must already be correct; preflight measures all three axes
  <=1degree over5s, saves the lever arm and fails otherwise. It sends no gimbal
  commands. No existing `integration_vio_ground_checker` fixture may be present:
  creation uses `allow_renaming:false`; failure never removes another fixture.
- ROS_DOMAIN_ID123 must be unused by source/clock publishers. Preflight checks
  discovery for2s and capture records publisher GIDs; use a different unused
  domain explicitly if needed. Existing services are not stopped.

The GL identity file must come from the active Gazebo render context. The
preflight accepts its actual `renderer=` format (also `GL_RENDERER=`) and rejects
llvmpipe/software/WARP. Retain identity plus container start metadata; a stale
identity file alone is not fresh hardware verification.

## Evidence reviewed on 14 November 2026

Archive `phase1-evidence-35355f4-FINAL.zip` SHA256:
`059c8fcf4e5bdb4c965207fff95fa41c3055e8e571a4115fc0b8a95f6d8afc00`.
The saved `camera02/evidence/disarmed-before.txt` (under `20261114-camera02/`)
shows strict decoding failed at generated codec line22720 on prefix108 (0x6c,
ASCII l). This is not a MAVLink frame prefix. The archive has no raw TCP bytes
or saved robust-check result; console text versus mid-stream framing cannot be
resolved from the traceback. BAD_DATA is never a valid heartbeat.

The separate `nvidia-diagnostic/result-fixed.txt` identifies Quadro T2000/NVIDIA,
but **camera02's own Ogre log says llvmpipe** and its metadata lacks explicit EGL
vendor selection. The separate diagnostic does not prove camera02 used a GPU.
TIMESYNC's seven positive result files agree with suite.json; bad_domain records
clock_domain_status=FAIL as expected. This is artificial-peer replay, not FCU
clock mapping. The archive contains only VIO DEPENDENCY.json, no source run.
Both camera02 containers were stopped in owned-final metadata. No restart here.

## Exact Linux sequence: startup first, probe second

Use the committed revision from [the Linux handoff](LINUX_GPU_HANDOFF.md).
Set PHASE1_SOURCE_SHA to the full SHA supplied with that handoff, then verify it
below. The historical evidence baseline remains35355f4; it is not the new code
revision. Do not merge branches or apply the old teammate ZIP. Use a fresh run
ID; preparation also reserves a directory and cannot be reused for execution.
No build/pull is performed by the launcher.

```bash
set -euo pipefail
# Run from the repository root on Linux NVIDIA, not Mac llvmpipe.
test "$(git branch --show-current)" = gazebo-sitl-integration
: "${PHASE1_SOURCE_SHA:?Set the full commit SHA from the Linux handoff}"
test "${#PHASE1_SOURCE_SHA}" -eq 40
test "$(git rev-parse HEAD)" = "$PHASE1_SOURCE_SHA"
git status --short
git cat-file -e d33f66008fd7b6de332c96dd324b8ddbd320c194^{commit}
test -f integration/linux_gpu/verified_disarmed.py
test -f integration/linux_gpu/run_source_runtime.py
docker compose version
docker image inspect gnss-phase1-camera:linux-gpu gnss-phase1-sitl:linux-gpu
# If either image is missing, STOP. Build using LINUX_GPU_HANDOFF.md first.
# If the VIO object is absent, obtain that exact dependency from its owner.
# Do not use current vio/ as a fallback. Four files are exported and hash-checked.
run_id=20261115-source1
mkdir -p integration/linux_gpu_runs
# This vendor path is INSIDE the NVIDIA container; override it explicitly if
# the host's NVIDIA runtime injects its vendor JSON at another verified path.
python3 -B integration/linux_gpu/run_source_runtime.py \
  --run-id "$run_id" --ros-domain 123 \
  --egl-vendor-file /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  --execute --probe
```

The **separate parent startup** starts only two NEW containers, checks linux/amd64
images, firmware commit/checksum/parser-test record, plugin and exact camera SDF,
network namespace/internal network, mounts, domain, active NVIDIA context and
Ogre corroboration. It sets the reviewed gimbal topics and measures all three
axes <=1degree using timestamped native poses. It runs native camera/IMU/clock/
pose topic inspection and a ROS topic inventory. No ROS `/clock` bridge is
started here: source probe must own it. Intrinsics remain640x480, horizontal
FOV2rad, fx205.4696, configured10Hz; scene/physics/rate are unchanged.

Only after startup PASS does the parent invoke the existing probe as a separate
`docker exec`, bounded to240s. The actual internal commands are recorded in
`evidence/source-runtime-commands.jsonl`, including:

```bash
# These are performed automatically on the NEW physics container, BEFORE probe:
python3 -B /opt/integration/linux_gpu/verified_disarmed.py \
  --expected-system 1 --expected-component 1 \
  --directory /run/integration/evidence/source-disarmed-before
# Then, after runtime preflight PASS:
bash /opt/integration/run_vio_source_probe.sh RUN_ID /run/integration/artifacts/vio
```

`--execute` authorizes only bounded DISARMED startup; it never sets the navigation
execution flag. Omitting `--probe` checks startup then stops, without VIO. Omitting
`--execute` only prepares/validates Compose; use another ID for an actual run.
The parent never renames/reuses existing containers. A name/network collision,
missing dependency, software renderer, wrong mount/domain/namespace or missing
DISARMED evidence causes STOP and HOLD. Do not rerun merely to obtain a PASS.

The expected FCU is explicit system1/component1 on this newly created isolated
endpoint (SYSID_THISMAV=1; autopilot component1). The new receive-only checker
uses robust framing with CRC, **discards and counts BAD_DATA**, requires MAVLink2,
ArduPilot/quadcopter role, >=3 DISARMED heartbeats spanning >=2s and no gap>3s.
Wrong identity/role/version, any armed heartbeat or stale state latches failure.
It saves raw TCP chunks, monotonic arrival/offsets and decoded packet bytes.
CRC/source checks are protocol validation, **not cryptographic authentication**.
Trust also depends on the isolated namespace and known owned FCU. The continuous
native witness uses the same policy throughout OFF/ON/OFF. The legacy strict
`disarmed.py` remains unchanged for historical reproducibility.

## Results, collection and owned cleanup

The parent always attempts log collection and stops only matching Compose-owned
container IDs in `finally`, including probe failure/timeout/Ctrl-C. It leaves
containers, network, state and all evidence intact; no `down`, removal or old
container restart. On Docker/host failure or SIGKILL, cleanup cannot be promised:
inspect IDs/status and report cleanup FAIL; do not kill by shared names.

```bash
out="integration/linux_gpu_runs/$run_id"
cat "$out/evidence/SOURCE_RUNTIME_RESULT.json"
# Created even on prerequisite/probe failure after runtime invocation.
# If probe ran, inspect the independent subscriber and full-window verdict:
test ! -f "$out/evidence/source-probe/SOURCE_RESULT.json" || \
  cat "$out/evidence/source-probe/SOURCE_RESULT.json"
docker inspect "gnss-gpu-physics-$run_id" "gnss-gpu-fcu-$run_id"
```

Required evidence: SOURCE_RUNTIME.json, resolved Compose/env, command/exit log,
owned-before/final snapshots (exact IDs and stopped state), firmware and renderer
checks, DISARMED tcp.bin/events/result before/after, mount/native topic preflight,
and the full copied `evidence/source-probe/` directory when run. That contains
independent `observer/input.jsonl` and actual ROS camera, IMU and `/clock` traces.
`source-ros-before.txt` should have no source or `/clock` publisher; live ROS
message validity is checked during the probe, not inferred from this inventory.

`SOURCE_RUNTIME_RESULT.json` separates startup PASS/BLOCKED, VIO PASS/FAIL/
NOT TESTED and cleanup PASS/FAIL. Any startup BLOCKED or cleanup FAIL stops the
sequence. A valid source PASS still leaves overall HOLD. Preserve failure logs
and raw stream even when no source directory exists. Collect a reviewed explicit
allowlist with [the existing ZIP tool](LINUX_EVIDENCE_REVIEW.md); exclude FCU
state/BIN and generated build trees. No automatic transfer or Git operation.

The launcher measures preflight, creates one visual-only1m fixture at z=.001m,
then captures **OFF10s → ON60s → OFF10s**. Fixture, camera settings, physics and
resource limits remain fixed across phases. It starts one camera ROS bridge,
one independent `observe_frame_bridge.py` subscriber and the existing source
with `--isolated-imu --profile --async-diagnostics --reliable-camera
--min-odom-subscribers 1`. These are actual separate processes, not mocks.
The source's own bridge is the only intended ROS `/clock` publisher in this
domain. Source startup/shutdown/export are labelled within ON; evaluate exact
monotonic boundaries, not nominal sleeps. Cleanup removes only a fixture whose
creation succeeded here, stops only owned process groups, and rechecks sustained expected-source DISARMED.
It does not stop containers; only the separate parent owns container cleanup. Empty observer/output.jsonl is expected: no frame
bridge is launched. observer/input.jsonl is the independent Odometry evidence.

## Required artifacts and time domains

|Artifact below run/|Purpose|
|---|---|
|preflight.json, mount_samples.json, mount.json|Source/VIO hashes, renderer, ROS domain/partition, measured mount/lever|
|phases.jsonl|OFF/ON/OFF boundaries in Linux monotonic seconds|
|native.jsonl, witness.json|Native camera acquisition labels and arrival times, native IMU, simulation clock, passive DISARMED heartbeat; witness errors/capacity|
|resources.json|Aligned cgroup CPU/throttling/pressure and per-process CPU ticks + process-leader schedstat, throughout all phases|
|source/ros_camera.jsonl, native_camera.jsonl|Acquisition-label joins between native arrival and ROS callback|
|source/imu.jsonl, native_imu.jsonl, clocks.jsonl|Raw acquisition/receipt times; IPC receipt/sequence in isolated rows|
|source/capture_meta.json|IPC age/batch metrics, worker health, overflow counts and clock publisher GIDs|
|source/stages.jsonl|Camera queue insertion/dequeue/replacement, callback/consumer, VIO stages, actual thread CPU and runqueue deltas where available|
|source/stream.jsonl|Validation-time clock details, publication age, IMU backlog, processing wall/CPU, quality/covariance and errors|
|source/publications.jsonl, observer/input.jsonl|Independent source-to-receiver timestamp matching and delivery latency|
|source/process_load.json, configuration.json, summary.json, gate_transitions.json|Process load, configuration, resets/gaps, gate state changes|
|*.log, collection.exit, launcher.exit, SOURCE_RESULT.json|Process/cleanup status and source verdict including final silence|

All host timestamps inside this container are Linux CLOCK_MONOTONIC; simulation
acquisition labels remain separate. Join camera samples by nanosecond-rounded
acquisition label and host events by monotonic time. Do not subtract simulation
seconds from host seconds. Native arrival is **not sensor-generation time**;
ROS bridge internal receive/publish is not instrumented. A negative difference
between two independent arrival witnesses indicates callback scheduling/order,
not negative physical latency. Process-leader schedstat is not all-thread CPU;
source critical-stage counters measure the actual consumer thread separately.
Wall minus thread CPU does not identify a particular lock or scheduler cause.
FCU container CPU is not part of this cgroup trace; inspect snapshots record its
limits, not a time-resolved FCU scheduler profile.

## Explicit verdict

`SOURCE_RESULT.json` returns PASS only for the **source-continuity** gate:

- Complete successful collection/cleanup, >=60s capture, readiness within10s,
  followed by >=50s observed operation. Startup is separately reported.
- Native camera remains640x480 with three bytes per pixel; no resolution/rate
  configuration is changed. Cgroup CPU evidence is available in every phase.
- At least two publications; independent input stamps match all publications
  exactly and in order, with no missing/duplicate/invalid messages.
- Every publication passes actual gate/frame/covariance checks and unchanged
  <=180ms simulation publication age; observer requires <=200ms simulation age
  and <=500ms clock receipt age. Future/nonfinite stamps fail validation.
- Both publication and independent reception gaps <=500ms, **including the
  tail from the last message to capture end**. Boundaries/maximum outage and
  deadline are recorded, not excluded as an unsuccessful segment.
- Worker healthy, stopped, no critical IMU gap>20ms, no reset/recovery, no
  buffer/profile loss, no unexplained post-readiness clock error, exactly one
  observed clock publisher, ordered raw/IPC data. Native `/clock` duplicates
  may be recorded; reversal is a failure.
- No message while the logged gate is blocked. Any incomplete evidence,
  process crash, failed cleanup or missing subscriber produces FAIL.

Fault injection/restart safety is **NOT TESTED** by normal OFF/ON/OFF. Offline
worker regressions cover failure cases but cannot establish live fault safety.
The200ms FCU/physics clock watchdog and permission boundary are not exercised
here; no transport authorization exists. No TTL or safety threshold is changed.
Even source PASS leaves overall decision **HOLD**, with forwarding/execution
false. Camera simulation/wall rates, actual clock progress, CPU/queue/IPC timing,
coverage and gaps are reported separately; source PASS is not a10–20Hz full
Phase1 rate/latency acceptance or proof that RTF was fixed.

## Decision tree after this one run

1. **Prerequisite/collection FAIL:** fix the named dependency, domain, mount,
   observer or evidence error. Do not infer estimator performance from a partial
   capture. A required file missing from the Linux mount is a handoff dependency.
2. **Native clock + camera/IMU arrival gaps coincide:** inspect native sim-label
   increments, CPU/throttling and per-process counters in the same window.
   Separate paused/stalled simulation from delayed witness scheduling; arrival
   traces alone do not prove Gazebo stopped generating data.
3. **Native streams continuous, ROS/IPC old:** use original callback age versus
   IPC receipt age and sequence. Callback delay suggests bridge/DDS/executor;
   fresh callbacks with old IPC indicate producer/drain/IPC scheduling. Neither
   conclusion is established by RTF alone.
4. **Callbacks/IPC fresh, consumer late:** join queue-dequeue/consumer and CPU/
   runqueue spans. Distinguish queue wait, measured CPU and unresolved blocking.
5. **Source continuous, observer missing/invalid:** inspect DDS discovery/QoS,
   clock ordering, observer age and exact missing stamps. Local counters do not
   qualify as successful delivery.
6. **All source checks PASS:** next task is reviewed amd64 firmware + TIMESYNC
   overlay admission, then continuous transport to both capture wire channels.
   The independent permission-deadline blocker remains; FCU forwarding still
   requires explicit approval and all preflight gates.

One additional offline finding: validation previously compared a newly received
clock against the *earlier frame-entry time*. A callback occurring during the
buffer snapshot could be falsely future-dated. Validation now samples host time
after the snapshot and records it; thresholds and acquisition labels are intact.
The regression reproduces this ordering case. It does **not** identify the
historical798.499ms gap's cause, which remains a live-measurement question.

Mac validation:155offline tests PASS (including10source-probe regressions);
10source-probe and8unchanged isolated-worker tests also PASS directly without
ROS. Python/shell syntax, local links and pinned VIO objects/hashes checked.
Linux live execution and the798.499ms blocker remain **NOT TESTED/unresolved**.
