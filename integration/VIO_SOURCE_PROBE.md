# One Linux GPU source-continuity experiment

**HOLD; execution_allowed=false; forwarding_enabled=false.** This is a static,
DISARMED source-only diagnostic. No MAVROS, ExternalNav, navigation-source
changes, movement, world pause/resume, simulator restart or image rebuild.
It does not establish full Phase1 acceptance or capture-to-FCU latency.

The earlier manual sequence in [the review](reports/VIO_ISOLATION_20261113.md)
is superseded by `run_vio_source_probe.sh`. That sequence lacked native
clock/IMU/resource witnesses outside ON and an executable full-window verdict.

## Required existing environment

- The teammate's already running hardware-GPU `gnss-sim-gazebo` and DISARMED
  `gnss-sim-fcu`, native `GZ_PARTITION=gnss-sim`, passive TCP heartbeat5760.
  These developer Compose files remain ZIP-only locally. This launcher does
  **not** build/start them or make the full amd64 Phase1 launcher ready.
- ROS2Jazzy, ros_gz_bridge, gz-sim8/transport13/msgs10, NumPy, OpenCV, `setsid`,
  `ps` and Python3 in the Gazebo container; its existing `/opt/integration`
  bind mount must expose this worktree. No pip/apt installation is performed.
- Transfer the current integration files, especially `isolated_imu.py`,
  `vio_sensor_capture.py`, `run_vio_stream.py`, `vio_source_probe.py` and
  `run_vio_source_probe.sh`. Also required: existing `observe_frame_bridge.py`,
  `stage_timing.py`, `vio_ground_fixture.py`, runner imports,
  `linux_gpu/{disarmed,prepare_vio}.py`,
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

## Exact Linux command

Run once from the repository root with a **new** run ID. The unchanged simulator
stack must already meet the prerequisites above. New evidence paths refuse
overwrite; a second attempt needs another ID and a stated hypothesis.

```bash
set -euo pipefail
run_id=sourceprobe-20261113-01
vio_export=vio-sourceprobe-20261113-01
python3 -B integration/linux_gpu/prepare_vio.py --output "integration/linux_gpu_runs/$vio_export"
out="integration/linux_gpu_runs/$run_id"
mkdir "$out"
docker inspect gnss-sim-gazebo gnss-sim-fcu > "$out/containers-before.json"
set +e
docker exec -e ROS_DOMAIN_ID=123 -e GZ_PARTITION=gnss-sim gnss-sim-gazebo bash /opt/integration/run_vio_source_probe.sh "$run_id" "/opt/integration/linux_gpu_runs/$vio_export" 2>&1 | tee "$out/launcher-console.log"
result=${PIPESTATUS[0]}
set -e
docker cp "gnss-sim-gazebo:/tmp/gnss-vio-source-$run_id" "$out/run"
docker inspect gnss-sim-gazebo gnss-sim-fcu > "$out/containers-after.json"
printf '%s\n' "$result" > "$out/docker-exec.exit"
python3 -B integration/vio_source_probe.py analyze --directory "$out/run" --result-name SOURCE_RESULT.rechecked.json
exit "$result"
```

If `docker exec` cannot start, preserve the console/inspect evidence; there may
be no container run directory to copy. Do not rerun merely to obtain a PASS.
Do not disconnect the Docker client mid-run: remote exec may keep running. The
source is bounded to120s, witness180s, and owned-process cleanup is bounded.
Check collection/cleanup exit files, not just the Docker client return code.

The launcher measures preflight, creates one visual-only1m fixture at z=.001m,
then captures **OFF10s → ON60s → OFF10s**. Fixture, camera settings, physics and
resource limits remain fixed across phases. It starts one camera ROS bridge,
one independent `observe_frame_bridge.py` subscriber and the existing source
with `--isolated-imu --profile --async-diagnostics --reliable-camera
--min-odom-subscribers 1`. These are actual separate processes, not mocks.
The source's own bridge is the only intended ROS `/clock` publisher in this
domain. Source startup/shutdown/export are labelled within ON; evaluate exact
monotonic boundaries, not nominal sleeps. Cleanup removes only a fixture whose
creation succeeded here, stops only owned process groups, and rechecks DISARMED.
It does not stop containers. Empty observer/output.jsonl is expected: no frame
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
