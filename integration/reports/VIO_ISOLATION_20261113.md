# VIO ingestion review — 13 November 2026

**HOLD; execution_allowed=false; forwarding_enabled=false.** No simulation,
flight, ExternalNav, commit, push or branch integration was performed here.

## Actual input and next blocker

Found `gazebo-sitl-integration_2026-11-11.zip` and
`WORK_REPORT_2026-11-11.md` in Downloads, without `(1)`. The embedded and external
reports match. ZIP SHA256:
`06ef01d68b7d96e46934ef93c3d2c4b48871363ebfaf1afca4d2d43cec7811df`.
CHANGES.patch contains the isolated-IMU update, not just the older camera fixes.
Recorded source hashes match the two ZIP VIO integration files. The current
worktree started at5cdbb7a, without any teammate ZIP changes applied.

Next demonstrated blocker is **continuous source availability**: publication
at t49.2/host492.906892540s is followed by t49.7/host493.705391556s — an actual
**798.499016ms publication gap**, exceeding the unchanged500ms host watchdog.
The isolated source was not connected to transport; this is not an observed
transport latch, but would fail its availability criterion. At t49.3 the source
reported `stale/uncomparable clock`; readiness reopened at49.7. That failed row
omits clock stamp/receipt age/IPC age, so the precise clock-failure cause is not
recoverable from supplied evidence. No raw IMU/clock/process-load/callback
traces are included for this run. The RTF root cause is likewise unresolved.

## Measured domains, not interchangeable quantities

Recomputed from supplied12Nov stream/publications (one static40s run):

|Quantity|Evidence|
|---|---|
|Consumed camera frames|275; acquisition intervals exactly100ms;10Hz simulation|
|Camera labels /consumer wall progress|6.87673Hz wall, ratio0.687673; not an independent physics-clock RTF trace|
|Local Odometry publications|259/275=94.18%; publication log agrees with source flags|
|Independent delivery|NOT TESTED: all published rows have odom_subscribers=0|
|IMU processing wall p50/p95/p99/max|21.206/45.246/69.981/83.438ms,266 successful processing rows|
|Total processing wall p50/p95/p99/max|54.138/92.198/117.579/129.983ms,266 rows;9 failed rows omitted this field|
|Frontend /spatial audit wall p95|22.753/17.747ms; not additive percentile budgets|
|Consumer→publish wall p95/max|89.916/127.302ms; same host monotonic clock|
|Source age at image callback|p50/p95/max15/22/52ms **simulation**, not host latency|
|Estimator-output age|p50/p95/max52/67.75/100ms simulation, not exact publication age|
|Odometry wall gap p95/max|188.674/798.499ms|
|Errors|8warmup,1stale clock; source summary0IMU gaps/0recoveries|
|Callback→consumer /IPC delay /thread CPU|NOT MEASURED in supplied run|
|Capture→FCU latency /wire deadline|NOT TESTED; no MAVROS or FCU route|

92ms host processing is not directly comparable to a100ms simulation period:
at measured ratio, nominal wall period is about145ms. No per-frame linear clock
mapping or capture→FCU latency is inferred from that ratio. The full-run gap is
not hidden by calculating metrics only on successful segments.

The teammate's0→259 and590→21ms result strongly supports separating callback
execution from the estimator. Original raw0-publication baseline and GIL/CPU
profiles are absent; exclusive attribution to GIL ownership was not measured.
Residual parent streams still include native IMU witness, native poses and the
camera node's simulation TimeSource. Their contribution is a hypothesis, not a
reason to remove safety checks or sensor witnesses. Camera-only quota and free
running GPS flight also have different workloads; RTF0.69 cannot be assigned to
one stage from those comparisons.

## What was integrated locally

Adopted the teammate's opt-in **spawned IMU+/clock receiver**, retaining default
in-process behavior. `vio/` and all teammate directories remain unchanged.
The ZIP's blocking Pipe/unbounded pending lists were not copied literally:

- `isolated_imu.py`: bounded32-batch channel,≤128rows/batch; queue full, bad
  sequence/session, reset/unordered IMU/clock, worker startup failure/exit latch
  unhealthy. Acquisition and callback timestamps remain unchanged.3ms is only
  the worker flush target, never a delivery guarantee.
- `vio_sensor_capture.py`: opt-in process integration, checked reads, IPC metrics,
  original callback ages, clock-publisher GID history, explicit shutdown/reaping
  and failure recording. Normal shutdown does not flush an indefinite tail;
  produced/consumed batch counts disclose it. No decimation/fabricated samples.
- `run_vio_stream.py`: late IPC cannot rejuvenate IMU watchdog time; worker
  health is checked before publishing. Clock diagnostics separate behind/ahead
  simulation time from stale/future host receipt; processing and IMU thread CPU
  recorded separately from wall time. Failed frame timing is retained too.
  Publication limit180ms, clock200ms and availability500ms remain unchanged.
- `launch_phase1.py`, `run_phase1_validation.py`, `run_latency_matrix.py`:
  explicit `--isolated-imu` forwarding; default is unchanged. This does not make
  the ARM64 launcher valid on amd64.
- `tests/test_isolated_imu.py`:8offline regressions, including actual spawned
  fixture process, backpressure, ordering/timestamps, exit/start failure, closed
  reads, shutdown, clock diagnostics and CLI parsing. Fixtures are synthetic
  IPC tests, not ROS/live sensor or flight verification.

Full145offline tests PASS in an existing ARM64 image, no simulator. Focused8
also PASS directly on Mac without ROS. Live ROS worker/QoS and performance of
this hardened variant are **NOT TESTED** until the Linux rerun.

Other reviewed ZIP changes remain ZIP-only: WSL D3D12 mode, root Git ownership
build fix, startup render/heartbeat readiness, analysis fallback, CPU-fallback
renderer rejection, build-job controls and developer sim wrappers. Bring over
the small integration-owned WSL/build/readiness fixes when rebuilding/running
those paths; do not apply the entire patch (it changes gazebo/ and adds flight
smoke commands). Existing Linux images/stack already exercised them. `sim.sh
smoke` and cleanup/prune commands from the teammate report are not authorized.

## amd64 path: exact remaining admission work

1. `launch_phase1.py` and `phase1.compose.yaml`: replace ARM64-only local image
   assumptions with a reviewed architecture/runtime manifest and explicit GPU
   override. Default Compose still forces software rendering; WSL support only
   in the ZIP camera/developer workflow does not fix the full pipeline.
2. Keep the existing ARM64 firmware pin intact. Reported amd64 artifact hash is
   `d8c8aa1126156c7c124fddb0dc0908704e0b9779a5b9fc5e40d24fa39b5f0d36`;
   this review does **not** admit it. Obtain full image IDs/repo digests, source
   commit, exact patch hashes, toolchain and5/5actual parser test output. Validate
   its applied-clock trace/cross-check contract, then explicitly review a new
   provenance record in `physics_clock_mapping.py` and the launcher. AP4.4.4
   source identity alone is not proof of binary timing behavior.
3. `phase1_transport.py`: use the same reviewed firmware record/path as launcher;
   do not independently accept a user-provided hash. Preserve clock brackets,
   durable epoch checks and post-MAVROS final interlock.
4. `MAVROS.isolated.Dockerfile` requires missing legacy
   `gnss-gazebo-ardupilot:dev`; use a reviewed amd64 camera/base image, verify
   actual available2.16.0 package builds, and record libmavconn/MAVROS versions.
   `build_mavros_time_patch.sh` must use that exact new builder and a NEW output
   prefix. Rebuild the existing signed TIMESYNC package, then repeat isolated
   ±offset/jitter/drift/reset tests before granting any capture permissions.
5. Configure image/binary/overlay mounts and `/tmp/patch_install` prefix
   consistently. Never copy the ARM64 overlay into amd64 or remove provenance
   checks to get past preparation. These changes need Linux build/runtime
   evidence; no binary whitelist or safety parameter was changed here.

## One next experiment: source-only static Linux run

**Updated reproducible entry point:** use [VIO_SOURCE_PROBE.md](../VIO_SOURCE_PROBE.md)
and `run_vio_source_probe.sh`. The manual sequence below is superseded: it did
not capture native clock/IMU/resources throughout OFF and did not enforce a
full-window verdict with independent delivery and trailing silence. No live
result is added by this update. The isolated worker is unchanged.

Use the already verified `gnss-sim-gazebo`/`gnss-sim-fcu` stack, confirmed
DISARMED, unchanged physics/GPS/EKF, real GPU and corrected mount. No MAVROS.
Transfer only the updated integration sources first (`isolated_imu.py`,
`vio_sensor_capture.py`, `run_vio_stream.py`); this Mac work is uncommitted.
Create a fresh output ID. Use a dedicated ROS domain for owned diagnostic
processes, keeping the same native `GZ_PARTITION=gnss-sim`. This avoids competing
clock bridges without stopping the teammate's services. Check graph history:
exactly one advancing `/clock` publisher is the intended configuration.

Run10s source-OFF native camera/resource capture, then60s source-ON with an
independent subscriber and profiling, then10s source-OFF again. Same fixture,
camera, native partition and resource limits throughout. This single controlled
experiment tests load correlation, not a full Phase1 acceptance. CPU time and
callback/IPC/clock evidence decide the next fix; no speculative QoS/rate change.

On Linux, from the worktree root (stop if prerequisites or fresh directories fail):

```bash
python3 -B integration/linux_gpu/prepare_vio.py --output integration/linux_gpu_runs/vio-20261113
docker exec gnss-sim-gazebo python3 -B /opt/integration/linux_gpu/disarmed.py
docker exec gnss-sim-gazebo cat /tmp/render_gl_identity.txt
```

Then run the following in that container via
`docker exec -e ROS_DOMAIN_ID=123 -e GZ_PARTITION=gnss-sim gnss-sim-gazebo bash`.
Only new diagnostic processes and `/tmp` files are created; do not run `smoke`:

```bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
review=/tmp/gnss-vio-20261113
mkdir "$review"
mkdir "$review/evidence"
export REVIEW_DIR="$review"
# Read-only pose measurement: no world pause/resume or gimbal commands.
python3 -B - <<'PY'
import json, math, os, sys, time
from pathlib import Path
import xml.etree.ElementTree as ET
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
sys.path.insert(0, '/opt/integration/isolated_blockers/20261109_a/code')
from mount_math import measure_mount, mount_pass
sensor = ET.parse('/opt/ardupilot_gazebo/models/gimbal_small_3d/model.sdf').find('.//sensor[@name="camera"]')
p = list(map(float, sensor.findtext('pose').split()))
r, pitch, y = p[3:]
cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(pitch),math.sin(pitch),math.cos(y),math.sin(y)
rs = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
samples = []
def pose(msg):
    if samples and time.monotonic()-samples[-1]['arrival_monotonic_s'] < .1:
        return
    def entry(name):
        found = [v for v in msg.pose if v.name == name]
        if len(found) != 1:
            raise ValueError('Ambiguous/missing pose: '+name)
        v = found[0]
        return dict(position=[v.position.x,v.position.y,v.position.z],quaternion_xyzw=[v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w])
    row = measure_mount(*(entry(n) for n in ('iris_with_standoffs','gimbal','pitch_link')),rs,p[:3])
    samples.append(dict(row,stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nsec,arrival_monotonic_s=time.monotonic()))
node=Node(); topic='/world/iris_runway/dynamic_pose/info'
assert node.subscribe(Pose_V,topic,pose)
time.sleep(5); node.unsubscribe(topic); time.sleep(.15)
directory=Path(os.environ['REVIEW_DIR'])/'evidence'/'integrated'
directory.mkdir()
(directory/'mount_samples.json').write_text(json.dumps(samples,indent=2)+'\n')
assert len(samples)>=10 and samples[-1]['stamp_ns']>samples[0]['stamp_ns'] and mount_pass(samples), 'HOLD: mount not measured within1degree'
summary=dict(mount_status='PASS',settled_snapshots=len(samples),max_angles_deg=[max(v['angles_deg'][i] for v in samples) for i in range(3)],final_lever_body_flu_m=samples[-1]['lever_body_flu_m'])
(directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
PY
python3 -B /opt/integration/vio_ground_fixture.py create --directory "$review/fixture" --size-m 1 --tile-size-m .02
cleanup() {
  for pid in $(jobs -pr); do kill -TERM "$pid" 2>/dev/null || true; done
  wait || true
  python3 -B /opt/integration/vio_ground_fixture.py remove --directory "$review/fixture"
}
trap cleanup EXIT
camera=/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge "$camera@sensor_msgs/msg/Image[gz.msgs.Image" --ros-args -r __node:=review_camera > "$review/camera.log" 2>&1 &
python3 -B /opt/integration/linux_gpu/sample_camera.py 10 "$review/before"
python3 -B /opt/integration/observe_frame_bridge.py --directory "$review/observer" --duration 70 > "$review/observer.log" 2>&1 &
observer=$!
python3 -B /opt/integration/linux_gpu/sample_camera.py 70 "$review/during" > "$review/sampler.log" 2>&1 &
sampler=$!
python3 -B /opt/integration/run_vio_stream.py --vio-dir /opt/integration/linux_gpu_runs/vio-20261113 --output-dir "$review/source" --duration 60 --ground-z .001 --mount-calibration "$review/evidence/integrated/summary.json" --profile --async-diagnostics --reliable-camera --isolated-imu --min-odom-subscribers 1 > "$review/source.log" 2>&1
wait "$observer" "$sampler"
python3 -B /opt/integration/linux_gpu/sample_camera.py 10 "$review/after"
```

Copy `/tmp/gnss-vio-20261113` to a NEW host ignored run folder with `docker cp`
after cleanup. `observer/input.jsonl` is the independent `/vio/odom` evidence;
empty output.jsonl is expected because no frame bridge is launched in this
source-only test. Record actual stage windows from monotonic stamps, including
startup/shutdown, not just nominal command durations. Export `process_load.json`,
capture_meta, raw IMU/clock/native IMU, ros_camera/native_camera and stages,
clock-publisher history, gate transitions and before/during/after sampler files.
Never call consumer→publish timing capture→FCU latency.

Success for A requires independent valid messages, no unexplained clock event,
no critical IMU gaps/IPC errors, bounded buffers, no stale publication and no
post-readiness gap>500ms across the entire window; report startup separately.
If it fails, preserve the first clock_check, publication check and IPC records;
do not lower readiness/freshness limits or automatically reset durable state.

## Dependency order to Phase1 acceptance

A. Stable source + independent ROS observer and clock/IPC evidence above.
B. Reviewed amd64 provenance + rebuilt TIMESYNC overlay, then continuous normal
real-VIO→MAVROS→both capture wire channels, final permissions valid; still no
FCU vision forwarding. Historical17.12ms enqueue budget issue remains untested.
C. Only after critical preflight/guard tests and explicit approval, a matched-
physics DISARMED live-camera EKF3 test: aligned time/origin/yaw/IMU, source
selection and innovations/rejections in DataFlash, not VISP/VISV counts alone.
D. Complete camera/IMU/tracking loss, stale, pause/reset, process/MAVROS restart
fault/recovery matrix with zero invalid wire leakage and explicit new reference
initialization after reset. Basic guard failure tests are prerequisites to C;
full matrix acceptance is required before flight.
E. Separately authorized GNSS-denied flight only after A–D and flight preflight.
GPU rendering and earlier GPS-backed flight do not satisfy C/E.

Local comparison/metric/test artifacts are under ignored
`integration/source_scope_audit/teammate-debug-20261113/`. Original ZIP, teammate
report and historical logs are unchanged. No live improvement is claimed here.
