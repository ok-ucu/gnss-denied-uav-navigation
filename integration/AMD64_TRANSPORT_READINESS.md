# amd64 runtime candidate and capture-only transport — 13 November 2026

**HOLD; execution_allowed=false; forwarding_enabled=false.** No amd64 image
build, native overlay load, GPU simulation or transport runtime was executed on
this Mac. Existing VIO changes/probes are preserved, not retuned.

## Audit and implemented boundary

The teammate's11Nov report and exported firmware-build record confirm a WSL
amd64 build from AP commit `9279df6191a772c098b2e4797c95ac954f258d88` and5actual
JSONParser tests. Reported binary SHA256 is
`d8c8aa1126156c7c124fddb0dc0908704e0b9779a5b9fc5e40d24fa39b5f0d36`.
The binary/image and its applied-clock cross-check proof are not supplied here;
that hash is **not admitted** to the runtime clock mapper.
First obtain the existing Linux image's read-only exports if available; do not
overwrite/rebuild its tag. The clean-build path below is for missing/incomplete
records or a fresh host. An older recipe's context mismatch remains HOLD for
separate review, not a reason to relabel its hashes as this new recipe.

The historical ARM64 pin remains
`047e4fbe21de2f88eba6c453e8aa3091ffd4b89b2b5568544f16cbf4c38d5970`.
`AppliedClockMapping` still rejects every other hash. No caller-provided hash,
ELF check, successful parser tests or candidate manifest can bypass this.

|Issue|Small change / remaining limit|
|---|---|
|SITL root Git query in docker-owned checkout|Collect commit/submodules/diff as the owner before USER root; no global safe.directory bypass|
|Large unpinned fetch / missing build records|Exact commit verified, shallow fetch with submodule fallback, bounded BUILD_JOBS; retain compiler/packages/patch/context/base/binary hashes and parser output|
|Cached ARM64 MAVROS base and architecture-specific Debian builds|NEW linux_gpu/MAVROS.Dockerfile uses public digest-pinned ROS base, explicit2.16.0 Debian versions, builds separate overlay at /tmp/patch_install|
|Physics image ENTRYPOINT sleep|Compose explicitly overrides entrypoint to bash; sleep bash... would fail with the Linux camera image|
|Hard-coded images/overlay/firmware path|Explicit image/overlay substitutions with historical defaults; exact firmware copied into new run/artifacts/arducopter and hash-bound to identity|
|Software rendering default|Default preserved; candidate-only dri/nvidia/wsl preview overrides; actual GPU evidence still required|
|Unreviewed native binary|runtime_candidate.py verifies build records/ELF and generates HOLD preparation only; launcher forbids --execute, transport rejects runtime_admitted=false|
|Hidden clock replay input|56KB fixture contains20unchanged historical ROS pairs plus clock; SHA/provenance retained under tests/fixtures; no historical logs edited|

The original `MAVROS.isolated.Dockerfile` remains the historical cached ARM64
recipe. Do not use it for native preparation. The old builder helper can now
take `PATCH_BUILDER_IMAGE` explicitly; the new recipe already builds the overlay
and needs no cached team image.

Capture routing is unchanged: FCU TCP5760 is private to the run's network
namespace; MAVROS binds UDP14891 with peer14890, where the capture interlock
receives candidate vision packets. Only TIMESYNC may cross its FCU relay;
read-only parameter/SYSTEM_TIME requests originate separately in the probe.
No vision packet is sent to FCU; no host ports are published. Guard storage
stays in the run-scoped persistent named volume and is not recreated on restart.

The official [ROS image manifest](https://github.com/docker-library/official-images/blob/master/library/ros)
lists Jazzy ros-base/noble for amd64 and arm64v8. MAVROS release2.16.0 also
declares2.16.0 in [libmavconn](https://github.com/mavlink/mavros/blob/2.16.0/libmavconn/package.xml)
and [mavros_msgs](https://github.com/mavlink/mavros/blob/2.16.0/mavros_msgs/package.xml).
This does not establish today's availability of those Debian builds: check on
Linux and HOLD if absent. No latest-version fallback is permitted.

## Linux preparation: build records, no simulation

Use the updated worktree, not the old5cdbb7a checkout alone. These commands need
internet for registry/apt/upstream dependencies and can be expensive; run outside
an acceptance window. They create only NEW images/containers/output directories,
never change working services. Choose a fresh ID and retain console failures.
The existing reviewed GPU physics image is a prerequisite; its full rebuild and
hardware checks remain described in [LINUX_GPU_HANDOFF.md](LINUX_GPU_HANDOFF.md).

```bash
set -euo pipefail
id=amd64-20261113-01
out="integration/linux_gpu_runs/$id"
mkdir "$out"
docker buildx imagetools inspect ubuntu:22.04 | tee "$out/ubuntu-index.txt"
docker buildx imagetools inspect ros:jazzy-ros-base-noble | tee "$out/ros-index.txt"
UBUNTU_BASE="ubuntu@$(awk '$1=="Digest:" {print $2; exit}' "$out/ubuntu-index.txt")"
ROS_BASE="ros@$(awk '$1=="Digest:" {print $2; exit}' "$out/ros-index.txt")"
[[ "$UBUNTU_BASE" =~ ^ubuntu@sha256:[0-9a-f]{64}$ ]]
[[ "$ROS_BASE" =~ ^ros@sha256:[0-9a-f]{64}$ ]]
docker run --platform linux/amd64 --name "gnss-p1-deps-$id" --entrypoint bash "$ROS_BASE" -lc 'apt-get update && apt-cache madison ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs ros-jazzy-libmavconn' > "$out/debian-versions.txt" 2>&1
MAVROS_DEB_VERSION=$(awk '$1=="ros-jazzy-mavros" && $3~/^2.16.0-/ {print $3; exit}' "$out/debian-versions.txt")
MAVROS_EXTRAS_DEB_VERSION=$(awk '$1=="ros-jazzy-mavros-extras" && $3~/^2.16.0-/ {print $3; exit}' "$out/debian-versions.txt")
MAVROS_MSGS_DEB_VERSION=$(awk '$1=="ros-jazzy-mavros-msgs" && $3~/^2.16.0-/ {print $3; exit}' "$out/debian-versions.txt")
LIBMAVCONN_DEB_VERSION=$(awk '$1=="ros-jazzy-libmavconn" && $3~/^2.16.0-/ {print $3; exit}' "$out/debian-versions.txt")
: "${MAVROS_DEB_VERSION:?HOLD unavailable2.16.0}" "${MAVROS_EXTRAS_DEB_VERSION:?HOLD unavailable2.16.0}" "${MAVROS_MSGS_DEB_VERSION:?HOLD unavailable2.16.0}" "${LIBMAVCONN_DEB_VERSION:?HOLD unavailable2.16.0}"
python3 -B integration/linux_gpu/prepare_context.py --kind sitl --output "$out/ctx-sitl"
python3 -B integration/linux_gpu/prepare_context.py --kind mavros --output "$out/ctx-mavros"
for tag in "gnss-p1-fcu-amd64:$id" "gnss-p1-transport-amd64:$id"; do
  if docker image inspect "$tag" >/dev/null 2>&1; then echo "Refuse existing tag: $tag"; exit 1; fi
done
docker build --pull --platform linux/amd64 --build-arg UBUNTU_BASE="$UBUNTU_BASE" --build-arg BUILD_JOBS=2 -t "gnss-p1-fcu-amd64:$id" "$out/ctx-sitl" 2>&1 | tee "$out/sitl-build.log"
docker build --pull --platform linux/amd64 --build-arg ROS_BASE="$ROS_BASE" --build-arg MAVROS_DEB_VERSION="$MAVROS_DEB_VERSION" --build-arg MAVROS_EXTRAS_DEB_VERSION="$MAVROS_EXTRAS_DEB_VERSION" --build-arg MAVROS_MSGS_DEB_VERSION="$MAVROS_MSGS_DEB_VERSION" --build-arg LIBMAVCONN_DEB_VERSION="$LIBMAVCONN_DEB_VERSION" -t "gnss-p1-transport-amd64:$id" "$out/ctx-mavros" 2>&1 | tee "$out/mavros-build.log"
```

Base digests and primary2.16.0 packages are explicit; transitive apt packages,
toolchain and GeographicLib data are not snapshot-pinned. Packages/source
records make differences reviewable, **not byte-identical builds guaranteed**.
For long-term byte reproducibility retain an apt snapshot/archive and datasets
hashes. If2.16.0 is absent, the remaining dependency is an authenticated archive
of those exact amd64 packages or a separately reviewed pinned-source build;
neither is silently substituted by this recipe.

Export records without starting either image. The copied install prefix must
be mounted back at `/tmp/patch_install`; relocating it arbitrarily is unsafe.

```bash
FCU_IMAGE=$(docker image inspect --format '{{.Id}}' "gnss-p1-fcu-amd64:$id")
TRANSPORT_IMAGE=$(docker image inspect --format '{{.Id}}' "gnss-p1-transport-amd64:$id")
PHYSICS_IMAGE=$(docker image inspect --format '{{.Id}}' gnss-phase1-camera:linux-gpu)
docker image inspect "$PHYSICS_IMAGE" "$FCU_IMAGE" "$TRANSPORT_IMAGE" > "$out/images.json"
python3 - "$out/images.json" <<'PY'
import json,sys
assert all(x['Architecture']=='amd64' and x['Os']=='linux' for x in json.load(open(sys.argv[1])))
PY
docker create --network none --name "gnss-p1-export-fcu-$id" "$FCU_IMAGE"
docker create --network none --name "gnss-p1-export-overlay-$id" "$TRANSPORT_IMAGE"
docker cp "gnss-p1-export-fcu-$id:/opt/phase1" "$out/firmware"
docker cp "gnss-p1-export-overlay-$id:/opt/phase1" "$out/mavros"
docker cp "gnss-p1-export-overlay-$id:/tmp/patch_install" "$out/patch_install"
python3 -B integration/runtime_candidate.py --firmware-dir "$out/firmware" --mavros-record-dir "$out/mavros" --overlay-dir "$out/patch_install" --architecture amd64 --physics-image "$PHYSICS_IMAGE" --fcu-image "$FCU_IMAGE" --transport-image "$TRANSPORT_IMAGE" --output "$out/candidate.json"
python3 -B integration/launch_phase1.py --run-id 20261113-amd64prep1 --runtime-candidate "$out/candidate.json" --gpu-mode wsl --isolated-imu
```

Choose `--gpu-mode nvidia` on native NVIDIA/Container Toolkit or `dri` on native
Mesa/DRM; WSL's override mirrors the measured teammate D3D12 contract. Preparation
only inspects images and renders Compose config. It starts no services.
Do **not** add `--execute` or manually start the generated Compose: candidate
admission is missing. Mapper pin, transport identity latch and permissions stay
closed. Historical ARM64 defaults remain available through the original path.
New preparations copy their firmware; do not restart historical run directories
with the new Compose or auto-migrate their recorded binaries/configurations.

## Isolated amd64 MAVROS validation, still no FCU

This is protocol/clock replay, not a live-camera test. The input fixture retains
actual recorded values/timestamps; only the peer clock offsets/jitter/drift and
declared reset lifecycle are test controls. Original sys_time is excluded by
the existing probe and sys_time_patched is loaded from the new overlay.

```bash
mkdir "$out/protocol"
docker run --platform linux/amd64 --name "gnss-p1-timesync-$id" --network none --cap-drop ALL --security-opt no-new-privileges --env ROS_DOMAIN_ID=82 --mount "type=bind,src=$PWD/integration,dst=/opt/integration,readonly" --mount "type=bind,src=$PWD/$out/protocol,dst=/evidence" --entrypoint bash "$TRANSPORT_IMAGE" /opt/integration/linux_gpu/check_timesync.sh /evidence/results > "$out/timesync-console.log" 2>&1
```

Required outputs: actual plugin-load console; ±1s/zero/jitter/drift/domain/reset
results; VPE/VSE raw packets with unique stamps and matching counts, finite PSD
21/9 covariance, timestamps unchanged, reset_counter0, offset errors within
the pre-existing20ms test budget; negative mixed-domain test must FAIL exit2.
The reset fixture tests upstream withholding, not an FCU-origin reinitialization
or automatic counter propagation. No result can admit a clock implementation.
`--network none` plus loopback-only check prevents any FCU network route.
No simulator or working ROS container participates.

## Transport deadline evidence and analysis

Two distinct failures are preserved in local ignored
`phase1_runs/20261111-nativeparts1/{REPORT.md,evidence/PERMISSION_HISTORY.json,evidence/WIRE_WAIT_BREAKDOWN.json}`.
Those private historical files are not needed to build/replay the workflow.

|Case|Evidence-based interpretation|
|---|---|
|t13.3s: upstream age180ms + clock uncertainty18.001ms|Only1.999ms remained at decision; frame conversion2.456ms crossed deadline; no MAVROS enqueue/wire measurement for this sample|
|t13.1s: enqueue end16771348584559ns, deadline16771365704059ns|17.119500ms left at enqueue end|
|Speed userspace receive16771369377475ns|20.792916ms after enqueue, already3.673416ms late|
|Final check16771370600184ns|Additional1.222709ms; final slack−4.896125ms; correct rejection|
|Next clock callback16771365725100ns|Already21.041us after deadline; later3.4905ms AP trace read worsened delay but did not first cross this deadline|

The remaining-field measurement in the first case was−3.950250ms, while the
marker host timestamp implies−3.951584ms: its field was computed1.334us earlier.
These are different instrumentation boundaries, not different deadlines.

There is no sender-system-call or kernel RX timestamp in this evidence. The
20.792916ms aggregate includes DDS, MAVROS/libmavconn and receiver scheduling;
it does not prove network latency, socket backpressure or which thread waited.
Even a future proof of early kernel RX would not authorize a late final check.
No receiver/scheduling optimization or safety-budget change was justified here.

`permission_deadlines.py` now joins actual enqueue spans with original
permissions and both wire channels. Missing spans are NOT TESTED; mapping log
time is not substituted as enqueue completion. `analyze_stage_latency.py`
corrects that metric and retains the explicitly named mapping-log interval.

```bash
python3 -B integration/permission_deadlines.py integration/phase1_runs/20261109-timing2/evidence/corrected_normal_transport1 --output integration/linux_gpu_runs/NEW-deadline-analysis.json
```

This command requires that historical local capture; it is optional and is not
a clean-clone dependency. For the next real transport run use its NEW transport
evidence directory instead. Separate sender→socket→receiver attribution needs
passive sender and kernel/receiver timestamps with verified clock domains;
do not treat userspace recvfrom as kernel arrival or grant extra TTL.

## Remaining dependency order

1. Linux builds and complete amd64 artifacts; isolated overlay ABI/load,
   TIMESYNC and durable reset results. Package availability is not verified here.
2. Review native AP applied-clock trace format/pacing, parser fields, production
   freshness and independent FCU cross-check. Only then consider an explicit
   reviewed binary admission change; never edit the candidate to self-approve.
3. The already prepared [VIO source-continuity run](VIO_SOURCE_PROBE.md) must
   PASS independently; the historical798.499ms gap remains unresolved.
4. One bounded real-VIO capture-only normal run with both wire channels,
   unchanged180ms/200ms/500ms checks, valid original permission at final boundary,
   no stale/invalid/duplicate packets and persistent epoch guards. If it fails,
   use the first aligned deadline failure, not speculative tuning.
5. After source/transport/critical fault preconditions and explicit approval:
   isolated matched-physics DISARMED live-camera EKF3 fusion with DataFlash
   source/innovation/rejection evidence. Full fault matrix and separately
   authorized flight remain later tasks. No source/nav switch is performed here.

Mac results belong to ignored
`source_scope_audit/amd64-transport-20261113/`: offline tests, Compose previews,
and reanalysis of16historical timing2 packet boundaries. These are not native
runtime measurements. Overall decision remains HOLD.

Offline validation:168/168tests PASS in a network-none ARM64 container on Mac;
the13new provenance/deadline tests also PASS directly on Mac. Default/WSL/NVIDIA/
DRM Compose configuration, Docker RUN shell/Python/Bash syntax, local Markdown
links, tiny build-context closure and recorded model/mount hashes checked.
No new amd64 binary, ROS package download, overlay compilation or runtime PASS.
