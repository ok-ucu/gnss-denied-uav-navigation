# Linux GPU Phase 1 handoff

**HOLD. execution_allowed=false; forwarding_enabled=false.**
Checkpoint before this handoff: `9298da8`, branch `gazebo-sitl-integration`.
Start with the [preserved measured summary](reports/RENDERING_20261111.md) and
[current status](PHASE1_STATUS.md). No flight, arming, mode changes, navigation
source switch, VIO→FCU route or ExternalNav approval is part of this handoff.

## What works, and the immediate question

Historical live Gazebo camera/IMU → FlowFrontend → VioEKF → quality-gated ROS
Odometry works. Independent ROS subscribers, NED/FRD↔ENU/FLU frame/covariance
conversion, real isolated MAVROS2.16.0 VPE/VSE wire output, separate signed-offset
TIMESYNC overlay and durable reset/egress checks were tested. Synthetic DISARMED
FCU reception/fusion is distinct from live-camera navigation. GT Z is still a
declared optical-flow depth aid; horizontal GT never enters VIO estimation.
None of this establishes live-camera GNSS-denied flight or continuous normal.

Actual Mac Docker rendering is **Mesa llvmpipe CPU rendering**, not hardware GPU:
actual GL context and Ogre log identify `EGL_MESA_device_software`. Camera remains
640×480 RGB, HFOV2.0rad/fx205.4696,10Hz simulation acquisition, but only4.1–4.3Hz
wall time in the camera-only captures. `CompositorPassScene::execute()` nested
interval union accounts for91.49% median Render wall time. Readback is short.
Reducing llvmpipe workers10→2 reduced throttling but did not improve latency.
Do not repeat that tuning or lower resolution/rate/scene quality to claim success.

The summary preserves dates, complete timing table, uncertainty and original
local report paths. Raw `phase1_runs/` recordings are intentionally **not in Git**.
Existing native diagnostic source is promoted unchanged where possible to
[linux_gpu/](linux_gpu/run_camera.py); its orchestration now accepts fresh IDs
and rebuilt images instead of hardcoded historical paths and ARM64 binaries.

## Dependencies and reproducibility boundary

|Component|Recorded pin/version|Linux preparation|
|---|---|---|
|ArduPilot Copter4.4.4|`9279df6191a772c098b2e4797c95ac954f258d88`|New isolated build with exact boolean/trace patches and actual C++ parser tests|
|ArduPilot Gazebo plugin|`f116b03552d05c8d0b0bed5ed67033a1183121d1`|Clone/build inside new camera image; upstream models/meshes remain external|
|ROS/Gazebo|Jazzy/Harmonic; measured sim8.15.0, sensors8.2.2, rendering8.2.3|`ros:jazzy-ros-base` + ROS vendor packages; record actual rebuilt versions|
|Renderer|OgreNext2.3.3 GL3Plus; measured Mesa25.2.8/LLVM20.1.2|Actual hardware GL/EGL context required on Linux; historical software device is comparison only|
|Python|ROS Jazzy Python3.12, NumPy/OpenCV; Matplotlib for figures|Included by camera recipe; no pip fallback or mock frames|
|MAVROS|2.16.0 + separate signed TIMESYNC plugin|Not needed or started for camera-only test; see full-stack limitations below|
|VIO|Person3 commit `d33f66008fd7b6de332c96dd324b8ddbd320c194`|Four source hashes verified by optional export helper; never edit/stage `vio/`|

Local `sha256:...` image IDs in [phase1.compose.yaml](phase1.compose.yaml) are not
pullable registry digests. Existing ARM64 binaries, Mesa profiler binary and
`timesync_patch_runs/.../patch_install` are absent after clone and must not be
copied into this commit. The new camera-only path does not need them.

**Validation scope of this handoff:** source/test closure, patch application,
native probe compilation against the existing installed ABI, preparation steps,
CLI/shell syntax and Compose resolution can be checked on this Mac without
simulation. New Linux Docker builds and hardware runtime are **NOT TESTED here**.
Build recipes pin AP/plugin source, but ROS/Ubuntu tags and apt repositories are
mutable; bit-identical cross-machine builds are not claimed. Save build logs,
image IDs, base repo digests, `packages.txt`, firmware hash and parser-test log.
Pin matching base digests/package snapshots separately for an immutable rebuild.

## Build on Linux, without historical runtime folders

Use native Linux amd64/arm64, Docker Engine + Compose v2, Python3.10+ on host,
network access for base images/Git/apt, and sufficient build disk/RAM. NVIDIA
requires host drivers and NVIDIA Container Toolkit; Intel/AMD requires working
host `/dev/dri` and compatible Mesa/kernel drivers. These are external host
prerequisites, not packages this repository silently installs. `gpus: all` needs
a Compose version supporting that key (2.30+). Do not emulate the ARM64 image on
x86 Linux. Do not alter unrelated simulators or services.

From the repository root on this branch, prepare **tiny allowlisted build
contexts**, excluding all local captures/upstream checkouts:

```bash
python3 -B integration/linux_gpu/prepare_context.py --kind camera --output integration/linux_gpu_runs/linux-camera-context
python3 -B integration/linux_gpu/prepare_context.py --kind sitl --output integration/linux_gpu_runs/linux-sitl-context
docker build --progress=plain -t gnss-phase1-camera:linux-gpu integration/linux_gpu_runs/linux-camera-context
docker build --progress=plain -t gnss-phase1-sitl:linux-gpu integration/linux_gpu_runs/linux-sitl-context
docker image inspect gnss-phase1-camera:linux-gpu gnss-phase1-sitl:linux-gpu
```

Contexts must be NEW. No deletion/overwrite is performed. Dockerfiles clone the
pinned upstream commits inside new images; no full upstream repository is
vendored in Git. SITL build requires actual `tests/test_sim_json` PASS before
producing the image. Artifacts inside that image:
`/opt/phase1/{arducopter,firmware.sha256,parser-tests.txt,ardupilot-commit.txt,packages.txt}`.
Camera image has `/opt/phase1/{native_parts_render.so,plugin-commit.txt,packages.txt}`.
The witness build checks a required Ogre2 virtual-thunk symbol. If apt/CMake/
compiler/ABI/parser tests fail, preserve the build log and stop; do not disable
the test or silently use an unpatched firmware. No Linux build success claimed.

## Prepare and execute the existing camera-only diagnostic

First inspect preparation only; no container starts:

```bash
python3 -B integration/linux_gpu/run_camera.py --run-id 20261112-gpuprep --mode dri
```

Use `--mode nvidia` instead for NVIDIA; `--mode software` is an explicitly
labelled control. The generated `artifacts/resolved-compose.yaml` records
devices/GPU requests,3CPU physics quota, private internal network, unique
partition, exact images and mounts. Only new `gnss-gpu-physics-<id>` and
`gnss-gpu-fcu-<id>` services exist. No host ports or ROS/VIO/MAVROS route.
Camera-only FCU adds only MAVLink2 serial protocol and DISARMED logging; it
does not apply the ExternalNav/GPS-disabled profile or switch navigation source.

After reviewing host GPU access, use a **different new ID** for execution:

```bash
python3 -B integration/linux_gpu/run_camera.py --run-id 20261112-gpurun1 --mode dri --duration 30 --execute
```

NVIDIA: substitute `--mode nvidia`. This performs the same native gimbal
preflight (three measured axes≤1°), same visual fixture and30s static camera
capture. Passive heartbeat checks confirm DISARMED before/after. Exact own
container IDs/labels are checked during finally cleanup; containers and raw
evidence are retained, never removed. Fixture removal is attempted before stop.
Inspect `evidence/owned-final.txt` and any `cleanup_error` after interruption.

Optional CPU stacks: add `--perf` to a fresh run. Only that new physics container
gets PERFMON/seccomp profiling permissions. Linux perf support is a host/kernel
prerequisite; failure is not a successful profile. It records cpu-clock:u99Hz,
DWARF stacks, plus native boundary wall/threadCPU and50ms scheduler samples.
Instrumentation overhead must be disclosed; it is not a latency guarantee.

Outputs in `integration/linux_gpu_runs/<id>/evidence/`:
`native_render/{render_gl_identity.txt,ogre2.log,camera_profile.json,
phase1_camera_boundary_1.csv,camera_profile.ppm}`, `METRICS.json`, metadata,
commands and logs; perf files only if requested. Re-analysis into a NEW file:

```bash
python3 -B integration/linux_gpu/analyze_camera.py integration/linux_gpu_runs/20261112-gpurun1/evidence/native_render --output integration/linux_gpu_runs/20261112-gpurun1/evidence/metrics-review.json
```

## Verify hardware rather than a flag

Read actual **Gazebo** `render_gl_identity.txt` and matching `GL_RENDERER`,
GL_VENDOR, selected EGL device in `ogre2.log`, together with the device inventory
and resolved Compose GPU allocation. Require a matching real GPU vendor/device;
`llvmpipe`, `softpipe`, `swrast`, `lavapipe`, SwiftShader/software are HOLD.
`nvidia-smi`, `LIBGL_ALWAYS_SOFTWARE=0`, a mounted device or EGL probe alone
does not prove Gazebo used hardware. The helper can label a corroborated
nonsoftware identity only **HARDWARE CANDIDATE**; operator verifies the selected
device. Enumeration of an unused software EGL device is not itself failure.

Collect full-window p50/p95/p99/max Render/Update wall and calling-thread CPU,
native publish and receiver gaps, wall versus acquisition Hz, acquisition-label
gaps, real image dimensions/content, per-thread runtime/runqueue/wchan,
cgroup CPU/throttled period/time deltas, actual versions/device/driver. Preserve
the200/180/500ms safety thresholds, intrinsic/mount/scene/physics/JSON settings.
Do not subtract simulation stamps from host monotonic time. Worker CPU is not
included in calling-thread CPU. Nested passes require interval union.

Camera investigation succeeds only if real hardware use is independently
confirmed, genuine640×480 frames preserve10Hz simulation acquisition and the
full-window timing/gap tail is reported and materially improves versus the
software baseline. A single short sample does not prove a hard continuity bound.
Native arrival gaps≥500ms or a reduction confined to one fragment cannot clear
the known availability gate. Hardware alone is not Phase1 GO.

## Before any full acceptance run

1. Validate the new Linux build/parser tests, real camera mount, device, scene,
   timestamps, IMU and JSON clock behavior. Review profile overhead and gaps.
2. Supply the pinned Person3 VIO dependency separately, without modifying their
   branch/directory. Optional verified export for tests:

```bash
git fetch origin d33f66008fd7b6de332c96dd324b8ddbd320c194
python3 -B integration/linux_gpu/prepare_vio.py --output integration/linux_gpu_runs/vio-source
VIO_SOURCE_DIR="$PWD/integration/linux_gpu_runs/vio-source" PYTHONPATH=integration python3 -B -m unittest discover -s integration/tests -v
```

3. Full historical launcher/transport still rejects a freshly compiled Linux
   binary: its tested ARM64 SHA-256 is `047e4fbe21de2f88eba6c453e8aa3091ffd4b89b2b5568544f16cbf4c38d5970`.
   Do not replace the hash automatically. A reviewed Linux artifact provenance
   record, parser/clock revalidation and explicit safe artifact-admission change
   are required. Camera-only binary is **not admitted** by these guards.
4. Rebuild real MAVROS2.16.0 and its separate signed TIMESYNC overlay for the
   Linux architecture. [Overlay build helper](build_mavros_time_patch.sh) requires
   a new builder image `gnss-mavros-isolated:jazzy-2.16.0`, then a NEW
   `PATCH_BUILD_OUTPUT`. [Existing Dockerfile](MAVROS.isolated.Dockerfile) still
   depends on a supplied `gnss-gazebo-ardupilot:dev` base and ARM64-dated apt
   versions; amd64 package availability/build is **unverified**. Do not edit the
   original MAVROS install or treat an absent generated overlay as present.
5. Close the independent **final wire permission deadline** issue: in historical
   nativeparts1 only17.1195ms remained after enqueue; userspace wire arrival
   followed20.792916ms later and final interlock rejected both packets. Preserve
   original acquisition stamps, expiry, durable checks and post-MAVROS interlock.
   Do not extend TTL, weaken freshness or relabel capture as FCU reception.
6. Only after focused checks and bounded continuous normal passes all required
   gates, consider the60s acceptance and pending DISARMED fault matrix. Full
   clock uncertainty, reset lifecycle, covariance calibration and live-camera
   fusion/flight readiness remain open. Separate explicit approval is required
   for any future FCU forwarding or flight. This handoff provides none.

No Linux simulation/build was executed for this Git handoff. See
[source boundary](SOURCE_DEPENDENCIES.md) and [scope](SOURCE_COMMIT.md).
