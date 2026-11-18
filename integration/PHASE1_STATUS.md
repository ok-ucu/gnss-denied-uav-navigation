# Phase 1 status — Linux GPU handoff

## Linux archive / source-runtime prerequisite — 2026-11-14

**HOLD; execution_allowed=false; forwarding_enabled=false.**
Read `phase1-evidence-35355f4-FINAL.zip` (SHA256
`059c8fcf4e5bdb4c965207fff95fa41c3055e8e571a4115fc0b8a95f6d8afc00`),
with HEAD35355f4 evidence. No archived source or container was modified.
Separate NVIDIA diagnostic identifies Quadro T2000/NVIDIA595.99.02; however,
camera02's actual Ogre log says llvmpipe and its metadata lacks explicit EGL
vendor selection. Do not combine those different contexts into a GPU camera
PASS. Both camera02 containers were stopped by their original launcher.
Seven positive TIMESYNC replay result files agree with the suite, with the
bad-domain FAIL expected. This is isolated artificial-peer evidence, not native
FCU clock admission. Robust heartbeat observation has no persistent artifact.
Original strict heartbeat traceback proves prefix108/0x6c failure; no raw TCP
capture establishes where that byte originated.

The source probe's documented `gnss-sim-*` prerequisite had no repository
startup implementation. Camera-only Compose creates/stops `gnss-gpu-*-<id>`;
renaming does not establish mounts, shared namespace, sensors or provenance.
Added a separate opt-in bounded NVIDIA runtime parent reusing that Compose and
verified SDFs. It exports pinned VIO d33f660, selects EGL vendor explicitly,
checks namespace/mounts/renderer/firmware/mount geometry, then optionally invokes
the existing OFF/ON/OFF probe. The probe never starts/restarts containers.
The parent stops only owned IDs in finally; no MAVROS or FCU vision route.

New expected-source receive-only heartbeat guard discards/counts BAD_DATA and
requires valid MAVLink2/CRC, configured IDs1/1, ArduPilot/quadcopter role and
>=3 DISARMED heartbeats over >=2s, with the existing3s heartbeat freshness bound.
The continuous native witness uses it too; legacy strict parser stays unchanged.
Protocol/source checks are not cryptographic authentication. Quadro renderer
recognition now checks the actual NVIDIA vendor without allowing software.

[Exact Linux startup, source-only test, evidence and cleanup commands](VIO_SOURCE_PROBE.md).
Mac preparation/Compose validation is not runtime acceptance. Live source and
capture transport remain NOT TESTED/BLOCKED; permission deadlines, native clock
admission and later live-VIO fusion still require separate gates. No simulations,
arming, forwarding, commit or push were performed here.

Offline validation of this addition: **198/198 tests PASS**, no skips; source
allowlist/import/Markdown-link audit and syntax checks PASS. New NVIDIA startup
Compose resolved successfully in PREPARE mode on Mac; actual Linux execution
remains NOT TESTED. Pinned VIO export and all four hashes verified locally.

## Independent review — 2026-11-14

**HOLD; execution_allowed=false; forwarding_enabled=false.**
Reviewed branch `gazebo-sitl-integration`, HEAD
`35355f4f5489b8f16aa32fa93cb5cc242b7fc952`, preserving existing local files.
[Audit, work classification and exact Linux evidence request](LINUX_EVIDENCE_REVIEW.md).
The teammate reports T2000 GPU/EGL/Ogre2, image and MAVROS builds PASS;
TIMESYNC replay reportedly PASS. Their changes and raw evidence have not been
transferred, so these results are not independently verified here. Live
DISARMED heartbeat is BLOCKED; camera-only and VIO OFF/ON/OFF remain unconfirmed.

Added offline real-codec/synthetic-socket regressions for malformed prefix/CRC,
BAD_DATA, packet fragmentation, armed/disarmed heartbeat, EOF and timeouts.
The production parser remains unchanged. Explicit characterization tests show
it accepts unexpected source IDs and MAVLink 1; they do not prove FCU identity
or MAVLink 2. No parser fix is justified by the missing Linux error evidence.
Added an offline explicit-allowlist evidence ZIP exporter with hashes and
provided PASS/FAIL/BLOCKED/NOT TESTED summaries, always marked unverified/HOLD.
It neither runs tests nor authorizes a runtime.

Validation: **184/184 offline tests PASS**, no skips, using pinned VIO sources;
source/import/Markdown-link audit PASS. Frames, covariance, TIMESYNC, clock
mapping, transport deadlines and production DISARMED code remain unchanged.
No Linux build, simulation, forwarding or flight was run. Next action: obtain
the blocked heartbeat traceback/raw capture and source/build/renderer/overlay
provenance from Linux before selecting a parser or runtime-policy change.

## Latest review — 2026-11-13

**HOLD; execution_allowed=false; forwarding_enabled=false.**
[Independent amd64 runtime and transport preparation](AMD64_TRANSPORT_READINESS.md):
native firmware/overlay build records and ELF checks now support PREPARE ONLY;
no amd64 binary is admitted to the clock mapper. Public-base2.16.0 overlay recipe,
explicit image/overlay paths, GPU preview overrides and source-bound firmware
copy added. Candidate execution and capture transport remain blocked pending
native clock/ABI/runtime evidence. No new Linux build or simulation was run.
Existing VIO work is unchanged. Deadline analysis now uses actual enqueue end,
not mapping-log time, and separates userspace receive→final validation. Sender,
kernel RX and socket wait remain unmeasured; no TTL/watchdog relaxation or
scheduling optimization was applied. The known17.1195ms budget versus20.792916ms
enqueue→userspace receive remains unresolved.
Validation:169/169offline tests PASS, including the source-package shell-input
regression for the isolated MAVROS replay YAML. Compose previews and build-input closure PASS. Linux package
availability, native builds/overlay load and capture runtime are NOT TESTED.

[Teammate evidence, isolated IMU changes and next Linux experiment](reports/VIO_ISOLATION_20261113.md).

The supplied 11–12 November ZIP confirms WSL2 RTX5060 hardware D3D12 rendering
and 10Hz simulation camera acquisition. These are teammate measurements, not a
new Mac live test. Isolated IMU ingestion produced259 Odometry records from275
frames; IMU processing wall median21.206ms, total processing p95 92.198ms.
However, the source has an unexplained stale-clock event followed by a798.499ms
publication gap, exceeding the unchanged500ms availability criterion. Published
rows all recorded zero subscribers; independent delivery was not verified.
The observed sim/consumer-wall progress ratio0.687673 does not identify the RTF
bottleneck. Supplied traces lack raw clock/IMU/IPC and aligned CPU evidence.

Locally implemented opt-in spawned IMU+/clock ingestion with bounded IPC,
ordering/session checks, latched failure, timestamp preservation and explicit
shutdown. All three Phase1 launchers now forward `--isolated-imu`; defaults stay
unchanged. Added clock-reason/IPC/CPU diagnostics;180ms publication,200ms clock,
500ms availability and firmware provenance checks remain intact.
**145/145 offline tests PASS**, including8new tests also passing on Mac.
The hardened worker's live ROS/QoS/performance remains **NOT TESTED**.

Next action is the linked source-only Linux OFF/ON/OFF capture with an
independent Odometry subscriber and one owned ROS clock publisher. Do not run
full acceptance until source continuity passes. The full amd64 launcher still
needs reviewed native firmware provenance, GPU configuration and a rebuilt,
validated MAVROS TIMESYNC overlay. Transport deadline and live-camera EKF3
fusion remain unverified. No new simulation, FCU forwarding or flight ran here.

The [executable source-probe procedure](VIO_SOURCE_PROBE.md) now supersedes the
manual experiment commands. It adds native clock/IMU/CPU witnesses across all
three phases and independent delivery/tail-gap analysis. Offline review also
corrected comparison of a fresh clock callback to an earlier frame-entry time;
validation now timestamps the check after reading its snapshot. This is an
ordering correction, not a demonstrated cause of the historical798.499ms gap.
No worker/performance optimization, live verification or threshold change.
Source-probe preparation:155offline tests PASS, including10new source-probe
regressions;10source-probe and8isolated-worker tests also PASS directly on Mac.
The Linux OFF/ON/OFF run is NOT TESTED; historical798.499ms gap remains unresolved.

## Previous Mac handoff — 2026-11-11

2026-11-11. **HOLD; execution_allowed=false; forwarding_enabled=false.**
Branch `gazebo-sitl-integration`; preceding checkpoint `9298da8`.

[Linux GPU handoff](LINUX_GPU_HANDOFF.md) ·
[Preserved measured summary](reports/RENDERING_20261111.md) ·
[Source/dependency boundary](SOURCE_DEPENDENCIES.md).

Actual Gazebo context and Ogre log confirm **Mesa llvmpipe CPU rendering**,
OpenGL3+/headless EGL, EGL_MESA_device_software. Native ARM64 container/host,
physics3CPU, no exposed /dev/dri. Gazebo8.15.0/sensors8.2.2/rendering8.2.3/
OgreNext2.3.3/Mesa25.2.8. Camera640×480 RGB/HFOV2.0/fx205.4696/10Hz simulation.

Camera-only baseline:129frames/30.046s=4.293Hz wall. Render wall p95/p99/max
286.148/373.062/489.189ms. Scene-pass nested union median91.49%; software
Mesa/JIT execution dominates CPU samples. Exact private shader kernels and
historical2s outlier attribution remain unresolved. Candidate LP_NUM_THREADS=2
reduced throttling but worsened p95/p99 and wall rate4.096Hz; NOT promoted.

Real camera/IMU→VioEKF→ROS Odometry, frame/covariance conversion and separate
MAVROS wire output have historical live evidence. Synthetic DISARMED EKF fusion
is separate; live-camera GNSS-denied flight is not established. GT Z remains a
transparent test aid. Horizontal GT is not fed into the estimator.

**Latest continuous normal FAIL. No new normal/flight/simulation run for this
handoff.** Independent wire deadline issue remains:17.1195ms left at enqueue,
20.792916ms to userspace capture arrival; final interlock correctly rejected.
180ms publication /200ms clock /500ms availability and all durable checks remain.
Acquisition→FCU uncertainty bound is NOT ESTABLISHED. No GO or FCU forwarding.

Handoff adds source-rebuild recipes and promotes native probes from ignored
runtime folders. Fresh camera-only path has no dependency on historical image
IDs/binary or VIO/MAVROS. Linux image build/hardware execution NOT TESTED here;
full historical pipeline additionally needs reviewed native firmware provenance
and rebuilt/validated MAVROS overlay. Existing strict binary pin remains intact.
Raw evidence paths and values are preserved in the linked summary; no raw logs,
frames, perf data, binaries or upstream repositories are shipped or deleted.

**One next action:** on the Linux teammate's GPU machine, follow the handoff's
preparation/build checks and one bounded camera-only diagnostic, verifying the
actual Gazebo hardware GL/EGL device before considering full normal validation.
