# Phase 1 status — Linux GPU handoff

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
Validation:168/168offline tests PASS, including13new native-candidate/deadline
regressions. Compose previews and build-input closure PASS. Linux package
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
