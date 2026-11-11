# Phase 1 status — Linux GPU handoff

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
