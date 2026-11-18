# Gazebo camera motion — 2026-11-04

**PASS: existing cmd_vel control and image-axis signs at identity attitude.**
Unchanged Person 2 SDF at `efd7f41cda82052417f4d5caae34bb25f2700ab7`, existing
ARM64 gz-sim runtime. A separate test publisher exercised VelocityControl;
the SITL motion bridge, ArduPilot and VIO were not run.

Commands: stationary 2.5 s → world +X 2.5 s at 0.2 m/s → pause 1.5 s →
world +Y 2.5 s at 0.2 m/s → stop 2.5 s. Independent pose/velocity came from the
existing Gazebo OdometryPublisher, not integration of command values.

| Phase | Measured model displacement, m | Measured static-feature shift | Status |
|---|---|---|---|
| Stationary | (0,0,0) | zero | PASS |
| World +X | (+0.4406,0,0) | down, +image v | PASS: sign |
| World +Y | (0,+0.4404,0) | right, +image u | PASS: sign |
| Stop | last 1 s: (0,0,0) | median approximately zero | PASS |

Height stayed 2 m, quaternion (0,0,0,1). Transition-inclusive pause/stop residuals
were 6.4/4.2 mm; they are not sustained motion. Collected 283 real PNG frames,
508 odometry samples and 225 command records. Image stamps: 6.667 → 16.798 s.

Same-phase Lucas–Kanade medians: +X (du,dv)=(+0.0001,+0.7467) px/pair over 16
usable pairs; +Y=(+0.7583,+0.0199) over 12. Whole-image phase correlation over 61
pairs each gave approximately (-0.0016,+1.7639) and (+1.7491,+0.0039), with median
responses 0.9313/0.9688. Both support the dominant-axis signs. Their different
magnitudes and sparse corner coverage mean **flow scale/velocity accuracy is NOT TESTED**.
The record is not proof of robust VIO feature tracking.

At yaw=0 with body +X forward and FLU +Y left, the signs support optical X right,
Y backward, Z down, matching Person 3's stated convention. Negative translation,
rotation/gyro compensation, arbitrary attitude, calibration, Person 3 frontend,
IMU axes, ArduPilot frame alignment and VIO fusion remain **NOT TESTED**.

A first integration-helper attempt **FAILed** on integer zero in ROS float fields;
explicit float conversion fixed it. The failed attempt was retained separately.
All test/analysis containers exited; no arm/takeoff or mock images.

Local primary archive (not committed): `integration/camera_probe_runs/20261104_motion_live_retry/`.
Analysis JSON SHA-256: `0dcac3679f41fa94b6af82206fb3926e885a8b2f9242eaacfc3c36ddc108107c`.
Primary PNG/CSV files are omitted here; checking tracking magnitudes requires that
archive or a fresh run. The README command recreates capture/LK recording, not the
archived auxiliary phase-correlation analysis.

[Reproduce motion capture](../README.md) · [Static camera](arm64_camera.md) ·
[Motion bridge limitation](motion_bridge.md)
