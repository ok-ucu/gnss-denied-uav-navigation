# ARM64 camera live test — 2026-11-04

**PASS: static camera transport only.** No ArduPilot, VIO, arm or takeoff ran.
Existing Person 2 Dockerfile/SDF: commit `efd7f41cda82052417f4d5caae34bb25f2700ab7`.
Runtime: ROS Jazzy, Gazebo Harmonic, Linux ARM64; local `gz-sim` image digest
`sha256:fede1060db6fe20fd4f388264b74591a65a75d1e372be7c15dbdb3b7be1179d4`.

| Check | Measured result | Status |
|---|---|---|
| Image type / shape / encoding | sensor_msgs/msg/Image; 640×480; rgb8; step 1920 | PASS |
| Wall-time delivery | 269 frames / 10.000278 s = 26.899 Hz | PASS, ≥20 Hz |
| Rate by header stamps | 29.857 Hz | Recorded |
| Image stamps | 6.403 → 15.379 s, strictly increasing | PASS |
| Clock | 9012 messages; 6.397 → 15.418 s | PASS |
| Nonzero content | 269/269 frames; first-frame bytes min/max 19/229 | PASS |
| IMU | sensor_msgs/msg/Imu; 1804 messages, 180.395 Hz wall time | Present; not a 200 Hz wall-time claim |

Simulation progressed 9.021 s over approximately 10 s wall time. The initial exact
Dockerfile build **FAILed** at apt index Hash Sum mismatch. A temporary stdin-only
HTTP→HTTPS Ubuntu mirror substitution succeeded with integrity checking enabled.
No tracked Dockerfile/model changed. Mutable base/apt versions limit future build
identity; the existing local image can be reused.

**NOT TESTED:** camera motion in this run, image-axis signs, IMU frame convention,
VIO/MAVROS fusion, ArduPilot synchronization, capture latency and GNSS-denied flight.
All created processes/containers stopped. No mock fallback.

Local primary archive (not committed): `integration/camera_probe_runs/20261104_arm64_live/`.
The saved summary JSON has SHA-256 `11123cb56a23ea5d5f43a4f05529f9397b02580b43898b6a8d0711a89177ad94`.
First RGB frame SHA-256:
`3fef29e09deb60f9ea9f95ca20f4ec099b53cd71c06c313ead01c717951f2699`.
This compact report does not include frame bytes or raw logs; independent rechecking
requires the local archive or another run.

[Reproduce the static test](../README.md) · [Motion test](gazebo_motion.md) ·
[Range diagnosis](sitl_range_inputs.md) · [Motion bridge](motion_bridge.md)
