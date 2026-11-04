# Existing motion bridge smoke-test — 2026-11-04

**BLOCKED: no real LOCAL_POSITION_NED; not an end-to-end PASS.**
Real ArduCopter 4.4.4 + existing ARM64 Gazebo. Person 2's script and SDF were used
byte-identically from `efd7f41cda82052417f4d5caae34bb25f2700ab7`; no model,
bridge logic, image or other-component code changed.

| Check | Actual observation | Status |
|---|---|---|
| Bridge UDP 14553 input | 13 heartbeat, 599 ATTITUDE, 598 RAW_IMU, 240 DISTANCE_SENSOR | PASS: real transport |
| MAVLink / arm state | V2, disarmed | PASS |
| id 32 request on independent UDP 14554 | 50 Hz request, ACK ACCEPTED | Request only |
| LOCAL_POSITION_NED | 0 at bridge and independent probe | FAIL / blocker |
| ROS Twist | one zero command at cleanup | Not forwarded movement |
| Independent Gazebo odometry | 1348 samples; pose stays (0,0,2) m | No movement |
| /clock | 26964 samples; 0.002 → 26.965 s | PASS |

The bridge sends velocity only after LOCAL_POSITION_NED, plus one zero command in
its finally block. Its startup message and successful process exit are not evidence
of forwarding. The independent probe also failed grounded range=0 cm. The
[previous diagnosis](sitl_range_inputs.md) explains the EKF publication gate.

The real bridge connection was observed with pymavlink's decoded message_hooks;
packets were logged, not replaced. Missing dependency checks prevented the original
script's automatic mock fallback. Existing host pymavlink 2.4.50 Python source was
mounted read-only in the ROS container without installing/rebuilding anything.
PYTHONPATH preserved ROS paths. Both containers shared a network namespace;
SITL_HOST=127.0.0.1, streamrate=-1, output 14553 for bridge and 14554 for diagnostics.
VIO 14551 and monitor 14552 were untouched. ROS→Gazebo Twist and Gazebo→ROS odometry
used standard ros_gz_bridge types. All created containers are stopped.

**NOT TESTED:** nonzero SITL-driven movement, NED→ENU forwarding direction,
pose/time consistency, VIO, flight. The bridge stores attitude but does not apply
it and forwards velocity without initial-position alignment; even a future message
PASS would not establish shared physics. It must not receive fabricated local
position to bypass this blocker. Real usable ExternalNav, EKF initialization and
actual id 32 packets are prerequisites for the next approved test.

Preparatory attempts were retained: a helper PYTHONPATH error was corrected, and
an empty native UDP raw-log was replaced by message_hooks. Neither attempt was
relabeled as a movement PASS. No arm/takeoff, synthetic ExternalNav or VIO ran.

Local primary archive (not committed): `integration/motion_bridge_runs/20261104_disarmed_smoke_observed/`.
Summary JSON SHA-256: `1a80b846aa1b8af3907a1bdcbf8873926fd749187a577d9f7c7f1ea9ac488b58`.
Packet/ROS logs and observational launch helpers remain local. This compact report
summarizes the original run; it does not provide a new standalone bridge launcher.
To repeat, use the unmodified script from the pinned Git commit in a ROS environment
with pymavlink, give it dedicated UDP 14553, bridge cmd_vel, and observe actual messages
on a separate output. Do not use the excluded archive as an installed dependency.

[Available probes/runtime commands](../README.md) · [Range/input commands](../RANGE_INPUTS.md) ·
[Standalone Gazebo motion test](gazebo_motion.md)
