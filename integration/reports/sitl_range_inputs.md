# SITL range/input diagnosis — 2026-11-04

**Overall FAIL for required VIO inputs; transport checks PASS.**
ArduCopter 4.4.4 firmware `9279df6191a772c098b2e4797c95ac954f258d88`, existing
pinned ARM64 image. GPS disabled, EKF3 ExternalNav configured, no ExternalNav
measurements supplied. No Gazebo, VIO, synthetic position injection, arm or takeoff.

| Check, streamrate=-1 control | Observed result | Status |
|---|---|---|
| Heartbeat, V2, disarmed | confirmed | PASS |
| Nine sonar parameters | live readback matches overlay, orientation 25, min/max 20/5000 cm | PASS |
| DISTANCE_SENSOR | 20 Hz | PASS: rate |
| Current range | 0 cm on ground | FAIL: below minimum |
| RAW_IMU / ATTITUDE | 49.875 / 49.875 Hz | PASS |
| LOCAL_POSITION_NED | zero packets | FAIL |
| Interval requests | command 511 ACK result=0 for all four requests | Accepted, not delivery proof |

## Established causes

In 4.4.4, `GCS_Common.cpp:2654-2657` returns before sending local position when AHRS
cannot supply origin-relative position/velocity. `AP_NavEKF3_Outputs.cpp:220-253`
returns false for horizontal position in AID_NONE, including with validOrigin.
After setting only geographic origin (-35.363261,149.165230,584 m AMSL), both EKF
origin messages and GPS_GLOBAL_ORIGIN confirmed it. Local-position packets still
remained absent; EKF flags stayed 167, constant-position mode without horizontal
position validity. Origin alone does not replace ExternalNav aiding.

Default MAVProxy streamrate=10 also overrode a 20000 µs request with 100000 µs.
Opt-in SITL_STREAMRATE=-1 retained 20000 µs and restored IMU/attitude rates, but did
not create a valid local-position solution. Default behavior was preserved.

The range backend classifies 0 cm below its 20 cm minimum as OutOfRangeLow;
`has_data()` still permits transmission. This status follows exact backend code
and measured bounds; id 132 does not directly expose its status enum. Delivery
is normal, but the reading is unusable as VIO scale. Internal simulated sonar is
not Gazebo range and does not establish camera height.

**NOT TESTED:** nonzero-height range, sensor-offset experiment, flight, VIO fusion,
camera/SITL physical or timestamp correspondence. No offset/glitch was applied.
All created containers stopped. No mock PASS.

Local archives (not committed): `integration/input_probe_runs/20261104_range_disarmed/`
and `integration/input_probe_runs/20261104_range_diagnosis/`.
Control probe JSON SHA-256: `95bdabc1188f3d85325f54d375139352531a17660e777c45c5de145d71432526`.
Raw parameter/source/packet logs are retained locally, not included in this summary.

[Reproduce the strict disarmed probe](../RANGE_INPUTS.md) ·
[Motion bridge blocker](motion_bridge.md) · [Integration overview](../README.md)
