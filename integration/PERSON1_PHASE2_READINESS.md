# Person 1 — Phase 2 Readiness

## Responsibility

ArduPilot / SITL / EKF3 / GPS / IMU / flight controller.

Scope: offline GPS/IMU log processing, coordinate conversion,
clock-domain preparation, and real-hardware readiness.

## Implemented components

| Component | File | Commit |
|---|---|---|
| DataFlash GPS/IMU/ATT export | ardupilot_log_parser.py | 523ada6 |
| GPS quality filtering | gps_quality_check.py | 900b88b |
| WGS84 to ENU/NED/TUM | gps_trajectory_export.py | 7d381f2 |
| Guarded FCU-to-host time conversion | dataflash_time_adapter.py | f4f2b01 |

## Verification

Combined offline test suite: 23 passed.

Input: existing ArduPilot SITL DataFlash BIN.

Observed:
- GPS: 129 raw rows, 128 accepted.
- IMU: 1586 rows across two instances.
- ATT: 317 rows.
- Exported GPS trajectory: 128 points.
- Unique GPS positions: 1.
- Maximum GPS displacement: 0 m.

The existing SITL GPS recording is stationary.
It does not validate VIO trajectory accuracy.

## Coordinate contract

- GPS input: latitude/longitude in degrees, altitude in metres.
- Reference ellipsoid: WGS84.
- ENU: east, north, up (metres).
- NED: north, east, down (metres).
- Local origin: first accepted GPS point.
- GPS position alone does not provide orientation.
- TUM identity quaternion is a placeholder, not measured attitude.
- Altitude datum remains unverified.

## Timestamp contract

DataFlash TimeUS:
FCU boot-relative microseconds.

Dataset camera:
Host monotonic nanoseconds.

DataFlash clock conversion requires an explicitly verified
same-session FCU-to-host clock model.

No verified model currently connects the inspected SITL BIN
to the camera recording.

The sample timesync.json must not be reused as proof of
synchronization for another FCU session.

## GPS Ground Truth limitations

GPS fix type and satellite count are quality indicators,
not independent accuracy measurements.

Ground Truth accuracy has not been established.

Before trajectory evaluation, verify:
- GPS receiver and antenna configuration.
- Position accuracy and reference source.
- Altitude datum.
- Clock synchronization.
- GPS update frequency.
- Coordinate-frame alignment.
- Dynamic trajectory coverage.

## Dependencies

Person 2:
Camera/IMU timestamp domains and recording format.

Person 3:
GPS Ground Truth accuracy and sampling requirements.

Person 4:
Trajectory comparison format and evaluation criteria.

## Hardware readiness

Pending physical flight controller and GPS/IMU identification.

Next hardware-stage tasks:
1. Identify FCU, ArduPilot firmware, GPS and IMU models.
2. Verify onboard logging and clock-domain information.
3. Collect an authorized, safe stationary baseline.
4. Validate GPS quality and timestamps.
5. Collect dynamic data only after safety authorization.
6. Verify synchronization before trajectory comparison.

## Safety status

Phase 1 runtime: HOLD / NO GO.

Offline parsing, static checks and unit tests are permitted.
No live forwarding, arming, fault injection, FCU parameter
changes or flight experiments are authorized by this report.
