# Person 1 — DataFlash GPS/IMU Exporter

## Purpose

Export GPS, GPS accuracy, IMU and attitude data from
ArduPilot DataFlash .BIN logs into CSV files.

This is an independent Phase 2 component.
It does not require running SITL or changing FCU parameters.

## Requirements

- Python 3
- pymavlink

## Usage

Run from the repository root:

    python3 integration/person1_phase2/dataflash_export.py       path/to/flight.BIN       integration/person1_phase2/output_run_001

The destination directory must not already exist.

## Output

- gps.csv: GPS coordinates, fix status, satellite count and accuracy
- imu.csv: gyroscope and accelerometer measurements
- attitude.csv: roll, pitch and yaw
- summary.json: counts, fix distribution and limitations

## Units

- GPS latitude/longitude: degrees
- GPS altitude: meters
- GPS horizontal/vertical accuracy: meters
- IMU angular velocity: rad/s
- IMU acceleration: m/s^2
- Attitude angles: degrees

## Timestamps

time_us is the original ArduPilot DataFlash timestamp.

timestamp_ap_ns = time_us * 1000.

These timestamps are NOT synchronized with Raspberry Pi,
camera or ROS timestamps.

Clock synchronization is required before GPS/VIO comparison.

## GPS validity

valid_fix = 1 when fix_type >= 3.

This is only a basic GPS fix check, not a guarantee
of actual measurement accuracy.

GPS accuracy, outliers, coordinate frames and timestamps
must also be checked before creating a reference trajectory.

## Verified SITL logs

ArduCopter 4.4.4.

00000002.BIN:
- GPS: 1989
- IMU: 20188
- ATT: 4037
- GPS with fix >= 3: 1988
- GPS/GPA matched: 1989

00000001.BIN:
- GPS: 0
- IMU: 112
- ATT: 22

## Tests

    python3 integration/person1_phase2/test_dataflash_export.py

The automated tests are self-contained and use synthetic
DataFlash messages. They do not require existing CSV files,
a running SITL instance, or physical flight hardware.

Verified:
- CSV counts and timestamps
- GPS/GPA matching
- Missing source handling
- Existing output protection
- Cleanup after simulated read failure

## Limitations

- Not validated on a physical flight controller
- Not synchronized with Raspberry Pi or camera
- No automatic GPS-to-local-coordinate conversion
- No VIO trajectory comparison
- No real-world GPS accuracy validation
- Some malformed logs may not be detected
- Atomic staging does not guarantee protection against
  every concurrent-write or power-loss scenario

## Safety

This script does not:
- Start SITL or Gazebo
- Change EKF3 parameters
- Enable VIO forwarding
- Arm motors
- Modify other team members' code

Phase 1 remains on HOLD.
