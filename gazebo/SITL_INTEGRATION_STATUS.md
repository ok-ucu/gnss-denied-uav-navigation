# ArduPilot SITL → Gazebo Integration Status

## Environment

- Host: macOS, Apple Silicon
- ArduPilot SITL: ArduCopter 4.4.4
- MAVLink: MAVProxy + pymavlink
- Gazebo: Gazebo Harmonic, ROS 2 Jazzy
- SITL Docker image: f5dffdb9f79e
- Gazebo Docker image: gz-sim:latest

## 1. SITL startup — PASS

ArduPilot SITL was successfully started inside Docker.

MAVProxy initially exited because the container was launched
without an open stdin. Running Docker with `-di` resolved this.

Verified log messages:

- Detected vehicle 1:1 on link 0
- AP: ArduPilot Ready
- AP: AHRS: EKF3 active

## 2. MAVLink transport — PASS

MAVProxy forwards MAVLink messages to the host:

    udp:host.docker.internal:14551

A real pymavlink receiver on the host confirmed:

- HEARTBEAT: PASS
- System ID: 1
- Component ID: 0
- LOCAL_POSITION_NED: 40 messages / 10 seconds
- ATTITUDE: 41 messages / 10 seconds

This test used the default GPS-enabled SITL configuration.

## 3. Gazebo Python dependencies — PASS

The Gazebo Docker image includes:

- Python 3.12
- ROS 2 Jazzy
- rclpy
- geometry_msgs
- lxml
- lxml
try_msgs
r image ince r image ince r image ince r imageng SITL container into a temporary host directory.

The Gazebo container successfully imported both pymavlink
and rclpy using PYTHONPATH, withoutand rclpy using PYTHO.

## 4. Remaining integration work

The following items have NOT yet been verified:

- Gazebo world startup during this integration run
- ROS 2 bridge publishing and receiving messages
- Live sitl_gazebo_sync.py execution without --mock
- Actual camera_drone movement driven by SITL
- GNSS-denied EKF3 operation with external V- GNSS-denied EKF3 operation with external V-ent - GNSS-denied EKF3 operation withele- GNSS-denied EKF3 operation with external V- Gner can import the re- GNSS-denion modules.

End-to-end SITL-driven Gazebo movement remains unverified.
