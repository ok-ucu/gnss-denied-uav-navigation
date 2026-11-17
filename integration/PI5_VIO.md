# Raspberry Pi 5 CPU-only VIO bring-up — OFFLINE IMPLEMENTED

Phase 1 HOLD / NO GO. `execution_allowed=false`, `forwarding_enabled=false`.
No hardware, ROS runtime, FCU or flight was exercised for this change.

[run_pi5_vio.py](run_pi5_vio.py) is a separate native entrypoint; existing Linux
GPU/flight launchers, transport policies and teammate files are unchanged.
[pi5_vio.py](pi5_vio.py) reuses the pinned `FlowFrontend`, `VioEKF`, image decoder,
`RosOdomPublisher`, spatial audit, quality gate and camera-lever correction.
No CUDA, Torch, TensorRT, Gazebo, Docker, MAVROS or FCU endpoint is needed.
This is velocity-aided inertial dead reckoning, not an attitude estimator,
SLAM system or terrain estimator. It still requires external attitude and scale.

## OS/dependency decision (checked 2026-11-17)

Recommend **Ubuntu Server 24.04 64-bit + ROS 2 Jazzy + system Python 3.12**.
Jazzy lists Ubuntu Noble ARM64 as Tier1 with binaries. Its reference dependencies
include Python3.12.3, NumPy1.26.4 and OpenCV4.6.0; these are reference versions,
not a claim about packages installed on an unavailable Pi. See
[REP2000](https://reps.openrobotics.org/rep-2000/) and
[Ubuntu python3-minimal](https://packages.ubuntu.com/noble/python3-minimal).

[Raspberry Pi OS 64-bit](https://www.raspberrypi.com/software/operating-systems/)
currently uses Debian Trixie; [its Python is3.13](https://packages.debian.org/trixie/python3).
Legacy [Bookworm uses3.11](https://packages.debian.org/bookworm/python3).
Jazzy lists Bookworm as Tier3/source-only, not the Ubuntu binary target; Trixie
is not listed as a Jazzy target. Do not install Ubuntu ROS debs into Raspberry
Pi OS or mix its Python with Jazzy binary extensions. Pure CPU algorithms can
potentially run there, but this native ROS runner deliberately admits only the
reviewed Jazzy/Python3.12 combination. A Pi OS ROS source build/port is NOT TESTED.

The core uses NumPy9x9 EKF and CPU OpenCV Shi-Tomasi/LK/RANSAC. No architecture
specific instruction or CUDA API is required by their Python source. The old
amd64 firmware hashes, NVIDIA EGL selection, Ogre2, Docker images and patched
MAVROS TIMESYNC overlay belong to simulation/FCU integration and are not loaded
here. MAVROS/FCU forwarding remains a separate blocked integration step.

Install ROS Jazzy using the
[official repository procedure](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
on the actual Ubuntu Pi first. With that apt repository configured:

```bash
sudo apt-get install ros-jazzy-ros-base python3-numpy python3-opencv
source /opt/ros/jazzy/setup.bash
uname -m
python3 -c 'import sys,numpy,cv2,rclpy; print(sys.version); print(numpy.__version__,cv2.__version__)'
printf '%s\n' "$ROS_DISTRO"
```

These are preparation commands, not performed by this implementation. Record
actual kernel, firmware, package/driver versions, OpenCV build information and
thermal/throttling metrics during hardware bring-up. Pi5 throughput/latency and
the required sensor cadence have NOT been established by Mac tests.

## Explicit hardware adapters and measurement contract

No `/dev/*`, I2C address, CSI index, serial port or ROS topic is guessed. Drivers
are deliberately outside this runner; select actual hardware and inspect its
ROS output. Fill a NEW local copy of [pi5_vio.example.json](pi5_vio.example.json).
The template contains nulls and `calibration_verified=false`: it MUST fail.
`calibration_evidence` and `clock_evidence` are existing files with relative
paths and SHA256 digests. They document operator-reviewed physical measurements;
a hash proves integrity, not physical calibration or clock correctness.

| Adapter | Required measured data | Required installation/evidence |
|---|---|---|
| Camera driver → `sensor_msgs/Image` | Raw calibrated pinhole/plumb_bob; mono8/rgb8/bgr8/rgba8/bgra8, exact dimensions, exposure acquisition stamp | CSI/libcamera or USB/V4L2 driver chosen after hardware inventory; K,D at the operating resolution/focus; image X body-right, image Y body-backward, view body-down within1°; camera-to-IMU translation in metres |
| IMU/AHRS driver → `sensor_msgs/Imu` | Specific force including gravity, m/s²; gyro rad/s; normalized **sensor→ENU** quaternion; finite nonzero accel/gyro/attitude covariance | Calibrated `R_body_imu`; axis/sign/bias checks; independent AHRS/orientation with documented yaw reference. A raw6-axis IMU with missing quaternion is insufficient |
| Downward range driver → `sensor_msgs/Range` | Physical slant metres, finite strict min/max, acquisition stamp | Body-down beam, measured range-sensor lever; measured conservative vertical uncertainty `range_sigma_m` including projection/calibration; local static horizontal ground plane; explicit maximum range age and tilt |
| Timestamp mapping in the drivers | Integer nanosecond acquisition stamps in `ros-system-unix-ns` | Device→host clock mapping, offset direction, drift/uncertainty and measured exposure semantics for ALL streams; conservative combined pairwise/host uncertainty bound must fit5ms IMU alignment |

IMU origin defines output body origin; its rotation to FRD is calibrated, not
inferred from a sensor name. Camera rotation (optical→FRD) must be
`[[0,-1,0],[1,0,0],[0,0,1]]` within1°. Translation is never discarded. If the
camera cannot be mounted this way, stop: changing FlowFrontend is outside scope.
Range must point down; its projected height includes the range lever at its own
acquisition attitude, then camera height includes the camera lever at frame time.
No barometric/GPS/FCU position, Gazebo pose or constant1m fallback is provided.
Without range or another separately designed metric-scale adapter: **BLOCKED**.
Changing terrain invalidates range-as-vertical-displacement; this mode is for
stationary/handheld bring-up over a known local ground plane, not flight approval.

`R_body_imu` maps IMU axes to FRD; quaternion maps IMU to ENU. The runner forms
`R_body_to_ned = R_enu_to_ned @ R_enu_imu @ R_body_imu.T` and rotates gyro/force
to FRD. The EKF adds NED gravity itself; no second compensation is applied.
At rest, expected FRD specific force is `[0,0,-9.80665]`. No constant force or
orientation is fed into the real runner. Images are rectified using measured K,D
to centred, equal-focal-length pinhole images, matching the frontend's model.

## Bounded commands, after hardware-specific configuration exists

From repository root; no dependency copies into teammate directories:

```bash
set -euo pipefail
: "${P5_CONFIG:?path to measured hardware JSON config}"
: "${P5_VIO:?NEW output path below integration/linux_gpu_runs}"
# Reuse existing pinned source exporter; despite its directory name it uses no GPU.
python3 -B integration/linux_gpu/prepare_vio.py --output "$P5_VIO"
# Offline admission only, no subscription/publication:
python3 -B integration/run_pi5_vio.py --config "$P5_CONFIG" --vio-dir "$P5_VIO"
```

Exporter requires Git object `d33f66008fd7b6de332c96dd324b8ddbd320c194`; all four
VIO files are checked against its existing fixed hashes. A missing object or
changed file is a blocker, never silently accepted. Use an existing verified
export without re-running the exporter if that directory already exists.

After approval for physical sensor-only bring-up, with drivers already running:

```bash
source /opt/ros/jazzy/setup.bash
: "${P5_EVIDENCE:?NEW JSONL path in a local ignored runtime directory}"
python3 -B integration/run_pi5_vio.py --config "$P5_CONFIG" --vio-dir "$P5_VIO" \
  --receive --duration 30 --output "$P5_EVIDENCE"
# Separate new output file, only after reviewing receipt/quality logs:
python3 -B integration/run_pi5_vio.py --config "$P5_CONFIG" --vio-dir "$P5_VIO" \
  --receive --publish --duration 30 --output "${P5_EVIDENCE}.candidate.jsonl"
```

`--receive` subscribes only; `--publish` additionally emits
`/integration/pi5/odom_candidate` with `pi5_odom_ned` / `pi5_imu_frd`. It cannot
select MAVROS topics, is not `/vio/odom`, supplies no transport epoch authority
and opens no MAVLink connection. Keep FCU disconnected. Do not attach production
frame bridge or transport during bring-up. No full hardware correctness is
claimed by a `BOUNDED_BRINGUP_COMPLETE` diagnostic result.

## Timing, quality and fail-closed behavior

Acquisition nanoseconds are preserved exactly in candidate Odometry. Relative
seconds are used only inside optical-flow/EKF math; no float Unix timestamp is
rounded back into a ROS stamp. `use_sim_time` must be false. Callback receipt and
CPU/wall processing durations use host monotonic/thread clocks separately.
No `/clock` bridge or published-time substitute is used. Exposure-to-host mapping
must be proven by the drivers; host receipt alone is not exposure evidence.

Raw IMU stamps are checked before propagation; callbacks also reject duplicates
and resets. Gaps >20ms, nonfinite values and unknown covariance latch failure. Causal held IMU is
integrated in stamp order to frame acquisition, with5ms alignment including clock
uncertainty; no interpolated/invented IMU samples or large-dt clamp. Range is
merged with IMU events and corrected at its acquisition time, once per new
measurement. Already committed estimator time is never rewound: late range
events fail closed. Startup uses only the latest causal range to define zero
vertical displacement. Holding its measured scale is bounded by the explicit
range-age contract; a held sample is not repeatedly fused as independent data.
The EKF's acceleration noise cannot be reduced below its default and grows to
cover the largest measured force covariance used in propagation.

Existing limits remain: frame gap300ms, final publication age180ms, adapter
receipt availability500ms. Spatial audit/inliers/contrast/PSD/finite-state and
readiness checks reuse the existing quality gate. IMU/range queues are bounded;
overflow fails. Camera keeps at most two queued frames, counts overwritten ones
and checks actual stamp gaps; it never republishes old Odometry. Image conversion
and estimation run outside receiver locks. A receiver thread is separate from
the CPU consumer; it is NOT a proven Pi latency/throughput guarantee.

On critical sensor/clock/estimator failures, exit FAIL and stop publication.
Low contrast/unavailable spatial support blocks output and requires readiness
again. There is no automatic reset/reinitialization on gaps; restart creates a
new local reference/session and must not be treated as continuous navigation.
Postprocessing and final publication recheck freshness/covariance. Angular twist
uses measured FRD gyro/covariance, replacing the upstream publisher's unknown
angular sentinel. Covariance consistency/calibration remains a hardware task;
PSD alone is insufficient. Logged sessions carry no flight/forwarding authority.
The upstream9-state EKF does not estimate attitude or propagate a complete joint
attitude/extrinsic uncertainty model; supplying measured orientation covariance
does not remove that limitation. Hardware consistency testing is required.

## Remaining acceptance work

Verify drivers, measured clock offsets/drift, frame extrinsics and metric range;
stationary gravity/bias then controlled handheld motion with independent reference;
Pi frequency/CPU/thermal/dropouts, independent subscriber delivery, measured
covariance consistency and resets. No numerical Pi FPS is promised. Raw6-axis
IMU attitude estimation, alternative scale adapters, arbitrary terrain/camera
mounts, Raspberry Pi OS native ROS builds and FCU/MAVROS integration are outside
this implementation. They remain explicit blockers where applicable.
