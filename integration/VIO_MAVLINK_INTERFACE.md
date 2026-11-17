# Phase 2 — Pi VIO / MAVLink interface

Software preparation only. Phase 1 HOLD / NO GO. `execution_allowed=false`,
`forwarding_enabled=false`. No physical forwarding route is implemented here.

## Actual producer and reusable components

[run_pi5_vio.py](run_pi5_vio.py) uses the pinned CPU FlowFrontend/VioEKF and
RosOdomPublisher; [PI5_VIO.md](PI5_VIO.md) describes its physical sensor inputs.
The candidate topic is `/integration/pi5/odom_candidate`, `nav_msgs/Odometry`.
Image/Imu/Range input topics and calibrations must be supplied explicitly.
Physical range is needed for optical-flow scale; no Gazebo height/GPS position
substitute is permitted. No CUDA/GPU/Docker is needed by this candidate path.

| Field | Actual semantics |
|---|---|
| `header.frame_id` | `pi5_odom_ned`: local metric NED reference, not automatically FCU origin/yaw |
| `child_frame_id` | `pi5_imu_frd`: **Pi IMU sensing point**, FRD axes |
| pose | Position in NED [m]; quaternion maps FRD body vectors into NED world |
| twist | Linear [m/s] and measured angular [rad/s] in body FRD |
| covariance | Full row-major 6×6 pose and twist, including cross terms; m², rad², (m/s)², (rad/s)² and corresponding mixed units |
| timestamp | Original acquisition integer nanoseconds, mapped by sensor adapters to `ros-system-unix-ns`; never publish time |
| candidate readiness | Producer quality checks passed; **not** FCU authorization |

## Explicit conversion

[physical_vio_contract.py](physical_vio_contract.py) validates Pi frame semantics,
time, readiness, session/reference and covariance before reusing
[vio_frame_bridge.py](vio_frame_bridge.py) and [vio_adapter.py](vio_adapter.py).
The original message is not mutated. There is no ROS publisher in this module.

```
N = [[0,1,0], [1,0,0], [0,0,-1]]  # NED -> ENU
B = diag(1,-1,-1)                 # FRD -> FLU
p_enu = N p_ned
R_enu_flu = N R_ned_frd B
v_flu = B v_frd; omega_flu = B omega_frd
P_pose_enu = diag(N,N) P_pose_ned diag(N,N)^T
P_twist_flu = diag(B,B) P_twist_frd diag(B,B)^T
v_world_enu = R_enu_flu v_flu
```

World-velocity covariance uses the existing adapter's full block rotation;
cross members are retained. It treats the orientation as known for this
rotation: unavailable pose/twist joint covariance and attitude-induced velocity
uncertainty are **not** reconstructed. Physical covariance calibration remains
NOT VERIFIED. Unknown angular covariance (`-1` sentinel), zero variance on any
twist axis, non-PSD/asymmetric/nonfinite matrices are rejected before the legacy
bridge could substitute an approximate angular variance.

Outputs retain `pi5_odom_enu / pi5_imu_flu`; they are NOT silently renamed to the
FCU body. A calibrated Pi-IMU-to-FCU rotation/translation, body origin, local
origin and yaw convention require Person 1 + Person 3 confirmation. A nonzero
lever arm changes position and velocity (`omega × lever`), with uncertainty;
this physical extrinsic correction is deliberately not guessed or implemented.

`mavlink_preview()` encodes **in-memory diagnostic bytes only**, reusing
[analyze_mavros_wire.py](analyze_mavros_wire.py)'s already tested MAVROS wire math.
Only VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE are encoded; no command
messages or I/O. The API requires the original `PiCandidateGuard` as `guard`,
current acquisition-domain `now_system_ns` and current monotonic `host_ns`.
The latest process-local admission is revalidated at encoding: latched loss/reset,
stale acquisition (including uncertainty), invalid numeric/covariance values,
mutated data, mismatched pose/speed/header timestamps and foreign/old admissions
are rejected. Pose and world speed are regenerated from the checked Odometry
and compared with the supplied pair. No timer or cached authorization is created.
It also requires an existing verified
[ClockMapping](navigation_clock.py), matching source session, fresh anchors,
explicit sender IDs and epoch zero. It preserves the Pi reference; generating
these bytes does not establish FCU reference alignment or permission to send.
The result is explicitly `non_transmittable=true`: one isolated two-packet
fixture, seq 0/1, restarting at zero each call. It is NOT a continuous wire-stream
encoder and cannot be substituted for a reviewed physical transport.

## Timing, availability and reset contract

Existing Pi limits are retained: publication age + supplied clock uncertainty
≤180 ms; receipt availability ≤500 ms host monotonic; IMU stamp gap ≤20 ms;
camera stamp gap ≤300 ms; sensor clock uncertainty <5 ms. These are software
guards, not claims about achieved hardware performance. No new sensor cadence
or GPS accuracy threshold is invented. System-clock acquisition time and host
monotonic arrival time are separate inputs. No ROS `/clock` on physical Pi.

`PiCandidateGuard` requires explicit source session, reference, integer epoch=0
and ready=true. Duplicates, resets, changed session/reference, stale/future
samples, invalid covariance and receipt loss latch failure. Call
`check_availability(monotonic_ns)` independently during silence. Recreate the
guard only after a separately reviewed new reference initialization; there is
no automatic recovery or last-position replay. This in-memory guard is **not**
a persistent TX authorization mechanism.

Current ROS Odometry has **no epoch/reference status side-channel**. The Pi
JSONL has a session UUID, but JSONL is not a runtime authorization channel.
Person 3 must supply a synchronized readiness/session/reference/epoch contract
before a physical subscriber-to-FCU forwarding path can be reviewed.

Preview wire timestamp = mapped FCU acquisition ns //1000 (≤999 ns truncation),
not copied UNIX time or callback time. An affine fit to arrival pairs is not
clock proof. Physical Pi↔FCU clock mapping, uncertainty, drift, restart and
wrap behavior require same-session evidence. Existing SITL 4.4.4 clock/firmware
proofs and patched MAVROS simulation overlay do not certify this physical FCU.

## ArduCopter 4.6.3 compatibility, not fusion proof

VPE carries local NED position and roll/pitch/yaw, 21 upper-triangular covariance
elements; VSE carries world NED velocity and 9 row-major covariance elements.
Both include reset_counter. MAVLink 2 framing is required by the diagnostic.
See the [MAVLink message definitions](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE).

The exact [4.6.3 GCS handlers](https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/libraries/GCS_MAVLink/GCS_Common.cpp#L3653-L3707)
perform offboard timestamp correction. VPE diagonal terms are reduced to scalar
position/angle errors; VSE covariance is not passed to its backend. The
[4.6.3 MAV visual-odometry backend](https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp#L23-L66)
uses configured velocity noise. Wire covariance integrity therefore does not
prove that EKF3 fuses the full matrix. VISP/VISV counts are receipt/logging
evidence, not numbers of accepted EKF updates.

The [ArduPilot ExternalNav guide](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
recommends ≥4 Hz and prefers ODOMETRY; this preparation retains the existing
VPE/VSE contract. Actual rate, gaps, configured VISO noise/delay/scale and EKF3
source selection must be verified on **this** firmware by Person 1. No parameter
write or navigation-source change is supplied. MAVROS 2.16.0 reset_counter=0
cannot express arbitrary restarts; the guard blocks changes rather than
pretending continuity. Physical MAVROS/native TIMESYNC remains NOT TESTED.

## Physical RX is separate

[physical_mavlink.py](physical_mavlink.py) opens an explicitly configured tty
**O_RDONLY**, configures host baud/raw mode, requests exclusive access and only
reads. It never requests streams, AUTOPILOT_VERSION, TIMESYNC or heartbeat rates.
Three valid expected-source DISARMED heartbeats spanning ≥2 s plus a matching
passively received AUTOPILOT_VERSION UID/4.6.3 establish protocol identity only.
Diagnostic PASS additionally requires at least two passive SYSTEM_TIME samples
with increasing boot time and progress within the existing 3 s telemetry bound.
Frozen/repeated samples do not refresh this evidence. Missing/stale boot evidence
produces `boot_continuity.status=NOT VERIFIED` and overall NOT VERIFIED (CLI exit
2), even when `protocol_identity_status=PASS`. Boot PASS covers **observed passive
progression**, never proof that every reboot was detected. An unobserved reboot
can escape detection; no boot identity or cryptographic guarantee is inferred.
The existing 3 s stale-heartbeat /10 s initial-heartbeat rules are reused.
BAD_DATA is counted, never identity evidence. EOF, I/O errors, armed/wrong-source
heartbeat, UID/version mismatch and observed FCU boot-clock rollback fail closed.
There is no reconnect, write/send API or physical forwarding option. A link that
does not passively emit AUTOPILOT_VERSION cannot pass; no query is sent to make
it pass. Reported UID/version/CRC checks are **not cryptographic authentication**,
board model attestation or exact firmware-binary provenance.

Host tty configuration/USB open can have driver/electrical side effects;
TIOCEXCL does not prove that no process already holds a descriptor. Port
ownership, UART2-to-SERIAL mapping and electrical connection require Person 1's
independent approval before opening a physical port. No port was opened here.
