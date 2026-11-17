# Person 4 — Phase 2 software handoff

Phase 1 HOLD / NO GO. Hardware integration NOT VERIFIED.
`execution_allowed=false`, `forwarding_enabled=false`. No flight commands,
physical TX route, parameter writes or automatic recovery are implemented.

## Readiness and dependencies

| Task | Prepared offline | Still blocked / NOT VERIFIED |
|---|---|---|
| 4.1 physical transport | Explicit USB/UART config, RX-only tty, separate protocol identity and observed boot-progress checks, raw capture and counters | Pi/FCU wiring, real port/USB behavior, passive UID/version/boot telemetry, firmware provenance; unobserved reboot detection not guaranteed; physical TX requires separate reviewed authorization |
| 4.2 VIO contract | Actual Pi frames, pure conversions, covariance/time/reset checks, memory-only MAVLink preview | Physical calibration, real scale/range, clock proof, Pi-to-FCU lever arm/yaw/origin, live epoch side-channel, actual MAVROS/FCU validation |
| 4.3 analysis | RMSE/drift with full-reference path denominator; Pi/DataFlash/CSV/GPS adapters, integer-ns alignment, world displacement error, conditional standard SE(3) translational RPE, JSON/CSV/plots, clock-proven boundary latency | Verified aligned attitudes for RPE, raw hardware BIN decoding, reference accuracy, rotational RPE, paired FCU reception latency and actual plots on this Mac |
| 4.4 handoff | Configuration, staged ground procedure, failure handling and evidence list below | Separate physical connection/test approval; no physical test conducted |

Reuse: [Pi CPU path](PI5_VIO.md), [physical contract](VIO_MAVLINK_INTERFACE.md),
[frame bridge](vio_frame_bridge.py), [adapter](vio_adapter.py),
[heartbeat guard](linux_gpu/verified_disarmed.py), [clock mapper](navigation_clock.py),
[BIN decoder](analyze_simulated_flight.py), [EKF statistics](analyze_ekf3_fusion.py).
Phase 1 SITL 4.4.4 transport/permits/flight guards and Stage 4 are unchanged.

## Before any hardware connection — Person 1 + Person 3

1. Person 1: exact Stellar H7 V2 revision, confirmed ArduCopter 4.6.3 build hash,
   reported FCU UID, system/component IDs, safe disarmed bench configuration and
   port ownership. Verify UART2 TX2/RX2 board-to-ArduPilot SERIAL mapping from
   board-specific documentation; do not assume SERIAL2. Confirm signal voltage,
   common ground, pinout, absence of conflicting peripherals and approved baud.
   No electrical assumptions or device paths are supplied by this work.
2. Person 1: provide read-only parameter export, boot identity and origin/home/yaw
   conventions; review VISO/EK3 source/noise/delay settings without changing them
   for this diagnostic. Physical ExternalNav admission is a later approval.
3. Person 3: calibrated camera/IMU/range topics, units, rigid transforms, exposure
   timestamp semantics and physical provenance. Confirm metric scale from a real
   height sensor/static-surface model; no constant height or horizontal GPS/GT
   input. Supply measured covariance, source session/reference/epoch status,
   readiness and explicit new-reference recovery protocol.
4. Joint: same-boot Pi acquisition→FCU time mapping with measured uncertainty,
   drift and reset evidence; calibrated Pi IMU→FCU extrinsics/lever arm and local
   reference. UNIX timestamps matching numerically are not sufficient proof.

Ubuntu Server 24.04 ARM64 + ROS 2 Jazzy/Python 3.12 is the existing Pi runner's
admitted platform. Raspberry Pi OS needs an independently verified ROS package/
Python path; do not bypass its host checks. Core adapters/analysis are stdlib;
CPU VIO needs NumPy/OpenCV and pinned teammate sources; candidate ROS conversion
uses NumPy through existing validators. Optional `pymavlink` is needed only for
**local raw BIN decoding**, Matplotlib only for plots. No dependency is installed
automatically. See [source dependencies](SOURCE_DEPENDENCIES.md).

## Commands — offline now, physical receive only after separate approval

From repository root, using a Python environment with existing NumPy/OpenCV:

```bash
export PYTHONPATH="$PWD/integration:$PWD/integration/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m unittest test_physical_mavlink test_physical_vio_contract test_physical_analysis -v
python3 -B integration/audit_source_scope.py
python3 -B integration/physical_mavlink.py --help
python3 -B integration/analyze_physical_vio.py --help
```

Fill a **new local** copy of
[physical_mavlink.example.json](physical_mavlink.example.json) with independently
confirmed `device`, `baud`, `expected_system`, `expected_component`, `expected_uid`.
The null-valued template intentionally fails. Do not guess a device pathname.
Configuration check below opens no port:

```bash
python3 -B integration/physical_mavlink.py --config "$RX_CONFIG"
```

Only after Person 1 authorizes a physical, disarmed, receive-only ground test:

```bash
python3 -B integration/physical_mavlink.py --config "$RX_CONFIG" \
  --receive --duration 10 --output "$NEW_RX_RUN"
```

`RX_CONFIG` and `NEW_RX_RUN` must be explicit operator-selected paths; output
must not already exist. This is **not** a forwarding command. Preserve
`rx.mavlink`, `events.jsonl`, `result.json`; protocol identity and observed boot
continuity have separate verdicts. Overall PASS requires both; neither authorizes
TX. Missing boot progress: NOT VERIFIED / exit 2, not PASS. Missing heartbeat/
version, armed state, disconnect, observed boot rollback or stale heartbeat: FAIL
/ exit 1, close the descriptor, save evidence,
no auto-reopen. A reboot not visible in passive telemetry is not guaranteed
detectable. Ctrl+C also closes and records failure. A close/restore error
is reported as FAIL. Do not terminate unrelated serial users by process name.

For Pi sensor bring-up, follow the existing [PI5_VIO.md](PI5_VIO.md) commands
after physical calibration/clock admission. Separate Pi candidate and FCU RX
logs by session. This work deliberately supplies **no command to connect a
candidate topic/MAVROS to a physical FCU**. Review time/reference/epoch contract
and a persistent final-boundary authorization mechanism first.

Stop the diagnostic normally or by Ctrl+C; review the saved result and device
closure. Any new process/FCU/VIO boot/session requires a new recorded test and
reference review. Never auto-reenable following a reset or reuse old permission.

## Offline trajectory report

[analyze_physical_vio.py](analyze_physical_vio.py) reads only files, never an FCU
endpoint. [analyze_trajectories.py](analyze_trajectories.py) reuses Person 4's
historical `368e531` RMSE/drift/bounded interpolation implementation. Reviewed
corrections retain integer nanoseconds and use every reference knot over the
evaluated interval for path length; a second aligner is not introduced.

```bash
python3 -B integration/analyze_physical_vio.py \
  --config "$ANALYSIS_CONFIG" --output "$NEW_ANALYSIS_RUN" --plots
```

Config paths are relative to the config file. Schema (no inferred defaults):

```
reference, candidate: {
  path, format, position_unit: "m", time_unit, clock_domain, session, reference_id,
  mapping: {
    reviewed: true, reviewer, evidence_path, evidence_sha256,
    source_clock, source_epoch, target_clock, target_epoch,
    uncertainty_ns, max_uncertainty_ns, max_anchor_age_ns,
    anchors: [{source_ns, target_ns, host_ns}, ... >=3 strictly ordered]
  }, ... format-specific fields
}
max_interpolation_gap_s: explicit positive bound
rpe_delta_samples: explicit positive integer
max_rpe_interval_s: explicit positive bound
source_availability_limit_ns: explicit criterion (existing Pi receipt limit: 500000000)
latency_jsonl: optional paired boundary log path
latency_clock_proof: optional reviewed, hashed shared-clock evidence (below)
orientation_pairs: optional reviewed, hashed exact-time attitude pairs (below)
```

Supported formats: `pi-jsonl` (actual Pi frame log, acquisition `ns`,
`ros-system-unix-ns`); `dataflash` (local raw `.BIN`, `us`, `FCU_BOOT_TIME_US`,
explicit `core`); `enu-csv` (`east,north,up`); `ned-csv` (`x,y,z`);
`gps-csv` (`lat,lon,alt` degrees/ellipsoid meters, `geodetic_origin` lat/lon/alt,
`altitude_datum=WGS84_ellipsoid_m`). CSV requires explicit `time_column` and
`time_unit` (`s/ms/us/ns`). Position magnitudes cannot establish clock units.

Both mappings must target the same named clock/epoch, cover all evaluated
samples and bind their source sessions. Review the external clock proof and
record its SHA-256; a hash/fit does not independently validate the physical
claim. Replay uses the mapper's calibrated interval, **not** a claim about live
anchor freshness. Same `reference_id` means independently established common
origin/yaw; no automatic origin shift, yaw fit or scale correction is performed.
Missing/mismatched proof or reset refuses metrics; it cannot yield readiness PASS.

Reports: `report.json`, `samples.csv`; optional `xy.png`, `xyz.png` only from
supplied samples. Missing Matplotlib is explicitly NOT TESTED. ATE here is
translation RMSE in the agreed reference. `world_displacement_error` is the
change in world-frame position residual; it is **not SE(3) RPE**. Standard
translational RPE is `||trans((Q_i^-1 Q_j)^-1 (P_i^-1 P_j))||`, calculated only
with verified normalized orientations in a common physical body/world convention.
Without them, `rpe.status=NOT VERIFIED`, `rmse_m=null`. No reference attitude is
invented or interpolated. Drift is change in position residual. Exclusions/overlap are
reported. Dropouts cover interior publication intervals; leading/trailing
availability needs independently measured observation bounds. An error relative
to GPS is **not** a certified true error without independent GPS accuracy proof.
The path denominator includes all recorded reference knots between the first
and last matched samples, including turns/loops missed by VIO sampling. Any
reference gap above `max_interpolation_gap_s` makes `truth_path_m` and drift
percentage null / NOT VERIFIED; covered distance and gap intervals are retained.

Optional `orientation_pairs` proof fields: `path`, `sha256`, `verified=true`,
`reviewer`, `body_frame`, `convention=body-to-world-ENU-xyzw`, `reference_id`,
`reference_sha256`, `candidate_sha256`, `reference_session`, `candidate_session`,
`target_clock`, `target_epoch`. The reviewed artifact is JSONL with strictly
increasing **mapped common-clock integer** `time_ns`, `reference_xyzw` and
`candidate_xyzw`. Every compared timestamp must match exactly. Hashes bind both
trajectory files and the attitude artifact. Review must attest calibrated common
body axes and the physical provenance; numeric SO(3) validation alone does not.
Missing proof leaves RPE unverified; an inconsistent supplied proof fails analysis.

The standalone `analyze_trajectories.load()` and `timestamp()` now return exact
integer nanoseconds. Call `compare(..., time_unit='ns')` for their output. Default
`compare` input remains seconds for explicit legacy callers. CSV unit parsing and
strict ordering occur before relative-second conversion; fractional ns and float
absolute-ns inputs are refused. The CLI's offset/max-gap arguments remain seconds.

DataFlash adapters export XKF1 trajectory plus descriptive XKF3 innovations,
XKF4/XKFS status, aiding text and VISP/VISV counts. They do not automatically
declare EKF3 fusion or count accepted updates. Confirm schema on actual 4.6.3
logs. No old synthetic analyzer verdict is reused as a hardware verdict.

Optional latency JSONL schema: `event=publish|forward`, `session`, integer
`epoch`, integer `acquisition_ns`, `host_boot_id`,
`clock_domain=HOST_MONOTONIC_NS`, `host_ns`, `time_namespace`, `clock_instance`;
forward additionally requires
`boundary=os_write_complete`. Duplicate, reversed or cross-boot pairs fail;
unpaired samples are counted. A shared-clock proof is also required:
`kind=shared-monotonic-instance`, `verified=true`, `reviewer`, `host_boot_id`,
`time_namespace`, `clock_instance`, integer `valid_from_ns`, `valid_to_ns`,
`evidence_path`, `evidence_sha256`, `trace_sha256`. The boot/namespace/clock IDs
and time interval must match both records; both proof artifact and trace hashes
are checked. Same boot ID alone is insufficient. Missing proof gives no measured
latency (NOT VERIFIED); mismatched namespaces/instances are rejected. This module
does not normalize clocks: cross-domain data need a separately verified export
into one clock instance with retained original timestamps and conversion evidence.
This measures publication→OS write completion,
not wire/FCU arrival. Existing capture-stage permission timing remains in
[permission_deadlines.py](permission_deadlines.py). FCU reception latency stays
NOT VERIFIED until paired receipt records and same-boot clock bounds exist.

## Evidence to return and recovery

- Person 1: approved wiring/port map/ownership; FCU boot/UID/build/version,
  read-only parameter export; RX raw bytes/chunk stamps/result; raw DataFlash
  BIN + format/version info, origin/yaw, VISP/VISV, XKF source/innovation/status.
- Person 3: original calibration/clock artifacts and hashes; sensor manifest,
  Pi source pin, raw acquisition stamps, exact Odometry samples/covariance,
  independent subscriber counts, epoch/readiness transitions and Pi JSONL.
- Joint: commands, software versions, session/boot IDs, reference extrinsics,
  reviewed mappings/uncertainty and availability observation bounds, report
  JSON/CSV. Preserve unsuccessful attempts. Do not infer capture→EKF latency
  from message counts or unaligned host/FCU timestamps.

No actual Pi/FCU/ROS/MAVROS, Linux serial driver or hardware clock test was run
by this offline preparation. A hardware fault requires operator inspection and
fresh explicit initialization, never a threshold relaxation or stale replay.
