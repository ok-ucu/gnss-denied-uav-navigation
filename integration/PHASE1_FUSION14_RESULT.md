# Phase 1 — fusion14 simulation integration result

## Outcome

Simulation-only ExternalNav integration demonstrated successfully.
Flight readiness and runtime admission remain HOLD.

## Verified evidence

- Run: `20261117-fusion14`
- VIO odometry publications: 369
- Transport odometry received: 369
- Mapped odometry pairs: 368
- Vision packets sent to FCU socket: 736
- ArduPilot DataFlash VISP records: 368
- ArduPilot DataFlash VISV records: 368
- EKF3 IMU0 and IMU1 explicitly reported using external nav data
- GPS_TYPE=0, EK3_SRC1_POSXY=6, EK3_SRC1_VELXY=6
- Isolated IMU unit tests: 8 passed
- No isolated IMU worker fault or transport watchdog in fusion14
- No armed samples or control commands

## Remaining limitations

- Last VISP: FCU time 62.749626 s
- EKF variance reported at 70.744356 s
- EKF3 IMU0 and IMU1 stopped aiding at 77.778992 s
- The causal relationship between stream termination and EKF warnings
  has not been independently established
- Maximum observed IPC delivery latency: 121.5 ms
- A hard IPC delivery-time bound has not been established
- This run does not establish real-hardware or flight readiness

## Safety disposition

- Decision: HOLD
- Runtime admitted: false
- Execution allowed: false
- Forwarding enabled: false
- Simulation remained DISARMED
- No ARM or TAKEOFF authorization

## Evidence locations

- `integration/phase1_runs/20261117-fusion14/evidence/`
- `integration/phase1_runs/20261117-fusion14/state/fcu/logs/00000001.BIN`

## Scope of completion

Phase 1 simulation ExternalNav ingress and EKF3 adoption demonstrated.
Full safety acceptance, bounded latency, loss-of-vision behavior,
and flight readiness remain open.
