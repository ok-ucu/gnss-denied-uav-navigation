# Phase 1 visual availability and inertial propagation

## Decision

Keep Phase 1 publication and transport fail-closed. IMU-only output remains
an unauthorised design candidate. This change filters identical IMU deliveries
and makes no change to navigation authority, output cadence or safety budgets.

Marta's abe2 reproduction describes the current architectural constraint.
A previous Linux change closes a separate hole by rejecting stale estimates
during readiness recovery too. The active watchdog uses DDS callback receipt
in host-monotonic time. The readiness span and source freshness use simulation
time. At 10 Hz, following one rejected frame, four consecutive acceptable
frames are required to span at least 250 ms. The next eligible acquisition is
500 ms sim time after the last eligible acquisition. For sim/host ratio R,
the host gap is approximately 500 ms / R plus the difference in processing
and delivery latency between the endpoints. At R=1 the nominal margin is
zero. At R=0.616 it is approximately −312 ms before endpoint jitter.
The readinessfresh01 evidence had zero of 344 five-period DDS windows at or
below 500 ms. Schedtrace111431 independently had a shortest window of
677.36 ms and a median of 809.86 ms. Isolated identical IMU delivery
does not need to enter this recovery path.

## Current estimator and interfaces

The pinned estimator propagates a nine-state position, velocity and
accelerometer-bias model using accelerometer input and a supplied orientation.
It updates covariance with F P Fᵀ + Q and symmetrises it. The IMU propagation
path already runs between camera updates. It uses the configured process
noise and a supplied orientation rather than a joint attitude uncertainty
model. In a stationary offline prediction from the default prior, using 5 ms
steps and no visual corrections, the position diagonal grew from 0.0100
to 0.01112 m² at 250 ms and 0.01682 m² at 500 ms. The velocity
diagonal grew from 0.0100 to 0.03500 and 0.06500 (m/s)² respectively.
Nominal covariance growth establishes forward propagation of model uncertainty. It does not calibrate drift under bias, orientation error,
vibration, correlated errors or camera loss. The source's existing vertical
simulation aid further limits what this simulator setup proves about
independent inertial navigation.

Today the source gate requires visual support, motion audit, IMU alignment,
clock validity, finite state, covariance bounds and readiness. The
transport consumes odometry plus an epoch token, handles clock-mapped
freshness and forwards vision position/speed pairs under an isolated permit.
There is no authenticated degraded-odometry mode in that contract. Sending
inertial estimates as ordinary ExternalNav would conceal the loss of
visual aiding. FCU receipt of VISP/VISV alone would establish neither EKF3
consistency nor that correlated prediction errors are correctly weighted.

A future inertial bridge would need a separately validated, explicit
degraded mode from source to transport. It would propagate from a
qualified visual anchor to each real, strictly increasing IMU acquisition
stamp, publish propagated covariance, and retain the anchor age, source
epoch, and reason for degraded state. Missing or invalid IMU, clock or
epoch changes, stale acquisition, excessive covariance, or expired anchor
would close the bridge. Return to visual authority would still require
the full visual readiness interval. Continuously publishing IMU samples
must not keep visual authority alive. It must be clear how the degraded
state is handled when its deadline expires.

No defensible maximum coast interval is established. Choose one only
after matched ground-truth evaluation of drift, covariance coverage and
FCU innovations under representative motion and loss modes. Replay on
recorded data and isolated DISARMED validation should precede any request
to permit degraded ExternalNav. Any runtime prototype must be opt-in,
off by default and isolated from production flight authority.

## Options

| Option | Benefit | Constraint |
| --- | --- | --- |
| Preserve fail-closed | Contains invalid navigation immediately | A single stale frame may trigger the watchdog |
| Remove measured pipeline stalls | Reduces stale-frame incidence | Does not change the cadence bound after an actual reset |
| Improve DDS/transport | Reduces late arrivals at the margin | Cannot recover a sample that the source publishes after the host deadline |
| Raise the rate of genuinely new VIO estimates | Shortens acquisition span needed after a reset | At 20 Hz the span is 350 ms sim, or approximately 568 ms host at R=0.616 before jitter. Requires real camera cadence, adequate IMU support and verified end-to-end throughput |
| Bounded inertial bridge | Could provide explicitly degraded continuity | Requires separate safety approval, validated uncertainty and transport/FCU semantics. No limit is approved |

Keep the current flight hold. Resolve the measured audit wait using
targeted futex/stack evidence, then assess camera cadence and throughput
against a positive measured host-time margin. No watchdog, readiness,
freshness, LK/RANSAC, independent motion audit or arming limit is changed.
