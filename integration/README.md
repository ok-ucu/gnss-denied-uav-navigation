# Phase 1 integration source package

**HOLD — `execution_allowed=false`, `forwarding_enabled=false`.**
This directory contains the real camera/IMU VIO integration, coordinate and
covariance conversion, clock/reset/egress guards, patched MAVROS overlay source,
capture-only Compose configuration, analysis and offline tests. This source
package does not prove live-camera EKF3 fusion or GNSS-denied flight.

[Exact proposed files](SOURCE_FILES.txt) · [Review/staging plan](SOURCE_COMMIT.md) ·
[External dependencies and bootstrap limits](SOURCE_DEPENDENCIES.md).

```text
Gazebo camera + IMU + simulation /clock
  -> existing teammate FlowFrontend/VioEKF (external vio/ dependency)
  -> quality-gated Odometry NED/FRD
  -> frame conversion ENU/FLU + pose/speed covariance
  -> explicit time mapping + durable epoch guard
  -> patched MAVROS -> final wire-level capture interlock
  -X-> FCU vision input (forwarding remains disabled)
```

Ground-truth Z is an explicitly disclosed test aid for optical-flow depth;
horizontal GT is not an estimator input. Correct mounting, preserved acquisition
timestamps and 21/9-element wire covariance do not establish flight readiness.
Timing, continuity, covariance calibration, reference and live fusion gates
remain open. Historical evidence remains locally under the ignored run folders.
`PHASE1_STATUS.md` is a local evidence-linked report, deliberately outside this
minimal source allowlist because its linked recordings are not part of a source
commit. No historical report or recording is moved or rewritten by this cleanup.

## Checks that do not start a simulator

```bash
python3 -B integration/audit_source_scope.py
PYTHONPATH=integration VIO_SOURCE_DIR="$PWD/vio" \
  python3 -B -m unittest discover -s integration/tests -v
git diff --check
git diff --cached --name-only
```

Full tests need NumPy and OpenCV plus the existing teammate `vio/` checkout.
ROS/Gazebo imports are generally lazy, but using the already installed ROS Jazzy
runtime is the verified full-suite environment. Missing dependencies are not
treated as a PASS. No package is installed by the audit script.

The latest full offline suite ran **108/108 PASS** in the existing ARM64 worker;
this source-scope review also verifies a copy containing only allowlisted files.
Exact local logs are under ignored `source_scope_audit/` or `phase1_runs/`.
Neither unit tests nor Compose resolution is a new live-navigation verification.

## Runtime entry points — prerequisites required, not executed by this audit

[launch_phase1.py](launch_phase1.py) refuses to overwrite an old run or reuse
container names. Its default is preparation only. A separately authorized
capture run uses [phase1.compose.yaml](phase1.compose.yaml), the actual patched
binary, gimbal model, real VIO and patched MAVROS. It never provides an FCU vision
forward switch or ARM/TAKEOFF command. [run_latency_matrix.py](run_latency_matrix.py)
adds labelled receive/quality faults and process/clock lifecycle checks within
that owned stack. It uses explicit new **capture** references, not FCU approval.

After the prerequisites in [SOURCE_DEPENDENCIES.md](SOURCE_DEPENDENCIES.md) are
supplied, preparation syntax is:

```bash
python3 -B integration/launch_phase1.py --run-id 20261110-review1
```

This command is documented, not run as part of the source-scope audit. It checks
installed image/artifact IDs and creates new local configuration; it is not a
fresh-machine dependency installer. No flight or forwarding approval follows
from preparing files.
