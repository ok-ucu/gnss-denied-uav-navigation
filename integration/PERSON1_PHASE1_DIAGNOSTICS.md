# Person 1 — Phase 1 Diagnostics

## Scope
ArduPilot SITL, EKF3 configuration, MAVROS contracts,
TIMESYNC and ExternalNav integration readiness.

## Environment
- Platform: Linux x86_64
- Branch: gazebo-sitl-integration
- Target firmware: ArduCopter 4.4.4
- Hardware FCU: not available
- Phase 1 runtime status: HOLD / NO GO

## EKF3 configuration

Status: PASS — offline structural validation.

- integration/ekf3_dry_run.parm: PASS
- integration/ekf3_sitl_candidate.parm: PASS
- EKF3 readiness tests: 5/5 PASS
- GPS-disabled ExternalNav candidate validated.
- Existing sitl-stand/ext_nav.parm does not match the validated profile.
- Candidate parameters have not been applied to an FCU.

Commit: ff8bf0d

## MAVROS diagnostics

Status: PASS — offline regression.

- MAVROS contract tests: 5/5 PASS
- MAVROS wire tests: 3/3 PASS
- Total: 8/8 PASS
- Tested timestamp, covariance, reset and message contracts.
- No live FCU forwarding was performed.

## TIMESYNC diagnostics

Status: PASS — offline regression only.

- Matched physics, runtime candidate and Phase 1 pipeline tests:
  30/30 PASS, 6 subtests PASS.
- TIMESYNC patch source exists.
- Patch has not been applied to working MAVROS.
- No saved TIMESYNC experiment report was found in this checkout.
- Native FCU TIMESYNC has not been demonstrated.

## Summary

| Component | Result |
|---|---|
| EKF3 offline configuration | PASS |
| EKF3 readiness tests | 5/5 PASS |
| MAVROS contract and wire tests | 8/8 PASS |
| Clock and pipeline regression | 30/30 PASS |
| Additional subtests | 6 PASS |
| Native SITL/MAVROS TIMESYNC | NOT VERIFIED |
| VIO to EKF3 fusion | NOT VERIFIED |
| Phase 1 runtime authorization | HOLD / NO GO |

## Safety boundary

This report documents offline validation only.
It does not authorize arming, live forwarding,
fault injection, FCU parameter changes or
isolated runtime experiments.

## Next dependencies

1. Review and reconcile the SITL parameter profile.
2. Obtain separate runtime safety approval.
3. Verify native SITL/MAVROS clock synchronization.
4. Verify ExternalNav fusion with EKF3.
