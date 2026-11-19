"""Offline observations only. The prototype has no network or runtime hooks."""
from fractions import Fraction

import pytest

from simulation_recovery import MS, Observation, SimulationRecovery, rate_bounds

IDENTITY = ('fixture-run', 'fixture-epoch', 'fixture-source')


def prototype():
    return SimulationRecovery(enabled=True, disarmed=True, run=IDENTITY[0],
                              epoch=IDENTITY[1], source=IDENTITY[2])


def drive(rate, fault=None):
    policy = prototype()
    rate = Fraction(rate)
    # Camera event times are exact in simulation and rounded down to host ns.
    cameras = {int(Fraction(slot*100*MS)/rate): slot
               for slot in range(int(10*rate*10)+1)}
    fault_slot = int(7*rate*10)+1
    events = set(range(0, 10_000*MS, 25*MS)) | set(cameras)
    for now in sorted(events):
        sim = int(now*rate)
        slot = cameras.get(now)
        active = sim >= (fault_slot-1)*100*MS
        bias = Fraction(1, 10) if fault == 'RTF disagreement' else 0
        for kind, stamp in [('gazebo', now), ('ros', sim), ('applied', int(now*(rate+bias))), ('imu', sim)]:
            if now % (25*MS):
                continue
            if active and fault == kind:
                continue
            if active and kind == 'ros' and fault == 'clock reset':
                stamp = 0
            if active and kind == 'ros' and fault == 'clock pause':
                stamp = policy.latest['ros'].stamp
            policy.observe(kind, Observation(now, stamp), identity=IDENTITY)
        if policy.state == 'RECOVERING' and now % (25*MS) == 0:
            policy.check_recovery_rate(now)
        if slot is None:
            continue
        stamp = slot*100*MS
        if slot >= fault_slot and fault == 'camera':
            continue
        if fault in (1, 3, 5) and fault_slot <= slot < fault_slot+fault:
            continue
        policy.observe('camera', Observation(now, stamp), identity=IDENTITY)
        if fault == 'lost rejection' and fault_slot <= slot < fault_slot+4:
            continue
        if fault and slot == fault_slot:
            if fault in ('pair', 'stale', 'covariance', 'DDS'):
                policy.complete(now, stamp, identity=IDENTITY, ready=True,
                                fresh=fault not in ('stale', 'DDS'), cov_valid=fault != 'covariance',
                                halves=('pose',) if fault == 'pair' else ('pose', 'velocity'))
            elif fault not in ('gazebo', 'ros', 'applied', 'imu', 'clock pause', 'clock reset'):
                policy.reject(now, stamp, identity=('foreign', *IDENTITY[1:]) if fault == 'source' else IDENTITY,
                              reason='inliers', healthy=fault != 'estimator')
        elif fault == 'double' and slot == fault_slot+1 or fault == 'cooldown' and slot == fault_slot+6:
            policy.reject(now, stamp, identity=IDENTITY, reason='inliers', healthy=True)
        elif policy.state != 'RECOVERING' or slot >= fault_slot+4:
            if fault == 'simulation deadline' and slot == fault_slot+4:
                # Observe a fresh clock 17 ms past acquisition, the minimum
                # common-clock age seen at the historical final wire check.
                late = now + int(Fraction(17*MS)/rate)
                policy.observe('ros', Observation(late, stamp+17*MS), identity=IDENTITY)
            policy.complete(now, stamp, identity=IDENTITY, ready=True, fresh=True,
                            cov_valid=True, halves=('pose', 'velocity'))
    return policy, int(Fraction(fault_slot*100*MS)/rate)


@pytest.mark.parametrize('rate', ['1', '0.65', '0.60', '0.50', '0.45', '0.25'])
def test_normal(rate):
    policy, _ = drive(rate)
    assert policy.state == ('BLOCKED' if rate == '0.25' else 'NORMAL')


@pytest.mark.parametrize('rate', ['0.65', '0.60', '0.50'])
def test_confirmed_single_rejection(rate):
    policy, at = drive(rate, 'single')
    recovered = at + int(Fraction(400*MS)/Fraction(rate))
    assert policy.transitions[0] == (at, 'RECOVERING', 'inliers')
    assert policy.transitions[1][0] == pytest.approx(recovered, abs=1)
    assert policy.transitions[1][1:] == ('NORMAL', 'complete fresh pair')
    assert policy.state == 'NORMAL'


@pytest.mark.parametrize('rate', ['0.65', '0.60', '0.50'])
@pytest.mark.parametrize('fault,reason', [
    ('double', 'rejection ineligible'), ('cooldown', 'rejection ineligible'),
    ('camera', 'camera liveness'), ('imu', 'imu liveness'), ('gazebo', 'gazebo liveness'),
    ('applied', 'applied liveness'), ('clock pause', 'ros liveness'),
    ('clock reset', 'ros timestamp reset'), ('RTF disagreement', 'inconsistent'),
    ('source', 'rejection ineligible'), ('estimator', 'rejection ineligible'),
    ('pair', 'incomplete pair'), ('stale', 'stale'), ('covariance', 'invalid'),
    ('DDS', 'stale'), (1, 'camera liveness'), (3, 'camera liveness'), (5, 'camera liveness'),
])
def test_fault_latches(rate, fault, reason):
    policy, at = drive(rate, fault)
    assert policy.state == 'BLOCKED'
    assert reason in policy.reason
    assert policy.transitions[-1][0] >= at-200*MS
    assert not policy.advance(20_000*MS)
    assert policy.state == 'BLOCKED'


def test_default_off_and_uncertainty_penalizes_boundary():
    assert SimulationRecovery().state == 'BLOCKED'
    assert SimulationRecovery(enabled=True, run=IDENTITY[0], epoch=IDENTITY[1],
                              source=IDENTITY[2]).state == 'BLOCKED'
    first = Observation(0, 0, 1, 1)
    last = Observation(2_000*MS, 1_000*MS, 1, 1)
    assert rate_bounds(first, last)[0] < Fraction(1, 2)


def test_literal_simulation_deadline_rejects_processing_after_last_slot():
    policy, _ = drive('0.60', 'simulation deadline')
    assert [state for _, state, _ in policy.transitions] == ['RECOVERING', 'BLOCKED']
    assert policy.reason == 'absolute recovery deadline'


def test_missing_rejection_evidence_cannot_extend_normal_deadline():
    policy, _ = drive('0.60', 'lost rejection')
    assert [state for _, state, _ in policy.transitions] == ['BLOCKED']
    assert policy.reason == 'normal availability deadline'
