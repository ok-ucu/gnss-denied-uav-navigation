"""Corrected offline acquisition/delivery contract, with no FCU connections."""
from fractions import Fraction

import pytest

from simulation_recovery import AcquisitionDeliveryRecovery, MS, Observation

IDENTITY = ('run', 'epoch', 'source')
PAIR = ('pose', 'velocity')


def recover(rate, fault=None, delay=0):
    p = AcquisitionDeliveryRecovery(enabled=True, disarmed=True, run='run', epoch='epoch', source='source')
    rate = Fraction(rate)
    host = lambda stamp: int(Fraction(stamp)/rate)
    camera = {host(i*100*MS): i for i in range(48)}
    last_stamp = 4_000*MS
    selected = last_stamp+500*MS
    offered = selected+100*MS if fault == 'late acquisition' else selected
    at = host(offered)+delay*MS
    delivered = at+(400*MS if fault == 'late delivery' else 5*MS)
    end = delivered+25*MS
    events = set(range(0, end, 10*MS)) | {at, delivered}
    events |= {t for t in camera if t <= end}
    for now in sorted(events):
        sim = int(now*rate)
        slot = camera.get(now)
        if slot is not None:
            sim = slot*100*MS
        for name in ('gazebo', 'imu', 'ros', 'applied'):
            if now > host(last_stamp+100*MS) and fault == name:
                continue
            stamp = (sim if name != 'applied' or fault != 'rate' or sim <= last_stamp+100*MS
                     else last_stamp+100*MS+(sim-last_stamp-100*MS)//2)
            if name == 'ros' and fault == 'reset' and now > host(last_stamp+100*MS):
                stamp = 0
            p.observe(name, Observation(now, stamp), identity=IDENTITY)
        if slot is not None and (fault != 'source age' or sim <= offered):
            p.observe('camera', Observation(now, sim), identity=IDENTITY)
            if sim <= last_stamp:
                p.complete(now, sim, identity=IDENTITY, ready=True, fresh=True, cov_valid=True, halves=PAIR)
            if sim == last_stamp+100*MS or (fault == 'second' and sim == last_stamp+200*MS):
                p.reject(now, sim, identity=('bad', 'epoch', 'source') if fault == 'foreign' else IDENTITY,
                         reason='inliers', healthy=True)
        if p.state == 'RECOVERING':
            p.check_recovery_rate(now)
        if now == at and fault != 'missing acceptance':
            original = p.last_pair
            if p.acquire(now, offered, identity=IDENTITY, source_clock=sim,
                         ready=True, estimator_ok=fault != 'estimator', cov_valid=True):
                assert p.state == 'RECOVERING'
                assert p.last_pair is original
        if now == delivered:
            p.complete(now, offered, identity=IDENTITY, ready=True,
                       fresh=fault != 'stale', cov_valid=True,
                       halves=('pose',) if fault == 'partial' else PAIR, wire_valid=True,
                       wire_stamps=(offered, offered+1) if fault == 'mismatch' else (offered, offered))
        if p.state == 'BLOCKED':
            break
    return p, at, delivered


@pytest.mark.parametrize('rate', ['0.65', '0.60', '0.50'])
@pytest.mark.parametrize('delay', [0, 35])
def test_independent_acquisition_and_delivery(rate, delay):
    p, _, delivered = recover(rate, delay=delay)
    assert [state for _, state, _ in p.transitions] == ['RECOVERING', 'NORMAL']
    assert p.last_pair == Observation(delivered, 4_500*MS)
    assert p.deadline.stamp == 4_510*MS


@pytest.mark.parametrize('fault', [
    'late delivery', 'late acquisition', 'stale', 'partial', 'mismatch', 'estimator',
    'second', 'ros', 'reset', 'imu', 'gazebo', 'rate', 'foreign', 'missing acceptance',
])
def test_recovery_failures_latch(fault):
    p, at, delivered = recover('0.65' if fault == 'late acquisition' else '0.50', fault, 10)
    if fault == 'late acquisition':
        assert delivered < p.deadline.host
        assert p.transitions[-1] == (at, 'BLOCKED', 'invalid or late recovery acquisition')
    if fault == 'late delivery':
        assert p.acquisition is not None
        assert p.transitions[-1][0] > p.deadline.host
        assert p.reason == 'absolute recovery deadline'
    assert p.state == 'BLOCKED'
    assert not p.advance(20_000*MS)


def test_source_age_boundary_in_actual_acquire():
    p, at, _ = recover('0.65', 'source age', delay=250)
    assert p.latest['camera'].stamp == 4_500*MS
    assert p.latest['ros'].stamp - p.latest['camera'].stamp > 157*MS
    assert p.transitions[-1] == (at, 'BLOCKED', 'invalid or late recovery acquisition')
