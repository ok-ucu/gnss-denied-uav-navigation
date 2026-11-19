"""Deterministic contract tests. No simulator process, ROS, or real sockets."""
import ast
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from run_vio_stream import clock_status, publication_block
from vio_latency_contract import SOURCE_PUBLICATION_MAX_AGE_S
from vio_quality_gate import QualityGate
import test_simulation_vision as wire_fixtures


RTFS = tuple(map(Fraction, ('1', '0.65', '0.5', '0.25')))
# Slots are counted from the last valid publication.
RECOVERY = [
    ('normal', 0, 1), ('missing', 1, 2), ('missing', 3, 8), ('missing', 5, 10),
    ('quality', 1, 5), ('quality', 3, 7), ('quality', 5, 9),
]
EXPECTED_OVERRUNS = [
    (False, False, True, True, False, True, True),
    (False, False, True, True, True, True, True),
    (False, False, True, True, True, True, True),
    (False, True, True, True, True, True, True),
]


def assess(gate, slot, rtf, *, gap=1, **changes):
    quality = dict(imu_age=0., frame_age=0., clock_age=0., max_imu_gap=.001,
                   inliers=154, contrast=30., covariance=np.eye(9)*.01,
                   state_finite=True, audit_ok=True, ekf_ok=True)
    quality.update(changes)
    sim = Fraction(slot, 10)
    return gate.assess(float(sim), float(sim/rtf), frame_gap=gap/10, **quality)


@pytest.mark.parametrize('rtf,overruns', list(zip(RTFS, EXPECTED_OVERRUNS)))
@pytest.mark.parametrize('case', range(len(RECOVERY)))
def test_gate_reacquisition_against_host_budget(rtf, overruns, case):
    fault, count, expected = RECOVERY[case]
    gate = QualityGate()
    for slot in range(10):
        assess(gate, slot, rtf)
    assert gate.ready
    previous = 9
    for offset in range(1, expected+1):
        if fault == 'missing' and offset <= count:
            continue
        slot = 9+offset
        ready = assess(gate, slot, rtf, gap=slot-previous,
                       inliers=0 if fault == 'quality' and offset <= count else 154)
        assert ready == (offset == expected)
        previous = slot
    # This comparison models the transport interval; wire admission and the
    # durable latch are tested separately with the production implementations.
    assert (Fraction(expected, 10)/rtf > Fraction(1, 2)) == overruns[case]


@pytest.fixture
def wire():
    fixture = wire_fixtures.SimulationVisionTests()
    try:
        fixture.setUp()
        route = fixture.route()
        packet, expected, gate = fixture.packet(route)
        yield fixture, route, packet, expected, gate
    finally:
        fixture.doCleanups()


@pytest.mark.parametrize('fault,reason', [
    ('host hard stop', 'VIO unavailable'),
    ('timestamp', 'acquisition timestamp mismatch'),
    ('covariance', 'Covariance'),
    ('DDS deadline', 'permission expired'),
])
def test_bad_wire_candidate_never_sends(wire, fault, reason):
    fixture, route, packet, expected, gate = wire
    last = fixture.now
    if fault == 'host hard stop':
        last -= 500_000_001
    elif fault == 'timestamp':
        packet.usec += 1
    elif fault == 'covariance':
        packet.covariance[0] = float('nan')
    else:
        expected['meta']['allowed_host_deadline_ns'] = fixture.now-1
    writes = []
    with pytest.raises(ValueError, match=reason):
        route.forward(packet, expected, gate, last, writes.append, clock=lambda:fixture.now)
    assert writes == []
    assert gate.durable.state['blocked']


def test_exact_host_boundary_with_other_permissions_still_current(wire):
    fixture, route, packet, expected, gate = wire
    writes = []
    assert route.forward(packet, expected, gate, fixture.now-500_000_000,
                         lambda data: writes.append(data) or len(data), clock=lambda: fixture.now)
    assert len(writes) == 1
    assert not gate.durable.state['blocked']


@pytest.mark.parametrize('fault', ['clock pause', 'clock reset', 'Gazebo applied clock stall'])
def test_clock_fault_latches_even_if_other_clock_is_alive(wire, fault):
    fixture, _, _, _, gate = wire
    stamp = 250_000_000
    now = fixture.now
    gate.observe_clock(stamp, now)
    if fault == 'clock pause':
        gate.observe_clock(stamp, now+200_000_001)
        assert gate.ros_clock_host == now-10_000_000
    elif fault == 'clock reset':
        gate.observe_clock(stamp-1, now+1)
        assert gate.reason == 'observed clock jump/reset'
    else:
        gate.observe_clock(stamp+1, now+200_000_001)
        assert gate.ros_clock_host == now+200_000_001
    assert not gate.watchdog(now+200_000_001)
    assert gate.durable.state['blocked']


@pytest.mark.parametrize('sensor,key,reset', [
    ('camera', 'last_frame_wall', False), ('IMU', 'last_imu_wall', True),
])
def test_source_stall_uses_host_not_sim_time(sensor, key, reset):
    # Exercise the real nested source tick with deterministic sensor receipts.
    source = Path(__file__).resolve().parents[1]/'run_vio_stream.py'
    main = next(node for node in ast.parse(source.read_text()).body
                if isinstance(node, ast.FunctionDef) and node.name == 'main')
    tick = next(node for node in main.body if isinstance(node, ast.FunctionDef) and node.name == 'tick')
    calls = []
    state = dict(last_frame_wall=10., last_imu_wall=10., last_seen_imu=3., frame_stamp=3.)
    now = 10.500001
    state['last_frame_wall' if key == 'last_imu_wall' else 'last_imu_wall'] = now
    env = dict(state=state, time=SimpleNamespace(monotonic=lambda:now), fault=lambda:None,
               block=lambda reason,t,reset=False:calls.append((reason,t,reset)),
               preintegrate=lambda _:None)
    exec(compile(ast.Module(body=[tick], type_ignores=[]), str(source), 'exec'), env)
    env['tick']({'imu':SimpleNamespace(latest=lambda:None)})
    assert calls == [(f'{sensor} wall watchdog', 3., reset)]


def test_partial_pair_is_not_completed_after_permit_deadline(wire):
    fixture, _, _, expected, gate = wire
    from navigation_egress import VisionEgress
    # Reuse the fixture egress that already consumed the pose half.
    # A new store is independently authorized, then used for this scenario.
    metadata = expected['meta']
    gate_path = gate.durable.path
    shadow_path = gate_path.with_name('partial.json')
    shadow_path.write_bytes(gate_path.read_bytes())
    egress = VisionEgress(shadow_path)
    try:
        egress.authorize(metadata['reference'], metadata['producer'], 'PASS')
        assert egress.approve(metadata)
        usec = metadata['fcu_ns']//1_000
        assert egress.check('VISION_POSITION_ESTIMATE', usec, 0, fixture.now)
        assert egress.pending[usec]['remaining'] == {'VISION_SPEED_ESTIMATE'}
        assert not egress.check('VISION_SPEED_ESTIMATE', usec, 0,
                                metadata['allowed_host_deadline_ns']+1)
        assert egress.authorization.state['blocked']
        assert not egress.pending
    finally:
        egress.close()


def test_stale_source_and_discovery_are_distinct_from_estimator_failure():
    gate = QualityGate()
    for slot in range(4):
        assess(gate, slot, Fraction(1))
    assert gate.ready
    clock = {'t': .3, 'arrival_monotonic_s': 10.}
    check = clock_status(.3, clock, now=10., limit=SOURCE_PUBLICATION_MAX_AGE_S)
    assert publication_block(gate, check, 1, 0, 2, 1) == 'waiting for downstream DDS discovery'
    assert gate.ready
    clock['t'] += SOURCE_PUBLICATION_MAX_AGE_S+.001
    stale = clock_status(.3, clock, now=10., limit=SOURCE_PUBLICATION_MAX_AGE_S)
    assert publication_block(gate, stale, 2, 1, 2, 1) == 'stale at publication'
    assert not gate.ready
    assert not assess(gate, 4, Fraction(1), ekf_ok=False)
    assert gate.reason == 'EKF not ready'
