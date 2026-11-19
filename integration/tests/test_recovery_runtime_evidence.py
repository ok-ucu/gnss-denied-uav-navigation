from run_vio_stream import rejection_evidence
from simulation_vision import record_local_pair_half


def test_local_pair_timestamp_is_second_successful_socket_completion():
    pose = 'VISION_POSITION_ESTIMATE'
    speed = 'VISION_SPEED_ESTIMATE'
    sample = {'meta': dict(acquisition_ns=100, fcu_ns=200, reference='ref', producer='p', epoch=0)}
    assert record_local_pair_half(sample, speed, {'end_host_ns': 1000}) is None
    assert record_local_pair_half(sample, speed, {'end_host_ns': 1100}) is None
    witness = record_local_pair_half(sample, pose, {'end_host_ns': 1200})
    assert witness['host_ns'] == 1200
    assert witness['half_end_host_ns'] == {speed: 1000, pose: 1200}
    assert witness['acquisition_ns'] == 100
    assert record_local_pair_half(sample, pose, {'end_host_ns': 1300}) is None


def test_only_evaluated_quality_rejection_produces_source_evidence():
    anchor = dict(session='p', epoch=1, stamp_ns=100, host_ns=200)
    row = dict(published=False, frontend_available=False, session='p', epoch=1,
               skip_reason='inliers')
    assert rejection_evidence(row, **anchor)['reason'] == 'inliers'
    for changes in (dict(published=True), dict(epoch=2), dict(session='foreign'),
                    dict(frontend_available=None), dict(skip_reason='sensor/clock warmup')):
        assert rejection_evidence(row | changes, **anchor) is None
