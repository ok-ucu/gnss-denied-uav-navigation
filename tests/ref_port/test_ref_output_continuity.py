"""Odometry output continuity vs. the FCU-side availability watchdog.

Reference behaviour (reimplemented, not copied):
  - ROS2Visualizer::callback_inertial -> visualize_odometry(): after initialisation,
    odometry is published for EVERY IMU message from fast_state_propagate(), so
    output continues while individual images fail to track.
      ov_msckf/src/ros/ROS2Visualizer.cpp#L269-L326, #L438-L448
  - ov_repub OdomRepublisher: nothing until the first odometry, then the latest
    message is republished at a fixed 20 Hz.        ov_repub/src/republisher.cpp
  - VioManager::feed_measurement_camera: no output until initialised.
      ov_msckf/src/core/VioManager.cpp#L310-L316

Our components: integration/vio_quality_gate.py QualityGate (phase-1 flight source),
integration/run_vio_stream.py frame loop, integration/phase1_transport.py:521 watchdog
(500 ms without odometry -> 'VIO availability watchdog' -> EMERGENCY_LAND), and
vio/vio_core.py QualityMonitor (sitl-stand MAVLink path).

ASSUMPTION: simulation time advances no faster than host time (RTF <= 1, as in all
recorded flights), so a publication gap of G simulation seconds is >= G host seconds
at the transport watchdog.
"""
import json

import numpy as np
import pytest

from conftest import ABE2_RUN, CAMERA_PERIOD_S, REF, WATCHDOG_S
from vio_quality_gate import QualityGate, TRANSIENT_IMAGE_REASONS

SRC_VIS = REF + "ov_msckf/src/ros/ROS2Visualizer.cpp#L269-L326"
SRC_REPUB = REF + "ov_repub/src/republisher.cpp"
GOOD = dict(imu_age=.001, frame_age=.01, clock_age=.01, max_imu_gap=.001, inliers=80, contrast=40,
            covariance=np.eye(9) * .01, state_finite=True, audit_ok=True, ekf_ok=True,
            consistency_ok=True, estimator_healthy=True)


def frame_inputs(reason):
    """Gate inputs run_vio_stream.py:532-539 produces for a frame that failed with `reason`."""
    kw = dict(GOOD)
    if reason is None:
        return kw
    if reason == 'inliers':                          # frontend returned None
        kw.update(inliers=0, audit_ok=False, ekf_ok=False)
    elif reason == 'distributed flow disagreement/unavailable':
        kw.update(audit_ok=False, ekf_ok=False)
    elif reason == 'contrast':
        kw.update(contrast=0, inliers=0, audit_ok=False, ekf_ok=False)
    else:
        raise ValueError(reason)
    return kw


def run_gate(frames, tolerate):
    """frames: [(t, failure_reason_or_None)] -> list of published stamps."""
    gate, published = QualityGate(tolerate_single_rejection=tolerate), []
    for t, reason in frames:
        if gate.assess(t, t, frame_gap=CAMERA_PERIOD_S, **frame_inputs(reason)):
            published.append(round(t, 6))
    return published


def worst_gap(published, start=None):
    pub = [p for p in published if start is None or p >= start]
    gaps = [(round(b - a, 6), a, b) for a, b in zip(pub, pub[1:])]
    return max(gaps) if gaps else (float('inf'), None, None)


def grid(t0, t1, failures):
    n = int(round((t1 - t0) / CAMERA_PERIOD_S))
    ts = [round(t0 + i * CAMERA_PERIOD_S, 6) for i in range(n + 1)]
    return [(t, failures.get(t)) for t in ts]


MODES = [pytest.param(False, id="default_gate"), pytest.param(True, id="C2_tolerate_single_rejection")]


# ----------------------------------------------------------------- edge cases
@pytest.mark.edge
@pytest.mark.parametrize("tolerate", MODES)
def test_single_rejected_frame_keeps_output_within_watchdog(tolerate):
    """One image fails tracking during HOVER. Ref keeps publishing IMU-propagated odometry."""
    pub = run_gate(grid(0.0, 3.0, {1.5: 'distributed flow disagreement/unavailable'}), tolerate)
    gap, a, b = worst_gap(pub, start=0.3)
    assert gap < WATCHDOG_S, (
        f"[{SRC_VIS}] one rejected frame at t=1.5 s opened an odometry gap of {gap:.3f} s "
        f"({a} -> {b}); transport watchdog trips at {WATCHDOG_S} s (phase1_transport.py:521). "
        f"Re-readiness needs >=3 frames AND >=0.25 s (vio_quality_gate.py:149), i.e. 4 frames at 10 Hz")


@pytest.mark.edge
@pytest.mark.parametrize("tolerate", MODES)
def test_three_frame_tracking_loss_keeps_output_within_watchdog(tolerate):
    """Liftoff-style loss: 3 consecutive frames (0.3 s) without usable flow."""
    fails = {1.5: 'distributed flow disagreement/unavailable', 1.6: 'inliers', 1.7: 'inliers'}
    pub = run_gate(grid(0.0, 3.0, fails), tolerate)
    gap, a, b = worst_gap(pub, start=0.3)
    assert gap < WATCHDOG_S, (
        f"[{SRC_VIS}] 0.3 s of lost tracking produced a {gap:.3f} s odometry gap ({a} -> {b}) "
        f">= watchdog {WATCHDOG_S} s; the reference would have published IMU-propagated odometry "
        f"throughout (fail-closed gate publishes nothing without a fused flow frame)")


@pytest.mark.edge
@pytest.mark.parametrize("tolerate", MODES)
@pytest.mark.skipif(not (ABE2_RUN / "flight-source" / "publications.jsonl").exists(), reason="abe2 evidence missing")
def test_single_rejection_at_measured_flight_rtf(tolerate):
    """The watchdog is HOST time (time.monotonic_ns), the gate runs on SIM time. Using the
    real-time factor measured in run abe2 (sim seconds per host second over its publications)
    the 500 ms host watchdog is only WATCHDOG_S*RTF of simulation time."""
    pubs = [json.loads(l) for l in (ABE2_RUN / "flight-source" / "publications.jsonl").read_text().splitlines() if l.strip()]
    rtf = (pubs[-1]['t'] - pubs[0]['t']) / (pubs[-1]['publish_monotonic_s'] - pubs[0]['publish_monotonic_s'])
    budget = WATCHDOG_S * min(rtf, 1.0)
    pub = run_gate(grid(0.0, 3.0, {1.5: 'inliers'}), tolerate)
    gap, a, b = worst_gap(pub, start=0.3)
    assert gap < budget, (
        f"measured flight RTF {rtf:.3f} -> host watchdog {WATCHDOG_S} s = {budget:.3f} s of sim time; "
        f"one rejected frame gives a {gap:.3f} s sim gap ({a} -> {b}) = {gap / rtf:.3f} s host")


@pytest.mark.edge
@pytest.mark.parametrize("tolerate", MODES)
@pytest.mark.skipif(not (ABE2_RUN / "flight-source" / "stream.jsonl").exists(), reason="abe2 evidence missing")
def test_recorded_abe2_liftoff_replayed_through_gate(tolerate):
    """Realistic replay: per-frame outcomes recorded in run 20261118-flightabe2d67e5306
    (CLIMB, watchdog at acquisition 41.7 s) fed through the CURRENT gate."""
    rows = [json.loads(l) for l in (ABE2_RUN / "flight-source" / "stream.jsonl").read_text().splitlines() if l.strip()]
    frames = []
    for r in rows:
        t = round(r['t'], 6)
        if not 40.0 <= t < 43.7:          # CLIMB until the first non-image ('stale at publication') event
            continue
        reason = r.get('skip_reason')
        if r.get('published') or (reason or '').startswith('readiness hysteresis'):
            frames.append((t, None))
        elif reason in TRANSIENT_IMAGE_REASONS:
            frames.append((t, reason))
        else:
            pytest.fail(f"unexpected recorded reason at {t}: {reason}")
    pub = run_gate(frames, tolerate)
    gap, a, b = worst_gap(pub)
    failed = [(t, r) for t, r in frames if r]
    assert gap < WATCHDOG_S, (
        f"recorded liftoff ({len(failed)} failed frames: {failed}) -> odometry gap {gap:.3f} s "
        f"({a} -> {b}) >= watchdog {WATCHDOG_S} s with tolerate_single_rejection={tolerate}")


@pytest.mark.edge
@pytest.mark.parametrize("tolerate", MODES)
def test_estimator_reset_outage_within_watchdog(tolerate):
    """Reset / re-init. One IMU glitch at T=1.5 (gap/duplicate) -> run_vio_stream.py:428-430
    block(reset=True) -> reset_after=T+0.25 (recovery_deadline) -> recovery_ready() needs
    0.25 s of continuous IMU after the glitch -> new epoch with no previous image
    (run_vio_stream.py:400-417, 462-495) -> readiness. The reference does not re-initialise
    on an IMU hiccup (zero-dt readings are dropped, Propagator.cpp#L368-L381)."""
    from run_vio_stream import recovery_deadline
    from vio_sensor_capture import recovery_ready
    T = 1.5
    imu = [dict(t=round(i * 0.001, 6)) for i in range(0, 3001) if round(i * 0.001, 6) != T]  # 2 ms hole at T
    imu.insert(int(T * 1000), dict(t=T - 0.001))                                            # duplicate stamp
    reset_after, epoch_started, frames = None, False, []
    for t, _ in grid(0.0, 3.0, {}):
        if t == T:
            reset_after = recovery_deadline(t)
            frames.append((t, 'inliers'))      # frame path raised -> gate blocked (any reason blocks)
            continue
        if reset_after is not None:
            recent = [m for m in imu if m['t'] <= t]
            if not recovery_ready(recent, t, reset_after):
                frames.append((t, 'inliers'))  # 'waiting for continuous IMU recovery'
                continue
            reset_after, epoch_started = None, True
            frames.append((t, 'inliers'))      # new epoch: no previous image -> frontend None
            continue
        frames.append((t, None))
    pub = run_gate(frames, tolerate)
    gap, a, b = worst_gap(pub, start=0.3)
    assert gap < WATCHDOG_S, (
        f"[{REF}ov_msckf/src/state/Propagator.cpp#L368-L381] a single IMU glitch at t={T} s caused an "
        f"estimator re-init with odometry gap {gap:.3f} s ({a} -> {b}) >= watchdog {WATCHDOG_S} s "
        f"(recovery {0.25} s + continuous-IMU window + 4-frame readiness)")


@pytest.mark.edge
def test_no_output_before_initialisation():
    """Ref: no odometry before initialisation (VioManager.cpp#L310-L316, republisher latest_msg_==nullptr)."""
    gate = QualityGate()
    ready = [gate.assess(t, t, **GOOD) for t in (0.0, 0.1, 0.2)]
    assert ready == [False, False, False], f"gate published before readiness window: {ready}"
    assert gate.assess(0.3, 0.3, **GOOD) is True


@pytest.mark.edge
@pytest.mark.parametrize("tolerate", MODES)
def test_failed_frame_itself_never_published(tolerate):
    """Fail-closed invariant (kept by both modes): a frame that failed is never published."""
    pub = run_gate(grid(0.0, 2.0, {1.0: 'inliers'}), tolerate)
    assert 1.0 not in pub, f"rejected frame t=1.0 was published: {pub}"


@pytest.mark.edge
@pytest.mark.parametrize("t2", [1.0, 0.95], ids=["duplicate", "backwards"])
def test_camera_timestamp_reset_blocks(t2):
    gate = QualityGate()
    for t in (0.7, 0.8, 0.9, 1.0):
        gate.assess(t, t, **GOOD)
    assert gate.ready
    assert gate.assess(t2, t2, **GOOD) is False and gate.reason == 'timestamp reset/duplicate', (
        f"frame stamp {t2} after 1.0 must block with 'timestamp reset/duplicate'; reason={gate.reason}")


@pytest.mark.edge
@pytest.mark.parametrize("field", ["imu_age", "frame_age", "clock_age", "max_imu_gap", "inliers", "contrast"])
def test_nonfinite_quality_input_blocks(field):
    gate = QualityGate()
    for t in (0.0, 0.1, 0.2, 0.3):
        gate.assess(t, t, **GOOD)
    kw = dict(GOOD, **{field: float('nan')})
    assert gate.assess(0.4, 0.4, **kw) is False and gate.reason == 'nonfinite timing/quality'


@pytest.mark.edge
def test_mavlink_path_stays_on_through_short_loss():
    """sitl-stand path (vio/vio_ekf.py): QualityMonitor keeps q >= MIN_SEND_Q (0.1) through
    0.3 s of rejected frames, so VISION_* keeps flowing at 30 Hz like the reference republisher."""
    from vio_core import QualityMonitor
    mon = QualityMonitor()
    t = 0.0
    for _ in range(20):
        mon.push(True, t)
        t += 0.05
    for _ in range(6):
        mon.push(False, t)
        t += 0.05
    q = mon.quality(t)
    assert q >= 0.1, f"[{SRC_REPUB}] q={q:.3f} < MIN_SEND_Q 0.1 after 0.3 s loss -> vio_ekf.py:453 stops sending"


@pytest.mark.edge
def test_mavlink_path_stops_after_stale_vision():
    """Ref republisher keeps re-sending the latest message forever; ours stops (q < 0.1)
    once no measurement was accepted for > grace+stale (1.2 s). Pin that it does stop."""
    from vio_core import QualityMonitor
    mon = QualityMonitor()
    mon.push(True, 0.0)
    assert mon.quality(1.25) < 0.1, f"q={mon.quality(1.25):.3f}: stale VIO must not be sent after 1.25 s"


# ---------------------------------------------------------------- normal path
@pytest.mark.parametrize("tolerate", MODES)
def test_clean_stream_publishes_every_frame_after_readiness(tolerate):
    pub = run_gate(grid(0.0, 2.0, {}), tolerate)
    gap, _, _ = worst_gap(pub)
    assert pub[0] == 0.3 and gap == pytest.approx(CAMERA_PERIOD_S), f"published={pub}"
