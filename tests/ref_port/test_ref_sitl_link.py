"""SITL MAVLink link (sitl-stand/send_vision.py, vio/vio_ekf.py payloads).

Reference behaviour (reimplemented, not copied) - ov_repub/src/republisher.cpp:
  * publishes NOTHING until the first odometry arrives (latest_msg_ == nullptr check),
  * then republishes the latest message at a fixed rate (20 Hz) WITH ITS ORIGINAL
    header stamp (it never re-stamps old data as new),
  * covariance comes from the estimator (ROS2Visualizer::visualize_odometry fills
    pose/twist covariance), it is never left as zeros.
MAVLink spec, VISION_POSITION_ESTIMATE.covariance: "If unknown, assign NaN value to
first element in the array."

MAVLink is mocked (no live SITL). Live checks are @pytest.mark.sitl.
"""
import math
import runpy
import sys
import time
import types

import numpy as np
import pytest

from conftest import REF, REPO

SRC_REPUB = REF + "ov_repub/src/republisher.cpp"
SEND_VISION = REPO / "sitl-stand" / "send_vision.py"


class _Stop(Exception):
    pass


class _Msg:
    def __init__(self, kind, **f):
        self._kind = kind
        self.__dict__.update(f)

    def get_type(self):
        return self._kind


def run_send_vision(monkeypatch, duration, local_from=None, local_until=None, rate=30.0,
                    att_from="local", local_resume=None):
    """Execute sitl-stand/send_vision.py against a scripted fake SITL; returns the sends.

    LOCAL_POSITION_NED streams over [local_from, local_until) and again from local_resume;
    ATTITUDE streams from att_from ("local" = together with LOCAL_POSITION_NED) to the end."""
    clock = [1000.0]
    t0 = clock[0]
    queue = []

    def stream(start, stop, make):
        k = 0
        while start + k / rate < stop:
            queue.append((start + k / rate, make(start + k / rate)))
            k += 1

    if local_from is not None:
        stop = local_until if local_until is not None else duration
        local = lambda ts: _Msg("LOCAL_POSITION_NED", x=1.0 + ts, y=2.0, z=-10.0, vx=1.0, vy=0.0, vz=0.0)
        stream(local_from, stop, local)
        if local_resume is not None:
            stream(local_resume, duration, local)
    attitude = lambda ts: _Msg("ATTITUDE", roll=0.01, pitch=0.02, yaw=0.3)
    if att_from == "local":
        if local_from is not None:
            stream(local_from, duration, attitude)
    elif att_from is not None:
        stream(att_from, duration, attitude)
    queue.sort(key=lambda e: e[0])
    sent = dict(pos=[], spd=[], origin=[], streams=[])

    class Mav:
        def set_gps_global_origin_send(self, *a):
            sent["origin"].append(a)

        def request_data_stream_send(self, *a):
            sent["streams"].append(a)

        def vision_position_estimate_send(self, usec, x, y, z, roll, pitch, yaw, covariance=None, reset_counter=0):
            sent["pos"].append(dict(t=clock[0] - t0, usec=usec, xyz=(x, y, z), att=(roll, pitch, yaw), cov=covariance))

        def vision_speed_estimate_send(self, usec, vx, vy, vz, covariance=None, reset_counter=0):
            sent["spd"].append(dict(t=clock[0] - t0, usec=usec, v=(vx, vy, vz), cov=covariance))

    class Master:
        target_system, target_component = 1, 1
        mav = Mav()

        def wait_heartbeat(self, *a, **k):
            return _Msg("HEARTBEAT")

        def recv_match(self, type=None, blocking=False, **k):
            if queue and queue[0][0] <= clock[0] - t0:
                return queue.pop(0)[1]
            return None

    mavutil = types.SimpleNamespace(
        mavlink_connection=lambda conn, **k: Master(),
        mavlink=types.SimpleNamespace(MAV_DATA_STREAM_POSITION=6, MAV_DATA_STREAM_EXTRA1=10))
    fake = types.ModuleType("pymavlink")
    fake.mavutil = mavutil
    monkeypatch.setitem(sys.modules, "pymavlink", fake)
    monkeypatch.setitem(sys.modules, "pymavlink.mavutil", mavutil)

    def sleep(s):
        clock[0] += s
        if clock[0] - t0 >= duration:
            raise _Stop

    monkeypatch.setattr(time, "time", lambda: clock[0])
    monkeypatch.setattr(time, "sleep", sleep)
    monkeypatch.setattr("builtins.print", lambda *a, **k: None)
    with pytest.raises(_Stop):
        runpy.run_path(str(SEND_VISION), run_name="__main__")
    return sent


# ----------------------------------------------------------------- edge cases
@pytest.mark.edge
def test_send_vision_sends_nothing_before_first_attitude(monkeypatch):
    """Ref republisher: no output while it has nothing to publish (latest_msg_ == nullptr).
    Before ATTITUDE arrives send_vision has no attitude of its own, so it must stay silent."""
    sent = run_send_vision(monkeypatch, duration=2.0, att_from=0.8, local_from=1.0)
    early = [s for s in sent["pos"] if s["t"] < 0.8]
    assert not early, (
        f"[{SRC_REPUB}] {len(early)} VISION_POSITION_ESTIMATE sent before the first ATTITUDE (t<0.8 s), "
        f"e.g. att={early[0]['att']}: the attitude was fabricated")


@pytest.mark.edge
def test_send_vision_bootstrap_before_first_position_is_explicit(monkeypatch):
    """ADAPTED (reference-specific behaviour does not transfer): the republisher's input is an
    independent estimator, ours is the FCU's own LOCAL_POSITION_NED, which EKF3 does not publish
    without external-nav aiding. So before the first LOCAL_POSITION_NED send_vision may send
    only an explicit 'at origin, at rest' start fix: real attitude and a LARGER covariance than
    normal fixes, so the FCU is told it is less certain."""
    sent = run_send_vision(monkeypatch, duration=2.0, att_from=0.2, local_from=1.0)
    boot = [s for s in sent["pos"] if 0.2 <= s["t"] < 1.0]
    normal = [s for s in sent["pos"] if s["t"] >= 1.1]
    assert boot and normal, f"expected bootstrap and normal sends, got {len(boot)} / {len(normal)}"
    b, n = boot[-1], normal[-1]
    assert b["xyz"] == (0.0, 0.0, 0.0) and b["att"] == (0.01, 0.02, 0.3), f"bootstrap fix: {b}"
    assert b["cov"] is not None and n["cov"] is not None, "covariance must be explicit for both kinds of fix"
    assert b["cov"][0] > n["cov"][0] > 0, (
        f"bootstrap position variance {b['cov'][0]} must exceed the normal one {n['cov'][0]} (> 0)")


@pytest.mark.edge
def test_send_vision_resumes_after_stale_period(monkeypatch):
    """After a stale period send_vision stops; when LOCAL_POSITION_NED returns it resumes (no bootstrap)."""
    sent = run_send_vision(monkeypatch, duration=4.0, local_from=0.1, local_until=1.5, local_resume=3.0)
    gap = [s for s in sent["pos"] if 2.05 <= s["t"] < 3.0]
    resumed = [s for s in sent["pos"] if s["t"] >= 3.05]
    assert not gap, f"{len(gap)} sends during the stale period (first at t={gap[0]['t']:.2f}s)"
    assert resumed and resumed[-1]["xyz"][0] > 3.9, f"did not resume with fresh data: {resumed[-1:] or 'none'}"


@pytest.mark.edge
def test_send_vision_does_not_restamp_stale_position(monkeypatch):
    """Ref republisher re-sends the latest message with its ORIGINAL stamp; ours must not
    present a position that stopped updating as a fresh measurement."""
    sent = run_send_vision(monkeypatch, duration=4.0, local_from=0.1, local_until=2.0)
    last_fresh = max(s["usec"] for s in sent["pos"] if s["t"] <= 2.05)
    stale = [s for s in sent["pos"] if s["t"] > 2.5 and s["usec"] > last_fresh]
    assert not stale, (
        f"[{SRC_REPUB}] LOCAL_POSITION_NED stopped at t=2.0 s, but {len(stale)} VISION_POSITION_ESTIMATE "
        f"with NEW timestamps were sent after t=2.5 s (last at t={stale[-1]['t']:.2f}s, usec={stale[-1]['usec']}) "
        f"carrying the frozen position {stale[-1]['xyz']} - send_vision.py:43 stamps wall time on every send")


@pytest.mark.edge
def test_send_vision_declares_covariance(monkeypatch):
    """Ref fills covariance from the estimator. MAVLink: unknown -> NaN in element 0.
    If send_vision.py omits it, pymavlink's default (all zeros) applies."""
    pytest.importorskip("pymavlink.dialects.v20.ardupilotmega")
    import inspect
    from pymavlink.dialects.v20 import ardupilotmega as m
    sent = run_send_vision(monkeypatch, duration=1.0, local_from=0.0)
    cov = sent["pos"][-1]["cov"]
    if cov is None:
        cov = inspect.signature(m.MAVLink.vision_position_estimate_send).parameters["covariance"].default
    c0 = cov[0]
    assert math.isnan(c0) or c0 > 0, (
        f"VISION_POSITION_ESTIMATE covariance effectively {list(cov)[:3]}... (pymavlink default): zero "
        f"variance claims a perfect fix; the spec requires NaN in element 0 when unknown, the reference "
        f"sends the estimator covariance")


@pytest.mark.edge
def test_vio_ekf_covariance_layout_matches_mavlink():
    """vio_ekf.py sends VioEKF.pose_cov()/speed_cov(). MAVLink: pose = row-major upper triangle of 6x6
    (x,y,z,roll,pitch,yaw) -> diagonal at 0,6,11,15,18,20; speed = 3x3 row-major -> 0,4,8."""
    from vio_core import VioEKF
    ekf = VioEKF()
    ekf.P[0, 0], ekf.P[1, 1], ekf.P[2, 2] = 0.11, 0.22, 0.33
    ekf.P[3, 3], ekf.P[4, 4], ekf.P[5, 5] = 0.44, 0.55, 0.66
    pose, speed = ekf.pose_cov(inflate=1.0, att_var=0.007), ekf.speed_cov(1.0)
    assert len(pose) == 21 and len(speed) == 9
    got = [pose[i] for i in (0, 6, 11, 15, 18, 20)]
    assert np.allclose(got, [0.11, 0.22, 0.33, 0.007, 0.007, 0.007]), f"pose diagonal at MAVLink indices: {got}"
    assert np.allclose([speed[i] for i in (0, 4, 8)], [0.44, 0.55, 0.66]), f"speed diagonal: {speed}"


@pytest.mark.edge
def test_vio_ekf_covariance_finite_and_psd_after_long_run():
    """Covariance handling: 10 min of 50 Hz prediction with 20 Hz velocity updates and gaps."""
    from vio_core import G, VioEKF
    ekf, rng = VioEKF(), np.random.default_rng(3)
    for i in range(30000):
        ekf.predict(np.array([0, 0, -G]) + rng.normal(0, .1, 3), np.eye(3), 0.02)
        if i % 3 == 0 and not (5000 <= i < 5500):          # 10 s camera blackout
            ekf.update_velocity_xy(rng.normal(0, .05, 2))
    p = ekf.P
    assert np.isfinite(p).all() and np.allclose(p, p.T) and np.linalg.eigvalsh(p).min() > -1e-12, (
        f"P not finite/symmetric/PSD: min eig {np.linalg.eigvalsh(p).min():.3e}")
    cov = ekf.pose_cov()
    assert all(math.isfinite(c) for c in cov) and min(cov[i] for i in (0, 6, 11)) > 0


# ---------------------------------------------------------------- normal path
def test_send_vision_rate_and_payload(monkeypatch):
    sent = run_send_vision(monkeypatch, duration=3.0, local_from=0.0)
    window = [s for s in sent["pos"] if 1.0 <= s["t"] < 2.0]
    assert 25 <= len(window) <= 35, f"expected ~30 Hz VISION_POSITION_ESTIMATE, got {len(window)} in 1 s"
    last = sent["pos"][-1]
    assert last["xyz"][2] == -10.0 and last["att"] == (0.01, 0.02, 0.3), f"payload mismatch: {last}"
    assert [s["usec"] for s in sent["pos"]] == [s["usec"] for s in sent["spd"]], "position/speed stamps differ"
    usec = [s["usec"] for s in sent["pos"]]
    assert all(b > a for a, b in zip(usec, usec[1:])), "VISION usec must be strictly increasing"


def test_send_vision_sets_origin_once(monkeypatch):
    sent = run_send_vision(monkeypatch, duration=0.5, local_from=0.0)
    assert len(sent["origin"]) == 1
    _, lat, lon, alt, _ = sent["origin"][0]
    assert (lat, lon, alt) == (-353632610, 1491652300, 584000), f"origin encoding: {(lat, lon, alt)}"


@pytest.mark.sitl
def test_live_sitl_ekf_accepts_vision():
    """Live: SITL started with sitl-stand/ext_nav.parm, output on udpin:127.0.0.1:14551.
    Feed static vision for 15 s and require EKF3 horizontal position (rel) to be valid."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
    hb = m.wait_heartbeat(timeout=15)
    assert hb is not None, "no HEARTBEAT from SITL on udpin:127.0.0.1:14551 within 15 s"
    m.mav.set_gps_global_origin_send(m.target_system, -353632610, 1491652300, 584000, 0)
    t0, flags = time.time(), 0
    while time.time() - t0 < 15:
        usec = int((time.time() - t0) * 1e6)
        cov = [0.01 if i in (0, 6, 11) else (0.001 if i in (15, 18, 20) else 0.0) for i in range(21)]
        m.mav.vision_position_estimate_send(usec, 0, 0, 0, 0, 0, 0, cov)
        m.mav.vision_speed_estimate_send(usec, 0, 0, 0, [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01])
        msg = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1 / 30)
        if msg is not None:
            flags = msg.flags
    assert flags & 8, f"EKF_STATUS_REPORT.flags=0x{flags:x}: EKF_POS_HORIZ_REL not set after 15 s of vision"


@pytest.mark.sitl
def test_live_send_vision_bootstraps_extnav():
    """Live (unverified offline): run sitl-stand/send_vision.py against SITL started with
    ext_nav.parm (Dockerfile CMD: --out 14550 and 14551). Observe on udpin:127.0.0.1:14550 that
    the explicit bootstrap lets EKF3 start external-nav aiding, i.e. LOCAL_POSITION_NED appears
    and EKF_POS_HORIZ_REL is set, within 30 s. Rebuild the image if send_vision.py runs inside it."""
    import subprocess
    from pymavlink import mavutil
    obs = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    assert obs.wait_heartbeat(timeout=15) is not None, "no HEARTBEAT on udpin:127.0.0.1:14550"
    proc = subprocess.Popen([sys.executable, str(SEND_VISION)], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        t0, flags, local = time.time(), 0, 0
        while time.time() - t0 < 30 and not (flags & 8 and local):
            msg = obs.recv_match(type=["EKF_STATUS_REPORT", "LOCAL_POSITION_NED"], blocking=True, timeout=1)
            if msg is not None and msg.get_type() == "EKF_STATUS_REPORT":
                flags = msg.flags
            elif msg is not None:
                local += 1
        assert flags & 8 and local, (
            f"after 30 s: EKF_STATUS_REPORT.flags=0x{flags:x} (need EKF_POS_HORIZ_REL 0x8), "
            f"LOCAL_POSITION_NED seen {local}x - the bootstrap did not start external-nav aiding")
    finally:
        proc.terminate()
        proc.wait(timeout=5)
