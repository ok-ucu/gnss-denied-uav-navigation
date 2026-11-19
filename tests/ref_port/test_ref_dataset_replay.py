"""Dataset-based checks on recordings/sample_flight (realistic input).

Ported from (reimplemented, not copied):
  - ov_msckf/src/test_sim_meas.cpp, ov_init/src/test_simulation.cpp: the measurement
    streams fed to the estimator are time-ordered with sane IMU / camera data.
  - ov_msckf/src/test_sim_repeat.cpp: two runs with the same input are identical.
  - ov_init/src/test_dynamic_init.cpp: after initialisation the estimate is compared
    with ground truth (after position/yaw alignment).
  - Propagator::select_imu_readings zero-dt removal, applied to our clock sync.
Dataset facts (meta.json): synthetic, 5 s, 10 m height, 2 m/s north, camera 640x480
30 fps, fx=461.629, IMU 100 Hz FRD; timesync.json has only an `imu` block.
"""
import json

import numpy as np
import pytest

from conftest import REF, REPO, SAMPLE_FLIGHT, SAMPLE_FX

pytestmark = pytest.mark.skipif(not SAMPLE_FLIGHT.exists(), reason="recordings/sample_flight missing")
CAM_PARAMS = REPO / "calibration" / "camera_params.json"
SRC_INIT = REF + "ov_init/src/test_dynamic_init.cpp"
GT_V = np.array([2.0, 0.0])          # meta.json note + local_position.csv vx=2.00, vy=0


@pytest.fixture(scope="module")
def ds():
    from dataset_reader import Dataset
    return Dataset(str(SAMPLE_FLIGHT), camera_params=str(CAM_PARAMS))


def replay(ds):
    """Feed sample_flight through our VIO exactly as vio_ekf.py does in 'ap' vertical mode."""
    from vio_core import VioEKF, rot_body_to_ned
    from vio_frontend import FlowFrontend
    ekf, fe = VioEKF(), FlowFrontend(fx=ds.params["fx"])
    att, gyro, last, h = np.zeros(3), np.zeros(3), None, None
    flows, accepted = [], 0
    for t, kind, d in ds.events():
        if kind == "imu":
            if last is not None and 0 < t - last < 0.2:
                ekf.predict(np.asarray(d[3:6]), rot_body_to_ned(*att), t - last)
            gyro, last = np.asarray(d[:3]), t
        elif kind == "att":
            att = np.asarray(d[:3])
        elif kind == "local":
            h = max(-d[2], 0.5)
            ekf.update_vertical(d[2], d[5])
        elif kind == "frame" and h is not None:
            res = fe.process(d, t, h, gyro, rot_body_to_ned(*att))
            if res is not None:
                flows.append(res.v_xy)
                ok, _ = ekf.update_velocity_xy(res.v_xy, res.sigma_scale)
                accepted += ok
    return np.array(flows), ekf, accepted


# ----------------------------------------------------------------- edge cases
@pytest.mark.edge
def test_timesync_has_only_imu_block_and_loaders_cope():
    """Clock mismatch: only IMU is clock-corrected; every stream must still share one clock."""
    from dataset_reader import Dataset
    from recording import load
    ts = json.loads((SAMPLE_FLIGHT / "timesync.json").read_text())
    assert set(ts) == {"imu"}, f"timesync.json blocks: {sorted(ts)}"
    d = Dataset(str(SAMPLE_FLIGHT), camera_params=str(CAM_PARAMS))
    rec = load(SAMPLE_FLIGHT, camera_params=CAM_PARAMS)
    firsts = {"imu": d._imu_t[0], "att": d._att_t[0], "frame": d._frame_t[0], "local": d._loc_t[0]}
    spread_ms = (max(firsts.values()) - min(firsts.values())) * 1e-6
    assert spread_ms < 1000 / 30, f"stream start stamps differ by {spread_ms:.1f} ms (> 1 frame): {firsts}"
    assert abs(rec.imu_t[0]) < 1e-3 and abs(rec.att_t[0]) < 1e-3, (
        f"recording.load: imu/attitude not on the frame clock (imu_t0={rec.imu_t[0]}, att_t0={rec.att_t[0]})")


@pytest.mark.edge
def test_clock_sync_removes_duplicate_and_backwards_stamps(tmp_path):
    """Ref select_imu_readings drops zero-dt readings; our timesync must emit strictly increasing
    stamps for IMU and attitude even if the autopilot clock repeats or steps back."""
    import timesync
    raw = tmp_path / "raw"
    raw.mkdir()
    t_ap = (np.arange(2000) * 10_000_000).astype(np.int64) + 5_000_000_000
    t_ap[700] = t_ap[699]                       # duplicate
    t_ap[1200] = t_ap[1100]                     # backwards step
    t_rx = t_ap + 1_700_000_000_000_000_000 + 1_000_000
    with open(raw / "imu_raw.csv", "w") as f:
        f.write("t_ap_ns,t_rx_ns,gx,gy,gz,ax,ay,az\n")
        for a, b in zip(t_ap, t_rx):
            f.write(f"{a},{b},0.0,0.0,0.0,0.0,0.0,-9.81\n")
    with open(raw / "attitude_raw.csv", "w") as f:
        f.write("t_ap_ns,t_rx_ns,roll,pitch,yaw,rr,pr,yr\n")
        for a, b in zip(t_ap[::3], t_rx[::3]):
            f.write(f"{a},{b},0.0,0.0,0.1,0.0,0.0,0.0\n")
    timesync.process_run(str(tmp_path))
    imu_t = np.loadtxt(tmp_path / "mav0" / "imu0" / "data.csv", delimiter=",", comments="#", usecols=0)
    att_t = np.loadtxt(tmp_path / "attitude.csv", delimiter=",", skiprows=1, usecols=0)
    for name, t in (("imu", imu_t), ("attitude", att_t)):
        bad = np.nonzero(np.diff(t) <= 0)[0]
        assert bad.size == 0, f"{name} stamps not strictly increasing after sync at rows {bad[:5]}"


@pytest.mark.edge
def test_streams_ordered_and_rates_match_meta(ds):
    """Ref test_sim_meas: monotonic IMU/camera streams at the configured rates."""
    imu_dt = np.diff(ds._imu_t) * 1e-9
    cam_dt = np.diff(ds._frame_t) * 1e-9
    assert (imu_dt > 0).all() and (cam_dt > 0).all(), "non-increasing timestamps in imu0 or cam0"
    assert abs(np.median(imu_dt) - 0.01) < 1e-4, f"IMU median dt {np.median(imu_dt):.5f} s != 0.010 (100 Hz)"
    assert abs(np.median(cam_dt) - 1 / 30) < 1e-4, f"camera median dt {np.median(cam_dt):.5f} s != 1/30"
    assert imu_dt.max() < 0.02 and cam_dt.max() < 0.05, f"gaps: imu {imu_dt.max():.3f}s cam {cam_dt.max():.3f}s"
    img = ds.image(0)
    assert img.shape == (480, 640) and img.dtype == np.uint8 and img.std() > 8
    assert ds.params["fx"] == pytest.approx(SAMPLE_FX)


@pytest.mark.edge
def test_events_merge_is_time_ordered(ds):
    ts = [t for t, _, _ in ds.events(kinds=("imu", "att", "local"))]
    assert all(b >= a for a, b in zip(ts, ts[1:])), "Dataset.events() yielded out-of-order events"


# ---------------------------------------------------------------- normal path
def test_imu_at_level_flight_reads_minus_g_in_frd(ds):
    acc = ds.imu()[:, 4:7]
    mean = acc.mean(axis=0)
    assert np.allclose(mean, [0, 0, -9.81], atol=0.2), f"mean specific force {mean.round(3)} != [0,0,-g] (FRD, level)"


def test_replay_velocity_matches_ground_truth(ds):
    """Ref test_dynamic_init.cpp: estimate vs GT. GT: 2 m/s NORTH at 10 m, level, yaw 0."""
    flows, ekf, accepted = replay(ds)
    assert len(flows) >= 0.85 * 149, f"frontend produced only {len(flows)}/149 measurements"
    mean = flows.mean(axis=0)
    assert np.linalg.norm(mean) == pytest.approx(2.0, abs=0.25), f"|v| = {np.linalg.norm(mean):.3f}, GT 2.0 m/s"
    assert np.allclose(mean, GT_V, atol=0.25), (
        f"[{SRC_INIT}] mean flow velocity NED_xy = {mean.round(3)}, ground truth {GT_V} "
        f"(local_position.csv vx=+2.00). Speed is right but the DIRECTION is "
        f"{'reversed' if mean[0] < 0 else 'wrong'}: frames in sample_flight move ground content "
        f"{'up' if mean[0] < 0 else '?'} (-y) while the frontend mount (vio_frontend.py:18-20, "
        f"camera_params T_cam_imu) says frame +y = body backward")
    assert np.allclose(ekf.x[3:5], GT_V, atol=0.3), f"final EKF velocity {ekf.x[3:5].round(3)} vs GT {GT_V}"


def test_replay_is_deterministic(ds):
    """Ref test_sim_repeat.cpp: the same input twice gives bit-identical results."""
    f1, e1, a1 = replay(ds)
    f2, e2, a2 = replay(ds)
    assert a1 == a2 and np.array_equal(f1, f2) and np.array_equal(e1.x, e2.x) and np.array_equal(e1.P, e2.P), (
        f"non-deterministic replay: accepted {a1} vs {a2}, max |dx| {np.abs(e1.x - e2.x).max():.3e}")
