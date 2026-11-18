#!/usr/bin/env python3
"""Run the team's VIO offline, from a recording on disk (no MAVLink, no ROS, no clock).

    python3 replay_vio.py RECORDING --out OUT [--camera-params calibration/camera_params.json]

The estimator classes (VioEKF, QualityMonitor, FlowFrontend) are imported
UNMODIFIED from vio/ (see vio_source.py). Only the loop around them is new:
it follows vio/vio_ekf.py's main loop step by step, but every "now" is a
timestamp from the recording, so a run is deterministic and repeatable.

Inputs taken from the recording, with the live equivalent in brackets:
  IMU            mav0/imu0/data.csv          [RAW_IMU]
  attitude       attitude.csv                [ATTITUDE]   or --attitude ahrs / gt
  height         distance.csv                [DISTANCE_SENSOR], local_position.csv
                                             [LOCAL_POSITION_NED] or ground truth
  frames         mav0/cam0                   [camera]

Outputs in OUT:
  vio.csv / vio.tum / vio_euroc.csv   trajectory (ENU world; see trajectory.py)
  states.csv     every output sample: state, covariance, quality, flags
  frames.csv     one row per camera frame: accepted / rejected and why
  run.json       settings, source hashes, counters, timing
  run.log        the console log
"""
import argparse
import collections
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np

import ahrs
import recording as rec_io
import vio_source
from rotations import R_to_euler, euler_to_R, quat_to_R, FRD_FLU, NED_ENU
from trajectory import Trajectory

MIN_SEND_Q = 0.1      # same thresholds as vio/vio_ekf.py
DEGRADED_Q = 0.5
RANGE_FRESH_S = 0.5


class Log:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def __call__(self, *parts):
        line = " ".join(str(p) for p in parts)
        print(line)
        self.f.write(line + "\n")

    def close(self):
        self.f.close()


def make_undistorter(calib, scale):
    """Return (function(img) -> img, fx) for the pinhole image the frontend expects."""
    size = (calib.width, calib.height)
    K, D = calib.K, np.array(calib.dist, dtype=float)
    remap = None
    if len(D) and np.any(D != 0):
        if calib.distortion_model in ("radtan", "plumb_bob", "radial-tangential"):
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, K, size, cv2.CV_16SC2)
        elif calib.distortion_model == "equidistant":
            m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, size, cv2.CV_16SC2)
        else:
            raise SystemExit(f"distortion model not supported: {calib.distortion_model}")
        remap = (m1, m2)

    def prepare(img):
        if img.shape[1] != calib.width or img.shape[0] != calib.height:
            raise SystemExit(f"frame is {img.shape[1]}x{img.shape[0]}, calibration says {calib.width}x{calib.height}")
        if remap is not None:
            img = cv2.remap(img, remap[0], remap[1], cv2.INTER_LINEAR)
        if scale != 1.0:
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return img

    return prepare, calib.fx * scale


def parse_overrides(items):
    """['sigma_vis=0.4', ...] -> {'sigma_vis': 0.4}"""
    out = {}
    for item in items or []:
        key, _, value = item.partition("=")
        out[key.strip()] = float(value)
    return out


def attitude_stream(rec, mode, yaw0, log):
    """Return (t, rpy) of the autopilot body (FRD -> NED) and the mode actually used."""
    if mode == "auto":
        mode = "recorded" if len(rec.att_t) else "ahrs"
    if mode == "recorded":
        if not len(rec.att_t):
            raise SystemExit("--attitude recorded: the recording has no attitude.csv")
        return rec.att_t, rec.att, mode
    if mode == "gt":
        if rec.gt is None or not rec.gt.has_orientation:
            raise SystemExit("--attitude gt: the recording has no ground-truth orientation")
        rpy = np.array([R_to_euler(NED_ENU @ quat_to_R(q) @ FRD_FLU) for q in rec.gt.q])
        log("ATTITUDE: taken from GROUND TRUTH (stand-in for the autopilot's ATTITUDE; not an independent run)")
        return rec.gt.t, rpy, mode
    rpy = ahrs.mahony(rec.imu_t, rec.gyro, rec.acc, yaw0=yaw0)
    log(f"ATTITUDE: own AHRS on the recorded IMU (no autopilot attitude in the recording); yaw starts at "
        f"{np.degrees(yaw0):.1f} deg and is gyro-only")
    return rec.imu_t, rpy, "ahrs"


def height_streams(rec, mode, log):
    """Return (use_range, ap_t, ap_z_down, ap_vz, description).

    'ap' in vio_ekf.py means LOCAL_POSITION_NED of the autopilot. Offline the same
    role is played by local_position.csv, or by ground-truth height when asked."""
    use_range = mode in ("range", "auto") and len(rec.range_t) > 0
    if mode == "range" and not use_range:
        raise SystemExit("--vertical range: the recording has no distance.csv")
    ap_t, ap_z, ap_vz, src = np.zeros(0), np.zeros(0), np.zeros(0), "none"
    if mode in ("ap", "auto") and len(rec.local_t):
        ap_t, ap_z, ap_vz, src = rec.local_t, rec.local[:, 2], rec.local[:, 5], "local_position.csv"
    elif mode == "gt" or (mode in ("ap", "auto") and not use_range and rec.gt is not None):
        if rec.gt is None:
            raise SystemExit("--vertical gt: the recording has no ground truth")
        up = rec.gt.p_enu[:, 2] - rec.gt.p_enu[0, 2]
        ap_t, ap_z, src = rec.gt.t, -up, "GROUND TRUTH height above the start point"
        ap_vz = np.gradient(ap_z, ap_t) if len(ap_t) > 1 else np.zeros(len(ap_t))
        log("HEIGHT: ground-truth height is used as the stand-in for the autopilot/baro height "
            "(not an independent run; flat ground at the start level is assumed)")
    log(f"HEIGHT: mode {mode}; range finder {'used' if use_range else 'not used'}; fallback source: {src}")
    return use_range, ap_t, ap_z, ap_vz, src


def run(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    log = Log(out / "run.log")
    _, src_info = vio_source.locate(args.vio_dir)
    from vio_core import QualityMonitor, VioEKF           # noqa: E402  (team's code)
    from vio_frontend import FlowFrontend                      # noqa: E402

    rec = rec_io.load(args.recording, args.camera_params, timeshift=args.timeshift)
    calib = rec.calib
    log(f"Recording {rec.root}: {len(rec.frame_t)} frames over {rec.duration:.1f} s, {len(rec.imu_t)} IMU rows, "
        f"{len(rec.att_t)} attitude rows, {len(rec.range_t)} range rows, ground truth: {rec.gt_source}")
    log(f"VIO sources: {src_info['origin']} (matches pin {src_info['pinned_commit'][:7]}: {src_info['matches_pin']})")
    log(f"Calibration: {calib.source}; fx {calib.fx:.1f}, time shift {calib.timeshift_cam_imu_s * 1e3:+.2f} ms, "
        f"mounting differs from the frontend's assumption by {calib.mount_deviation_deg():.1f} deg, "
        f"optical axis {calib.optical_axis_tilt_deg():.1f} deg off body-down")

    prepare, fx = make_undistorter(calib, args.scale)
    R_vb_imu = calib.R_viobody_imu            # IMU axes -> camera-aligned body used by the frontend
    att_t, att_rpy, att_mode = attitude_stream(rec, args.attitude, np.radians(args.yaw0_deg), log)
    use_range, ap_t, ap_z, ap_vz, ap_src = height_streams(rec, args.vertical, log)

    # Defaults are exactly what vio_ekf.py uses. --ekf / --frontend change constructor arguments
    # for what-if runs only; such runs are labelled in run.json and in the log.
    ekf_args, frontend_args = parse_overrides(args.ekf), parse_overrides(args.frontend)
    ekf = VioEKF(**ekf_args)
    monitor = QualityMonitor()
    frontend = FlowFrontend(fx=fx, **frontend_args)
    if ekf_args or frontend_args:
        log(f"NON-DEFAULT estimator parameters: VioEKF {ekf_args}, FlowFrontend {frontend_args}")

    # ---- merged event list: (time, order, kind, index) ----
    t_end = rec.frame_t[-1] if args.end is None else args.end
    events = []
    for kind, order, ts in (("att", 0, att_t), ("imu", 1, rec.imu_t), ("range", 2, rec.range_t if use_range else []),
                            ("ap", 3, ap_t), ("frame", 4, rec.frame_t)):
        ts = np.asarray(ts)
        idx = np.nonzero((ts >= args.start) & (ts <= t_end))[0]
        events += [(float(ts[i]), order, kind, int(i)) for i in idx]
    events.sort()

    # ---- state of the loop (names follow vio_ekf.py) ----
    R_body = None                  # autopilot body FRD -> NED
    R_vb = None                    # camera-aligned body -> NED (what the EKF/frontend get)
    rpy = (0.0, 0.0, 0.0)
    gyro_vb = np.zeros(3)
    gyro_angle, gyro_span = np.zeros(3), 0.0      # integrated rate since the previous frame (--gyro mean)
    last_imu_t = None
    range_buf = collections.deque(maxlen=3)
    h_range, t_range, t_range_upd = None, -1e9, -1e9
    h_local, last_vert = 1.0, -1e9
    z_init = False
    last_out = -1e9
    accepted = rejected = 0
    degraded = paused = False
    counters = collections.Counter()
    last_nis, last_inl, last_sig, last_h, h_src = float("nan"), 0, float("nan"), float("nan"), "none"
    proc_ms = []

    states_f = open(out / "states.csv", "w", newline="")
    states = csv.writer(states_f)
    states.writerow(["t", "trigger", "n", "e", "d", "vn", "ve", "vd", "roll", "pitch", "yaw",
                     "quality", "inflate", "sent", "accepted", "rejected", "nis", "n_inliers", "sigma_v",
                     "h_used", "h_src", "P_nn", "P_ne", "P_nd", "P_ee", "P_ed", "P_dd", "P_vn", "P_ve", "P_vd",
                     "ba_x", "ba_y", "ba_z", "n_resets"])
    frames_f = open(out / "frames.csv", "w", newline="")
    frames = csv.writer(frames_f)
    frames.writerow(["t", "file", "result", "nis", "n_inliers", "sigma_v", "sigma_scale", "v_n", "v_e",
                     "h_used", "h_src", "tilt_deg", "contrast", "dt_prev", "proc_ms"])
    rows = []                      # for the trajectory export
    prev_frame_t = None

    def emit(t, trigger):
        nonlocal degraded, paused
        q = monitor.quality(t)
        infl = monitor.inflate(q)
        sent = q >= MIN_SEND_Q
        if not sent and not paused:
            paused = True
            counters["output_pauses"] += 1
            log(f"[{t:8.2f}] !!! VIO: no usable data, output paused (quality {q:.2f})")
        elif sent and paused:
            paused = False
            log(f"[{t:8.2f}] VIO output resumed")
        if q < DEGRADED_Q and not degraded:
            degraded = True
            counters["degraded_events"] += 1
            log(f"[{t:8.2f}] !!! VIO DEGRADED (quality {q:.2f}), covariance inflated x{infl:.1f}")
        elif q >= DEGRADED_Q and degraded:
            degraded = False
            log(f"[{t:8.2f}] VIO recovered")
        P = ekf.P
        states.writerow([f"{t:.6f}", trigger, *(f"{v:.5f}" for v in ekf.x[0:6]), *(f"{a:.6f}" for a in rpy),
                         f"{q:.4f}", f"{infl:.3f}", int(sent), accepted, rejected, f"{last_nis:.4f}", last_inl,
                         f"{last_sig:.5f}", f"{last_h:.3f}", h_src,
                         *(f"{v:.6g}" for v in (P[0, 0], P[0, 1], P[0, 2], P[1, 1], P[1, 2], P[2, 2],
                                                 P[3, 3], P[4, 4], P[5, 5])),
                         *(f"{v:.5f}" for v in ekf.x[6:9]), ekf.n_resets])
        rows.append((t, ekf.x[0:3].copy(), ekf.x[3:6].copy(), rpy, P[0:3, 0:3] * infl, q, int(sent)))

    wall0 = time.perf_counter()
    for t, _, kind, i in events:
        if kind == "att":
            rpy = tuple(float(a) for a in att_rpy[i])
            R_body = euler_to_R(*rpy)
            R_vb = R_body @ R_vb_imu.T
        elif kind == "imu":
            counters["imu"] += 1
            f_vb = R_vb_imu @ rec.acc[i]
            if R_vb is not None and last_imu_t is not None:
                dt = t - last_imu_t
                if 0.0 < dt < 0.2:
                    ekf.predict(f_vb, R_vb, dt)
                    counters["predict"] += 1
                elif 0.2 <= dt < 3.0:
                    ekf.predict_kinematic(dt)        # IMU gap: position coasts on velocity
                    counters["predict_kinematic"] += 1
                else:
                    counters["imu_skipped"] += 1     # dt <= 0 or a jump of 3 s and more
            gyro_vb = R_vb_imu @ rec.gyro[i]
            if last_imu_t is not None and 0.0 < t - last_imu_t < 0.2:
                gyro_angle += gyro_vb * (t - last_imu_t)
                gyro_span += t - last_imu_t
            last_imu_t = t
        elif kind == "range":
            counters["range"] += 1
            d, lo, hi = rec.range_m[i]
            if R_body is not None and lo <= d < hi - 0.05:
                range_buf.append(d)
                cos_tilt = max(float(R_body[2, 2]), 0.5)
                h_range, t_range = float(np.median(range_buf)) * cos_tilt, t
                if not z_init:
                    ekf.x[2] = -h_range
                    z_init = True
                elif t - t_range_upd >= 0.1:
                    ekf.update_height(-h_range)
                    t_range_upd = t
            else:
                counters["range_invalid"] += 1
        elif kind == "ap":
            h_local = max(-float(ap_z[i]), 0.5)
            if args.vertical in ("ap", "gt") or (args.vertical == "auto" and t - t_range >= RANGE_FRESH_S):
                if not z_init:
                    ekf.x[2], ekf.x[5] = ap_z[i], ap_vz[i]
                    z_init = True
                elif t - last_vert >= 0.1:
                    ekf.update_vertical(ap_z[i], ap_vz[i])
                    last_vert = t
        elif kind == "frame" and R_vb is not None:
            counters["frames"] += 1
            w0 = time.perf_counter()
            raw = cv2.imread(str(rec.frame_files[i]), cv2.IMREAD_GRAYSCALE)
            if raw is None:
                counters["frames_unreadable"] += 1
                continue
            gray = prepare(raw)
            if h_range is not None and t - t_range < RANGE_FRESH_S:
                h, h_src = h_range, "range"
            elif len(ap_t):
                h, h_src = h_local, "ap"
            else:
                h, h_src = 0.0, "none"
            last_h = h
            # vio_ekf.py hands the frontend the latest gyro sample; "mean" hands it the average
            # rate over the interval between the two frames, which is what the image motion saw
            gyro_used = gyro_angle / gyro_span if (args.gyro == "mean" and gyro_span > 0) else gyro_vb
            gyro_angle, gyro_span = np.zeros(3), 0.0
            res = frontend.process(gray, t, h, gyro_used, R_vb)
            result, sig_scale, v_meas, nis = "no_measurement", float("nan"), (float("nan"), float("nan")), float("nan")
            if res is None:
                monitor.push(False, t)
                rejected += 1
                counters["frames_no_measurement"] += 1
            else:
                ok, nis = ekf.update_velocity_xy(res.v_xy, res.sigma_scale)
                monitor.push(ok, t)
                accepted += ok
                rejected += (not ok)
                last_nis, last_inl, last_sig = nis, res.n_inliers, res.sigma_v
                sig_scale, v_meas = res.sigma_scale, res.v_xy
                result = "accepted" if ok else "rejected_by_gate"
                counters["frames_accepted" if ok else "frames_gated"] += 1
            ms = (time.perf_counter() - w0) * 1000.0
            proc_ms.append(ms)
            frames.writerow([f"{t:.6f}", rec.frame_files[i].name, result, f"{nis:.4f}",
                             res.n_inliers if res else 0, f"{res.sigma_v:.5f}" if res else "", f"{sig_scale:.3f}",
                             f"{v_meas[0]:.4f}", f"{v_meas[1]:.4f}", f"{h:.3f}", h_src,
                             f"{np.degrees(np.arccos(np.clip(R_vb[2, 2], -1, 1))):.2f}", f"{float(gray.std()):.1f}",
                             f"{t - prev_frame_t:.4f}" if prev_frame_t is not None else "", f"{ms:.2f}"])
            prev_frame_t = t
            if result == "accepted":
                emit(t, "frame")
                last_out = t
                last_nis = float("nan")
        if R_vb is not None and t - last_out >= 1.0 / args.rate:
            emit(t, "tick")
            last_out = t
            last_nis = float("nan")
    wall = time.perf_counter() - wall0
    states_f.close()
    frames_f.close()

    # ---- trajectory export ----
    keep = [r for r in rows if r[6]] if args.only_sent else rows
    summary = dict(recording=str(rec.root), out=str(out), vio_source=src_info,
                   settings=dict(attitude=att_mode, vertical=args.vertical, gyro=args.gyro, height_fallback=ap_src, scale=args.scale,
                                 fx_used=fx, rate=args.rate, start=args.start, end=t_end, yaw0_deg=args.yaw0_deg,
                                 only_sent=args.only_sent, camera_params=calib.source,
                                 ekf_overrides=ekf_args, frontend_overrides=frontend_args,
                                 timeshift_cam_imu_s=calib.timeshift_cam_imu_s,
                                 mount_deviation_deg=round(calib.mount_deviation_deg(), 2)),
                   counters=dict(counters), accepted=int(accepted), rejected=int(rejected),
                   velocity_resets=int(ekf.n_resets), states_written=len(rows), states_exported=len(keep),
                   wall_s=round(wall, 2), realtime_factor=round((t_end - args.start) / wall, 2) if wall > 0 else None,
                   frame_proc_ms=dict(mean=round(float(np.mean(proc_ms)), 2), p95=round(float(np.percentile(proc_ms, 95)), 2),
                                      max=round(float(np.max(proc_ms)), 2)) if proc_ms else None)
    if keep:
        traj = Trajectory.from_ned([r[0] for r in keep], [r[1] for r in keep], rpy=[r[3] for r in keep],
                                   v_ned=[r[2] for r in keep], cov_p_ned=[r[4] for r in keep],
                                   t0_ns=rec.t0_ns, name="vio",
                                   extra=dict(quality=np.array([f"{r[5]:.4f}" for r in keep]),
                                              sent=np.array([r[6] for r in keep])))
        summary["trajectory"] = traj.write_all(out, "vio")
    (out / "run.json").write_text(json.dumps(summary, indent=2) + "\n")
    log(f"Frames: {counters['frames']} processed, {accepted} accepted, {counters['frames_gated']} rejected by the gate, "
        f"{counters['frames_no_measurement']} without a measurement; velocity resets {ekf.n_resets}")
    log(f"IMU: {counters['predict']} predictions, {counters['predict_kinematic']} gap steps (0.2-3 s), "
        f"{counters['imu_skipped']} skipped")
    if proc_ms:
        log(f"Per-frame processing on this machine: mean {np.mean(proc_ms):.1f} ms, p95 {np.percentile(proc_ms, 95):.1f} ms; "
            f"whole replay {wall:.1f} s for {t_end - args.start:.1f} s of data")
    log(f"Wrote {out}/vio.csv, vio.tum, vio_euroc.csv, states.csv, frames.csv, run.json")
    log.close()
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", help="recording folder (dataset/recorder.py format, EuRoC or convert_insane.py output)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera-params", default=None, help="camera_params.json (default: RECORDING/camera_params.json)")
    ap.add_argument("--attitude", choices=["auto", "recorded", "ahrs", "gt"], default="auto",
                    help="recorded = attitude.csv (autopilot); ahrs = own filter on the IMU; gt = ground truth")
    ap.add_argument("--yaw0-deg", type=float, default=0.0, help="initial heading for --attitude ahrs")
    ap.add_argument("--vertical", choices=["auto", "range", "ap", "gt"], default="auto",
                    help="height source, as in vio_ekf.py; gt = ground-truth height as a stand-in for 'ap'")
    ap.add_argument("--gyro", choices=["sample", "mean"], default="sample",
                    help="rate used to remove rotation from the optical flow: sample = latest IMU sample "
                         "(what vio_ekf.py does), mean = average over the interval between the two frames")
    ap.add_argument("--timeshift", type=float, default=None,
                    help="override timeshift_cam_imu_s of the calibration file (t_imu = t_cam + shift)")
    ap.add_argument("--scale", type=float, default=1.0, help="downscale frames before optical flow")
    ap.add_argument("--rate", type=float, default=30.0, help="minimum output rate, Hz")
    ap.add_argument("--start", type=float, default=0.0, help="seconds from the first frame")
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--only-sent", action="store_true",
                    help="export only samples the live VIO would send (quality >= 0.1)")
    ap.add_argument("--ekf", action="append", default=[], metavar="KEY=VALUE",
                    help="what-if: VioEKF constructor argument, e.g. --ekf sigma_vis=0.4 (default: as vio_ekf.py)")
    ap.add_argument("--frontend", action="append", default=[], metavar="KEY=VALUE",
                    help="what-if: FlowFrontend constructor argument, e.g. --frontend sigma_ref=0.4")
    ap.add_argument("--vio-dir", default=None, help="folder with vio_core.py / vio_frontend.py")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
