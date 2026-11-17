#!/usr/bin/env python3
"""Check camera-IMU calibration on an ordinary flight recording (no target needed).

    python3 calib_check.py RECORDING --out OUT [--camera-params camera_params.json]

Idea: when the drone rotates, the image of the ground moves. For a camera that
looks down, the image motion at the centre, divided by the focal length, equals
the body angular rate (plus a slow term from translation):

    flow_x / fx = gyro_x - v_cam_x / depth        flow_y / fx = gyro_y - v_cam_y / depth

The fast part (vibration, attitude corrections) is rotation. Comparing the fast
part of the image motion with the gyro gives, without any calibration target:

  * TIME OFFSET camera -> IMU: the lag that maximises the correlation.
    Same sign as Kalibr's `timeshift_cam_imu` (t_imu = t_cam + shift). The value
    printed is the REMAINING offset on top of the one already in camera_params.json.
  * AXES / EXTRINSIC ROTATION: correlation must be strongly positive on the
    matching axis and weak on the crossed one. A negative or crossed result
    means the camera is mounted differently from T_cam_imu.
  * FOCAL LENGTH: the regression slope flow/gyro must be ~1.
  * How much noise the VIO's rotation compensation leaves, in rad/s, for
    (a) one gyro sample per frame, as vio_ekf.py does, and
    (b) the gyro averaged over the interval between the two frames.
    Multiply by the height above ground to get the velocity error in m/s.

This complements Kalibr (calibration/README.md); it does not replace it. It needs
some rotation in the recording (ordinary flight is enough) and textured ground.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

import recording as rec_io
import vio_source
from replay_vio import make_undistorter


def measure_flow(rec, scale, start, end, log=print):
    """Per-frame image motion at the centre, in rad/s, in the camera-aligned body axes.

    Uses the team's FlowFrontend with height 1, zero gyro and identity attitude,
    which makes its output the raw normalised flow."""
    from vio_frontend import FlowFrontend
    prepare, fx = make_undistorter(rec.calib, scale)
    frontend = FlowFrontend(fx=fx)
    t_out, flow, t_prev_out = [], [], []
    prev_t = None
    for t, path in zip(rec.frame_t, rec.frame_files):
        if t < start or (end is not None and t > end):
            continue
        img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        res = frontend.process(prepare(img), t, 1.0, np.zeros(3), np.eye(3))
        if res is not None and prev_t is not None:
            # v_xy = [flow_y / fx, -flow_x / fx]  ->  rotation-equivalent rates about body x and y
            t_out.append(t)
            t_prev_out.append(prev_t)
            flow.append([-res.v_xy[1], res.v_xy[0]])
        prev_t = t
    log(f"flow measured on {len(t_out)} frame pairs")
    return np.array(t_prev_out), np.array(t_out), np.array(flow)


def highpass(t, x, window_s):
    """Remove the slow part with a moving average of about window_s."""
    x = np.atleast_2d(x.T).T
    dt = np.median(np.diff(t))
    n = max(3, int(round(window_s / dt)) | 1)
    kernel = np.ones(n) / n
    pad = n // 2
    out = np.empty_like(x)
    for k in range(x.shape[1]):
        padded = np.pad(x[:, k], pad, mode="edge")
        out[:, k] = x[:, k] - np.convolve(padded, kernel, mode="valid")
    return out


class GyroIntegral:
    """Mean angular rate over an interval, from the integrated gyro."""

    def __init__(self, t, gyro):
        self.t = t
        steps = 0.5 * (gyro[1:] + gyro[:-1]) * np.diff(t)[:, None]
        self.angle = np.vstack([np.zeros(3), np.cumsum(steps, axis=0)])
        self.gyro = gyro

    def mean_rate(self, t0, t1):
        a0 = np.column_stack([np.interp(t0, self.t, self.angle[:, k]) for k in range(3)])
        a1 = np.column_stack([np.interp(t1, self.t, self.angle[:, k]) for k in range(3)])
        return (a1 - a0) / (t1 - t0)[:, None]

    def sample(self, t):
        """Last sample at or before t: what vio_ekf.py holds when a frame arrives."""
        idx = np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 1)
        return self.gyro[idx]


def corr(a, b):
    a, b = a - a.mean(), b - b.mean()
    den = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / den) if den > 0 else 0.0


def analyse(rec, t_prev, t, flow, max_lag=0.15, step=0.002, highpass_s=1.0):
    R = rec.calib.R_viobody_imu
    gyro = GyroIntegral(rec.imu_t, rec.gyro @ R.T)
    flow_hp = highpass(t, flow, highpass_s)

    lags = np.arange(-max_lag, max_lag + step / 2, step)
    score = np.zeros(len(lags))
    for i, lag in enumerate(lags):
        g = highpass(t, gyro.mean_rate(t_prev + lag, t + lag)[:, :2], highpass_s)
        score[i] = 0.5 * (corr(flow_hp[:, 0], g[:, 0]) + corr(flow_hp[:, 1], g[:, 1]))
    best = int(np.argmax(score))
    lag = lags[best]
    if 0 < best < len(lags) - 1:            # parabolic refinement
        y0, y1, y2 = score[best - 1:best + 2]
        den = y0 - 2 * y1 + y2
        if den < 0:
            lag += 0.5 * (y0 - y2) / den * step

    g_best = highpass(t, gyro.mean_rate(t_prev + lag, t + lag)[:, :2], highpass_s)
    g_zero = highpass(t, gyro.mean_rate(t_prev, t)[:, :2], highpass_s)
    g_inst = highpass(t, gyro.sample(t)[:, :2], highpass_s)
    energy = [float(np.dot(g_best[:, k], g_best[:, k])) for k in range(2)]
    slope = [float(np.dot(flow_hp[:, k], g_best[:, k]) / energy[k]) if energy[k] > 0 else float("nan") for k in range(2)]
    result = dict(
        frame_pairs=int(len(t)),
        gyro_rate_rms_rad_s=[round(float(np.sqrt(np.mean(g_zero[:, k] ** 2))), 4) for k in range(2)],
        time_offset_s=round(float(lag), 4),
        correlation_at_best_lag=round(float(score[best]), 3),
        correlation_at_zero_lag=round(float(score[np.argmin(np.abs(lags))]), 3),
        axis_correlation=dict(
            xx=round(corr(flow_hp[:, 0], g_best[:, 0]), 3), yy=round(corr(flow_hp[:, 1], g_best[:, 1]), 3),
            xy=round(corr(flow_hp[:, 0], g_best[:, 1]), 3), yx=round(corr(flow_hp[:, 1], g_best[:, 0]), 3)),
        slope_flow_over_gyro=[round(s, 3) for s in slope],
        residual_rad_s=dict(
            no_compensation=round(float(np.sqrt(np.mean(flow_hp ** 2))), 4),
            one_gyro_sample_per_frame=round(float(np.sqrt(np.mean((flow_hp - g_inst) ** 2))), 4),
            gyro_mean_over_interval=round(float(np.sqrt(np.mean((flow_hp - g_zero) ** 2))), 4),
            gyro_mean_over_interval_at_best_lag=round(float(np.sqrt(np.mean((flow_hp - g_best) ** 2))), 4)),
        lags_s=[round(float(x), 4) for x in lags], lag_scores=[round(float(x), 4) for x in score])
    return result


def verdict(r, calib):
    notes = []
    c = r["axis_correlation"]
    enough_rotation = min(r["gyro_rate_rms_rad_s"]) > 0.01
    if not enough_rotation:
        return ["NOT ENOUGH ROTATION in the recording (gyro rate RMS below 0.01 rad/s): time offset, axes and "
                "focal length cannot be checked from it; fly or shake the drone a little and repeat"]
    if r["correlation_at_best_lag"] < 0.5:
        notes.append("WEAK correlation between image motion and gyro (< 0.5): check texture, exposure, "
                     "timestamps, or the extrinsic rotation")
    if c["xx"] < 0 or c["yy"] < 0:
        notes.append("NEGATIVE correlation on a matching axis: axis sign differs from T_cam_imu")
    if max(abs(c["xy"]), abs(c["yx"])) > max(c["xx"], c["yy"]):
        notes.append("CROSSED axes correlate better than matching ones: camera is rotated about its optical axis "
                     "relative to T_cam_imu (90 deg?)")
    for k, name in enumerate("xy"):
        if enough_rotation and not 0.85 <= r["slope_flow_over_gyro"][k] <= 1.15:
            notes.append(f"slope on axis {name} is {r['slope_flow_over_gyro'][k]}: focal length or distortion "
                         "model off by that factor (expected 1.0 +- 0.15)")
    if abs(r["time_offset_s"]) > 0.005:
        notes.append(f"remaining camera->IMU time offset {r['time_offset_s'] * 1e3:+.1f} ms: add it to "
                     f"timeshift_cam_imu_s (now {calib.timeshift_cam_imu_s * 1e3:+.2f} ms) and re-check with Kalibr")
    if not notes:
        notes.append("OK: axes, focal length and time offset are consistent with the calibration file")
    return notes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera-params", default=None)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--max-lag", type=float, default=0.15, help="search range, seconds")
    ap.add_argument("--timeshift", type=float, default=None, help="override timeshift_cam_imu_s of the calibration file")
    ap.add_argument("--vio-dir", default=None)
    args = ap.parse_args()

    vio_source.locate(args.vio_dir)
    rec = rec_io.load(args.recording, args.camera_params, timeshift=args.timeshift)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t_prev, t, flow = measure_flow(rec, args.scale, args.start, args.end)
    if len(t) < 50:
        raise SystemExit("fewer than 50 usable frame pairs: nothing to analyse")
    r = analyse(rec, t_prev, t, flow, max_lag=args.max_lag)
    r["notes"] = verdict(r, rec.calib)
    r["calibration"] = dict(file=rec.calib.source, timeshift_cam_imu_s=rec.calib.timeshift_cam_imu_s,
                            mount_deviation_deg=round(rec.calib.mount_deviation_deg(), 2),
                            optical_axis_tilt_deg=round(rec.calib.optical_axis_tilt_deg(), 2))
    (out / "calib_check.json").write_text(json.dumps(r, indent=2) + "\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 3.5))
        ax.plot(np.array(r["lags_s"]) * 1e3, r["lag_scores"], color="#2a78d6", lw=2)
        ax.axvline(r["time_offset_s"] * 1e3, color="#555", lw=1, ls="--")
        ax.set_xlabel("lag added to camera stamps, ms")
        ax.set_ylabel("correlation flow vs gyro")
        ax.set_title(f"camera->IMU time offset: {r['time_offset_s'] * 1e3:+.1f} ms", loc="left")
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(out / "calib_check_time_offset.png", dpi=130)
    except ImportError:
        pass

    print(f"frame pairs: {r['frame_pairs']}; gyro rate RMS x/y: {r['gyro_rate_rms_rad_s']} rad/s")
    print(f"time offset camera->IMU (remaining): {r['time_offset_s'] * 1e3:+.1f} ms "
          f"(correlation {r['correlation_at_best_lag']}, at zero lag {r['correlation_at_zero_lag']})")
    print(f"axis correlation: {r['axis_correlation']}")
    print(f"slope flow/gyro (focal length check): {r['slope_flow_over_gyro']}")
    print(f"rotation-compensation residual, rad/s: {r['residual_rad_s']}")
    for note in r["notes"]:
        print(" -", note)


if __name__ == "__main__":
    main()
