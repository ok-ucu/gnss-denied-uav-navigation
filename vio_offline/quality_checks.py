#!/usr/bin/env python3
"""Quality checks for a recording and for VIO / baseline runs on it.

    python3 quality_checks.py RECORDING [--vio VIO_OUT] [--baseline BASELINE_OUT] --out REPORT_DIR

Three groups, each line marked OK / WARN / FAIL:

  INPUT (the recording itself; run this right after a flight to decide whether
  it must be re-recorded)
    IMU rate, gaps, time going backwards, units; camera rate and dropped frames;
    attitude and range-finder coverage; whether the streams overlap; ground truth.

  OUR VIO (needs --vio, the folder written by replay_vio.py)
    tracking loss   share of frames accepted, intervals without an accepted frame,
                    time spent "degraded" (quality < 0.5) and "paused" (< 0.1)
    covariance      NIS statistics (the filter's own consistency), and, with ground
                    truth, actual error divided by the reported sigma
    drift           ATE, final error, % of path, with ground truth
    IMU gaps        how often the estimator had to coast without accelerometer data
    divergence      non-finite or absurd state, error beyond a hard bound

  BASELINE (needs --baseline, the folder written by baseline/run_baseline.py)
    initialisation time, coverage, number of tracked features, scale, drift,
    estimated camera-IMU time offset.

Writes REPORT_DIR/quality.json and quality.md. Exit code 1 if any FAIL.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

import recording as rec_io
from evaluate import evaluate_one
from trajectory import read_csv

CHI2_95_DOF2 = 5.99


class Report:
    def __init__(self):
        self.sections = []
        self.data = {}

    def section(self, name):
        self.sections.append((name, []))
        self.data[name] = {}
        self._name = name

    def add(self, status, key, text, value=None):
        self.sections[-1][1].append((status, text))
        self.data[self._name][key] = dict(status=status, text=text, value=value)

    @property
    def failed(self):
        return any(s == "FAIL" for _, lines in self.sections for s, _ in lines)

    def markdown(self, title):
        out = [f"# {title}", ""]
        for name, lines in self.sections:
            out += [f"## {name}", "", "| | Check |", "|---|---|"]
            out += [f"| {s} | {t} |" for s, t in lines]
            out.append("")
        return "\n".join(out)

    def console(self):
        for name, lines in self.sections:
            print(f"\n== {name}")
            for s, t in lines:
                print(f"[{s:<4}] {t}")


def grade(value, ok, warn, higher_is_better=True):
    """OK / WARN / FAIL from two thresholds."""
    if higher_is_better:
        return "OK" if value >= ok else "WARN" if value >= warn else "FAIL"
    return "OK" if value <= ok else "WARN" if value <= warn else "FAIL"


def stream_stats(t):
    if len(t) < 2:
        return None
    dt = np.diff(t)
    med = float(np.median(dt))
    return dict(n=int(len(t)), rate_hz=1.0 / med if med > 0 else 0.0, median_dt=med, max_gap=float(dt.max()),
                gaps_over_3_periods=int((dt > 3 * med).sum()), backwards=int((dt <= 0).sum()),
                span=(float(t[0]), float(t[-1])))


def intervals(t, bad, min_len):
    """[start, end] of runs where bad is True and the run lasts at least min_len seconds."""
    out, start = [], None
    for i, flag in enumerate(bad):
        if flag and start is None:
            start = i
        if start is not None and (not flag or i == len(bad) - 1):
            end = i if flag else i - 1
            if t[end] - t[start] >= min_len:
                out.append((float(t[start]), float(t[end])))
            start = None
    return out


# ---------- INPUT ----------
def check_input(rep, rec):
    rep.section("Input: recording")
    imu = stream_stats(rec.imu_t)
    cam = stream_stats(rec.frame_t)
    if imu is None or cam is None:
        rep.add("FAIL", "streams", "IMU or camera stream is missing or has a single row")
        return
    rep.add(grade(imu["rate_hz"], 95, 45), "imu_rate",
            f"IMU rate {imu['rate_hz']:.0f} Hz over {imu['n']} rows (VIO needs >= 50, OpenVINS/Kalibr want >= 100-200)",
            round(imu["rate_hz"], 1))
    dt = np.diff(rec.imu_t)
    n_coast, n_skip = int(((dt >= 0.2) & (dt < 3.0)).sum()), int((dt >= 3.0).sum())
    status = "FAIL" if n_skip or imu["backwards"] else "WARN" if n_coast or imu["gaps_over_3_periods"] > 0.01 * imu["n"] else "OK"
    rep.add(status, "imu_gaps",
            f"IMU gaps: longest {imu['max_gap'] * 1e3:.0f} ms; {imu['gaps_over_3_periods']} longer than 3 periods; "
            f"{n_coast} of 0.2-3 s (VIO coasts without accelerometer); {n_skip} of 3 s or more (VIO skips); "
            f"{imu['backwards']} stamps not increasing",
            dict(max_gap_s=imu["max_gap"], over_3_periods=imu["gaps_over_3_periods"], coast=n_coast, skip=n_skip,
                 backwards=imu["backwards"]))
    f_norm = float(np.median(np.linalg.norm(rec.acc, axis=1)))
    rep.add(grade(abs(f_norm - 9.81), 0.6, 1.5, False), "imu_units",
            f"median accelerometer magnitude {f_norm:.2f} m/s^2 (expect ~9.8: checks units and scale); "
            f"max gyro magnitude {float(np.abs(rec.gyro).max()):.2f} rad/s", round(f_norm, 3))
    hp = rec.acc - np.array([np.convolve(rec.acc[:, k], np.ones(21) / 21, "same") for k in range(3)]).T
    vib = float(np.sqrt(np.mean(hp ** 2)))
    rep.add(grade(vib, 1.0, 3.0, False), "vibration",
            f"accelerometer vibration {vib:.2f} m/s^2 RMS after removing the slow part "
            "(above ~1 hurts any inertial method; above 3 expect trouble)", round(vib, 3))

    dtc = np.diff(rec.frame_t)
    dropped = int((dtc > 1.5 * cam["median_dt"]).sum())
    rep.add(grade(cam["rate_hz"], 10, 5), "camera_rate",
            f"camera rate {cam['rate_hz']:.1f} Hz over {cam['n']} frames, {rec.duration:.1f} s", round(cam["rate_hz"], 2))
    share = dropped / max(len(dtc), 1)
    rep.add(grade(share, 0.01, 0.05, False) if cam["max_gap"] < 1.0 else "FAIL", "camera_gaps",
            f"camera gaps: {dropped} intervals longer than 1.5 frame periods ({share * 100:.1f} %), longest "
            f"{cam['max_gap'] * 1e3:.0f} ms; {cam['backwards']} stamps not increasing",
            dict(dropped_intervals=dropped, max_gap_s=cam["max_gap"]))
    lead, tail = rec.frame_t[0] - rec.imu_t[0], rec.imu_t[-1] - rec.frame_t[-1]
    rep.add("OK" if lead >= 0 and tail >= 0 else "WARN", "overlap",
            f"IMU starts {lead:+.2f} s before the first frame and ends {tail:+.2f} s after the last one "
            "(both should be positive)", dict(lead_s=float(lead), tail_s=float(tail)))

    att = stream_stats(rec.att_t)
    if att:
        rep.add(grade(att["rate_hz"], 20, 5), "attitude",
                f"autopilot attitude {att['rate_hz']:.0f} Hz, longest gap {att['max_gap'] * 1e3:.0f} ms", round(att["rate_hz"], 1))
    else:
        rep.add("WARN", "attitude", "no attitude.csv: the VIO needs roll/pitch/yaw from the autopilot; "
                                    "replay falls back to its own AHRS (gyro-only heading)")
    if len(rec.range_t):
        d, lo, hi = rec.range_m[:, 0], rec.range_m[:, 1], rec.range_m[:, 2]
        valid = (d >= lo) & (d < hi - 0.05)
        rng = stream_stats(rec.range_t)
        rep.add(grade(valid.mean(), 0.9, 0.5), "range",
                f"range finder {rng['rate_hz']:.0f} Hz, {valid.mean() * 100:.0f} % of readings inside its limits, "
                f"{float(d[valid].min()) if valid.any() else float('nan'):.1f}-{float(d[valid].max()) if valid.any() else float('nan'):.1f} m",
                round(float(valid.mean()), 3))
    else:
        rep.add("WARN", "range", "no distance.csv: flow scale will come from the autopilot height (not independent)")
    c = rec.calib
    dev = c.mount_deviation_deg()
    rep.add("OK" if dev <= 10 else "WARN", "mounting",
            f"calibration {Path(c.source).name}: camera mounting differs from the one vio_frontend.py assumes by "
            f"{dev:.1f} deg" + (" (offline replay rotates the IMU to compensate; the LIVE vio_ekf.py has the mounting "
                                "hard-coded and would be wrong with this camera)" if dev > 10 else "")
            + f"; optical axis {c.optical_axis_tilt_deg():.1f} deg off body-down; time shift "
              f"{c.timeshift_cam_imu_s * 1e3:+.1f} ms", round(dev, 2))
    if rec.gt is None:
        rep.add("WARN", "ground_truth", "no ground truth in the recording: drift cannot be computed")
    else:
        gt = stream_stats(rec.gt.t)
        rep.add("OK", "ground_truth",
                f"ground truth from {rec.gt_source}: {gt['rate_hz']:.1f} Hz, path {rec.gt.path_length():.0f} m, "
                f"covers {gt['span'][0]:.1f}..{gt['span'][1]:.1f} s", dict(path_m=rec.gt.path_length()))


# ---------- OUR VIO ----------
def read_dicts(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def check_vio(rep, rec, vio_dir, max_speed):
    rep.section("Our VIO: " + str(vio_dir))
    vio_dir = Path(vio_dir)
    run = json.loads((vio_dir / "run.json").read_text())
    frames = read_dicts(vio_dir / "frames.csv")
    states = read_dicts(vio_dir / "states.csv")
    s = run["settings"]
    rep.add("OK" if run["vio_source"]["matches_pin"] else "WARN", "source",
            f"estimator sources: {run['vio_source']['origin']}, identical to the pinned vio-setup commit: "
            f"{run['vio_source']['matches_pin']}; attitude: {s['attitude']}, height: {s['vertical']} "
            f"(fallback {s['height_fallback']}), gyro for flow: {s.get('gyro', 'sample')}, "
            f"time shift {s['timeshift_cam_imu_s'] * 1e3:+.1f} ms")
    for flag, text in (("gt" == s["attitude"], "attitude"), ("GROUND TRUTH" in s["height_fallback"], "height")):
        if flag:
            rep.add("WARN", f"uses_gt_{text}", f"this run takes {text} from ground truth: not an independent estimate")

    # tracking
    t = np.array([float(r["t"]) for r in frames])
    result = np.array([r["result"] for r in frames])
    acc = result == "accepted"
    n = max(len(frames), 1)
    rep.add(grade(acc.mean() if len(acc) else 0.0, 0.8, 0.5), "accepted_share",
            f"frames: {acc.sum()} accepted ({acc.mean() * 100:.0f} %), {(result == 'rejected_by_gate').sum()} rejected by "
            f"the innovation gate ({(result == 'rejected_by_gate').mean() * 100:.0f} %), "
            f"{(result == 'no_measurement').sum()} without a flow measurement "
            f"({(result == 'no_measurement').mean() * 100:.0f} %)",
            dict(accepted=int(acc.sum()), gated=int((result == "rejected_by_gate").sum()),
                 no_measurement=int((result == "no_measurement").sum()), frames=n))
    lost = intervals(t, ~acc, 1.0)
    lost_total = sum(b - a for a, b in lost)
    longest = max((b - a for a, b in lost), default=0.0)
    rep.add(grade(longest, 1.0, 3.0, False), "tracking_loss",
            f"tracking loss (>= 1 s without an accepted frame): {len(lost)} intervals, {lost_total:.1f} s in total, "
            f"longest {longest:.1f} s" + (f"; worst: {', '.join(f'{a:.0f}-{b:.0f} s' for a, b in sorted(lost, key=lambda x: x[0] - x[1])[:4])}" if lost else ""),
            dict(intervals=lost, total_s=lost_total, longest_s=longest))
    ts = np.array([float(r["t"]) for r in states])
    q = np.array([float(r["quality"]) for r in states])
    dts = np.diff(ts, append=ts[-1])
    span = max(ts[-1] - ts[0], 1e-9)
    degraded, paused = float(dts[q < 0.5].sum() / span), float(dts[q < 0.1].sum() / span)
    rep.add(grade(degraded, 0.05, 0.25, False), "quality",
            f"self-reported quality: degraded (< 0.5) {degraded * 100:.0f} % of the time, output paused (< 0.1) "
            f"{paused * 100:.0f} %; {run['counters'].get('degraded_events', 0)} degraded events, "
            f"{run['velocity_resets']} velocity resets", dict(degraded_share=degraded, paused_share=paused,
                                                              velocity_resets=run["velocity_resets"]))

    # covariance: NIS
    nis = np.array([float(r["nis"]) for r in frames if r["result"] != "no_measurement" and r["nis"] not in ("", "nan")])
    nis = nis[nis > 0]
    if len(nis) >= 30:
        mean, below = float(nis.mean()), float(np.mean(nis < CHI2_95_DOF2))
        median = float(np.median(nis))
        status = "FAIL" if (median > 6 or below < 0.5) else "WARN" if (mean > 4 or below < 0.9) else "OK"
        rep.add(status, "nis",
                f"NIS of the flow velocity update (2 dof): median {median:.1f}, mean {mean:.1f} (expect ~2), "
                f"{below * 100:.0f} % below {CHI2_95_DOF2} (expect ~95 %). Large values = the filter trusts the flow "
                "(or itself) more than the data supports", dict(mean=mean, median=median, share_below_chi2_95=below))
    c = run["counters"]
    rep.add("OK" if not c.get("predict_kinematic") and not c.get("imu_skipped") else "WARN", "imu_gaps_seen",
            f"estimator IMU steps: {c.get('predict', 0)} normal, {c.get('predict_kinematic', 0)} coasting (0.2-3 s gap), "
            f"{c.get('imu_skipped', 0)} skipped")
    if run.get("frame_proc_ms"):
        p = run["frame_proc_ms"]
        rep.add(grade(p["p95"], 50, 100, False), "processing",
                f"processing per frame on this computer: mean {p['mean']} ms, p95 {p['p95']} ms "
                "(the brief allows 50 ms frame -> MAVLink on the on-board computer; this is not that computer)", p)

    # divergence and drift
    state = np.array([[float(r[k]) for k in ("n", "e", "d", "vn", "ve", "vd")] for r in states])
    finite = bool(np.isfinite(state).all())
    speed = float(np.nanmax(np.linalg.norm(state[:, 3:6], axis=1)))
    rep.add("OK" if finite and speed < max_speed else "FAIL", "state_sanity",
            f"state finite: {finite}; highest estimated speed {speed:.1f} m/s (bound {max_speed:g})", speed)
    if rec.gt is not None and (vio_dir / "vio.csv").exists():
        est = read_csv(vio_dir / "vio.csv")
        m, series = evaluate_one(est, rec.gt, "yaw")
        if "error" not in m:
            rep.data[rep._name]["metrics"] = m
            path = m["reference_path_m"]
            bound = max(25.0, 0.25 * path)
            rep.add("FAIL" if m["ate_horizontal_m"]["max"] > bound else "OK", "divergence",
                    f"largest horizontal error {m['ate_horizontal_m']['max']:.1f} m against a divergence bound of "
                    f"{bound:.0f} m (25 m or 25 % of the {path:.0f} m path)", m["ate_horizontal_m"]["max"])
            yaw = m["alignment"]["yaw_deg"]
            if s["attitude"] == "recorded":
                # with the autopilot's attitude the track is north-referenced, so no rotation should be needed
                rep.add(grade(abs(yaw), 10, 30, False), "heading",
                        f"heading: the track had to be rotated by {yaw:+.1f} deg to fit ground truth (expect ~0 with "
                        "autopilot attitude; ~180 = motion sign reversed, ~90 = axes swapped)", yaw)
            else:
                rep.add("OK", "heading", f"heading: track rotated by {yaw:+.1f} deg to fit ground truth "
                                         f"(attitude source '{s['attitude']}' has no absolute heading, so this is expected)", yaw)
            rep.add(grade(m["drift_final_percent"], 2.0, 5.0, False), "drift",
                    f"drift (yaw-aligned to ground truth): ATE RMSE {m['ate_horizontal_m']['rmse']:.2f} m, final error "
                    f"{m['final_error_horizontal_m']:.2f} m = {m['drift_final_percent']:.1f} % of path, "
                    f"scale {m['scale_from_sim3']:.3f}", m["drift_final_percent"])
            cc = m.get("covariance_check")
            if cc:
                status = grade(cc["share_within_3_sigma"], 0.95, 0.7)
                if status == "OK" and cc["error_over_sigma_median"] < 0.2:
                    status = "WARN"          # safe, but the autopilot would give the data almost no weight
                rep.add(status, "covariance_vs_truth",
                        f"reported position sigma vs actual error: sigma median {cc['sigma_h_median_m']:.2f} m, "
                        f"error/sigma median {cc['error_over_sigma_median']:.1f} (expect ~1), "
                        f"{cc['share_within_3_sigma'] * 100:.0f} % of samples within 3 sigma (expect ~99 %). "
                        "Low share = overconfident; ratio far below 1 = covariance much larger than the real error", cc)


# ---------- BASELINE ----------
def check_baseline(rep, rec, base_dir):
    rep.section("Baseline: " + str(base_dir))
    base_dir = Path(base_dir)
    run = json.loads((base_dir / "run.json").read_text())
    if not (base_dir / "baseline.csv").exists():
        rep.add("FAIL", "states", f"no trajectory: the baseline did not initialise (exit code {run['exit_code']})")
        return
    est = read_csv(base_dir / "baseline.csv")
    rep.add("OK", "version", f"{run['baseline']}, commit {run['openvins_commit'][:10]}, IMU noise preset "
                             f"{run['imu_noise_preset']}, init {run['init']}")
    first = est.t[0] - run["start"]
    rep.add(grade(first, 10, 30, False), "init", f"initialised {first:.1f} s after its start time; "
                                                 f"{len(est)} poses until {est.t[-1]:.1f} s", float(first))
    feats = est.extra["n_slam"] + est.extra["n_msckf"]
    low = intervals(est.t, feats < 10, 1.0)
    rep.add("OK" if not low else "WARN", "features",
            f"tracked features used per update: median {np.median(feats):.0f}; {len(low)} intervals of >= 1 s with fewer than 10"
            + (f" ({', '.join(f'{a:.0f}-{b:.0f} s' for a, b in low[:4])})" if low else ""), dict(intervals=low))
    speed = float(np.linalg.norm(est.v_enu, axis=1).max())
    finite = bool(np.isfinite(est.p_enu).all())
    rep.add("OK" if finite and speed < 40 else "FAIL", "state_sanity", f"state finite: {finite}; highest speed {speed:.1f} m/s", speed)
    dt0, dt1 = run["timeshift_cam_imu_s"], float(est.extra["dt_cam_imu"][-1])
    rep.add("OK" if abs(dt1 - dt0) < 0.01 else "WARN", "time_offset",
            f"camera-IMU time offset: given {dt0 * 1e3:+.1f} ms, estimated online {dt1 * 1e3:+.1f} ms", dt1)
    p = est.extra["proc_ms"]
    rep.add("OK", "processing", f"processing per frame: mean {p.mean():.1f} ms, p95 {np.percentile(p, 95):.1f} ms")
    if rec.gt is not None:
        m, _ = evaluate_one(est, rec.gt, "yaw")
        if "error" not in m:
            rep.data[rep._name]["metrics"] = m
            bound = max(25.0, 0.25 * m["reference_path_m"])
            rep.add("FAIL" if m["ate_horizontal_m"]["max"] > bound else "OK", "divergence",
                    f"largest horizontal error {m['ate_horizontal_m']['max']:.1f} m against a bound of {bound:.0f} m",
                    m["ate_horizontal_m"]["max"])
            rep.add(grade(m["drift_final_percent"], 2.0, 5.0, False), "drift",
                    f"drift (yaw-aligned): ATE RMSE {m['ate_horizontal_m']['rmse']:.2f} m, final error "
                    f"{m['final_error_horizontal_m']:.2f} m = {m['drift_final_percent']:.1f} % of path", m["drift_final_percent"])
            rep.add(grade(abs(m["scale_from_sim3"] - 1.0), 0.05, 0.15, False), "scale",
                    f"scale {m['scale_from_sim3']:.3f} (monocular VIO gets scale only from the accelerometer)", m["scale_from_sim3"])
            cc = m.get("covariance_check")
            if cc:
                rep.add(grade(cc["share_within_3_sigma"], 0.95, 0.7), "covariance_vs_truth",
                        f"reported sigma vs actual error: error/sigma median {cc['error_over_sigma_median']:.1f}, "
                        f"{cc['share_within_3_sigma'] * 100:.0f} % within 3 sigma", cc)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--camera-params", default=None)
    ap.add_argument("--vio", action="append", default=[], help="replay_vio.py output folder (repeatable)")
    ap.add_argument("--baseline", action="append", default=[], help="run_baseline.py output folder (repeatable)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-speed", type=float, default=40.0, help="sanity bound on estimated speed, m/s")
    args = ap.parse_args()

    rec = rec_io.load(args.recording, args.camera_params)
    rep = Report()
    check_input(rep, rec)
    for d in args.vio:
        check_vio(rep, rec, d, args.max_speed)
    for d in args.baseline:
        check_baseline(rep, rec, d)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "quality.json").write_text(json.dumps(rep.data, indent=2, default=float) + "\n")
    (out / "quality.md").write_text(rep.markdown(f"Quality checks: {rec.name}") + "\n")
    rep.console()
    print(f"\nwrote {out}/quality.md and quality.json")
    raise SystemExit(1 if rep.failed else 0)


if __name__ == "__main__":
    main()
