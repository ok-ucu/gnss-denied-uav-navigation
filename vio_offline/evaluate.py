#!/usr/bin/env python3
"""Compare estimated trajectories with a reference: ATE, drift, RPE, plots.

    python3 evaluate.py --ref RECORDING_OR_FILE --est vio=OUT/vio.csv --est baseline=OUT2/baseline.csv --out REPORT_DIR

--ref is a recording folder (its ground truth is used: groundtruth.csv, EuRoC
state_groundtruth_estimate0, or the recorder's GPS) or a trajectory file
(*.tum / *.csv from these tools, e.g. Person 1's gps_enu.tum).
All files are ENU, time in seconds from the first camera frame (trajectory.py).

Alignment (--align):
  yaw     rotation about the vertical + translation. Default: heading and
          origin are not observable by VIO, so this is the fair comparison.
  origin  only shift both tracks to start at the same point (keeps the
          estimate's own heading; meaningful when it claims north-referenced NED).
  se3     full 6-DoF rigid fit;   sim3: se3 + scale (shows a scale error).
--align-seconds N fits the alignment on the first N seconds only, so the
error afterwards is DRIFT and not a best fit over the whole flight.

Numbers reported per estimate:
  ATE          RMSE / mean / median / max of the position error after alignment
  final error  position error at the last common sample
  drift %      final error and max error divided by the reference path length
  RPE          error of relative motion over fixed path segments (default 10 m), in % of the segment
  scale        estimate/reference scale from a sim3 fit (1.0 = correct scale)
  coverage     share of the reference time span for which the estimate exists
"""
import argparse
import json
from pathlib import Path

import numpy as np

import recording as rec_io
from rotations import rot_z
from trajectory import read_any


# ---------- association ----------
def associate(est, ref, max_gap=0.5):
    """Interpolate the reference at the estimate's timestamps.

    Returns (mask over est rows, reference positions at est[mask]). Samples
    outside the reference span, or inside a reference gap longer than max_gap,
    are dropped."""
    t = est.t
    idx = np.searchsorted(ref.t, t)
    inside = (idx > 0) & (idx < len(ref.t))
    idx = np.clip(idx, 1, len(ref.t) - 1)
    gap = ref.t[idx] - ref.t[idx - 1]
    mask = inside & (gap <= max_gap)
    p = np.column_stack([np.interp(t[mask], ref.t, ref.p_enu[:, k]) for k in range(3)])
    return mask, p


# ---------- alignment ----------
def umeyama(src, dst, with_scale=False):
    """Least-squares R, t, s with dst ~ s * R @ src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(b.T @ a / len(src))
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1.0
    R = U @ D @ Vt
    var = (a ** 2).sum() / len(src)
    s = float(np.trace(np.diag(S) @ D) / var) if with_scale and var > 0 else 1.0
    return R, mu_d - s * R @ mu_s, s


def yaw_fit(src, dst):
    """Rotation about z + translation, least squares."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    angle = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    R = rot_z(angle)
    return R, mu_d - R @ mu_s, 1.0


def align(est_p, ref_p, mode, fit_mask=None):
    """Return (aligned estimate, info). fit_mask selects the samples used for the fit."""
    fit = slice(None) if fit_mask is None else fit_mask
    if mode == "origin":
        R, t, s = np.eye(3), ref_p[fit][0] - est_p[fit][0], 1.0
    elif mode == "yaw":
        R, t, s = yaw_fit(est_p[fit], ref_p[fit])
    elif mode == "se3":
        R, t, s = umeyama(est_p[fit], ref_p[fit])
    elif mode == "sim3":
        R, t, s = umeyama(est_p[fit], ref_p[fit], with_scale=True)
    else:
        raise ValueError(mode)
    info = dict(mode=mode, yaw_deg=round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 2), scale=round(s, 4),
                translation=[round(float(x), 3) for x in t])
    return s * est_p @ R.T + t, info


# ---------- metrics ----------
def stats(err):
    return dict(rmse=round(float(np.sqrt(np.mean(err ** 2))), 3), mean=round(float(err.mean()), 3),
                median=round(float(np.median(err)), 3), max=round(float(err.max()), 3),
                p95=round(float(np.percentile(err, 95)), 3))


def rpe(est_p, ref_p, segment_m):
    """Relative position error over reference path segments of segment_m (yaw-aligned per segment start
    is NOT applied: this is the error of the displacement vector length-normalised)."""
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref_p, axis=0), axis=1))])
    out = []
    j = 0
    for i in range(len(dist)):
        while j < len(dist) and dist[j] - dist[i] < segment_m:
            j += 1
        if j >= len(dist):
            break
        d_ref, d_est = ref_p[j] - ref_p[i], est_p[j] - est_p[i]
        out.append(np.linalg.norm(d_est - d_ref) / (dist[j] - dist[i]) * 100.0)
    return np.array(out)


def evaluate_one(est, ref, mode="yaw", align_seconds=None, segment_m=10.0, max_gap=0.5):
    mask, ref_p = associate(est, ref, max_gap)
    if mask.sum() < 10:
        return dict(error="fewer than 10 samples overlap the reference in time"), None
    t, est_p = est.t[mask], est.p_enu[mask]
    fit_mask = None
    if align_seconds:
        fit_mask = t <= t[0] + align_seconds
        if fit_mask.sum() < 5:
            fit_mask = None
    aligned, info = align(est_p, ref_p, mode, fit_mask)
    err_vec = aligned - ref_p
    err = np.linalg.norm(err_vec, axis=1)
    err_h = np.linalg.norm(err_vec[:, :2], axis=1)
    path = float(np.sum(np.linalg.norm(np.diff(ref_p, axis=0), axis=1)))
    _, sim3 = align(est_p, ref_p, "sim3")
    seg = rpe(aligned, ref_p, segment_m)
    ref_span = ref.t[-1] - ref.t[0]
    result = dict(
        samples=int(mask.sum()), t_first=round(float(t[0]), 2), t_last=round(float(t[-1]), 2),
        coverage_of_reference=round(float((t[-1] - t[0]) / ref_span), 3) if ref_span > 0 else None,
        longest_gap_s=round(float(np.max(np.diff(t))), 2) if len(t) > 1 else None,
        alignment=dict(info, fitted_on=f"first {align_seconds:g} s" if fit_mask is not None else "whole overlap"),
        reference_path_m=round(path, 2),
        estimate_path_m=round(float(np.sum(np.linalg.norm(np.diff(aligned, axis=0), axis=1))), 2),
        ate_3d_m=stats(err), ate_horizontal_m=stats(err_h),
        vertical_error_m=stats(np.abs(err_vec[:, 2])),
        final_error_m=round(float(err[-1]), 3), final_error_horizontal_m=round(float(err_h[-1]), 3),
        drift_final_percent=round(float(err_h[-1] / path * 100), 2) if path > 0 else None,
        drift_max_percent=round(float(err_h.max() / path * 100), 2) if path > 0 else None,
        rpe_percent=dict(segment_m=segment_m, **stats(seg)) if len(seg) else None,
        scale_from_sim3=sim3["scale"])
    # reported uncertainty vs actual error (only meaningful with origin alignment or short fit window)
    if est.cov_p_enu is not None:
        cov = est.cov_p_enu[mask]
        sigma_h = np.sqrt(np.clip(cov[:, 0, 0] + cov[:, 1, 1], 0, None))
        ok = sigma_h > 0
        if ok.any():
            ratio = err_h[ok] / sigma_h[ok]
            result["covariance_check"] = dict(
                sigma_h_median_m=round(float(np.median(sigma_h[ok])), 3),
                sigma_h_final_m=round(float(sigma_h[ok][-1]), 3),
                error_over_sigma_median=round(float(np.median(ratio)), 2),
                error_over_sigma_p95=round(float(np.percentile(ratio, 95)), 2),
                share_within_3_sigma=round(float(np.mean(ratio <= 3.0)), 3),
                note="error/sigma should be around 1 and mostly below 3; much larger = overconfident")
    series = dict(t=t, aligned=aligned, ref=ref_p, err=err, err_h=err_h)
    return result, series


# ---------- plots ----------
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87cb0", "#008300"]


def plot(series_by_name, ref, out_dir, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    ax.plot(ref.p_enu[:, 0], ref.p_enu[:, 1], color="#333333", lw=2.2, label="reference")
    for (name, s), color in zip(series_by_name.items(), COLORS):
        ax.plot(s["aligned"][:, 0], s["aligned"][:, 1], color=color, lw=1.6, label=name)
        ax.plot(*s["aligned"][0, :2], "o", color=color, ms=5)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("east, m")
    ax.set_ylabel("north, m")
    ax.set_title(f"{title}: top view", loc="left")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(Path(out_dir) / "trajectory_top.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.5, 6), sharex=True)
    for (name, s), color in zip(series_by_name.items(), COLORS):
        axes[0].plot(s["t"], s["err_h"], color=color, lw=1.6, label=name)
        axes[1].plot(s["t"], s["aligned"][:, 2] - s["ref"][:, 2], color=color, lw=1.6, label=name)
    axes[0].set_ylabel("horizontal error, m")
    axes[0].set_title(f"{title}: position error over time", loc="left")
    axes[1].set_ylabel("vertical error, m")
    axes[1].set_xlabel("time from first frame, s")
    for a in axes:
        a.grid(alpha=0.25)
    axes[0].legend(frameon=False)
    fig.tight_layout()
    fig.savefig(Path(out_dir) / "error_over_time.png", dpi=130)
    plt.close(fig)


def load_reference(path):
    path = Path(path)
    if path.is_dir():
        gt, source = rec_io.ground_truth(path)
        if gt is None:
            raise SystemExit(f"{path} has no ground truth (groundtruth.csv, EuRoC state estimate or GPS)")
        return gt, source
    return read_any(path, "reference"), str(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--est", action="append", required=True, metavar="NAME=FILE")
    ap.add_argument("--out", required=True)
    ap.add_argument("--align", choices=["yaw", "origin", "se3", "sim3"], default="yaw")
    ap.add_argument("--align-seconds", type=float, default=None)
    ap.add_argument("--segment", type=float, default=10.0, help="RPE segment length, m")
    ap.add_argument("--start", type=float, default=None, help="evaluate from this time (s from first frame)")
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--common-window", action="store_true",
                    help="evaluate every estimate on the time span where ALL estimates exist")
    ap.add_argument("--title", default=None)
    ap.add_argument("--ref-time-offset", type=float, default=0.0,
                    help="seconds added to the reference timestamps (for a reference file on another time base, "
                         "e.g. a GPS track whose time starts at its first fix)")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ref, ref_source = load_reference(args.ref)
    ref.t = ref.t + args.ref_time_offset
    ests = {}
    for item in args.est:
        name, _, path = item.partition("=")
        ests[name] = read_any(path, name)
    lo = max([args.start if args.start is not None else -np.inf] +
             ([e.t[0] for e in ests.values()] if args.common_window else []))
    hi = min([args.end if args.end is not None else np.inf] +
             ([e.t[-1] for e in ests.values()] if args.common_window else []))

    report = dict(reference=ref_source, reference_rows=len(ref), reference_has_orientation=bool(ref.has_orientation),
                  window=[None if not np.isfinite(lo) else round(float(lo), 2),
                          None if not np.isfinite(hi) else round(float(hi), 2)],
                  estimates={})
    series = {}
    for name, est in ests.items():
        est = est.sliced((est.t >= lo) & (est.t <= hi))
        result, s = evaluate_one(est, ref, args.align, args.align_seconds, args.segment)
        report["estimates"][name] = result
        if s is not None:
            series[name] = s
    (out / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    if series:
        try:
            plot(series, ref, out, args.title or Path(args.ref).name)
        except ImportError:
            print("matplotlib not installed: plots skipped")

    print(f"reference: {ref_source}; alignment: {args.align}"
          + (f" on the first {args.align_seconds:g} s" if args.align_seconds else ""))
    head = f"{'estimate':<12}{'ATE rmse':>10}{'max':>9}{'final':>9}{'drift %':>9}{'RPE %':>8}{'scale':>8}{'cover':>7}"
    print(head)
    for name, r in report["estimates"].items():
        if "error" in r:
            print(f"{name:<12}{r['error']}")
            continue
        print(f"{name:<12}{r['ate_horizontal_m']['rmse']:>10.2f}{r['ate_horizontal_m']['max']:>9.2f}"
              f"{r['final_error_horizontal_m']:>9.2f}{r['drift_final_percent']:>9.2f}"
              f"{(r['rpe_percent'] or {}).get('mean', float('nan')):>8.1f}{r['scale_from_sim3']:>8.3f}"
              f"{r['coverage_of_reference']:>7.2f}")
    print("(horizontal errors in metres; drift % = final horizontal error / reference path length)")


if __name__ == "__main__":
    main()
