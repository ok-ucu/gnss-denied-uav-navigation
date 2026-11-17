#!/usr/bin/env python3
"""One command for a new recording: checks, our VIO, the baseline, comparison.

    python3 vio_offline/pipeline.py RECORDING --out OUT [--camera-params calibration/camera_params.json]

Steps (each one is also a standalone script, see vio_offline/README.md):
  1. calib_check.py      camera-IMU time offset, axes, focal length from the flight itself
  2. replay_vio.py       our VIO, exactly as vio_ekf.py runs it  -> OUT/vio
  3. run_baseline.py     OpenVINS in Docker                      -> OUT/baseline   (skip with --no-baseline)
  4. evaluate.py         both against ground truth, same window  -> OUT/eval       (needs ground truth)
  5. quality_checks.py   recording, VIO and baseline checks      -> OUT/quality

OUT/SUMMARY.md collects the verdicts and the numbers. A step that fails does
not stop the others; the summary says which ones failed.

Extra arguments for a step:  --vio-args "--timeshift -0.037 --gyro mean"
                             --baseline-args "--init static --set try_zupt=true"
"""
import argparse
import json
import shlex
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent


def step(name, cmd, log_dir, results):
    print(f"\n=== {name}\n$ {' '.join(shlex.quote(str(c)) for c in cmd)}")
    t0 = time.time()
    with open(log_dir / f"{name}.log", "w") as f:
        proc = subprocess.run([str(c) for c in cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        f.write(proc.stdout)
    tail = "\n".join(proc.stdout.strip().splitlines()[-12:])
    print(tail)
    results[name] = dict(exit_code=proc.returncode, seconds=round(time.time() - t0, 1),
                         command=" ".join(shlex.quote(str(c)) for c in cmd))
    return proc.returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera-params", default=None)
    ap.add_argument("--no-baseline", action="store_true")
    ap.add_argument("--vio-args", default="", help="extra arguments for replay_vio.py")
    ap.add_argument("--baseline-args", default="", help="extra arguments for baseline/run_baseline.py")
    ap.add_argument("--eval-args", default="", help="extra arguments for evaluate.py")
    ap.add_argument("--scale", type=float, default=1.0, help="frame downscale for calib_check and replay")
    args = ap.parse_args()

    out = Path(args.out).resolve()
    logs = out / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    py = sys.executable
    cam = ["--camera-params", args.camera_params] if args.camera_params else []
    results = {}

    step("1_calib_check", [py, HERE / "calib_check.py", args.recording, "--out", out / "calib_check",
                           "--scale", args.scale, *cam], logs, results)
    step("2_replay_vio", [py, HERE / "replay_vio.py", args.recording, "--out", out / "vio", "--scale", args.scale,
                          *cam, *shlex.split(args.vio_args)], logs, results)
    if not args.no_baseline:
        step("3_baseline", [py, HERE / "baseline" / "run_baseline.py", args.recording, "--out", out / "baseline",
                            *cam, *shlex.split(args.baseline_args)], logs, results)
    ests = []
    if (out / "vio" / "vio.csv").exists():
        ests += ["--est", f"vio={out / 'vio' / 'vio.csv'}"]
    if (out / "baseline" / "baseline.csv").exists():
        ests += ["--est", f"baseline={out / 'baseline' / 'baseline.csv'}"]
    if ests:
        step("4_evaluate", [py, HERE / "evaluate.py", "--ref", args.recording, *ests, "--out", out / "eval",
                            "--common-window", *shlex.split(args.eval_args)], logs, results)
    quality = [py, HERE / "quality_checks.py", args.recording, "--out", out / "quality", *cam]
    if (out / "vio" / "run.json").exists():
        quality += ["--vio", out / "vio"]
    if (out / "baseline" / "run.json").exists():
        quality += ["--baseline", out / "baseline"]
    step("5_quality", quality, logs, results)

    # ---- summary ----
    lines = [f"# Offline VIO run: {Path(args.recording).name}", "",
             f"Recording: `{args.recording}`  ", f"Output: `{out}`", "", "## Steps", "",
             "| Step | Exit code | Seconds |", "|---|---|---|"]
    lines += [f"| {name} | {r['exit_code']} | {r['seconds']} |" for name, r in results.items()]
    lines += ["", "Exit code 1 from `5_quality` means at least one check is FAIL; from `4_evaluate` that there is "
                  "no ground truth in the recording.", ""]
    if (out / "calib_check" / "calib_check.json").exists():
        c = json.loads((out / "calib_check" / "calib_check.json").read_text())
        lines += ["## Calibration check", "",
                  f"- remaining camera->IMU time offset: {c['time_offset_s'] * 1e3:+.1f} ms "
                  f"(correlation {c['correlation_at_best_lag']})",
                  f"- axis correlation: {c['axis_correlation']}", f"- slope flow/gyro: {c['slope_flow_over_gyro']}",
                  *[f"- {n}" for n in c["notes"]], ""]
    if (out / "eval" / "metrics.json").exists():
        m = json.loads((out / "eval" / "metrics.json").read_text())
        lines += ["## Accuracy against ground truth", "", f"Reference: {m['reference']}; window {m['window']} s; "
                  "yaw + translation alignment; horizontal errors.", "",
                  "| Estimate | ATE RMSE, m | Max, m | Final, m | Drift, % of path | Scale | Path, m |", "|---|---|---|---|---|---|---|"]
        for name, r in m["estimates"].items():
            if "error" in r:
                lines.append(f"| {name} | {r['error']} | | | | | |")
            else:
                lines.append(f"| {name} | {r['ate_horizontal_m']['rmse']} | {r['ate_horizontal_m']['max']} | "
                             f"{r['final_error_horizontal_m']} | {r['drift_final_percent']} | {r['scale_from_sim3']} | "
                             f"{r['reference_path_m']} |")
        lines += ["", "Plots: `eval/trajectory_top.png`, `eval/error_over_time.png`.", ""]
    if (out / "quality" / "quality.md").exists():
        lines += (out / "quality" / "quality.md").read_text().splitlines()[1:]
    (out / "SUMMARY.md").write_text("\n".join(lines) + "\n")
    (out / "pipeline.json").write_text(json.dumps(results, indent=2) + "\n")
    print(f"\nSummary: {out / 'SUMMARY.md'}")


if __name__ == "__main__":
    main()
