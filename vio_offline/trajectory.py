#!/usr/bin/env python3
"""Trajectory container and export: CSV, TUM and EuRoC.

All exported files use one convention so that our VIO, the baseline and the
ground truth can be compared directly:

  world ENU (x east, y north, z up), body FLU, Hamilton quaternion body->world,
  time = seconds from the first camera frame of the recording (`t0_ns` is kept
  in the sidecar JSON and in the CSV so absolute stamps can be restored).

Formats:
  *.tum         "t x y z qx qy qz qw"                     (evo, rpg_trajectory_evaluation)
  *_euroc.csv   "#timestamp [ns], p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z"
  *.csv         everything above plus NED columns, covariance and quality

Command line: convert between the formats.
    python3 trajectory.py in.tum --out-dir out/ --name vio
"""
import argparse
import csv
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from rotations import NED_ENU, R_to_quat, euler_to_R, ned_frd_to_enu_flu

IDENTITY_Q = np.array([0.0, 0.0, 0.0, 1.0])


@dataclass
class Trajectory:
    t: np.ndarray                       # seconds from t0_ns
    p_enu: np.ndarray                   # (N, 3)
    q: np.ndarray                       # (N, 4) x y z w, body FLU -> ENU
    v_enu: np.ndarray = None            # (N, 3) or None
    cov_p_enu: np.ndarray = None        # (N, 3, 3) position covariance or None
    t0_ns: int = 0
    name: str = "trajectory"
    has_orientation: bool = True        # False: q is a placeholder (e.g. GPS)
    extra: dict = field(default_factory=dict)   # per-row extra columns

    def __len__(self):
        return len(self.t)

    @property
    def p_ned(self):
        return self.p_enu @ NED_ENU.T

    def path_length(self):
        return float(np.sum(np.linalg.norm(np.diff(self.p_enu, axis=0), axis=1))) if len(self) > 1 else 0.0

    def sliced(self, mask):
        pick = lambda a: None if a is None else a[mask]
        return Trajectory(self.t[mask], self.p_enu[mask], self.q[mask], pick(self.v_enu),
                          pick(self.cov_p_enu), self.t0_ns, self.name, self.has_orientation,
                          {k: np.asarray(v)[mask] for k, v in self.extra.items()})

    # ---------- constructors ----------
    @classmethod
    def from_ned(cls, t, p_ned, rpy=None, v_ned=None, cov_p_ned=None, **kw):
        """From NED positions and (optionally) FRD roll/pitch/yaw in radians."""
        p_ned = np.asarray(p_ned, dtype=float).reshape(-1, 3)
        n = len(p_ned)
        if rpy is None:
            q = np.tile(IDENTITY_Q, (n, 1))
            kw.setdefault("has_orientation", False)
        else:
            q = np.array([R_to_quat(ned_frd_to_enu_flu(euler_to_R(*a))) for a in np.asarray(rpy)])
        v = None if v_ned is None else np.asarray(v_ned, dtype=float) @ NED_ENU.T
        cov = None
        if cov_p_ned is not None:
            cov = np.einsum("ij,njk,lk->nil", NED_ENU, np.asarray(cov_p_ned, dtype=float), NED_ENU)
        return cls(np.asarray(t, dtype=float), p_ned @ NED_ENU.T, q, v, cov, **kw)

    # ---------- writers ----------
    def write_tum(self, path):
        with open(path, "w", encoding="utf-8") as f:
            f.write("# timestamp tx ty tz qx qy qz qw  (ENU world, FLU body, t = s from first frame)\n")
            for t, p, q in zip(self.t, self.p_enu, self.q):
                f.write(f"{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}\n")

    def write_euroc(self, path):
        v = self.v_enu if self.v_enu is not None else np.full((len(self), 3), np.nan)
        with open(path, "w", encoding="utf-8") as f:
            f.write("#timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], "
                    "q_RS_y [], q_RS_z [], v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1]\n")
            for t, p, q, vel in zip(self.t, self.p_enu, self.q, v):
                t_ns = self.t0_ns + int(round(t * 1e9))
                f.write(f"{t_ns},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},{q[3]:.7f},{q[0]:.7f},{q[1]:.7f},{q[2]:.7f},"
                        f"{vel[0]:.6f},{vel[1]:.6f},{vel[2]:.6f}\n")

    def write_csv(self, path):
        ned = self.p_ned
        head = ["t_s", "t_ns", "east_m", "north_m", "up_m", "north_ned_m", "east_ned_m", "down_ned_m",
                "qx", "qy", "qz", "qw"]
        if self.v_enu is not None:
            head += ["v_east", "v_north", "v_up"]
        if self.cov_p_enu is not None:
            head += ["cov_ee", "cov_en", "cov_eu", "cov_nn", "cov_nu", "cov_uu"]
        head += list(self.extra)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(head)
            for i in range(len(self)):
                row = [f"{self.t[i]:.6f}", self.t0_ns + int(round(self.t[i] * 1e9)),
                       *(f"{x:.6f}" for x in self.p_enu[i]), *(f"{x:.6f}" for x in ned[i]),
                       *(f"{x:.7f}" for x in self.q[i])]
                if self.v_enu is not None:
                    row += [f"{x:.6f}" for x in self.v_enu[i]]
                if self.cov_p_enu is not None:
                    c = self.cov_p_enu[i]
                    row += [f"{x:.6g}" for x in (c[0, 0], c[0, 1], c[0, 2], c[1, 1], c[1, 2], c[2, 2])]
                row += [self.extra[k][i] for k in self.extra]
                w.writerow(row)

    def write_all(self, out_dir, stem=None):
        """Write <stem>.csv, <stem>.tum, <stem>_euroc.csv and <stem>.json."""
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = stem or self.name
        self.write_csv(out_dir / f"{stem}.csv")
        self.write_tum(out_dir / f"{stem}.tum")
        self.write_euroc(out_dir / f"{stem}_euroc.csv")
        info = dict(name=self.name, rows=len(self), t0_ns=int(self.t0_ns),
                    time="seconds from t0_ns (first camera frame of the recording)",
                    world="ENU", body="FLU", quaternion="Hamilton body->world, TUM order x y z w",
                    has_orientation=bool(self.has_orientation),
                    has_velocity=self.v_enu is not None, has_covariance=self.cov_p_enu is not None,
                    path_length_m=round(self.path_length(), 3))
        (out_dir / f"{stem}.json").write_text(json.dumps(info, indent=2) + "\n")
        return info


# ---------- readers ----------
def read_tum(path, name=None, t0_ns=0):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                rows.append([float(x) for x in line.replace(",", " ").split()[:8]])
    a = np.array(rows, dtype=float).reshape(-1, 8)
    q = a[:, 4:8]
    has_orientation = bool(len(q)) and not np.allclose(q, q[0])
    return Trajectory(a[:, 0], a[:, 1:4], q, t0_ns=t0_ns, name=name or Path(path).stem,
                      has_orientation=has_orientation)


def read_csv(path, name=None):
    """Read a file written by Trajectory.write_csv."""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t = col("t_s")
    t0_ns = int(rows[0]["t_ns"]) - int(round(t[0] * 1e9)) if rows else 0
    p = np.column_stack([col("east_m"), col("north_m"), col("up_m")])
    q = np.column_stack([col("qx"), col("qy"), col("qz"), col("qw")])
    v = np.column_stack([col("v_east"), col("v_north"), col("v_up")]) if rows and "v_east" in rows[0] else None
    cov = None
    if rows and "cov_ee" in rows[0]:
        ee, en, eu, nn, nu, uu = (col(k) for k in ("cov_ee", "cov_en", "cov_eu", "cov_nn", "cov_nu", "cov_uu"))
        cov = np.array([[ee, en, eu], [en, nn, nu], [eu, nu, uu]]).transpose(2, 0, 1)
    known = {"t_s", "t_ns", "east_m", "north_m", "up_m", "north_ned_m", "east_ned_m", "down_ned_m",
             "qx", "qy", "qz", "qw", "v_east", "v_north", "v_up",
             "cov_ee", "cov_en", "cov_eu", "cov_nn", "cov_nu", "cov_uu"}
    extra = {}
    for k in (rows[0].keys() if rows else []):
        if k not in known:
            try:
                extra[k] = col(k)
            except ValueError:
                extra[k] = np.array([r[k] for r in rows])
    return Trajectory(t, p, q, v, cov, t0_ns=t0_ns, name=name or Path(path).stem,
                      has_orientation=not np.allclose(q, q[0]) if len(q) else False, extra=extra)


def read_any(path, name=None):
    path = Path(path)
    return read_csv(path, name) if path.suffix == ".csv" else read_tum(path, name)


def main():
    ap = argparse.ArgumentParser(description="Convert a trajectory between CSV / TUM / EuRoC.")
    ap.add_argument("trajectory", help="*.tum or *.csv written by these tools")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--name", default=None)
    args = ap.parse_args()
    traj = read_any(args.trajectory, args.name)
    print(json.dumps(traj.write_all(args.out_dir), indent=2))


if __name__ == "__main__":
    main()
