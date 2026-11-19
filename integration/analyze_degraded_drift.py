"""Offline measurement: drift of the REAL IMU+height-only estimator (Phase A).

Read-only analysis of preserved source evidence (stream.jsonl). No production
change, no ROS, no estimator re-implementation.

Method. In the source, a frame whose flow is not accepted skips only
ekf.update_velocity_xy; IMU prediction and update_height still run
(run_vio_stream.py: frame_step 473, update_height 505, update_velocity_xy 602-603).
A run of consecutive non-accepted frames that follows an accepted frame is
therefore a genuine IMU+height-only window of the estimator that would be
published in degraded mode. For each such window anchored at the last accepted
frame t0 we measure, at every later frame t0+tau inside the window:

  horizontal position drift  |(p_est - p_gt)(t0+tau) - (p_est - p_gt)(t0)|
  horizontal velocity drift  |(v_est - v_gt)(t0+tau) - (v_est - v_gt)(t0)|
  estimator covariance growth (P_xx+P_yy, P_vx+P_vy) relative to t0

GT (gt_ned) is LOGGING ONLY in the source and is used here only as the
reference. GT velocity is a central difference of logged GT positions.
The constant offset between the estimator origin and the GT origin cancels in
the anchored difference; a constant yaw misalignment does not, so it is fitted
on accepted frames and reported.

Also measured: the simulation/host time ratio between consecutive frames.
"""
import argparse
import json
import math
from pathlib import Path
import statistics

DEFAULT_RUNS = (
    'integration/phase1_runs/20261118-flightfbf7d8854ed1/evidence/flight-source',
    'integration/phase1_runs/20261118-flightabe2d67e5306/evidence/flight-source',
    'integration/debug_evidence/20261117-flight4414cb818f2a/flight-source',
    'integration/debug_evidence/20261117-flight6f32c08e7aaf/flight-source',
)
FRAME_S = .1


def load(directory):
    rows = [json.loads(line) for line in (Path(directory)/'stream.jsonl').read_text().splitlines() if line.strip()]
    return rows


def gt_velocity(estimated):
    """Central difference of logged GT positions (NED), None at edges/gaps."""
    by_t = {round(r['t'], 6): r for r in estimated}
    out = {}
    for r in estimated:
        a, b = by_t.get(round(r['t']-FRAME_S, 6)), by_t.get(round(r['t']+FRAME_S, 6))
        if a is None or b is None:
            continue
        out[round(r['t'], 6)] = [(b['gt_ned'][k]-a['gt_ned'][k])/(b['t']-a['t']) for k in range(3)]
    return out


def fit_yaw(estimated, gtv):
    """Least-squares yaw between estimator and GT horizontal velocity (accepted frames)."""
    s = c = 0.
    for r in estimated:
        g = gtv.get(round(r['t'], 6))
        if g is None or not r.get('flow_accepted'):
            continue
        ex, ey = r['state'][3], r['state'][4]
        c += ex*g[0]+ey*g[1]
        s += ex*g[1]-ey*g[0]
    return math.atan2(s, c)


def rotate(v, yaw):
    return [math.cos(yaw)*v[0]-math.sin(yaw)*v[1], math.sin(yaw)*v[0]+math.cos(yaw)*v[1]]


def windows(estimated):
    """(anchor, [rejected rows]) for each non-accepted run after an accepted frame."""
    out, anchor, current = [], None, []
    for r in estimated:
        if anchor is not None and (r.get('epoch') != anchor.get('epoch')
                                   or r.get('ekf_internal_resets') != anchor.get('ekf_internal_resets')):
            anchor, current = None, []   # new estimator epoch: never a continuous window
        if r.get('flow_accepted'):
            if anchor is not None and current:
                out.append((anchor, current))
            anchor, current = r, []
        elif anchor is not None:
            current.append(r)
    if anchor is not None and current:
        out.append((anchor, current))
    return out


def measure(directory):
    rows = load(directory)
    estimated = sorted((r for r in rows if 'state' in r and 'gt_ned' in r), key=lambda r: r['t'])
    gtv = gt_velocity(estimated)
    yaw = fit_yaw(estimated, gtv)
    samples = []
    for anchor, rejected in windows(estimated):
        def err(r):
            p = rotate(r['state'][:2], yaw)
            return [p[0]-r['gt_ned'][0], p[1]-r['gt_ned'][1]]
        def verr(r):
            g = gtv.get(round(r['t'], 6))
            if g is None:
                return None
            v = rotate(r['state'][3:5], yaw)
            return [v[0]-g[0], v[1]-g[1]]
        e0, v0 = err(anchor), verr(anchor)
        P0 = anchor['covariance']
        for r in rejected:
            e, v, P = err(r), verr(r), r['covariance']
            samples.append(dict(
                run=Path(directory).parts[-3] if 'phase1_runs' in str(directory) else Path(directory).parts[-2],
                anchor_t=anchor['t'], t=r['t'], tau=round(r['t']-anchor['t'], 6), window_frames=len(rejected),
                pos_drift_m=math.hypot(e[0]-e0[0], e[1]-e0[1]),
                vel_drift_m_s=None if v is None or v0 is None else math.hypot(v[0]-v0[0], v[1]-v0[1]),
                pos_var_xy_m2=P[0][0]+P[1][1], pos_var_xy_growth_m2=P[0][0]+P[1][1]-P0[0][0]-P0[1][1],
                vel_var_xy=P[3][3]+P[4][4], vel_var_xy_growth=P[3][3]+P[4][4]-P0[3][3]-P0[4][4],
                pos_var_max_diag_m2=max(P[i][i] for i in range(9)),
                reason=r.get('skip_reason') or (r.get('audit') or {}).get('reason')))
    ratios = [(b['t']-a['t'])/(b['wall']-a['wall']) for a, b in zip(rows, rows[1:])
              if b['wall'] > a['wall'] and abs(b['t']-a['t']-FRAME_S) < 1e-6]
    return dict(directory=str(directory), estimated_frames=len(estimated),
                accepted=sum(bool(r.get('flow_accepted')) for r in estimated),
                fitted_yaw_deg=math.degrees(yaw), samples=samples,
                sim_host_ratio=dict(n=len(ratios), median=statistics.median(ratios),
                                    p10=sorted(ratios)[len(ratios)//10], p01=sorted(ratios)[len(ratios)//100],
                                    min=min(ratios)) if ratios else None)


def summarize(results, bins=(.1, .2, .3, .4, .5, .6, .7, .8, .9, 1.)):
    samples = [s for r in results for s in r['samples']]
    table = []
    for b in bins:
        sel = [s for s in samples if abs(s['tau']-b) < 1e-6]
        if not sel:
            table.append(dict(tau_s=b, n=0))
            continue
        pos = sorted(s['pos_drift_m'] for s in sel)
        vel = sorted(s['vel_drift_m_s'] for s in sel if s['vel_drift_m_s'] is not None)
        table.append(dict(tau_s=b, n=len(sel), runs=sorted({s['run'] for s in sel}),
                          pos_drift_median_m=statistics.median(pos), pos_drift_max_m=pos[-1],
                          vel_drift_median_m_s=statistics.median(vel) if vel else None,
                          vel_drift_max_m_s=vel[-1] if vel else None,
                          est_pos_sigma_xy_max_m=math.sqrt(max(s['pos_var_xy_m2'] for s in sel)),
                          est_pos_var_growth_max_m2=max(s['pos_var_xy_growth_m2'] for s in sel),
                          est_vel_var_growth_max=max(s['vel_var_xy_growth'] for s in sel),
                          max_diag_m2=max(s['pos_var_max_diag_m2'] for s in sel)))
    return table


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs', nargs='*', default=list(DEFAULT_RUNS))
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    results = [measure(d) for d in a.runs]
    report = dict(runs=[{k: v for k, v in r.items() if k != 'samples'} for r in results],
                  table=summarize(results), samples=[s for r in results for s in r['samples']])
    text = json.dumps(report, indent=2)
    if a.output:
        a.output.write_text(text+'\n')
    print(json.dumps(dict(runs=report['runs'], table=report['table']), indent=2))


if __name__ == '__main__':
    main()
