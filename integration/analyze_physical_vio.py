"""Offline Phase 2 adapters; reuses Person 4 trajectory/clock/BIN analyzers.

No port/ROS connections, hidden trajectory fitting or inferred clock proof.
Reports descriptive errors, not hardware readiness or EKF acceptance counts.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from analyze_trajectories import compare, gps_to_enu, timestamp
from analyze_ekf3_fusion import statistics_of
from navigation_clock import ClockMapping
from vio_frame_bridge import quat_matrix


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Finite numeric value required')
    return value


def ns(value):
    if type(value) is not int or value < 0:
        raise ValueError('Raw integer nanoseconds required')
    return value


def json_rows(path):
    rows = []
    with Path(path).open() as stream:
        for line in stream:
            if not line.strip():
                raise ValueError('Empty/partial JSONL record')
            row = json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            if not isinstance(row, dict):
                raise ValueError('JSONL object required')
            rows.append(row)
    return rows


def raw_time(value, unit):
    if unit not in ('ns', 'us', 'ms', 's'):
        raise ValueError('Explicit supported timestamp units required')
    return timestamp(value, unit)


def trajectory(spec, base, decoder=None):
    """Raw recording order is checked BEFORE any mapping, no sorting/dedup."""
    path = base/spec['path']
    kind = spec['format']
    if spec.get('position_unit') != 'm' or not spec.get('session') or not spec.get('clock_domain'):
        raise ValueError('Explicit meter units, session and clock domain required')
    extras = {}
    if kind == 'pi-jsonl':
        if spec['time_unit'] != 'ns' or spec['clock_domain'] != 'ros-system-unix-ns':
            raise ValueError('Pi logs require raw acquisition ns in ros-system-unix-ns')
        records = json_rows(path)
        if any(r.get('session') != spec['session'] for r in records):
            raise ValueError('Mixed/unbound Pi session')
        frames = [r for r in records if r.get('event') == 'frame']
        if any(type(r.get('published')) is not bool or type(r.get('ready')) is not bool for r in frames):
            raise ValueError('Explicit ready/published booleans required')
        all_stamps = [ns(r['stamp_ns']) for r in frames]
        if any(b <= a for a, b in zip(all_stamps, all_stamps[1:])):
            raise ValueError('Pi raw timestamp reset/duplicate')
        rows = [(ns(r['stamp_ns']), r['p']) for r in frames if r.get('published') is True and r.get('ready') is True]
        if any(r.get('published') is True and r.get('ready') is not True for r in frames):
            raise ValueError('Published unready candidate')
        extras = dict(frames=len(frames), published=len(rows), recovery_events=[r for r in records if r.get('event') == 'recovery'])
        axes = 'ned'
    elif kind == 'dataflash':
        if spec['time_unit'] != 'us' or spec['clock_domain'] != 'FCU_BOOT_TIME_US' or type(spec.get('core')) is not int:
            raise ValueError('DataFlash requires FCU boot us and explicit EKF core')
        if decoder is None:
            from analyze_simulated_flight import decode_bin
            decoder = decode_bin
        records = decoder(path)
        # Validate each logged stream: interleaved cores/types are not sorted.
        previous = {}
        for r in records:
            if 'TimeUS' not in r:
                continue
            key = (r['mavpackettype'], r.get('C'))
            t = ns(r['TimeUS'])
            if key in previous and t < previous[key]:
                raise ValueError('DataFlash boot timestamp reset')
            previous[key] = t
        selected = [r for r in records if r['mavpackettype'] == 'XKF1' and r.get('C') == spec['core']]
        rows = [(ns(r['TimeUS'])*1000, [r[k] for k in ('PN', 'PE', 'PD')]) for r in selected]
        innovations = [r for r in records if r['mavpackettype'] == 'XKF3' and r.get('C') == spec['core']]
        extras = dict(aiding_messages=[r for r in records if r['mavpackettype'] == 'MSG' and
                       any(x in r['Message'].lower() for x in ('aiding', 'external nav'))],
            innovations={k: statistics_of([number(r[k]) for r in innovations if k in r]) for k in ('IVN', 'IVE', 'IVD', 'IPN', 'IPE', 'IPD')},
            estimator_status=[r for r in records if r['mavpackettype'] in ('XKF4', 'XKFS') and r.get('C') == spec['core']],
            reception_counts={k: sum(r['mavpackettype'] == k for r in records) for k in ('VISP', 'VISV')},
            fusion='NOT VERIFIED: receipt/status/innovations require source-selection review, not packet-count inference')
        axes = 'ned'
    elif kind in ('enu-csv', 'ned-csv', 'gps-csv'):
        with path.open(newline='') as stream:
            raw = list(csv.DictReader(stream))
        axes = kind.split('-')[0]
        keys = {'enu': ('east', 'north', 'up'), 'ned': ('x', 'y', 'z'), 'gps': ('lat', 'lon', 'alt')}[axes]
        rows = [(raw_time(r[spec['time_column']], spec['time_unit']), [float(r[k]) for k in keys]) for r in raw]
        if axes == 'gps':
            if spec.get('altitude_datum') != 'WGS84_ellipsoid_m':
                raise ValueError('GPS needs explicit ellipsoid altitude; no MSL substitution')
            origin = spec['geodetic_origin']
            if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
                raise ValueError('Finite common geodetic origin required')
            rows = [(t, gps_to_enu(*p, origin)) for t, p in rows]
            extras['reference_accuracy'] = 'NOT VERIFIED: GPS fix is not ground truth'
            axes = 'enu'
    else:
        raise ValueError('Unknown trajectory format')
    if len(rows) < 2:
        raise ValueError('At least two admitted samples required')
    out = []
    for t, p in rows:
        ns(t)
        if out and t <= out[-1][0]:
            raise ValueError('Raw timestamp reset/duplicate/order')
        if len(p) != 3:
            raise ValueError('XYZ position required')
        p = tuple(number(v) for v in p)
        out.append((t, (p[1], p[0], -p[2]) if axes == 'ned' else p))
    extras['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return out, extras


def mapping(spec, base):
    """Existing affine mapper; offline replay at last anchor, not live freshness.

    External proof is supplied/reviewed by the operator, never inferred from fit.
    A hash binds the proof artifact but does not authenticate its physical claim.
    """
    proof = spec['mapping']
    if proof.get('reviewed') is not True or not proof.get('reviewer'):
        raise ValueError('Clock proof requires explicit external review')
    evidence = base/proof['evidence_path']
    if hashlib.sha256(evidence.read_bytes()).hexdigest() != proof['evidence_sha256']:
        raise ValueError('Clock proof hash mismatch')
    if proof['source_clock'] != spec['clock_domain'] or proof['source_epoch'] != spec['session']:
        raise ValueError('Clock proof source domain/session mismatch')
    model = ClockMapping(proof['anchors'], source_epoch=spec['session'], verified=True,
        uncertainty_ns=proof['uncertainty_ns'], max_uncertainty_ns=proof['max_uncertainty_ns'],
        max_age_ns=proof['max_anchor_age_ns'], extrapolation_ns=0)
    if not model.verified:
        raise ValueError('Unverified clock uncertainty')
    return model, (proof['target_clock'], proof['target_epoch'])


def pair_indices(metrics, delta_samples, max_pair_gap_s):
    if type(delta_samples) is not int or delta_samples < 1:
        raise ValueError('Explicit positive RPE sample interval required')
    if not math.isfinite(max_pair_gap_s) or max_pair_gap_s <= 0:
        raise ValueError('Explicit bounded RPE interval required')
    times = metrics['matched_times_ns']
    return [(i-delta_samples, i) for i in range(delta_samples, len(times))
            if (times[i]-times[i-delta_samples])/1e9 <= max_pair_gap_s]


def world_displacement_error(metrics, delta_samples, max_pair_gap_s):
    """World-frame displacement error; explicitly NOT standard SE(3) RPE."""
    errors = metrics['error_vectors_m']
    values = [math.dist(errors[i], errors[j]) for i, j in pair_indices(metrics, delta_samples, max_pair_gap_s)]
    return dict(metric='world-frame displacement error (not SE3 RPE)', delta_samples=delta_samples,
                pairs=len(values), rmse_m=math.sqrt(sum(v*v for v in values)/len(values)) if values else None)


def orientation(q):
    if len(q) != 4 or any(isinstance(v, bool) or not math.isfinite(v) for v in q):
        raise ValueError('Finite xyzw orientation required')
    if abs(sum(v*v for v in q)-1) > 1e-9:
        raise ValueError('Verified orientation quaternion must be normalized')
    norm = math.sqrt(sum(v*v for v in q))
    return quat_matrix([v/norm for v in q])


def se3_translational_rpe(metrics, reference_xyzw, candidate_xyzw, delta_samples,
                          max_pair_gap_s, *, verified=False):
    """||trans((Q_i^-1 Q_j)^-1 (P_i^-1 P_j))||, no yaw/scale fitting."""
    result = dict(metric='standard SE3 translational RPE', status='NOT VERIFIED',
                  pairs=0, rmse_m=None, delta_samples=delta_samples)
    if verified is not True:
        result['reason'] = 'No externally verified, aligned orientations'
        return result
    count = len(metrics['matched_times_ns'])
    if len(reference_xyzw) != count or len(candidate_xyzw) != count:
        raise ValueError('RPE orientation/sample count mismatch')
    rq, rp = [orientation(q) for q in reference_xyzw], [orientation(q) for q in candidate_xyzw]
    q, p = metrics['reference_positions_m'], metrics['candidate_positions_m']
    values = []
    for i, j in pair_indices(metrics, delta_samples, max_pair_gap_s):
        # Relative translations expressed in their respective starting bodies.
        tq = [sum(rq[i][b][a]*(q[j][b]-q[i][b]) for b in range(3)) for a in range(3)]
        tp = [sum(rp[i][b][a]*(p[j][b]-p[i][b]) for b in range(3)) for a in range(3)]
        # The final Q_rel^-1 rotation preserves this vector's Euclidean norm.
        values.append(math.dist(tp, tq))
    result.update(status='PASS' if values else 'NOT VERIFIED', pairs=len(values),
                  rmse_m=math.sqrt(sum(v*v for v in values)/len(values)) if values else None,
                  scope='Metric calculation only; not hardware readiness')
    return result


def verified_attitudes(config, metrics, details, target_identity, base):
    """Optional exact-time sidecar. Never synthesize/interpolate reference yaw."""
    proof = config.get('orientation_pairs')
    if proof is None:
        return None
    if proof.get('verified') is not True or not proof.get('reviewer') or not proof.get('body_frame'):
        raise ValueError('Orientation provenance not verified')
    if proof.get('convention') != 'body-to-world-ENU-xyzw':
        raise ValueError('Explicit common-body ENU orientation convention required')
    for name in ('reference', 'candidate'):
        if proof[name+'_sha256'] != details[name]['sha256'] or proof[name+'_session'] != config[name]['session']:
            raise ValueError('Orientation proof not bound to trajectory/session')
    if ((proof['target_clock'], proof['target_epoch']) != target_identity or
            proof['reference_id'] != config['reference']['reference_id']):
        raise ValueError('Orientation clock/reference mismatch')
    path = base/proof['path']
    if hashlib.sha256(path.read_bytes()).hexdigest() != proof['sha256']:
        raise ValueError('Orientation artifact hash mismatch')
    rows = json_rows(path)
    times = [ns(r['time_ns']) for r in rows]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Orientation timestamp reset/duplicate')
    for row in rows:
        orientation(row['reference_xyzw']); orientation(row['candidate_xyzw'])
    by_time = dict(zip(times, rows))
    if any(t not in by_time for t in metrics['matched_times_ns']):
        raise ValueError('Missing exact-time orientation; no implicit attitude interpolation')
    aligned = [by_time[t] for t in metrics['matched_times_ns']]
    return ([r['reference_xyzw'] for r in aligned], [r['candidate_xyzw'] for r in aligned])


def latency(rows, clock_proof=None):
    """Measured publish->forward completion only. No FCU receive-time inference."""
    events = {}
    for r in rows:
        if r.get('event') not in ('publish', 'forward'):
            continue
        if r.get('clock_domain') != 'HOST_MONOTONIC_NS' or not r.get('host_boot_id') or not r.get('session'):
            raise ValueError('Latency needs explicit same-host monotonic domain/boot/session')
        ns(r['acquisition_ns']); ns(r['host_ns']); ns(r['epoch'])
        key = (r['session'], r['epoch'], r['acquisition_ns'])
        channel = events.setdefault(key, {})
        if r['event'] in channel:
            raise ValueError('Duplicate latency boundary')
        channel[r['event']] = r
    durations = []; unverified = 0
    for channel in events.values():
        if set(channel) != {'publish', 'forward'}:
            continue
        a, b = channel['publish'], channel['forward']
        if a['host_boot_id'] != b['host_boot_id'] or b['host_ns'] < a['host_ns']:
            raise ValueError('Unaligned/reset/reversed latency clocks')
        if a.get('time_namespace') != b.get('time_namespace'):
            raise ValueError('Mismatched time namespaces; normalize with verified evidence first')
        if a.get('clock_instance') != b.get('clock_instance'):
            raise ValueError('Mismatched monotonic clock instances')
        if b.get('boundary') != 'os_write_complete':
            raise ValueError('Forward boundary must name observed OS write completion')
        if clock_proof is None:
            unverified += 1
            continue
        if (clock_proof.get('verified') is not True or not clock_proof.get('reviewer') or
                clock_proof.get('kind') != 'shared-monotonic-instance'):
            raise ValueError('Unverified shared monotonic clock proof')
        for key in ('host_boot_id', 'time_namespace', 'clock_instance'):
            if not clock_proof.get(key) or a.get(key) != clock_proof[key]:
                raise ValueError('Clock proof does not match recorded '+key)
        lo, hi = ns(clock_proof['valid_from_ns']), ns(clock_proof['valid_to_ns'])
        if not lo <= a['host_ns'] <= b['host_ns'] <= hi:
            raise ValueError('Latency outside verified clock interval')
        durations.append((b['host_ns']-a['host_ns'])/1e6)
    return dict(status='PASS' if durations else 'NOT VERIFIED',
                publish_to_forward_ms=statistics_of(durations), pairs=len(durations),
                unverified_pairs=unverified,
                unpaired_samples=sum(set(c) != {'publish', 'forward'} for c in events.values()),
                fcu_reception_latency='NOT VERIFIED: needs paired FCU receipt and same-boot timing bounds',
                wire_latency='NOT VERIFIED: OS write completion is not wire arrival')


def analyze(config, base, decoder=None):
    result = dict(status='NOT VERIFIED', decision='HOLD', execution_allowed=False, forwarding_enabled=False,
                  physical_integration='NOT VERIFIED', checks={}, streams={})
    try:
        reference, detail = trajectory(config['reference'], base, decoder)
        candidate, more = trajectory(config['candidate'], base, decoder)
        result['streams'] = dict(reference=detail, candidate=more)
        result['checks']['structure'] = 'PASS'
        if not config['reference'].get('reference_id') or config['reference']['reference_id'] != config['candidate'].get('reference_id'):
            raise ValueError('Shared origin/yaw/reference not explicitly established; no fitted alignment')
        a, identity = mapping(config['reference'], base)
        b, other = mapping(config['candidate'], base)
        if identity != other or not all(identity):
            raise ValueError('Different/unidentified mapped clock domains/epochs')
        mapped = []
        for rows, spec, model in ((reference, config['reference'], a), (candidate, config['candidate'], b)):
            stream = [(model.map(t, model.last['host_ns'], spec['session'])['fcu_ns'], p) for t, p in rows]
            if any(y[0] <= x[0] for x, y in zip(stream, stream[1:])):
                raise ValueError('Mapping collapsed/reversed timestamp order')
            mapped.append(stream)
        reference, candidate = mapped
        metrics = compare(reference, candidate, synchronized=True,
                          max_gap=number(config['max_interpolation_gap_s']), time_unit='ns')
        delta, gap = config['rpe_delta_samples'], number(config['max_rpe_interval_s'])
        attitudes = verified_attitudes(config, metrics, result['streams'], identity, base)
        result.update(metrics=metrics, world_displacement_error=world_displacement_error(metrics, delta, gap),
                      rpe=se3_translational_rpe(metrics, *(attitudes or ([], [])), delta, gap,
                                               verified=attitudes is not None))
        gaps = [(y[0]-x[0])/1e9 for x, y in zip(candidate, candidate[1:])]
        limit = ns(config['source_availability_limit_ns'])/1e9
        if limit <= 0:
            raise ValueError('Explicit source availability criterion required')
        origin = metrics['time_origin_ns']
        result['continuity'] = dict(gaps_s=statistics_of(gaps),
            dropout_intervals_s=[((x[0]-origin)/1e9, (y[0]-origin)/1e9) for x, y in zip(candidate, candidate[1:]) if (y[0]-x[0])/1e9 > limit],
            scope='Between recorded publications only; leading/trailing loss needs independent observation bounds')
        result['checks']['observed_continuity'] = 'FAIL' if result['continuity']['dropout_intervals_s'] else 'PASS: interior intervals only'
        result['checks'].update(clock_mapping='PASS: externally reviewed artifact + mathematical replay only',
                                trajectory_metrics='PASS', reference_accuracy=detail.get('reference_accuracy', 'NOT VERIFIED: independent accuracy proof required'))
        if config.get('latency_jsonl'):
            path = base/config['latency_jsonl']; clock_proof = config.get('latency_clock_proof')
            if clock_proof is not None:
                proof = base/clock_proof['evidence_path']
                if (hashlib.sha256(proof.read_bytes()).hexdigest() != clock_proof['evidence_sha256'] or
                        hashlib.sha256(path.read_bytes()).hexdigest() != clock_proof['trace_sha256']):
                    raise ValueError('Latency clock evidence/trace hash mismatch')
            result['latency'] = latency(json_rows(path), clock_proof)
        else:
            result['latency'] = dict(status='NOT VERIFIED', reason='No paired publication/forward records')
        result['analysis_status'] = 'PASS'
        result['clock_scope'] = 'Offline calibrated interval; no live clock freshness/FCU timing proof inferred'
    except (ValueError, KeyError, TypeError, OSError, ImportError) as exc:
        result.update(analysis_status='FAIL', reason=str(exc))
    return result


def write_outputs(result, output, make_plots=False):
    output.mkdir(exist_ok=False)
    if make_plots and result.get('analysis_status') == 'PASS':
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            m = result['metrics']; ref, cand = m['reference_positions_m'], m['candidate_positions_m']
            fig, ax = plt.subplots()
            for name, rows in (('reference', ref), ('VIO', cand)):
                ax.plot([p[0] for p in rows], [p[1] for p in rows], label=name)
            ax.set(xlabel='East [m]', ylabel='North [m]'); ax.axis('equal'); ax.legend()
            fig.savefig(output/'xy.png'); plt.close(fig)
            fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
            for name, rows in (('reference', ref), ('VIO', cand)):
                ax.plot(*zip(*rows), label=name)
            ax.set(xlabel='East [m]', ylabel='North [m]', zlabel='Up [m]'); ax.legend()
            fig.savefig(output/'xyz.png'); plt.close(fig)
            result['plots'] = 'PASS: supplied recorded data only'
        except ImportError:
            result['plots'] = 'NOT TESTED: install matplotlib in the analysis environment'
    with (output/'samples.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['relative_seconds', 'error_east_m', 'error_north_m', 'error_up_m'])
        m = result.get('metrics', {})
        for t, e in zip(m.get('matched_times_s', []), m.get('error_vectors_m', [])):
            writer.writerow([t]+list(e))
    (output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--plots', action='store_true')
    a = p.parse_args()
    result = analyze(json.loads(a.config.read_text()), a.config.resolve().parent)
    write_outputs(result, a.output, a.plots)
    print(json.dumps(result, allow_nan=False))
    return 0 if result.get('analysis_status') == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
