"""Compare explicitly synchronized ENU trajectories; standard library only."""
import argparse
import bisect
import csv
from datetime import datetime
import json
import math
import sys
import warnings


def timestamp(value, unit):
    if unit == 'iso':
        dt = datetime.fromisoformat(value.replace('Z', '+00:00'))
        if dt.tzinfo is None:
            raise ValueError('ISO timestamps require timezone')
        return dt.timestamp()
    result = float(value) / {'s': 1, 'ms': 1e3, 'us': 1e6, 'ns': 1e9}[unit]
    if not math.isfinite(result):
        raise ValueError('Non-finite timestamp')
    return result


def ecef(lat, lon, alt):
    lat, lon = math.radians(lat), math.radians(lon)
    a, e2 = 6378137.0, 6.69437999014e-3
    n = a / math.sqrt(1-e2*math.sin(lat)**2)
    return ((n+alt)*math.cos(lat)*math.cos(lon),
            (n+alt)*math.cos(lat)*math.sin(lon),
            (n*(1-e2)+alt)*math.sin(lat))


def gps_to_enu(lat, lon, alt, origin):
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError('GPS requires degrees, not scaled MAVLink integers')
    delta = [a-b for a, b in zip(ecef(lat, lon, alt), ecef(*origin))]
    phi, lam = map(math.radians, origin[:2])
    x, y, z = delta
    return (-math.sin(lam)*x + math.cos(lam)*y,
            -math.sin(phi)*math.cos(lam)*x - math.sin(phi)*math.sin(lam)*y + math.cos(phi)*z,
            math.cos(phi)*math.cos(lam)*x + math.cos(phi)*math.sin(lam)*y + math.sin(phi)*z)


def load(path, fmt='enu', time_column='timestamp', unit='s', origin=None, offset=0):
    rows = []
    with open(path, newline='') as stream:
        reader = csv.DictReader(stream)
        if fmt == 'enu':
            columns = ('east', 'north', 'up')
            if not all(k in (reader.fieldnames or []) for k in columns):
                columns = ('east_m', 'north_m', 'up_m')
            if not all(k in (reader.fieldnames or []) for k in columns):
                raise ValueError('ENU requires east,north,up or east_m,north_m,up_m')
        for row in reader:
            t = timestamp(row[time_column], unit)+offset
            if fmt == 'enu':
                pos = tuple(float(row[k]) for k in columns)
            elif fmt == 'ned':
                pos = (float(row['y']), float(row['x']), -float(row['z']))
            else:
                if origin is None:
                    raise ValueError('GPS requires shared --origin LAT LON ALT')
                pos = gps_to_enu(*(float(row[k]) for k in ('lat', 'lon', 'alt')), origin)
            if not all(math.isfinite(v) for v in pos):
                raise ValueError('Non-finite position')
            if rows and t <= rows[-1][0]:
                raise ValueError('Timestamps must strictly increase; no silent sorting/reset')
            rows.append((t, pos))
    if len(rows) < 2:
        raise ValueError('At least two samples required per trajectory')
    return rows


def compare(truth, vio, synchronized=False, max_gap=0.2):
    if not synchronized:
        warnings.warn('Clock synchronization unconfirmed: refusing trajectory comparison')
        raise ValueError('Confirm a shared clock or measured time offset with --synchronized')
    if max_gap <= 0:
        raise ValueError('max-gap must be positive')
    times = [row[0] for row in truth]
    errors, matched = [], []
    for t, position in vio:
        i = bisect.bisect_left(times, t)
        if i < len(times) and times[i] == t:
            reference = truth[i][1]
        elif i == 0 or i == len(times) or times[i]-times[i-1] > max_gap:
            continue
        else:
            alpha = (t-times[i-1])/(times[i]-times[i-1])
            reference = tuple(a+alpha*(b-a) for a, b in zip(truth[i-1][1], truth[i][1]))
        errors.append(tuple(a-b for a, b in zip(position, reference)))
        matched.append((t, reference))
    if len(errors) < 2:
        raise ValueError('Insufficient synchronized overlap within max-gap')
    skipped = len(vio)-len(errors)
    if skipped:
        warnings.warn(f'{skipped}/{len(vio)} VIO samples excluded (overlap/gaps)')
    horizontal = [math.hypot(*e[:2]) for e in errors]
    spatial = [math.sqrt(sum(v*v for v in e)) for e in errors]
    drift = tuple(b-a for a, b in zip(errors[0], errors[-1]))
    drift_h = math.hypot(*drift[:2])
    drift_3d = math.sqrt(sum(v*v for v in drift))
    duration = matched[-1][0]-matched[0][0]
    distance = sum(math.dist(a[1], b[1]) for a, b in zip(matched, matched[1:]))
    return dict(samples=len(errors), excluded=skipped, duration_s=duration,
                horizontal_error_m=horizontal, spatial_error_m=spatial,
                horizontal_rmse_m=math.sqrt(sum(e*e for e in horizontal)/len(errors)),
                spatial_rmse_m=math.sqrt(sum(e*e for e in spatial)/len(errors)),
                horizontal_drift_m=drift_h, spatial_drift_m=drift_3d,
                spatial_drift_m_per_s=drift_3d/duration,
                truth_path_m=distance,
                spatial_drift_percent=100*drift_3d/distance if distance else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('truth')
    parser.add_argument('vio')
    for prefix in ('truth', 'vio'):
        parser.add_argument(f'--{prefix}-format', choices=['enu', 'ned', 'gps'], default='enu')
        parser.add_argument(f'--{prefix}-time-column', default='timestamp')
        parser.add_argument(f'--{prefix}-time-unit', choices=['s', 'ms', 'us', 'ns', 'iso'], default='s')
    parser.add_argument('--origin', nargs=3, type=float, metavar=('LAT', 'LON', 'ALT'))
    parser.add_argument('--vio-time-offset', type=float, default=0,
                        help='Measured offset in seconds, added to VIO time')
    parser.add_argument('--synchronized', action='store_true')
    parser.add_argument('--max-gap', type=float, default=0.2)
    args = parser.parse_args()
    try:
        trajectories = []
        for prefix in ('truth', 'vio'):
            trajectories.append(load(getattr(args, prefix),
                getattr(args, prefix+'_format'), getattr(args, prefix+'_time_column'),
                getattr(args, prefix+'_time_unit'), args.origin,
                args.vio_time_offset if prefix == 'vio' else 0))
        print(json.dumps(compare(*trajectories, args.synchronized, args.max_gap), indent=2,
                         allow_nan=False))
        return 0
    except (ValueError, KeyError, OSError) as exc:
        print(f'Analysis failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
