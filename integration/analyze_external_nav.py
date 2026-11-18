"""Offline ExternalNav wire validation. No FCU commands or EKF acceptance claims."""
import argparse
import json
import math
from pathlib import Path
import struct

from vio_adapter import expand_upper_triangle, validate_covariance

TYPES = ('VISION_POSITION_ESTIMATE', 'VISION_SPEED_ESTIMATE')


def read_capture(path, raw=False):
    """Read standard timestamped tlog or raw concatenated MAVLink packets.

    Tlog timestamps describe the capture point, not necessarily FCU reception.
    Strict framing/CRC errors abort rather than silently hiding capture damage.
    """
    from pymavlink.dialects.v20 import ardupilotmega
    decoder = ardupilotmega.MAVLink(None)
    with open(path, 'rb') as stream:
        while True:
            first = stream.read(1)
            if not first:
                return
            arrival = None
            if not raw:
                stamp = first + stream.read(7)
                if len(stamp) != 8:
                    raise ValueError('Truncated tlog timestamp')
                arrival = (struct.unpack('>Q', stamp)[0] & ~3) / 1e6
                first = stream.read(1)
            if first not in (b'\xfd', b'\xfe'):
                raise ValueError('Expected MAVLink 1/2 packet marker')
            version = 2 if first == b'\xfd' else 1
            header = stream.read(9 if version == 2 else 5)
            if len(header) != (9 if version == 2 else 5):
                raise ValueError('Truncated MAVLink header')
            size = header[0] + 2 + (13 if version == 2 and header[1] & 1 else 0)
            tail = stream.read(size)
            if len(tail) != size:
                raise ValueError('Truncated MAVLink payload')
            try:
                message = decoder.parse_char(first + header + tail)
            except Exception as exc:
                raise ValueError(f'Invalid MAVLink packet: {exc}') from exc
            if message is None or message.get_type() == 'BAD_DATA':
                raise ValueError('Undecodable MAVLink packet')
            yield message, arrival, version


def validate_measurement(message):
    kind = message.get_type()
    fields = ('x', 'y', 'z', 'roll', 'pitch', 'yaw') if kind == TYPES[0] else ('x', 'y', 'z')
    if not all(math.isfinite(getattr(message, field)) for field in fields):
        raise ValueError('Non-finite measurement')
    cov = message.covariance
    if kind == TYPES[0]:
        validate_covariance(expand_upper_triangle(cov))
    else:
        validate_covariance(cov, 3)


def analyze(records, min_rate=20., max_gap=.2, system=None, component=None):
    if not all(math.isfinite(x) and x > 0 for x in (min_rate, max_gap)):
        raise ValueError('Rate and gap thresholds must be finite and positive')
    streams = {kind: [] for kind in TYPES}
    bounds = []
    for message, arrival, version in records:
        if arrival is not None:
            if not math.isfinite(arrival):
                raise ValueError('Non-finite capture timestamp')
            bounds.append(arrival)
        if system is not None and message.get_srcSystem() != system:
            continue
        if component is not None and message.get_srcComponent() != component:
            continue
        if message.get_type() in streams:
            streams[message.get_type()].append((message, arrival, version))
    if any(b < a for a, b in zip(bounds, bounds[1:])):
        raise ValueError('Capture clock reset/out-of-order arrival timestamps')
    report = {'criteria': {'min_rate_hz': min_rate, 'max_gap_s': max_gap},
              'time_basis': 'capture observation timestamps; sender usec is a separate clock',
              'capture_to_ekf_latency': 'NOT TESTED', 'ekf_acceptance': 'NOT TESTED',
              'streams': {}}
    for kind, rows in streams.items():
        errors, stamps, counters = [], [], []
        sources = sorted({(m.get_srcSystem(), m.get_srcComponent()) for m, _, _ in rows})
        if not rows:
            errors.append('Required stream missing')
        if len(sources) > 1:
            errors.append('Multiple senders: select --system/--component before comparing timing')
        for index, (message, _, version) in enumerate(rows):
            if version != 2:
                errors.append(f'Message {index}: MAVLink 2 required')
            try:
                validate_measurement(message)
            except (ValueError, AttributeError, TypeError) as exc:
                errors.append(f'Message {index}: {exc}')
            stamps.append(message.usec)
            counters.append(getattr(message, 'reset_counter', None))
        resets = [i for i in range(1, len(stamps)) if stamps[i] < stamps[i-1]]
        duplicates = [i for i in range(1, len(stamps)) if stamps[i] == stamps[i-1]]
        if resets or duplicates:
            errors.append('Sender timestamps are not strictly increasing (reset/duplicate)')
        arrivals = [arrival for _, arrival, _ in rows if arrival is not None]
        timed = len(arrivals) == len(rows) and len(arrivals) >= 2 and arrivals[-1] > arrivals[0]
        gaps, dropouts, rate = [], [], None
        if timed:
            gaps = [b-a for a, b in zip(arrivals, arrivals[1:])]
            rate = (len(arrivals)-1)/(arrivals[-1]-arrivals[0])
            edges = [(bounds[0], arrivals[0], 'leading'),
                     *[(a, b, 'internal') for a, b in zip(arrivals, arrivals[1:])],
                     (arrivals[-1], bounds[-1], 'trailing')]
            dropouts = [{'start_capture_s': a, 'end_capture_s': b, 'gap_s': b-a,
                         'location': location, 'recovered': location != 'trailing',
                         'first_return_capture_s': b if location != 'trailing' else None}
                        for a, b, location in edges if b-a > max_gap + 1e-9]
            if rate + 1e-9 < min_rate:
                errors.append('Observed rate below threshold')
            if dropouts:
                errors.append('Observed absence exceeds maximum gap')
        report['streams'][kind] = {
            'status': 'FAIL' if errors else ('PASS' if timed else 'NOT TESTED'),
            'count': len(rows), 'sources': sources, 'errors': errors,
            'timing_status': 'PASS' if timed and not dropouts and rate + 1e-9 >= min_rate else
                             ('FAIL' if timed else 'NOT TESTED'),
            'observed_rate_hz': rate, 'max_interarrival_gap_s': max(gaps) if gaps else None,
            'interarrival_gaps_s': gaps, 'capture_timestamps_s': arrivals,
            'sender_timestamps_usec': stamps, 'sender_clock_domain': 'unknown: boot or epoch',
            'sender_reset_indices': resets, 'sender_duplicate_indices': duplicates,
            'reset_counters': counters, 'dropouts': dropouts,
            'recovery_status': ('PASS' if any(d['recovered'] for d in dropouts) else
                                'NOT TESTED'),
            'recovery_meaning': 'Observed packet return only; not stable VIO or EKF recovery'}
    statuses = [s['status'] for s in report['streams'].values()]
    report['status'] = 'FAIL' if 'FAIL' in statuses else ('NOT TESTED' if 'NOT TESTED' in statuses else 'PASS')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--raw', action='store_true', help='Raw packets without capture timestamps')
    parser.add_argument('--min-rate', type=float, default=20.)
    parser.add_argument('--max-gap', type=float, default=.2)
    parser.add_argument('--system', type=int)
    parser.add_argument('--component', type=int)
    args = parser.parse_args(argv)
    try:
        report = analyze(read_capture(args.capture, args.raw), args.min_rate, args.max_gap,
                         args.system, args.component)
    except (OSError, ValueError, ImportError) as exc:
        print(json.dumps({'status': 'FAIL', 'error': str(exc)}))
        return 1
    print(json.dumps(report, indent=2, allow_nan=False))
    return {'PASS': 0, 'FAIL': 1, 'NOT TESTED': 2}[report['status']]


if __name__ == '__main__':
    raise SystemExit(main())
