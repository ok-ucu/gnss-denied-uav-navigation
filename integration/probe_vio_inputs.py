#!/usr/bin/env python3
"""Measure real FCU input streams; never substitutes mock data or proves camera sync."""
import argparse
import json
import math
from pathlib import Path
import time

RATES = {'DISTANCE_SENSOR': 10, 'RAW_IMU': 30, 'ATTITUDE': 30,
         'LOCAL_POSITION_NED': 30}
IDS = {'DISTANCE_SENSOR': 132, 'RAW_IMU': 27, 'ATTITUDE': 30,
       'LOCAL_POSITION_NED': 32}


def request_intervals(client, source, clock):
    """Serialize command 511: ACK contains command ID, not requested message ID."""
    results = []
    for kind, hz in [('DISTANCE_SENSOR', 20), ('RAW_IMU', 50),
                     ('ATTITUDE', 50), ('LOCAL_POSITION_NED', 50)]:
        client.mav.command_long_send(*source, 511, 0, IDS[kind], 1e6/hz, 0, 0, 0, 0, 0)
        entry = {'message': kind, 'message_id': IDS[kind], 'requested_hz': hz,
                 'ack_result': None}
        deadline = clock() + 1
        while clock() < deadline:
            msg = client.recv_match(blocking=True, timeout=.1)
            if msg is None or (msg.get_srcSystem(), msg.get_srcComponent()) != source:
                continue
            if (msg.get_type() == 'COMMAND_ACK' and msg.command == 511
                    and getattr(msg, 'target_system', 0) in (0, 245)
                    and getattr(msg, 'target_component', 0) in (0, 191)):
                entry['ack_result'] = msg.result
                break
        results.append(entry)
    return results


def summarize(samples, duration, heartbeat, errors=(), armed=False):
    """Arrival rates use the entire observation window, including missing data."""
    failures = list(errors)
    streams = {}
    for kind, threshold in RATES.items():
        times = samples[kind]
        rate = len(times) / duration
        edges = [0.0] + times + [duration]
        streams[kind] = {'count': len(times), 'hz': rate, 'minimum_hz': threshold,
                         'max_gap_s': max(b - a for a, b in zip(edges, edges[1:]))}
        if rate < threshold:
            failures.append(f'{kind}: {rate:.3f} Hz < {threshold}')
    if not heartbeat:
        failures.append('No autopilot heartbeat')
    return {'status': 'FAIL' if failures else 'PASS', 'duration_s': duration,
            'heartbeat': heartbeat, 'armed_observed': armed,
            'mavlink2': heartbeat and not any('MAVLink 2' in x for x in failures),
            'streams': streams, 'failures': sorted(set(failures)),
            'scope': 'MAVLink arrival streams only; no Gazebo/camera physical or time validation'}


def probe(client, duration=10, heartbeat_timeout=15, request_streams=False,
          clock=time.monotonic):
    if not all(math.isfinite(x) and x > 0 for x in (duration, heartbeat_timeout)):
        raise ValueError('Duration and heartbeat timeout must be finite and positive')
    samples = {kind: [] for kind in RATES}
    errors = []
    ranges = []
    heartbeat = False
    armed = False
    source = None
    deadline = clock() + heartbeat_timeout
    while clock() < deadline:
        msg = client.recv_match(blocking=True, timeout=min(.2, max(0, deadline-clock())))
        if msg and msg.get_type() == 'HEARTBEAT' and msg.autopilot != 8:
            source = (msg.get_srcSystem(), msg.get_srcComponent())
            heartbeat = True
            armed = bool(msg.base_mode & 128)
            if not msg.get_msgbuf() or msg.get_msgbuf()[0] != 0xFD:
                errors.append('Heartbeat is not MAVLink 2')
            break
    if not heartbeat:
        return summarize(samples, duration, False, errors)
    requests = request_intervals(client, source, clock) if request_streams else []
    for entry in requests:
        if entry['ack_result'] != 0:
            errors.append(f"{entry['message']}: interval ACK {entry['ack_result']} (0 required)")
    start = clock()
    try:
        while clock() - start < duration:
            msg = client.recv_match(blocking=True, timeout=min(.1, max(0, duration-(clock()-start))))
            elapsed = clock() - start
            if msg is None or elapsed > duration:
                continue
            if (msg.get_srcSystem(), msg.get_srcComponent()) != source:
                continue
            kind = msg.get_type()
            if kind == 'HEARTBEAT':
                armed |= bool(msg.base_mode & 128)
            if kind not in samples:
                continue
            samples[kind].append(elapsed)
            if not msg.get_msgbuf() or msg.get_msgbuf()[0] != 0xFD:
                errors.append(f'{kind}: packet is not MAVLink 2')
            if kind == 'DISTANCE_SENSOR':
                values = {'orientation': msg.orientation, 'current_cm': msg.current_distance,
                          'min_cm': msg.min_distance, 'max_cm': msg.max_distance}
                values['range_status'] = ('out_of_range_low' if msg.current_distance < msg.min_distance
                                          else 'out_of_range_high' if msg.current_distance > msg.max_distance
                                          else 'within_bounds')
                ranges.append(values)
                if msg.orientation != 25:
                    errors.append('DISTANCE_SENSOR: orientation != 25')
                if msg.min_distance != 20 or msg.max_distance != 5000:
                    errors.append('DISTANCE_SENSOR: expected min/max 20/5000 cm')
                if not (0 < msg.min_distance <= msg.current_distance <= msg.max_distance):
                    errors.append('DISTANCE_SENSOR: invalid range')
    except (OSError, EOFError) as exc:
        errors.append(f'Transport error: {exc}')
    result = summarize(samples, duration, heartbeat, errors, armed)
    result['source'] = source
    result['distance_samples_cm'] = ranges
    result['requested_intervals'] = request_streams
    result['interval_requests'] = requests
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conn', default='udpin:0.0.0.0:14553')
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--heartbeat-timeout', type=float, default=15)
    parser.add_argument('--request-streams', action='store_true')
    parser.add_argument('--output', type=Path, help='JSON report; parent must exist')
    args = parser.parse_args(argv)
    client = None
    try:
        from pymavlink import mavutil
        from pymavlink.dialects.v20 import ardupilotmega
        client = mavutil.mavlink_connection(args.conn, source_system=245)
        # Explicit V2 encoder even when pymavlink was imported without MAVLINK20.
        client.mav = ardupilotmega.MAVLink(client, srcSystem=245, srcComponent=191)
        result = probe(client, args.duration, args.heartbeat_timeout, args.request_streams)
    except (ImportError, OSError, EOFError, ValueError, RuntimeError) as exc:
        result = {'status': 'FAIL', 'failures': [str(exc)]}
    finally:
        if client is not None:
            client.close()
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + '\n')
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
