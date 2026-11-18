"""Odometry continuity through every flight phase, from transport evidence.

Reads the transport's own logs (odom.jsonl, optional flight.jsonl) exactly as
phase1_transport.py writes them, so it runs on a live run directory
(<run>/evidence/flight-transport) and on preserved debug evidence alike.

Maximum allowed gap, derived rather than chosen:
  * The source publishes one odometry sample per camera frame, stamped with the
    frame's acquisition time, so the expected publish period T is the camera
    period (10 Hz in the Gazebo flight: 188 of 194 recorded intervals are
    exactly 100 ms).
  * The source reads the camera through a latest-only slot
    (vio_sensor_capture.RosFrameSource), so one frame may be superseded when a
    single frame's processing jitters past T. That is not loss of the estimate.
  * Two consecutive missing frames mean the source is not sustaining the
    sensor rate. Hence MAX GAP = 2 T (200 ms at 10 Hz), the same bound the
    flight analyzer applies to mission publications.
  * Cross-check: 2 T of simulation time is at most 2 T of wall time at
    real-time factor <= 1, inside the transport's 500 ms availability watchdog.

Domains are never mixed: gaps are measured between acquisition stamps
(simulation ns). Host time is used only to say which flight phase a sample
arrived in, using the transport's own callback_entry_host_ns.
"""
import argparse
import bisect
import json
from pathlib import Path
import statistics
import sys

GAP_FRAMES = 2


def read_jsonl(path):
    rows = []
    with Path(path).open() as f:
        for number, line in enumerate(f, 1):
            if not line.strip():
                raise ValueError('Blank JSONL line %d in %s' % (number, path))
            rows.append(json.loads(line))
    return rows


def expected_period_ns(stamps_ns):
    """Sensor period from the stream's own most common interval (the camera)."""
    intervals = [b-a for a, b in zip(stamps_ns, stamps_ns[1:]) if b > a]
    if len(intervals) < 2:
        raise ValueError('Need at least three odometry samples to establish the sensor period')
    return statistics.mode(intervals)


def phase_windows(flight_rows):
    """[(state, start_host_ns)] in order, starting with PREFLIGHT."""
    transitions = [(r['host_ns'], r['state']) for r in flight_rows if r.get('event') == 'flight_transition']
    transitions.sort()
    return [('PREFLIGHT', None)]+[(state, host) for host, state in transitions]


def phase_at(windows, host_ns):
    starts = [h for _, h in windows[1:]]
    return windows[bisect.bisect_right(starts, host_ns)][0]


def analyze(odom_rows, flight_rows=None, period_ns=None):
    rows = sorted(odom_rows, key=lambda r: r['acquisition_ns'])
    stamps = [r['acquisition_ns'] for r in rows]
    if any(type(s) is not int for s in stamps):
        raise ValueError('acquisition_ns must be integer nanoseconds')
    duplicates = sum(b == a for a, b in zip(stamps, stamps[1:]))
    arrival = [r['callback_entry_host_ns'] for r in odom_rows]
    reordered = sum(b < a for a, b in zip([r['acquisition_ns'] for r in odom_rows][:-1],
                                          [r['acquisition_ns'] for r in odom_rows][1:]))
    period = period_ns or expected_period_ns(stamps)
    max_gap = GAP_FRAMES*period
    windows = phase_windows(flight_rows or [])
    gaps = []
    for previous, current in zip(rows, rows[1:]):
        gap = current['acquisition_ns']-previous['acquisition_ns']
        if gap > max_gap:
            gaps.append(dict(from_acquisition_ns=previous['acquisition_ns'], to_acquisition_ns=current['acquisition_ns'],
                             gap_ns=gap, missing_frames=round(gap/period)-1,
                             phase=phase_at(windows, previous['callback_entry_host_ns']),
                             resumed_phase=phase_at(windows, current['callback_entry_host_ns'])))
    expected = round((stamps[-1]-stamps[0])/period)+1
    phases = {}
    for r in rows:
        name = phase_at(windows, r['callback_entry_host_ns'])
        phases.setdefault(name, dict(received=0, gaps=0))['received'] += 1
    for g in gaps:
        phases.setdefault(g['phase'], dict(received=0, gaps=0))['gaps'] += 1
    states = [s for s, _ in windows]
    return dict(
        status='FAIL' if gaps or duplicates else 'PASS',
        period_ns=period, max_allowed_gap_ns=max_gap,
        bound='%d x sensor period (latest-only frame slot may supersede one frame)' % GAP_FRAMES,
        first_acquisition_ns=stamps[0], last_acquisition_ns=stamps[-1],
        received=len(stamps), expected=expected, missing=expected-len(set(stamps)),
        duplicates=duplicates, arrival_reordered=reordered,
        arrival_monotonic=all(b >= a for a, b in zip(arrival, arrival[1:])),
        largest_gap_ns=max((g['gap_ns'] for g in gaps), default=max((b-a for a, b in zip(stamps, stamps[1:])), default=0)),
        gaps=gaps, phases={k: phases[k] for k in sorted(phases, key=lambda k: states.index(k) if k in states else 99)},
        flight_states=states)


def analyze_directory(directory, period_ns=None):
    directory = Path(directory)
    odom = read_jsonl(directory/'odom.jsonl')
    flight = read_jsonl(directory/'flight.jsonl') if (directory/'flight.jsonl').is_file() else None
    return analyze(odom, flight, period_ns)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('directory', type=Path, help='Directory holding odom.jsonl (and flight.jsonl), e.g. <run>/evidence/flight-transport')
    p.add_argument('--period-ns', type=int, help='Override the sensor period instead of deriving it from the stream')
    a = p.parse_args(argv)
    report = analyze_directory(a.directory, a.period_ns)
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
