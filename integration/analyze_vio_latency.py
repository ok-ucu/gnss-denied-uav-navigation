"""VIO source latency/backlog metrics and a replay of the frame loop.

Inputs are only a run's committed evidence: flight-source/stream.jsonl and
flight-transport/{stages,flight,mapping}.jsonl. Metric definitions (identical
for measured and replayed columns):

  acquisition->publish     simulation time at the end of the frame's source
                           processing (the publish point) minus acquisition_ns
  acquisition->conversion  transport pair_input_age: simulation clock when the
                           transport starts converting the sample minus
                           acquisition_ns
  slot wait                simulation time the frame sat in the latest-only
                           camera slot before processing started

  over limit               processed frames whose acquisition->publish age
                           exceeds the conversion limit (200 ms - mapping
                           uncertainty - slowest recorded transport pair): the
                           source blocks them as stale or the transport expires
                           them; either breaks continuity
  over source limit        processed frames whose end-of-processing age exceeds
                           the contract source publication limit (157 ms); the
                           check runs 2.5-13.5 ms earlier, so this over-counts
                           the frames the source gate would block
  max gap                  largest interval between frames under the conversion limit

All metrics cover every processed frame. Measured conversion age exists only
for frames that actually reached the transport; replayed conversion age is
replayed publish age plus the measured median publish->conversion offset.

Simulation<->host time comes from the transport's own /clock callbacks
(transport_clock_callback: source_ns at callback_host_ns), both CLOCK_MONOTONIC
on one host. Replay: the recorded per-frame costs (non-IMU work and per-sample
IMU cost) are run through a model of the capture loop (poll latest frame ->
process -> tick -> 2 ms sleep). Policy 'frame' integrates IMU on the frame path
(as recorded); 'idle' integrates it in ticks, TICK_CHUNK samples at a time,
never past a frame that may still be processed; 'idle+admit' additionally
applies vio_admission (skip a frame predicted to miss the publication age);
'idle+admit+5ms' also propagates the IMU in 5 ms blocks (measured cost factor). The 'frame' replay must reproduce the
measured ages before the 'idle' replay is believed.
"""
import argparse
import bisect
import json
from pathlib import Path
import statistics

from vio_imu_preintegration import TICK_CHUNK, STAMP_EPSILON_S
from vio_latency_contract import SOURCE_PUBLICATION_MAX_AGE_S

LOOP_SLEEP_S = .002
IMU_PERIOD_S = .001
IMU_DELIVERY_S = .003   # isolated IMU flush target (vio_sensor_capture)


def jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


class Timebase:
    """Piecewise-linear simulation<->host mapping from observed /clock receipts."""
    def __init__(self, pairs):
        pairs = sorted(pairs)
        self.host, self.sim = [], []
        for host, sim in pairs:
            if not self.host or (host > self.host[-1] and sim > self.sim[-1]):
                self.host.append(host)
                self.sim.append(sim)
        if len(self.host) < 2:
            raise ValueError('Need at least two /clock receipts')

    @staticmethod
    def _interp(xs, ys, x):
        i = min(max(bisect.bisect_left(xs, x), 1), len(xs)-1)
        x0, x1, y0, y1 = xs[i-1], xs[i], ys[i-1], ys[i]
        return y0+(y1-y0)*(x-x0)/(x1-x0)

    def covers(self, host_s):
        return self.host[0] <= host_s <= self.host[-1]

    def to_sim(self, host_s):
        return self._interp(self.host, self.sim, host_s)

    def to_host(self, sim_s):
        return self._interp(self.sim, self.host, sim_s)


def quantiles(values):
    v = sorted(values)
    if not v:
        return None
    pick = lambda q: v[min(len(v)-1, int(round(q*(len(v)-1))))]
    return dict(n=len(v), p50=pick(.5), p95=pick(.95), max=v[-1])


def load(run):
    run = Path(run)
    stream = [r for r in jsonl(run/'flight-source/stream.jsonl') if 'processing_s' in r]
    # Preserved evidence keeps transport logs in flight-transport/ (newer) or at top level (older).
    transport = run/'flight-transport' if (run/'flight-transport').is_dir() else run
    stages = jsonl(transport/'stages.jsonl')
    flight = jsonl(transport/'flight.jsonl')
    mapping = jsonl(transport/'mapping.jsonl') if (transport/'mapping.jsonl').is_file() else []
    clock = Timebase([(r['callback_host_ns']/1e9, r['source_ns']/1e9) for r in stages
                      if r['stage'] == 'transport_clock_callback'])
    # Only frames whose whole processing lies inside observed /clock coverage;
    # never extrapolate the simulation<->host mapping.
    stream = [r for r in stream if clock.sim[0] <= r['t'] and clock.host[0] <= r['wall']
              and r['wall']+r['processing_s'] <= clock.host[-1] and r['t']+.3 <= clock.sim[-1]]
    transitions = sorted((r['host_ns']/1e9, r['state']) for r in flight if r.get('event') == 'flight_transition')
    climb = next((h for h, s in transitions if s == 'CLIMB'), None)
    after = next((h for h, s in transitions if climb is not None and h > climb), None)
    climb_window = None if climb is None else (clock.to_sim(climb), clock.to_sim(after) if after else float('inf'))
    pair_age = {r['source_ns']: r['age_sim_ns']/1e9 for r in stages
                if r['stage'] == 'pair_input_age' and r.get('age_sim_ns') is not None}
    pair_cost = [(r['end_host_ns']-r['start_host_ns'])/1e9 for r in stages if r['stage'] == 'lifecycle_pair']
    uncertainty = max((m['uncertainty_ns'] for m in mapping), default=None)
    return dict(stream=stream, clock=clock, climb_window=climb_window, pair_age=pair_age,
                pair_cost=pair_cost, uncertainty_s=None if uncertainty is None else uncertainty/1e9)


def conversion_limit(data):
    """Oldest acquisition->conversion age that still leaves the transport time
    to finish one pair: 200 ms - mapping uncertainty - slowest recorded pair."""
    return .2-data['uncertainty_s']-max(data['pair_cost'])


def measured(data):
    clock, rows = data['clock'], []
    for r in data['stream']:
        end = r['wall']+r['processing_s']
        rows.append(dict(t=r['t'], start=r['wall'], end=end, publish_age=clock.to_sim(end)-r['t'],
                         slot_wait=clock.to_sim(r['wall'])-r['t']-(r.get('source_age') or 0),
                         published=bool(r.get('published')),
                         conversion_age=data['pair_age'].get(round(r['t']*1e9))))
    return rows


POLICIES = ('frame', 'idle', 'idle+admit', 'idle+admit+5ms')
# IMU cost with 5 ms propagation blocks relative to per-sample propagation,
# measured on vio_imu_preintegration.advance (3000 samples: 30.3 / 118.5 ms).
BLOCK_IMU_COST_FACTOR = .256


def replay(data, policy, period=.1, publication_max_age=None):
    """Replay recorded costs through the capture loop under a policy."""
    if policy not in POLICIES:
        raise ValueError('policy must be one of %s' % (POLICIES,))
    from vio_admission import Admission
    publication_max_age = SOURCE_PUBLICATION_MAX_AGE_S if publication_max_age is None else publication_max_age
    admission = Admission(publication_max_age) if policy.startswith('idle+admit') else None
    imu_factor = BLOCK_IMU_COST_FACTOR if policy.endswith('+5ms') else 1.
    clock, recorded = data['clock'], data['stream']
    costs = []
    for r in recorded:
        imu = r.get('imu_processing_wall_s') or 0.
        count = (r.get('imu_batch') or {}).get('count') or 0
        costs.append((r['t'], max(0., r['processing_s']-imu), imu_factor*imu/count if count else 0.))
    times = [c[0] for c in costs]
    def cost_at(t):
        i = min(range(len(times)), key=lambda k: abs(times[k]-t))
        return costs[i]
    delivery = {round(r['t']*1e9): r.get('source_age') for r in recorded if r.get('source_age') is not None}
    median_delivery = statistics.median(delivery.values())
    first, last = times[0], times[-1]
    grid = [round(first+k*period, 9) for k in range(int(round((last-first)/period))+1)]
    arrival = [clock.to_host(t+delivery.get(round(t*1e9), median_delivery)) for t in grid]
    n_imu = int(round((last-first+period)/IMU_PERIOD_S))
    imu_t = [first-period+(k+1)*IMU_PERIOD_S for k in range(n_imu)]
    imu_arrival = [clock.to_host(s)+IMU_DELIVERY_S for s in imu_t]
    imu_cost = [cost_at(s)[2] for s in imu_t]
    now = recorded[0]['wall']
    consumed, integrated, last_t, out, superseded, skipped = -1, 0, None, [], 0, 0
    end_time = recorded[-1]['wall']+recorded[-1]['processing_s']
    while now < end_time:
        ready = bisect.bisect_right(arrival, now)-1
        if ready > consumed:
            superseded += ready-consumed-1 if consumed >= 0 else 0
            consumed = ready
            t = grid[ready]
            if admission is not None and not admission.admit(clock.to_sim(now)-t):
                skipped += 1
                last_t = t
                now += LOOP_SLEEP_S
                continue
            _, base, _ = cost_at(t)
            work = base
            while integrated < n_imu and imu_t[integrated] <= t+1e-9:
                work += imu_cost[integrated]
                integrated += 1
            start = now
            now += work
            if admission is not None:
                admission.processed(clock.to_sim(now)-clock.to_sim(start))
            out.append(dict(t=t, start=start, end=now, publish_age=clock.to_sim(now)-t,
                            slot_wait=clock.to_sim(start)-t-delivery.get(round(t*1e9), median_delivery)))
            last_t = t
        if policy != 'frame' and last_t is not None:
            newest = bisect.bisect_right(arrival, now)-1
            bound = (grid[newest] if newest >= 0 and grid[newest] > last_t else last_t+period)-STAMP_EPSILON_S
            done = 0
            while (done < TICK_CHUNK and integrated < n_imu and imu_t[integrated] <= bound
                   and imu_arrival[integrated] <= now):
                now += imu_cost[integrated]
                integrated += 1
                done += 1
        now += LOOP_SLEEP_S
    offsets = [data['pair_age'][round(r['t']*1e9)]-m['publish_age']
               for r, m in zip(recorded, measured(data)) if round(r['t']*1e9) in data['pair_age']]
    offset = statistics.median(offsets) if offsets else 0.
    for row in out:
        row['conversion_age'] = row['publish_age']+offset
    return out, dict(superseded=superseded, skipped=skipped, transport_offset_s=offset)


def window(rows, lo, hi):
    return [r for r in rows if lo <= r['t'] <= hi]


def summarize(rows, key):
    q = quantiles([r[key]*1e3 for r in rows if r.get(key) is not None])
    return None if q is None else {k: (round(v, 1) if k != 'n' else v) for k, v in q.items()}


def report(run):
    data = load(run)
    meas = measured(data)
    replays = {policy: replay(data, policy) for policy in POLICIES}
    before, before_meta = replays['frame']
    lo, hi = data['climb_window']
    # Model validation: replayed vs measured publish age on the same frames.
    by_t = {round(r['t'], 3): r for r in before}
    errors = [abs(by_t[round(m['t'], 3)]['publish_age']-m['publish_age'])*1e3
              for m in meas if round(m['t'], 3) in by_t]
    limit = conversion_limit(data)
    result = dict(limit_conversion_age_ms=round(limit*1e3, 1),
                  mapping_uncertainty_ms=data['uncertainty_s']*1e3,
                  slowest_pair_ms=round(max(data['pair_cost'])*1e3, 2),
                  climb_window_sim_s=[round(lo, 3), round(hi, 3)],
                  replay_validation=dict(frames=len(errors), median_abs_error_ms=round(statistics.median(errors), 1),
                                         p95_abs_error_ms=round(quantiles(errors)['p95'], 1)),
                  superseded={p: m['superseded'] for p, (_, m) in replays.items()},
                  skipped_by_admission={p: m['skipped'] for p, (_, m) in replays.items()},
                  transport_offset_ms=round(before_meta['transport_offset_s']*1e3, 1))
    columns = [('measured', meas)]+[('replay_'+p, rows) for p, (rows, _) in replays.items()]
    table = {}
    for name, rows in columns:
        table[name] = {}
        for scope, sel in (('CLIMB', window(rows, lo, hi)), ('flight', rows)):
            # Identical definitions in every column (see module doc).
            usable = sorted(r['t'] for r in sel if r['publish_age'] <= limit)
            table[name][scope] = dict(publish=summarize(sel, 'publish_age'), slot_wait=summarize(sel, 'slot_wait'),
                                      conversion=summarize(sel, 'conversion_age'),
                                      over_limit=sum(r['publish_age'] > limit for r in sel),
                                      over_source_limit=sum(r['publish_age'] > SOURCE_PUBLICATION_MAX_AGE_S for r in sel),
                                      max_gap_ms=round(max((b-a for a, b in zip(usable, usable[1:])), default=0)*1e3, 1))
    result['table'] = table
    pick = lambda rows: next((r for r in rows if abs(r['t']-37.9) < 1e-6), None)
    result['acquisition_37_9'] = {name: None if pick(rows) is None else
                                  {k: round(pick(rows)[k]*1e3, 1) for k in ('publish_age', 'slot_wait', 'conversion_age')
                                   if pick(rows).get(k) is not None}
                                  for name, rows in columns}
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('run', type=Path, help='Evidence directory holding flight-source/ and flight-transport/')
    print(json.dumps(report(p.parse_args(argv).run), indent=2))


if __name__ == '__main__':
    main()
