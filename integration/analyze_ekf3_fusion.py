"""Summarize a stationary synthetic EKF3 bench; no per-message acceptance count."""
import argparse
import json
import math
from pathlib import Path
import statistics


def statistics_of(values):
    return dict(count=len(values), minimum=min(values), maximum=max(values), mean=statistics.mean(values)) if values else None


def analyze(directory, fcu_directory="fcu", stream_directory="attempt02"):
    p = Path(directory)
    df = json.loads((p / fcu_directory / 'decoded.json').read_text())
    rows = df['records']
    wire = [json.loads(line) for line in (p / stream_directory / 'wire.jsonl').read_text().splitlines()]
    decisions = json.loads((p / stream_directory / 'decisions.json').read_text())
    result = {'scope': 'SYNTHETIC static ground trajectory; not camera VIO or flight', 'channels': {}, 'cores': {}}
    for name, log, length in [('VISION_POSITION_ESTIMATE', 'VISP', 21), ('VISION_SPEED_ESTIMATE', 'VISV', 9)]:
        incoming = [r for r in wire if r['direction'] == 'to_fcu' and r['message']['mavpackettype'] == name]
        received = [r for r in rows if r['mavpackettype'] == log]
        times = [r['TimeUS'] for r in received]
        gaps = [(b-a)/1e6 for a, b in zip(times, times[1:])]
        result['channels'][name] = dict(wire=len(incoming), fcu=len(received),
            timestamps_exact=[r['message']['usec'] for r in incoming] == [r['RTimeUS'] for r in received],
            rate_hz=(len(times)-1)*1e6/(times[-1]-times[0]) if len(times)>1 else None,
            receiver_gaps_seconds=statistics_of(gaps),
            corrected_age_ms=statistics_of([r['TimeUS']/1000-r['CTimeMS'] for r in received]),
            covariance_finite_and_size=all(len(r['message']['covariance']) == length and all(math.isfinite(x) for x in r['message']['covariance']) for r in incoming),
            ignored=sorted({r['Ign'] for r in received}), reset=sorted({r['Rst'] for r in received}))
    positions = [r for r in rows if r['mavpackettype'] == 'VISP']
    lo, hi = positions[0]['TimeUS'], positions[-1]['TimeUS']
    result['stream_window_boot_us'] = [lo, hi]
    result['params'] = {k:v for k,v in df['params'].items() if k.startswith(('EK3_SRC', 'VISO', 'GPS_TYPE', 'AHRS_EKF_TYPE'))}
    result['messages'] = [r for r in rows if r['mavpackettype'] == 'MSG' and any(s in r['Message'] for s in ['external nav', 'aiding', 'origin'])]
    for core in (0, 1):
        core_result = {}
        for phase, start, end in [('before', lo-5e6, lo), ('steady', lo+5e6, hi), ('dropout', hi+10e6, hi+25e6)]:
            subset = [r for r in rows if r.get('C') == core and start < r['TimeUS'] < end]
            status = [r for r in subset if r['mavpackettype'] == 'XKF4']
            innovations = [r for r in subset if r['mavpackettype'] == 'XKF3']
            estimates = [r for r in subset if r['mavpackettype'] == 'XKF1']
            core_result[phase] = dict(status_count=len(status), flags={k:sorted({r[k] for r in status}) for k in ['FS','TS','SS']},
                ratios={k:statistics_of([r[k] for r in status]) for k in ['SV','SP','SH']},
                innovation_max_abs={k:max((abs(r[k]) for r in innovations),default=None) for k in ['IVN','IVE','IPN','IPE']},
                xy_rmse=math.sqrt(statistics.mean([r['PN']**2+r['PE']**2 for r in estimates])) if estimates else None,
                source_sets=sorted({r['SS'] for r in subset if r['mavpackettype']=='XKFS'}),
                ekf_state_variances={key: statistics_of([r[key] for r in subset if r['mavpackettype']=='XKV1']) for key in ['V04','V05','V07','V08']} )
        result['cores'][str(core)] = core_result
    result['faults'] = {}
    wire_stamps={r['message']['usec'] for r in wire if r['direction']=='to_fcu' and r['message']['mavpackettype'].startswith('VISION_')}
    for case in sorted({r['case'] for r in decisions}):
        phase=[r for r in decisions if r['case']==case]
        result['faults'][case]=dict(allowed=sum(r['allowed'] for r in phase), blocked=sum(not r['allowed'] for r in phase), leaks=sum(not r['allowed'] and r['stamp_ns']//1000 in wire_stamps for r in phase))
    hb=[r for r in wire if r['direction']=='from_fcu' and r['message']['mavpackettype']=='HEARTBEAT']
    result['heartbeat']=dict(count=len(hb),all_disarmed=bool(hb) and all(not r['message']['base_mode'] & 128 for r in hb),all_v2=bool(hb) and all(r['mavlink2'] for r in hb))
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--fcu-directory', default='fcu')
    parser.add_argument('--stream-directory', default='attempt02')
    args=parser.parse_args()
    print(json.dumps(analyze(args.directory, args.fcu_directory, args.stream_directory), indent=2, allow_nan=False))
