"""Diagnostic clock regression. Unknown Gazebo one-way delay always prevents GO."""
import argparse
import bisect
import json
from pathlib import Path
import statistics
from navigation_clock import ClockMapping


def analyze(directory, topology=None, anchors_output=None):
    p=Path(directory)
    topology=topology or {}
    load=lambda name:[json.loads(s) for s in (p/name).read_text().splitlines()]
    gz=load('gazebo_clock.jsonl');fcu_all=load('fcu_clock.jsonl')
    fcu=[r for r in fcu_all if r.get('kind')=='timesync']
    hosts=[r['host_ns'] for r in gz];pairs=[]
    for row in fcu:
        index=bisect.bisect_left(hosts,row['host_ns'])
        if not 0<index<len(gz):continue
        a,b=gz[index-1],gz[index]
        if b['host_ns']-a['host_ns']>50_000_000:continue
        source=a['source_ns']+round((b['source_ns']-a['source_ns'])*(row['host_ns']-a['host_ns'])/(b['host_ns']-a['host_ns']))
        pairs.append(dict(source_ns=source,target_ns=row['source_ns'],host_ns=row['host_ns']))
    base=dict(decision='HOLD',clock_mapping='HOLD',pairs=len(pairs),execution_allowed=False,forwarding_enabled=False,uncertainty_bound_verified=False,independent_physics=None,pause_resume='NOT ASSESSED: requires separate event-window evidence')
    if len(pairs)<6:
        return dict(base,reason='Insufficient bracketed pairs for fit and held-out check')
    try:
        model=ClockMapping(pairs,source_epoch='diagnostic-clock-epoch',verified=False)
        training=ClockMapping(pairs[:len(pairs)//2],source_epoch='diagnostic-only')
    except ValueError as error:
        return dict(base,reason=str(error))
    heldout_error=max(abs(row['target_ns']-training.estimate(row['source_ns'])) for row in pairs[len(pairs)//2:])
    differences=[b['source_ns']-a['source_ns'] for a,b in zip(gz,gz[1:])]
    hb=[r for r in fcu_all if r.get('kind')=='heartbeat']
    result=dict(decision='HOLD',clock_mapping='HOLD',execution_allowed=False,forwarding_enabled=False,pairs=len(pairs),fit_slope_fcu_per_gazebo=model.slope,
                fit_max_residual_ms=model.residual_ns/1e6,heldout_max_error_ms=heldout_error/1e6,
                first_raw_fcu_minus_gazebo_ns=pairs[0]['target_ns']-pairs[0]['source_ns'],
                last_raw_fcu_minus_gazebo_ns=pairs[-1]['target_ns']-pairs[-1]['source_ns'],
                gazebo_sim_per_host=(gz[-1]['source_ns']-gz[0]['source_ns'])/(gz[-1]['host_ns']-gz[0]['host_ns']),
                fcu_boot_per_host=(fcu[-1]['source_ns']-fcu[0]['source_ns'])/(fcu[-1]['host_ns']-fcu[0]['host_ns']),
                timesync_rtt_ms=dict(maximum=max(r['rtt_ns'] for r in fcu)/1e6,mean=statistics.mean(r['rtt_ns'] for r in fcu)/1e6),
                gazebo_clock_resets=sum(x<=0 for x in differences),
                all_fcu_disarmed=all(not r['base_mode']&128 for r in hb) if hb else None,
                independent_physics=(True if topology.get('fcu_model')=='quad' else False if topology.get('fcu_model')=='JSON' and topology.get('shared_plugin_endpoint') is True else None),
                physics_evidence_scope=topology.get('scope','UNKNOWN: cannot infer physics from a clock regression'),
                uncertainty_bound_verified=False,
                pause_resume=topology.get('pause_resume','NOT ASSESSED: requires separate event-window evidence'),
                proof_limit='Receive-paired fit, not proven acquisition-to-FCU time; transport/scheduling uncertainty unbounded; no fusion inference')
    if anchors_output is not None:
        with Path(anchors_output).open('x') as output:json.dump(pairs,output,indent=2);output.write('\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory')
    parser.add_argument('--topology',type=Path)
    parser.add_argument('--anchors-output',type=Path)
    args=parser.parse_args()
    topology=json.loads(args.topology.read_text()) if args.topology else None
    print(json.dumps(analyze(args.directory,topology,args.anchors_output),indent=2,allow_nan=False))
