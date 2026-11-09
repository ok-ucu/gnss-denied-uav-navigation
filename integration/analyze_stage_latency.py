"""Real stage timings and qualification of the DISARMED capture fault matrix.

Simulation labels and Linux monotonic events are kept separate. Native-camera
callback arrival is not a wall-time exposure witness. No acquisition-to-FCU
latency, FCU reception or fusion is inferred from a capture-only wire stream.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np


def records(path):
    if not path.exists():return []
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def quantiles(values):
    values=np.asarray(values,float)
    if not len(values):return dict(n=0,p50=None,p95=None,p99=None,max=None)
    if not np.isfinite(values).all():raise ValueError('Nonfinite latency evidence')
    return dict(n=len(values),**dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100])))))


def stage_summary(rows):
    grouped=defaultdict(list);cpu=defaultdict(list)
    for row in rows:
        if 'start_host_ns' in row:
            grouped[row['stage']].append((row['end_host_ns']-row['start_host_ns'])/1e6)
            cpu[row['stage']].append(row['cpu_ns']/1e6)
    cgroup=[row['values'] for row in rows if row['stage']=='cgroup_cpu']
    return dict(wall_ms={name:quantiles(values) for name,values in grouped.items()},
                process_cpu_ms={name:quantiles(values) for name,values in cpu.items()},
                cgroup_delta=None if len(cgroup)<2 else {k:cgroup[-1][k]-cgroup[0][k] for k in cgroup[0]},
                profile_rows_dropped=sum(row.get('dropped',0) for row in rows if row['stage']=='profile_summary'))


def case_metrics(root,transport_name,source_name):
    transport=root/'evidence'/transport_name;source=root/'evidence'/source_name
    stages=records(transport/'stages.jsonl');sensor_stages=records(source/'stages.jsonl')
    mapping=records(transport/'mapping.jsonl');wire=records(transport/'wire.jsonl')
    odom=records(transport/'odom.jsonl');frames=records(source/'stream.jsonl')
    source_summary=json.loads((source/'summary.json').read_text()) if (source/'summary.json').exists() else None
    callbacks={row['source_ns']:row['host_ns'] for row in sensor_stages if row['stage']=='camera_callback_entry'}
    outputs={row['source_ns']:row['host_ns'] for row in sensor_stages if row['stage']=='vio_estimator_output'}
    enqueue={row['acquisition_ns']:row['end_host_ns'] for row in sensor_stages if row['stage']=='vio_ros_enqueue'}
    callbacks_transport={row['acquisition_ns']:row['callback_entry_host_ns'] for row in odom}
    by_wire={row['fcu_ns']//1000:row for row in mapping}
    before_egress=[];final_egress=[];slack=[];original_stamps=[]
    for row in wire:
        permission=by_wire.get(row['message']['usec'])
        if permission is None:continue
        before_egress.append((row['arrival_host_ns']-permission['publish_host_ns'])/1e6)
        final_egress.append((row['checked_host_ns']-permission['approved_host_ns'])/1e6)
        slack.append((permission['allowed_host_deadline_ns']-row['checked_host_ns'])/1e6)
        original_stamps.append(row['message']['usec']==permission['fcu_ns']//1000)
    callback_stages=[row for row in stages if row['stage']=='callback_validation']
    source_errors=defaultdict(int)
    for row in frames:
        if row.get('error'):source_errors[row['error']]+=1
    blocks=[row for row in stages if row['stage']=='lifecycle_block']
    refs=[row for row in records(transport/'events.jsonl') if row['event']=='diagnostic_reference_initialized']
    first_block=next((row for row in blocks if refs and row['host_ns']>refs[0]['host_ns']),None)
    source_reasons=[row for row in frames if row.get('error')=='stale at publication']
    result=dict(transport=transport_name,source=source_name,source_summary=source_summary,
                transport_profile=stage_summary(stages),source_profile=stage_summary(sensor_stages),
                callback_to_vio_output_wall_ms=quantiles([(outputs[s]-callbacks[s])/1e6 for s in outputs if s in callbacks]),
                callback_to_vio_enqueue_wall_ms=quantiles([(enqueue[s]-callbacks[s])/1e6 for s in enqueue if s in callbacks]),
                vio_enqueue_to_transport_callback_wall_ms=quantiles([(callbacks_transport[s]-enqueue[s])/1e6 for s in callbacks_transport if s in enqueue]),
                acquisition_to_vio_output_sim_ms=quantiles([(row['estimator_output_clock_s']-row['t'])*1000 for row in frames if 'estimator_output_clock_s' in row]),
                mavros_enqueue_to_udp_read_wall_ms=quantiles(before_egress),
                approval_to_final_wire_check_wall_ms=quantiles(final_egress),
                final_wire_deadline_slack_ms=quantiles(slack),
                original_mapped_acquisition_microseconds_preserved=bool(original_stamps and all(original_stamps)),
                odom_callbacks=len(callback_stages),validated_odom_callbacks=len(odom),
                callback_rejected_count=len(callback_stages)-len(odom),
                source_stale_publication_count=len(source_reasons),source_error_counts=dict(source_errors),
                source_publications=len(records(source/'publications.jsonl')),mavros_pairs_enqueued=len(mapping),
                wire_received=len(wire),wire_valid=sum(row['valid_capture'] for row in wire),
                wire_invalid=sum(not row['valid_capture'] for row in wire),
                wire_after_deadline=sum(row.get('deadline_host_ns') is not None and row['checked_host_ns']>row['deadline_host_ns'] for row in wire),
                first_block_after_reference=first_block,explicit_reference=bool(refs),
                clock_mapping_summary=json.loads((transport/'summary.json').read_text()) if (transport/'summary.json').exists() else None,
                acquisition_wall_latency='NOT TESTED: no monotonic exposure/render witness',
                fcu_vision_latency='NOT TESTED: forwarding=false')
    return result,wire,stages


def fault_verdict(positive,already_blocked,blocked,leaks,reopened):
    if leaks or reopened:return 'FAIL'
    if not positive or already_blocked:return 'NOT TESTED'
    return 'PASS' if blocked else 'FAIL'


def analyze(root,tag,output):
    output.mkdir(exist_ok=False)
    results=json.loads((root/'evidence'/(tag+'_matrix/results.json')).read_text())
    commands=records(root/'evidence'/(tag+'_matrix/commands.jsonl'))
    cases=[]
    for row in results:
        metric,wire,stages=case_metrics(root,row['transport'],row['source'])
        case=row['case'];boundary=None
        if case in ('camera_loss','imu_loss','tracking_loss','bad_covariance','stale'):
            control=next((i for i,c in enumerate(commands) if c.get('event')=='fault_control' and c.get('fault')==case),None)
            if control is not None:
                # Immediately preceding Linux-side fsync+rename command returns
                # its actual monotonic commit witness. Not host UTC arithmetic.
                boundary=int(commands[control-1]['stdout'].strip())
        elif case in ('pause_resume','clock_reset'):
            path=root/'evidence'/(tag+'_matrix')/(case+'.json')
            if path.exists():boundary=json.loads(path.read_text())['start_host_ns']
        else:
            relevant=[c for c in commands if c.get('command') and c.get('host_utc_ns',0)>=row.get('injection_utc_ns',0) and 'kill -TERM' in ' '.join(c['command'])]
            if row.get('injected') and relevant:
                # Source/operator UTC is not used as a nanosecond clock mapping.
                # The owned lifecycle stop/disconnect is its local witness.
                expected=('owner shutdown',) if case=='process_restart' else ('owned MAVROS process stopped','MAVROS disconnected')
                boundary=next((c['host_ns'] for c in stages if c['stage']=='lifecycle_block' and c.get('reason') in expected),None)
        first=metric['first_block_after_reference']
        already=bool(boundary is not None and first and first['host_ns']<boundary)
        before=[] if boundary is None else [w for w in wire if w['valid_capture'] and w['checked_host_ns']<=boundary]
        positive=bool(before and boundary-before[-1]['checked_host_ns']<=500_000_000)
        blocked=bool(boundary is not None and first and 0<=first['host_ns']-boundary<=2_000_000_000
                     and (case=='process_restart' or first.get('reason')!='owner shutdown'))
        leaks=[] if boundary is None or first is None else [w for w in wire if w['valid_capture'] and w['checked_host_ns']>max(boundary,first['host_ns'])]
        reopened=False
        if row.get('restarted_transport'):
            restart,rewire,restages=case_metrics(root,row['restarted_transport'],row['source'])
            metric['restart']=restart;reopened=bool(restart['wire_valid'] or restart['explicit_reference'])
        if case=='normal':
            status='PASS' if metric['wire_valid']>=10 and not metric['wire_invalid'] and metric['source_summary'] and not metric['source_summary']['gaps'] and not metric['source_summary']['recoveries'] else 'FAIL'
        else:status=fault_verdict(positive,already,blocked,len(leaks),reopened)
        metric.update(case=case,status=status,local_fault_commit_host_ns=boundary,
                      positive_within_500ms_before_fault=positive,already_blocked_before_fault=already,
                      valid_packets_after_latch=len(leaks),reopened_after_restart=reopened,
                      scope='Static real camera VIO; labelled local receive/quality faults; capture only')
        cases.append(metric)
    before,_,_=case_metrics(root,'before_normal_transport1','before_normal_vio')
    result=dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,before=before,cases=cases,
                notes=['Quantiles are observed, not hard real-time bounds. Counts n matter.',
                       'MAVROS enqueue means ROS publish completion; DDS/MAVROS processing is inside enqueue-to-UDP-read aggregate.',
                       'Simulation acquisition labels are never replaced. Host wall exposure time is unknown.',
                       'Camera/IMU source may already be invalid before an injected fault: these cases are NOT TESTED, not PASS.'])
    (output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(decision='HOLD',cases=[dict(case=c['case'],status=c['status'],wire_valid=c['wire_valid'],wire_invalid=c['wire_invalid']) for c in cases]),indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--tag',default='final')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.output.resolve().is_relative_to(Path(__file__).resolve().parent):raise ValueError('integration output only')
    analyze(args.root,args.tag,args.output)
