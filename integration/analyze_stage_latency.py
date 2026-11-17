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
from permission_deadlines import timeline


def records(path):
    if not path.exists():return []
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def quantiles(values):
    values=np.asarray(values,float)
    if not len(values):return dict(n=0,min=None,p50=None,p95=None,p99=None,max=None)
    if not np.isfinite(values).all():raise ValueError('Nonfinite latency evidence')
    return dict(n=len(values),min=float(values.min()),**dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100])))))


def stage_summary(rows):
    grouped=defaultdict(list);cpu=defaultdict(list)
    for row in rows:
        if 'start_host_ns' in row:
            grouped[row['stage']].append((row['end_host_ns']-row['start_host_ns'])/1e6)
            measured=row.get('cpu_ns',row.get('process_cpu_ns'))
            if measured is not None:cpu[row['stage']].append(measured/1e6)
    cgroup=[row['values'] for row in rows if row['stage']=='cgroup_cpu']
    return dict(wall_ms={name:quantiles(values) for name,values in grouped.items()},
                process_cpu_ms={name:quantiles(values) for name,values in cpu.items()},
                cgroup_delta=None if len(cgroup)<2 else {k:cgroup[-1][k]-cgroup[0][k] for k in cgroup[0]},
                profile_rows_dropped=sum(row.get('dropped',0) for row in rows if row['stage']=='profile_summary'))


def clock_path_stats(stages,wire,start=None,end=None):
    def in_window(t):return (start is None or t>=start) and (end is None or t<=end)
    received=[r for r in stages if r['stage']=='clock_anchor_received' and in_window(r['produced_host_ns'])]
    validated={r['source_ns']:r for r in stages if r['stage']=='clock_anchor_validated'}
    decisions=[r for r in stages if r['stage']=='clock_authorization' and in_window(r['decision_host_ns'])]
    final=[r['clock'] for r in wire if r['valid_capture'] and 'clock' in r]
    return dict(production_intervals_wall_ms=quantiles([(b['produced_host_ns']-a['produced_host_ns'])/1e6 for a,b in zip(received,received[1:])]),
                production_to_read_wall_ms=quantiles([(r['received_host_ns']-r['produced_host_ns'])/1e6 for r in received]),
                read_to_validated_wall_ms=quantiles([(validated[r['source_ns']]['host_ns']-r['received_host_ns'])/1e6 for r in received if r['source_ns'] in validated]),
                authorization_latest_applied_age_wall_ms=quantiles([r['latest_applied_age_ns']/1e6 for r in decisions if r['latest_applied_age_ns'] is not None]),
                authorization_ros_age_wall_ms=quantiles([r['ros_age_ns']/1e6 for r in decisions if r['ros_age_ns'] is not None]),
                valid_wire_latest_applied_age_wall_ms=quantiles([r['latest_applied_age_ns']/1e6 for r in final if r['latest_applied_age_ns'] is not None]),
                valid_wire_ros_age_wall_ms=quantiles([r['ros_age_ns']/1e6 for r in final if r['ros_age_ns'] is not None]),
                valid_wire_freshness_violations=sum(r.get('latest_applied_age_ns') is None or r.get('ros_age_ns') is None
                    or not 0<=r['latest_applied_age_ns']<=200_000_000 or not 0<=r['ros_age_ns']<=200_000_000 for r in final))


def case_metrics(root,transport_name,source_name):
    transport=root/'evidence'/transport_name;source=root/'evidence'/source_name
    stages=records(transport/'stages.jsonl');sensor_stages=records(source/'stages.jsonl')
    mapping=records(transport/'mapping.jsonl');wire=records(transport/'wire.jsonl')
    odom=records(transport/'odom.jsonl');frames=records(source/'stream.jsonl')
    source_summary=json.loads((source/'summary.json').read_text()) if (source/'summary.json').exists() else None
    callbacks={row['source_ns']:row['host_ns'] for row in sensor_stages if row['stage']=='camera_callback_entry'}
    native={row['source_ns']:row['host_ns'] for row in sensor_stages if row['stage']=='native_camera_arrival'}
    outputs={row['source_ns']:row['host_ns'] for row in sensor_stages if row['stage']=='vio_estimator_output'}
    enqueue={row['acquisition_ns']:row['end_host_ns'] for row in sensor_stages if row['stage']=='vio_ros_enqueue'}
    callbacks_transport={row['acquisition_ns']:row['callback_entry_host_ns'] for row in odom}
    by_wire={row['fcu_ns']//1000:row for row in mapping}
    transport_enqueue={r['acquisition_ns']:r['end_host_ns'] for r in stages if r['stage']=='mavros_enqueue'}
    before_egress=[];mapping_log_to_read=[];final_egress=[];slack=[];original_stamps=[];callback_to_wire=[]
    for row in wire:
        permission=by_wire.get(row['message']['usec'])
        if permission is None:continue
        # Mapping audit occurs AFTER ROS publish; it is not enqueue completion.
        # Missing span means unmeasured latency, never a substituted log time.
        stamp=permission['acquisition_ns']
        if stamp in transport_enqueue:
            before_egress.append((row['arrival_host_ns']-transport_enqueue[stamp])/1e6)
        mapping_log_to_read.append((row['arrival_host_ns']-permission['publish_host_ns'])/1e6)
        final_egress.append((row['checked_host_ns']-permission['approved_host_ns'])/1e6)
        slack.append((permission['allowed_host_deadline_ns']-row['checked_host_ns'])/1e6)
        original_stamps.append(row['message']['usec']==permission['fcu_ns']//1000)
        stamp=permission['acquisition_ns']
        if stamp in callbacks:callback_to_wire.append((row['checked_host_ns']-callbacks[stamp])/1e6)
    callback_stages=[row for row in stages if row['stage']=='callback_validation']
    source_errors=defaultdict(int)
    for row in frames:
        if row.get('error'):source_errors[row['error']]+=1
    blocks=[row for row in stages if row['stage']=='lifecycle_block']
    refs=[row for row in records(transport/'events.jsonl') if row['event']=='diagnostic_reference_initialized']
    first_block=next((row for row in blocks if refs and row['host_ns']>refs[0]['host_ns']),None)
    source_reasons=[row for row in frames if row.get('error')=='stale at publication']
    clock_callbacks=[row for row in stages if row['stage']=='transport_clock_callback']
    watermarks=[row for row in stages if row['stage']=='common_clock_watermark']
    starts=[r['host_ns'] for r in sensor_stages if r['stage']=='sensor_capture_started']
    ends=[r['host_ns'] for r in sensor_stages if r['stage']=='sensor_capture_finished']
    capture_markers_valid=len(starts)==len(ends)==1 and starts[0]<ends[0]
    capture_start=starts[0] if capture_markers_valid else None
    capture_end=ends[0] if capture_markers_valid else None
    reference_stamp=refs[0].get('initial_acquisition_ns') if refs else None
    source_publications=records(source/'publications.jsonl')
    eligible=None if reference_stamp is None else {round(r['t']*1e9) for r in source_publications if round(r['t']*1e9)>reference_stamp}
    eligible_rows=[] if reference_stamp is None else [r for r in source_publications if round(r['t']*1e9)>reference_stamp]
    mapped_stamps={r['acquisition_ns'] for r in mapping}
    result=dict(transport=transport_name,source=source_name,source_summary=source_summary,
                permission_boundaries=timeline(stages,mapping,wire),
                transport_profile=stage_summary(stages),source_profile=stage_summary(sensor_stages),
                callback_to_vio_output_wall_ms=quantiles([(outputs[s]-callbacks[s])/1e6 for s in outputs if s in callbacks]),
                native_camera_arrival_to_ros_callback_wall_ms=quantiles([(callbacks[s]-native[s])/1e6 for s in callbacks if s in native]),
                camera_callback_to_final_wire_check_wall_ms=quantiles(callback_to_wire),
                callback_to_vio_enqueue_wall_ms=quantiles([(enqueue[s]-callbacks[s])/1e6 for s in enqueue if s in callbacks]),
                vio_enqueue_to_transport_callback_wall_ms=quantiles([(callbacks_transport[s]-enqueue[s])/1e6 for s in callbacks_transport if s in enqueue]),
                acquisition_to_vio_output_sim_ms=quantiles([(row['estimator_output_clock_s']-row['t'])*1000 for row in frames if 'estimator_output_clock_s' in row]),
                mavros_enqueue_to_udp_read_wall_ms=quantiles(before_egress),
                mapping_log_to_udp_read_wall_ms=quantiles(mapping_log_to_read),
                approval_to_final_wire_check_wall_ms=quantiles(final_egress),
                final_wire_deadline_slack_ms=quantiles(slack),
                permission_budget_wall_ms=quantiles([(r['allowed_host_deadline_ns']-r['approved_host_ns'])/1e6 for r in mapping]),
                transport_clock_callback_gap_wall_ms=quantiles([(b['host_ns']-a['host_ns'])/1e6 for a,b in zip(clock_callbacks,clock_callbacks[1:])]),
                common_watermark_age_wall_ms=quantiles([(r['host_ns']-r['applied_host_ns'])/1e6 for r in watermarks]),
                ros_clock_label_lag_behind_applied_sim_ms=quantiles([(r['latest_applied_ns']-r['ros_clock_ns'])/1e6 for r in watermarks]),
                original_mapped_acquisition_microseconds_preserved=bool(original_stamps and all(original_stamps)),
                odom_callbacks=len(callback_stages),validated_odom_callbacks=len(odom),
                callback_rejected_count=len(callback_stages)-len(odom),
                source_stale_publication_count=len(source_reasons),source_error_counts=dict(source_errors),
                source_publications=len(source_publications),mavros_pairs_enqueued=len(mapping),
                source_capture_start_host_ns=capture_start,source_capture_end_host_ns=capture_end,
                capture_markers_valid=capture_markers_valid,capture_start_marker_count=len(starts),capture_end_marker_count=len(ends),
                source_capture_wall_s=None if capture_start is None or capture_end is None else (capture_end-capture_start)/1e9,
                reference_acquisition_ns=reference_stamp,eligible_source_count=None if eligible is None else len(eligible),
                missing_eligible_source=None if eligible is None else sorted(eligible-mapped_stamps),
                source_after_reference_sim_gaps_ms=quantiles([(b['t']-a['t'])*1000 for a,b in zip(eligible_rows,eligible_rows[1:])]),
                source_after_reference_wall_gaps_ms=quantiles([(b['publish_monotonic_s']-a['publish_monotonic_s'])*1000 for a,b in zip(eligible_rows,eligible_rows[1:])]),
                clock_path=clock_path_stats(stages,wire,capture_start,capture_end),
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


def normal_verdict(metric):
    """Wire continuity criterion only, never motion/accuracy/fusion approval."""
    summary=metric['source_summary']
    first=metric.get('first_block_after_reference')
    end=metric.get('source_capture_end_host_ns')
    interrupted=bool(first and first.get('reason')!='owner shutdown'
                     and (end is None or first.get('host_ns',0)<end))
    complete=(metric['wire_valid']>=10 and metric['wire_valid']==2*metric['mavros_pairs_enqueued']
              and not metric['wire_invalid'])
    source_missing=metric.get('missing_eligible_source')
    fresh=not metric.get('clock_path',{}).get('valid_wire_freshness_violations',0)
    gaps=[(metric.get(key,{}).get('min'),metric.get(key,{}).get('max'),limit) for key,limit in
          [('source_after_reference_sim_gaps_ms',200.000001),('source_after_reference_wall_gaps_ms',500.)]]
    continuous=all(lo is None or (lo>0 and hi<=limit) for lo,hi,limit in gaps)
    profiled=metric.get('capture_markers_valid',True) and not any(metric.get(key,{}).get('profile_rows_dropped',0) for key in ('transport_profile','source_profile'))
    return 'PASS' if complete and continuous and profiled and not source_missing and fresh and not interrupted and summary and not summary['gaps'] and not summary['recoveries'] else 'FAIL'


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
        elif row.get('injected') and row.get('injection_utc_ns') is not None:
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
            status=normal_verdict(metric)
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
