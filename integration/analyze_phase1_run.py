"""Offline evaluation of the complete capture-only run, preserving failed phases.

GT is read here for evaluation only. A declared first 50 wall seconds is also
reported separately; it is not selected by estimator success. No alignment by
yaw, scale or segment: one initial XY translation for all available states.
Reset-containing/static results cannot qualify a moving navigation trajectory.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def read(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def stats(values):
    values = np.asarray(values, float)
    if not len(values):
        return None
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite recorded statistics')
    return dict(min=float(values.min()), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(values.max()))


def intervals(rows, arrival='arrival_monotonic_s'):
    stamps = np.asarray([row['t'] for row in rows])
    host = np.asarray([row[arrival] for row in rows])
    differences = np.diff(stamps)
    return dict(count=len(rows), timestamp_resets=int(sum(differences < 0)),
                positive_sim_gaps_s=stats(differences[differences > 0]),
                wall_gaps_s=stats(np.diff(host)),
                wall_rate_hz=float((len(rows)-1)/(host[-1]-host[0])) if len(rows)>1 and host[-1]>host[0] else None,
                sim_rate_hz=float((len(rows)-1)/(stamps[-1]-stamps[0])) if len(rows)>1 and np.all(differences>0) else None)


def evaluate(rows, poses, ground_z):
    """Bounded GT interpolation, no extrapolation or interpolation through reset."""
    valid = [row for row in rows if 'state' in row]
    if len(valid)<3 or len(poses)<3:
        return dict(status='NOT TESTED', reason='insufficient states/GT')
    pt = np.asarray([row['t'] for row in poses])
    t = np.asarray([row['t'] for row in valid])
    if np.any(np.diff(pt)<=0) or np.any(np.diff(t)<=0) or max(np.diff(pt))>.05:
        return dict(status='NOT TESTED', reason='unordered or gapped evaluation timestamps')
    if t[0]<pt[0] or t[-1]>pt[-1]:
        return dict(status='NOT TESTED', reason='GT does not bracket all states; no extrapolation')
    position = np.array([next(p['p'] for p in row['poses'] if p['name']=='iris_with_gimbal') for row in poses])[:,[1,0,2]]
    position[:,2] = -position[:,2]+ground_z
    velocity = np.gradient(position,pt,axis=0)
    gt = np.column_stack([np.interp(t,pt,position[:,i]) for i in range(3)])
    gv = np.column_stack([np.interp(t,pt,velocity[:,i]) for i in range(3)])
    state = np.asarray([row['state'] for row in valid])
    covariance = np.asarray([row['covariance'] for row in valid])
    error = state[:,:3]-gt
    error[:,:2] = (state[:,:2]-state[0,:2])-(gt[:,:2]-gt[0,:2])
    ve = state[:,3:6]-gv
    position_nees = [float(e@np.linalg.solve(c[:2,:2],e)) for e,c in zip(error[:,:2],covariance)]
    velocity_nees = [float(e@np.linalg.solve(c[3:5,3:5],e)) for e,c in zip(ve[:,:2],covariance)]
    epochs = sorted(set(row.get('epoch',0) for row in valid))
    extent = float(np.linalg.norm(gt[:,:2]-gt[0,:2],axis=1).max())
    result = dict(status='DESCRIPTIVE ONLY', states=len(valid), total_frames=len(rows),
                  gt_xy_extent_m=extent, xy_rmse_m=float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
                  velocity_xy_rmse_m_s=float(np.sqrt(np.mean(np.sum(ve[:,:2]**2,axis=1)))),
                  max_spatial_error_m=float(np.linalg.norm(error,axis=1).max()),
                  final_xy_drift_m=float(np.linalg.norm(error[-1,:2])),
                  epochs=epochs, continuous_epoch=len(epochs)==1,
                  coverage=len(valid)/len(rows), accepted_flow_fraction=sum(row.get('flow_accepted',False) for row in rows)/len(rows),
                  position_nees=stats(position_nees),velocity_nees=stats(velocity_nees),
                  position_95ellipse_coverage=float(np.mean(np.array(position_nees)<=5.991464547)),
                  velocity_95ellipse_coverage=float(np.mean(np.array(velocity_nees)<=5.991464547)),
                  covariance_calibrated=False, motion_test='NOT TESTED' if extent<.2 else 'requires independent motion criteria',
                  note='All available states, including blocked ones. One initial XY translation only. GT Z is also an estimator aid; static NEES cannot calibrate motion covariance.')
    return result,(t,state,gt,gv,error,covariance)


def analyze(run, output):
    output.mkdir(exist_ok=False)
    evidence = run/'evidence'
    source = evidence/'vio'
    config = json.loads((source/'configuration.json').read_text())
    start = config['started_monotonic_s']
    rows = read(source/'stream.jsonl')
    sent = read(source/'publications.jsonl')
    received = read(evidence/'observer/input.jsonl')
    normal = [row for row in rows if 0<=row['wall']-start<50]
    poses = [row for row in read(source/'poses.jsonl') if 0<=row['arrival_monotonic_s']-start<50.5]
    evaluation = evaluate(normal,poses,config['ground_z'])
    plot_data = None
    if isinstance(evaluation,tuple):
        evaluation,plot_data = evaluation
    transitions = json.loads((source/'gate_transitions.json').read_text())
    def gate_ready(at):
        preceding = [row for row in transitions if row['wall']<=at]
        return bool(preceding and preceding[-1]['ready'])
    observed = {round(row['t']*1e9):row for row in received}
    delivery = [observed[round(row['t']*1e9)]['arrival_monotonic_s']-row['publish_monotonic_s']
                for row in sent if round(row['t']*1e9) in observed]
    faults = []
    injections = json.loads((source/'fault_events.json').read_text())
    for index,event in enumerate(injections):
        if event['fault'] is None:
            continue
        ended = next((other for other in injections[index+1:] if other['previous']==event['fault']),None)
        publications = None if ended is None else [row for row in sent
            if event['applied_monotonic_s']<=row['publish_monotonic_s']<ended['applied_monotonic_s']]
        next_injection = None if ended is None else next((other for other in injections[index+1:]
            if other['fault'] is not None and other['applied_monotonic_s']>ended['applied_monotonic_s']),None)
        recovery = [] if ended is None else [row for row in sent if row['publish_monotonic_s']>=ended['applied_monotonic_s']
            and (next_injection is None or row['publish_monotonic_s']<next_injection['applied_monotonic_s'])]
        faults.append(dict(name=event['fault'],effective_start_host_s=event['applied_monotonic_s'],
                           publications_during_effective_fault=None if publications is None else len(publications),
                           recovered_locally_before_next_fault=bool(recovery),
                           local_block_status='NOT TESTED' if publications is None else ('PASS' if not publications else 'FAIL'),
                           post_mavros_positive_to_blocked_transition='NOT TESTED: transport already latched earlier',
                           scope='Labelled local receive/quality injection, not physical Gazebo outage'))
    channels = {}
    for name in ('native_camera','ros_camera','native_imu','imu','clocks'):
        data = read(source/(name+'.jsonl'))
        normal_data = [row for row in data if 0<=row['arrival_monotonic_s']-start<50]
        channels[name] = dict(full=intervals(data),normal=intervals(normal_data))
    transports = {}
    for name in ('transport_1','transport_2'):
        path = evidence/name
        summary = json.loads((path/'summary.json').read_text())
        summary['events'] = read(path/'events.jsonl')
        # The original summary's owner-shutdown reason is not the initiating fault.
        summary['first_latched_reason'] = next((row['reason'] for row in summary['events'] if row['event']=='latched_closed'),None)
        transports[name] = summary
    applied = np.loadtxt(evidence/'applied.csv',delimiter=',',ndmin=2)
    before_faults = [row for row in rows if row['wall']-start<60]
    pauses = {name:json.loads((evidence/(name+'.json')).read_text()) for name in ('pause','resume','reset')}
    reset_after = channels['clocks']['full']['timestamp_resets']
    last_applied_ns = int(applied[-1,6])
    result = dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,
                  source_summary=json.loads((source/'summary.json').read_text()),
                  scope='DISARMED stationary capture with local faults; NOT live FCU fusion or flight',
                  declared_normal_window_wall_s=[0,50],normal_evaluation=evaluation,
                  full_duration_wall_s=rows[-1]['wall']-start, full_frames=len(rows),
                  state_frames=sum('state' in row for row in rows), channels=channels,
                  publications=len(sent),independent_received=len(received),invalid_received=sum(not row['valid'] for row in received),
                  publications_while_quality_blocked=sum(not gate_ready(row['publish_monotonic_s']) for row in sent),
                  publish=intervals(sent,'publish_monotonic_s'),
                  publish_to_independent_subscriber_wall_s=stats(delivery),
                  source_callback_to_estimator_output_wall_s=stats([row['estimator_output_monotonic_s']-
                       next(camera['arrival_monotonic_s'] for camera in read(source/'ros_camera.jsonl') if abs(camera['t']-row['t'])<1e-8)
                       for row in before_faults if 'estimator_output_monotonic_s' in row]),
                  processing_wall_s=stats([row['processing_s'] for row in before_faults if 'processing_s' in row]),
                  acquisition_to_estimator_sim_age_s=stats([row['estimator_output_clock_s']-row['t'] for row in before_faults if 'estimator_output_clock_s' in row]),
                  acquisition_to_fcu_latency='NOT TESTED; no FCU vision reception and no exposure-to-host witness',
                  acquisition_to_fcu_uncertainty_bound='NOT ESTABLISHED; applied label bracket is not a capture/transport bound',
                  applied_clock=dict(rows=len(applied),parsed_true_rows=int(sum(applied[:,1]==1)),
                      disabled_parser_pacing_rows=int(sum(applied[:,2]==0)),
                      source_minus_fcu_label_s=stats(applied[:,0]-applied[:,4]/1e6),
                      source_gap_s=stats(np.diff(applied[:,0])),last_host_ns=last_applied_ns,
                      rows_after_reset_request=int(sum(applied[:,6]>pauses['reset']['end_host_ns']))),
                  world_control_service_ack={name:row['success'] for name,row in pauses.items()},
                  actual_ros_clock_resets=reset_after, faults=faults,transports=transports,
                  isolation=json.loads((evidence/'isolation.json').read_text()))
    if plot_data is not None:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        t,state,gt,gv,error,covariance = plot_data
        figure,axes = plt.subplots(3,2,figsize=(11,8),sharex=True)
        for axis in range(2):
            axes[0,axis].plot(t-t[0],state[:,axis]-state[0,axis],label='VIO')
            axes[0,axis].plot(t-t[0],gt[:,axis]-gt[0,axis],label='GT evaluation')
            axes[1,axis].plot(t-t[0],state[:,axis+3],label='VIO velocity')
            axes[1,axis].plot(t-t[0],gv[:,axis],label='GT velocity')
            axes[2,axis].plot(t-t[0],error[:,axis],label='position error')
        for ax in axes.flat:
            ax.legend();ax.grid();ax.set_xlabel('simulation seconds (declared normal window)')
        figure.suptitle('Static descriptive comparison. Reset-containing capture is HOLD.')
        figure.tight_layout();figure.savefig(output/'static_trajectory.png');plt.close(figure)
    (output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(decision=result['decision'],publications=len(sent),received=len(received),
                          normal_evaluation=evaluation),indent=2))
    return result


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True,help='NEW directory; existing evidence never overwritten')
    args = parser.parse_args()
    if not args.output.resolve().is_relative_to(Path(__file__).resolve().parent):
        raise ValueError('Output must be inside integration/')
    analyze(args.run,args.output)
