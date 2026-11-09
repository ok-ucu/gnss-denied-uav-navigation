"""Independent capture diagnostics and GT evaluation; never feeds GT to VIO."""
import argparse
import json
from pathlib import Path


def analyze(root):
    import numpy as np
    def read(name):return [json.loads(x) for x in (root/(name+'.jsonl')).read_text().splitlines()]
    def stats(rows):
        t=np.array([r['t'] for r in rows]);a=np.array([r['arrival_monotonic_s'] for r in rows])
        dt=np.diff(t);da=np.diff(a)
        return {'count':len(rows),'rate_sim_hz':float((len(t)-1)/(t[-1]-t[0])),
                'rate_wall_hz':float((len(a)-1)/(a[-1]-a[0])),
                'max_sim_gap_s':float(dt.max()),'max_arrival_gap_s':float(da.max()),
                'arrival_p99_s':float(np.quantile(da,.99)),
                'sim_gap_p99_s':float(np.quantile(dt,.99)),
                'gaps_over_20ms':[{'start':float(t[i]),'end':float(t[i+1]),'dt':float(d)} for i,d in enumerate(dt) if d>.020000001],
                'callback_build_p99_s':float(np.quantile([r.get('callback_build_s',0) for r in rows],.99)),
                'callback_build_max_s':max((r.get('callback_build_s',0) for r in rows),default=0)}
    imu=read('imu');native=read('native_imu');clocks=read('clock');poses=read('poses');samples=read('samples')
    summary=json.loads((root/'summary.json').read_text())
    report={'imu_ros':stats(imu),'imu_native':stats(native),'clock':stats(clocks),
            'capture_meta':json.loads((root/'capture_meta.json').read_text()),
            'runner_status':summary['status'],'recovery_events':summary['recovery_events']}
    nr={round(r['t']*1e6):r for r in native};rr={round(r['t']*1e6):r for r in imu}
    lo=max(min(nr),min(rr));hi=min(max(nr),max(rr));ns={t for t in nr if lo<=t<=hi};rs={t for t in rr if lo<=t<=hi}
    delays=[rr[t]['arrival_monotonic_s']-nr[t]['arrival_monotonic_s'] for t in ns&rs]
    report['native_ros_comparison']={'native_not_received_ros':len(ns-rs),
                                     'ros_not_received_native':len(rs-ns),
                                     'paired':len(delays),
                                     'arrival_difference_mean_s':float(np.mean(delays)),
                                     'arrival_difference_p99_s':float(np.quantile(delays,.99)),
                                     'arrival_difference_max_s':max(delays),
                                     'meaning':'ROS callback arrival minus native callback arrival, not sensor-to-EKF latency'}
    valid=[r for r in samples if r['status']=='SHADOW_VALID']
    pt=[];gp=[]
    for r in poses:
        p=next(p['p'] for p in r['poses'] if p['name']=='iris_with_gimbal')
        pt.append(r['t']);gp.append([p[1],p[0],-p[2]])
    pt=np.array(pt);gp=np.array(gp)
    config=json.loads((root/'configuration.json').read_text())
    gp[:,2]+=float(config.get('ground_z',0))
    vgt=np.gradient(gp,pt,axis=0)
    segment_metrics=[];all_errors=[];all_verrors=[]
    for segment in sorted({r['segment'] for r in valid}):
        rows=[r for r in valid if r['segment']==segment]
        times=np.array([r['t'] for r in rows]);states=np.array([r['state'] for r in rows])
        gt=np.column_stack([np.interp(times,pt,gp[:,i]) for i in range(3)])
        gv=np.column_stack([np.interp(times,pt,vgt[:,i]) for i in range(3)])
        error=states[:,:3]-gt
        error[:,:2]=(states[:,:2]-states[0,:2])-(gt[:,:2]-gt[0,:2])
        ve=states[:,3:6]-gv
        all_errors.extend(error);all_verrors.extend(ve)
        segment_metrics.append({'segment':segment,'samples':len(rows),'start':float(times[0]),'end':float(times[-1]),
                                'gt_horizontal_displacement_m':float(np.linalg.norm(gt[-1,:2]-gt[0,:2])),
                                'vio_horizontal_displacement_m':float(np.linalg.norm(states[-1,:2]-states[0,:2]))})
    if valid:
        e=np.array(all_errors);ve=np.array(all_verrors)
        report['evaluation']={'scope':'valid samples only, each reset segment independently translation-aligned in XY',
                              'coverage':len(valid)/len(samples),'segments':segment_metrics,
                              'trajectory_rmse_3d_m':float(np.sqrt(np.mean(np.sum(e*e,axis=1)))),
                              'trajectory_rmse_horizontal_m':float(np.sqrt(np.mean(np.sum(e[:,:2]**2,axis=1)))),
                              'trajectory_max_error_3d_m':float(np.linalg.norm(e,axis=1).max()),
                              'velocity_rmse_3d_m_s':float(np.sqrt(np.mean(np.sum(ve*ve,axis=1)))),
                              'velocity_rmse_horizontal_m_s':float(np.sqrt(np.mean(np.sum(ve[:,:2]**2,axis=1)))),
                              'velocity_max_error_m_s':float(np.linalg.norm(ve,axis=1).max()),
                              'accepted_flow':sum(r.get('flow_accepted',False) for r in samples),
                              'rejected_flow':sum(r.get('flow_accepted') is False for r in samples),
                              'covariance_min_eigenvalue':min(r['covariance_min_eigenvalue'] for r in valid),
                              'covariance_max_asymmetry':max(r['covariance_max_asymmetry'] for r in valid),
                              'covariance_max_eigenvalue':max(float(np.linalg.eigvalsh(r['covariance']).max()) for r in valid),
                              'covariance_trace_min':min(float(np.trace(r['covariance'])) for r in valid),
                              'covariance_trace_max':max(float(np.trace(r['covariance'])) for r in valid),
                              'no_flow_samples':sum('flow_v_xy' not in r for r in valid),
                              'replay_processing_p99_s':float(np.quantile([r['replay_processing_wall_s'] for r in valid],.99))}
    report['gt_motion']={'horizontal_extent_m':float(np.max(np.linalg.norm(gp[:,:2]-gp[0,:2],axis=1))),
                          'max_horizontal_speed_m_s':float(np.linalg.norm(vgt[:,:2],axis=1).max())}
    # Declared local validation thresholds, not a project-wide performance proof.
    ev=report.get('evaluation',{})
    report['motion_validation']='PASS' if (summary['status']=='PASS' and report['gt_motion']['horizontal_extent_m']>=.2
        and ev.get('coverage',0)>=.95 and ev.get('trajectory_rmse_horizontal_m',1e9)<.2
        and ev.get('velocity_rmse_horizontal_m_s',1e9)<.15) else 'FAIL'
    report['criteria']={'min_gt_extent_m':.2,'min_coverage':.95,'max_horizontal_rmse_m':.2,'max_horizontal_velocity_rmse_m_s':.15,
                        'no_rejected_samples_or_recovery_in_validated_run':True}
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);args=p.parse_args()
    print(json.dumps(analyze(args.run),indent=2,allow_nan=False))
