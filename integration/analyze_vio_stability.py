"""Full-trial metrics, shadow checks and empirical covariance diagnostics.

GT XY is used only here, after capture. No sensor or flight commands.
"""
import argparse
import json
from pathlib import Path


def verified_reliable(endpoints):
    # RMW reliability id 1 is RELIABLE; history/depth may be unknown in discovery.
    return bool(endpoints) and all(e.get('reliability') in ('1','RELIABLE','ReliabilityPolicy.RELIABLE') for e in endpoints)


def full_trajectory_valid(rows, recovery_events, segments):
    first=next((i for i,r in enumerate(rows) if r['status']=='SHADOW_VALID'),len(rows))
    return (first<len(rows) and not recovery_events and segments==1
            and all(r['status']=='WARMUP' for r in rows[:first])
            and all(r['status']=='SHADOW_VALID' for r in rows[first:]))


def analyze_suite(root):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from analyze_vio_run import analyze
    criteria=json.loads((root/'criteria.json').read_text())
    results=[]
    for index in range(1,4):
        trial=root/f'trial_{index}';capture=trial/'capture'
        metrics=analyze(capture)
        summary=json.loads((capture/'summary.json').read_text())
        rows=[json.loads(x) for x in (capture/'samples.jsonl').read_text().splitlines()]
        valid=[r for r in rows if r['status']=='SHADOW_VALID']
        first_valid=next((i for i,r in enumerate(rows) if r['status']=='SHADOW_VALID'),len(rows))
        uninterrupted=all(r['status']=='SHADOW_VALID' for r in rows[first_valid:])
        complete=full_trajectory_valid(rows,summary['recovery_events'],summary['segments'])
        poses=[json.loads(x) for x in (capture/'poses.jsonl').read_text().splitlines()]
        pt=np.array([r['t'] for r in poses]);gp=np.array([next(p['p'] for p in r['poses'] if p['name']=='iris_with_gimbal') for r in poses])[:,[1,0,2]];gp[:,2]*=-1
        gp[:,2]+=.001
        gv=np.gradient(gp,pt,axis=0)
        t=np.array([r['t'] for r in valid]);states=np.array([r['state'] for r in valid]);cov=np.array([r['covariance'] for r in valid])
        gt=np.column_stack([np.interp(t,pt,gp[:,j]) for j in range(3)])
        velocity=np.column_stack([np.interp(t,pt,gv[:,j]) for j in range(3)])
        error=states[:,:3]-gt;error[:,:2]=(states[:,:2]-states[0,:2])-(gt[:,:2]-gt[0,:2])
        ve=states[:,3:6]-velocity
        pn=np.array([float(e@np.linalg.solve(p[:2,:2],e)) for e,p in zip(error[:,:2],cov)])
        vn=np.array([float(e@np.linalg.solve(p[3:5,3:5],e)) for e,p in zip(ve[:,:2],cov)])
        confidence={'position_xy_nees_mean':float(pn.mean()),'velocity_xy_nees_mean':float(vn.mean()),
                    'position_xy_95ellipse_coverage':float(np.mean(pn<=5.991464547)),
                    'velocity_xy_95ellipse_coverage':float(np.mean(vn<=5.991464547)),
                    'position_2sigma_marginal_coverage':np.mean(np.abs(error)<=2*np.sqrt(np.diagonal(cov,axis1=1,axis2=2)[:,:3]),axis=0).tolist(),
                    'velocity_2sigma_marginal_coverage':np.mean(np.abs(ve)<=2*np.sqrt(np.diagonal(cov,axis1=1,axis2=2)[:,3:6]),axis=0).tolist(),
                    'interpretation':'Empirical only; correlated samples, GT-assisted Z and initial XY translation alignment. Not a formal independent NEES calibration test.',
                    'overconservative_indication':bool(pn.mean()<.05 or vn.mean()<.05)}
        shadow=[]
        for j,r in enumerate(valid):
            if 'flow_v_xy' in r and abs(velocity[j,2])>.08 and np.linalg.norm(velocity[j,:2])<.04:
                f=np.array(r['flow_v_xy']);prediction=np.array([velocity[j,2]/9,-5*velocity[j,2]/9])
                actual_error=float(np.linalg.norm(f-velocity[j,:2]));shadow_error=float(np.linalg.norm(f-prediction))
                shadow.append({'t':float(t[j]),'flow_error_m_s':actual_error,'shadow_model_error_m_s':shadow_error,
                               'candidate':bool(actual_error>.15 and shadow_error<actual_error)})
        ev=metrics['evaluation'];ros=metrics['imu_ros'];native=metrics['imu_native']
        endpoints=metrics['capture_meta']['qos'].get('publisher_endpoints',[])
        reliable=verified_reliable(endpoints)
        failures=[]
        checks={'continuous_full_trajectory':complete,
                'no_critical_ros_gap':ros['max_sim_gap_s']<=criteria['max_imu_gap_s']+1e-9,
                'no_critical_native_gap':native['max_sim_gap_s']<=criteria['max_imu_gap_s']+1e-9,
                'reliable_endpoint_verified':reliable,
                'runner_checks':summary['status']=='PASS',
                'coverage':ev['coverage']>=criteria['min_coverage'],
                'trajectory_rmse':ev['trajectory_rmse_horizontal_m']<=criteria['max_horizontal_trajectory_rmse_m'],
                'velocity_rmse':ev['velocity_rmse_horizontal_m_s']<=criteria['max_horizontal_velocity_rmse_m_s'],
                'max_spatial_error':ev['trajectory_max_error_3d_m']<=criteria['max_spatial_error_m'],
                'actual_movement':metrics['gt_motion']['horizontal_extent_m']>=criteria['min_gt_extent_m'],
                'flow_fraction':ev['accepted_flow']/len(valid)>=criteria['min_flow_fraction'],
                'covariance_error_coverage':min(confidence['position_xy_95ellipse_coverage'],confidence['velocity_xy_95ellipse_coverage'])>=criteria['min_position_velocity_95ellipse_coverage'],
                'no_observed_shadow_candidates':not any(r['candidate'] for r in shadow)}
        failures=[k for k,v in checks.items() if not v]
        flight=[json.loads(x) for x in (trial/'flight.jsonl').read_text().splitlines()]
        landed=any(r.get('event')=='landed_disarmed_confirmed' for r in flight)
        if not landed:failures.append('landed_disarmed_not_confirmed')
        load=json.loads((capture/'process_load.json').read_text());cpu=[]
        for a,b in zip(load,load[1:]):
            dt=b['arrival_monotonic_s']-a['arrival_monotonic_s'];old={r['pid']:r for r in a['cpu_ticks']}
            for r in b['cpu_ticks']:
                if r['pid'] in old:
                    before=old[r['pid']];pct=100*(r['utime']+r['stime']-before['utime']-before['stime'])/b['clock_ticks_per_second']/dt
                    cpu.append({'pid':r['pid'],'cpu_percent':pct})
        predeclared_failures=[f for f in failures if f!='no_observed_shadow_candidates']
        result={'trial':index,'status':'FAIL' if failures else 'PASS',
                'predeclared_motion_status':'FAIL' if predeclared_failures else 'PASS',
                'predeclared_motion_failures':predeclared_failures,
                'release_status':'HOLD' if failures else 'PASS',
                'extra_safety_veto':'shadow_candidate' if 'no_observed_shadow_candidates' in failures else None,
                'failures':failures,'checks':checks,
                'full_capture_wall_s':json.loads((trial/'orchestration.json').read_text())['total_wall_s'],
                'simulation_duration_s':float(t[-1]-t[0]),'frames':len(rows),'valid_frames':len(valid),
                'full_trajectory_metrics_valid':complete,'metrics':metrics,'confidence':confidence,
                'shadow':{'checked_samples':len(shadow),'candidates':sum(r['candidate'] for r in shadow),'samples':shadow,
                          'meaning':'No candidates is evidence only for this fixture/trajectory, not a general shadow detector'},
                'cpu_load':{'max_per_process_percent':max((r['cpu_percent'] for r in cpu),default=None),
                            'meaning':'100 percent equals one core; raw PID ticks/ps in process_load.json'},
                'landed_disarmed':landed,'publication_enabled':False}
        results.append(result)
        (trial/'reviewed_stability_metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        fig,axes=plt.subplots(3,2,figsize=(12,10),sharex=True)
        relative=t-t[0]
        for j in range(2):
            axes[0,j].plot(relative,error[:,j],label='position error');axes[0,j].fill_between(relative,-2*np.sqrt(cov[:,j,j]),2*np.sqrt(cov[:,j,j]),alpha=.25,label='2 sigma')
            axes[1,j].plot(relative,ve[:,j],label='velocity error');axes[1,j].fill_between(relative,-2*np.sqrt(cov[:,j+3,j+3]),2*np.sqrt(cov[:,j+3,j+3]),alpha=.25,label='2 sigma')
        axes[2,0].plot(relative,pn,label='position NEES');axes[2,1].plot(relative,vn,label='velocity NEES')
        for ax in axes[2]:ax.axhline(5.991464547,color='r',label='2D 95% ellipse')
        for ax in axes.flat:ax.grid();ax.legend();ax.set_xlabel('simulation seconds')
        fig.tight_layout();fig.savefig(trial/'covariance_errors.png',dpi=130);plt.close(fig)
    result={'status':'PASS' if all(r['status']=='PASS' for r in results) else 'FAIL',
            'predeclared_motion_status':'PASS' if all(r['predeclared_motion_status']=='PASS' for r in results) else 'FAIL',
            'release_status':'HOLD' if any(r['release_status']=='HOLD' for r in results) else 'PASS',
            'criteria':criteria,'trials':results,'publication_enabled':False}
    (root/'reviewed_stability_summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':result['status'],'trials':[{'trial':r['trial'],'status':r['status'],'failures':r['failures'],
               'trajectory_rmse':r['metrics']['evaluation']['trajectory_rmse_horizontal_m'],
               'velocity_rmse':r['metrics']['evaluation']['velocity_rmse_horizontal_m_s']} for r in results]},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('suite',type=Path);a=p.parse_args();analyze_suite(a.suite)
