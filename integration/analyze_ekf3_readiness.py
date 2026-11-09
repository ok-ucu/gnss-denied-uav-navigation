"""Offline readiness diagnostics on complete recorded real motion runs.

GT is used here only for independent evaluation. Does not change covariance,
filter estimates, quality thresholds, flight parameters or historical evidence.
"""
import argparse
from collections import Counter
import json
from pathlib import Path


def covariance_diagnostic(errors,covariances):
    import numpy as np
    e=np.asarray(errors,float);p=np.asarray(covariances,float)
    if e.ndim!=2 or e.shape[1]!=2 or p.shape!=(len(e),2,2) or not len(e):
        raise ValueError('Expected nonempty Nx2 errors and Nx2x2 covariance')
    if not np.isfinite(e).all() or not np.isfinite(p).all():raise ValueError('Nonfinite diagnostics')
    if not np.allclose(p,np.swapaxes(p,1,2)) or np.linalg.eigvalsh(p).min()<=0:
        raise ValueError('Expected symmetric positive definite covariance for NEES')
    nis=np.array([x@np.linalg.solve(c,x) for x,c in zip(e,p)])
    # Diagnostic only: successive state errors are not independent experiments.
    corr=[]
    for i in range(2):
        v=e[:,i];corr.append(float(np.corrcoef(v[:-1],v[1:])[0,1]) if len(v)>2 and v.std()>1e-12 else None)
    return dict(samples=len(e),nees_mean=float(nis.mean()),expected_independent_gaussian_mean=2,
        ellipse50_coverage=float(np.mean(nis<=1.38629436112)),ellipse95_coverage=float(np.mean(nis<=5.991464547)),
        axis_rmse=np.sqrt(np.mean(e*e,axis=0)).tolist(),rms_reported_sigma=np.sqrt(np.mean(np.diagonal(p,axis1=1,axis2=2),axis=0)).tolist(),
        lag1_error_correlation=corr,calibration='NOT VERIFIED: correlated samples, GT/initial alignment limitations; no covariance retuning')


def analyze(root):
    import numpy as np
    def read(p):return [json.loads(s) for s in p.read_text().splitlines()]
    run=root/'source_trial';stream=run/'source/stream';rows=read(stream/'stream.jsonl');states=[r for r in rows if 'state' in r]
    native=read(stream/'native_camera.jsonl');ros=read(stream/'ros_camera.jsonl');clocks=read(stream/'clocks.jsonl')
    cm=np.array([c['arrival_monotonic_s'] for c in clocks]);ct=np.array([c['t'] for c in clocks]);nm={round(r['t']*1e9):r for r in native}
    native_age=[];callback_delta=[];conversion=[]
    for r in ros:
        ns=round(r['t']*1e9)
        if ns in nm:
            n=nm[ns];callback_delta.append(r['arrival_monotonic_s']-n['arrival_monotonic_s'])
            i=np.searchsorted(cm,n['arrival_monotonic_s'],side='right')-1
            if i>=0 and n['arrival_monotonic_s']-cm[i]<=.02:native_age.append(ct[i]-n['t'])
        conversion.append(r['callback_end_monotonic_s']-r['arrival_monotonic_s'])
    def stats(v):return dict(mean=float(np.mean(v)),p95=float(np.percentile(v,95)),max=float(max(v)),min=float(min(v))) if v else None
    camera_timing=dict(native_receipt_header_age_sim_s=stats(native_age),native_to_ros_receipt_wall_s=stats(callback_delta),ros_callback_conversion_wall_s=stats(conversion),note='Independent receipt witnesses; negative receipt ordering possible; no wall acquisition timestamp')
    profile=json.loads((run/'metrics.json').read_text());evaluation=json.loads((run/'source/metrics.json').read_text())
    poses=read(stream/'poses.jsonl');pt=np.array([p['t'] for p in poses]);gp=np.array([next(p['p'] for p in r['poses'] if p['name']=='iris_with_gimbal') for r in poses])
    gp=gp[:,[1,0,2]];gp[:,2]*=-1;gv=np.gradient(gp,pt,axis=0)
    t=np.array([r['t'] for r in states]);x=np.array([r['state'] for r in states]);p=np.array([r['covariance'] for r in states])
    gt=np.column_stack([np.interp(t,pt,gp[:,i]) for i in range(3)]);vel=np.column_stack([np.interp(t,pt,gv[:,i]) for i in range(3)])
    pe=(x[:,:2]-x[0,:2])-(gt[:,:2]-gt[0,:2]);ve=x[:,3:5]-vel[:,:2]
    phase=np.where(-gt[:,2]<.3,'ground',np.where(abs(vel[:,2])>.08,'vertical',np.where(np.linalg.norm(vel[:,:2],axis=1)>.04,'horizontal','hover')))
    phases={}
    for name in sorted(set(phase)):
        ix=phase==name;subset=[r for i,r in enumerate(states) if ix[i]]
        phases[name]=dict(samples=int(ix.sum()),xy_rmse=float(np.sqrt(np.mean(np.sum(pe[ix]**2,axis=1)))),
             accepted=sum(r['flow_accepted'] for r in subset),rejected_audit=sum(r['frontend_available'] and not r['flow_accepted'] for r in subset),
             no_frontend=sum(not r['frontend_available'] for r in subset),
             audit_reasons=Counter(r['audit']['reason'] for r in subset if r['frontend_available'] and not r['flow_accepted']))
    errors=Counter(r['error'] for r in rows if 'error' in r)
    result=dict(input=str(root),camera=profile['camera'],flow=profile['flow'],source_sim_hz=profile['output_sim_hz'],
        source_count=profile['source_count'],frame_count=len(rows),state_count=len(states),publication_frame_fraction=profile['source_count']/len(rows),
        acquisition_age_sim_s=dict(mean=float(np.mean([r['source_age'] for r in states])),max=float(max(r['source_age'] for r in states))),
        camera_timing=camera_timing,callback_and_processing=profile['stage_wall_s'],latency=profile['latency'],errors=errors,
        transitions=Counter(r['reason'] for r in json.loads((stream/'gate_transitions.json').read_text())),phases=phases,
        position_covariance=covariance_diagnostic(pe,p[:,:2,:2]),velocity_covariance=covariance_diagnostic(ve,p[:,3:5,3:5]),
        historical_motion_evaluation=evaluation['evaluation'],
        rate_verdict='FAIL: observed output <10 Hz' if profile['output_sim_hz']<10 else 'PASS',
        ceiling='Native 10Hz gives no margin for real rejections. No evidence of stable >=10Hz; >10 requires a genuine faster sensor/configuration.')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps([analyze(r) for r in a.runs],indent=2,allow_nan=False)+'\n')
