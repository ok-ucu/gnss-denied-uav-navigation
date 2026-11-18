"""Offline evidence preflight, never a sender or approval service.

PASS means supplied live evidence satisfies this review contract. It does not
initialize a reference, open a route or prove claims beyond cited artifacts.
"""
import argparse
import hashlib
import json
from pathlib import Path

REQUIRED=('clock_mapping','pause_resume','clock_lifecycle','reference','physics',
          'camera_imu_alignment','quality_epoch_bridge','durable_process_restart',
          'patched_mavros_lifecycle','freshness_faults','covariance_delay')


def evaluate(evidence,base):
    checks=[]
    for name in REQUIRED:
        entry=evidence.get(name,{})
        reasons=[]
        if entry.get('status')!='PASS':reasons.append(entry.get('reason','missing or unpassed evidence'))
        if entry.get('scope')!='live':reasons.append('live-runtime evidence required; fixtures/source review are insufficient')
        artifacts=entry.get('artifacts',[])
        if not artifacts:reasons.append('no hashed evidence artifacts')
        for artifact in artifacts:
            path=(Path(base)/artifact.get('path','')).resolve()
            expected=artifact.get('sha256')
            if not path.is_file() or not isinstance(expected,str) or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
                reasons.append('missing/changed artifact: '+artifact.get('path',''))
        if name=='clock_mapping' and entry.get('uncertainty_bound_verified') is not True:
            reasons.append('arrival regression is not a verified transport/time uncertainty bound')
        if name=='physics' and entry.get('same_physical_state') is not True:
            reasons.append('independent quad physics is incompatible with moving Gazebo VIO')
        checks.append(dict(check=name,status='HOLD' if reasons else 'PASS',reasons=reasons))
    return dict(decision='PASS' if all(c['status']=='PASS' for c in checks) else 'HOLD',
                checks=checks,execution_allowed=False,forwarding_enabled=False,
                scope='Evidence review only; operator initialization and independent transport interlock still required')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=evaluate(json.loads(args.evidence.read_text()),args.evidence.parent)
    with args.output.open('x') as out:json.dump(result,out,indent=2);out.write('\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['decision']=='PASS' else 2)
