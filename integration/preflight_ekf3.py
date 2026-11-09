"""Read-only dry-run readiness report. NEVER opens MAVLink or applies parameters."""
import argparse
import json
import math
from pathlib import Path

EXPECTED={'AHRS_EKF_TYPE':3,'EK3_ENABLE':1,'EK3_SRC_OPTIONS':0,'EK3_SRC1_POSXY':6,'EK3_SRC1_VELXY':6,
          'EK3_SRC1_POSZ':1,'EK3_SRC1_VELZ':0,'EK3_SRC1_YAW':1,'VISO_TYPE':1,'VISO_SCALE':1,
          'VISO_ORIENT':0,'VISO_POS_X':0,'VISO_POS_Y':0,'VISO_POS_Z':0,'GPS_TYPE':0,'GPS_TYPE2':0}
PORTS={'vio':14551,'monitor':14552,'motion':14553,'mavros':14555,'isolated_receiver':14590,'isolated_mavros':14591}


def parse_profile(text):
    result={}
    for line in text.splitlines():
        line=line.split('#')[0].strip()
        if not line:continue
        name,value=line.split()
        value=float(value)
        if name in result or not math.isfinite(value):raise ValueError('Duplicate/nonfinite parameter')
        result[name]=value
    return result


def check_profile(profile,ports):
    errors=[f'{k}: expected {v}' for k,v in EXPECTED.items() if profile.get(k)!=v]
    if 'GPS1_TYPE' in profile:errors.append('GPS1_TYPE is not the 4.4.4 GPS_TYPE parameter')
    if set(ports)!=set(PORTS) or len(set(ports.values()))!=len(ports):errors.append('Missing/colliding UDP roles')
    if any(type(p) is not int or not 1<=p<=65535 for p in ports.values()):errors.append('Invalid UDP port')
    return errors


def motion_checks(diagnostic):
    e=diagnostic['historical_motion_evaluation']
    checks={'rate':diagnostic['source_sim_hz']>=10,'accepted_flow':e['accepted_flow_fraction']>=.95,
            'state_coverage':e['state_coverage']>=.98,'xy_accuracy':e['xy_rmse_m']<=.2,
            'velocity_accuracy':e['velocity_xy_rmse_m_s']<=.15,'max_error':e['max_spatial_error_m']<=.4,
            'no_reset':e['full_trajectory_no_reset']}
    return {k:'PASS' if v else 'FAIL' for k,v in checks.items()}


def assess_offset(statuses,peer_offset_ns):
    return 'PASS' if statuses and all(abs(s['observed_offset_ns']+peer_offset_ns)<=20_000_000 for s in statuses) else 'FAIL'


def build(profile,diagnostics,clock_reports=()):
    errors=check_profile(profile,PORTS)
    return dict(profile_structure='FAIL' if errors else 'PASS',profile_errors=errors,
        isolated_clock_controls=[dict(mode=r['mode'],result='NOT TESTED' if r['mode']=='NONE' else assess_offset(r['timesync_statuses'],r.get('peer_offset_ns',1_000_000_000))) for r in clock_reports],
        target='ArduCopter4.4.4 review-only horizontal ExternalNav candidate; NOT applied',ports=PORTS,
        ports_status='PLAN ONLY: live routing not configured or tested',
        motion=[dict(input=d['input'],checks=motion_checks(d)) for d in diagnostics],
        covariance_calibration='NOT VERIFIED',actual_fcu_clock_and_fusion='NOT TESTED by this dry-run',
        delay_and_noise_values='NOT APPROVED: VISO_DELAY_MS, VISO_VEL_M_NSE, VISO_POS_M_NSE, VISO_YAW_M_NSE',
        fcu_origin_reset_rollback='NOT TESTED: operator-controlled disposable FCU experiment required',
        decision='HOLD',forwarding_allowed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--clock-report',type=Path,action='append',default=[])
    p.add_argument('--diagnostics',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=build(parse_profile(a.profile.read_text()),json.loads(a.diagnostics.read_text()),[json.loads(p.read_text()) for p in a.clock_report])
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    raise SystemExit(2 if result['decision']=='HOLD' else 0)
