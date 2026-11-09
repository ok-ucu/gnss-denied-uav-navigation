"""Generate a review-only plan. No sockets, processes or parameter application."""
import argparse
import json
from pathlib import Path


def evaluate(results,reset):
    required={'plus','minus','zero','jitter','drift','epoch','reset'}
    return (required.issubset({r['case'] for r in results}) and all(r['timestamp_pass'] and r['offset_pass'] for r in results)
            and reset.get('blocked_pairs',0)>0 and reset.get('wire_leaks')==0 and reset.get('explicit_reinitializations')==1)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--reset-summary',type=Path,required=True)
    a=p.parse_args();results=json.loads(a.results.read_text())
    passed=evaluate(results,json.loads(a.reset_summary.read_text()))
    result=dict(decision='CONDITIONAL GO: isolated DISARMED receipt/time/freshness diagnostic only' if passed else 'HOLD',
        execution_allowed=False,working_sitl_target_allowed=False,
        scope='Plan only; no FCU process or ExternalNav was started by this program',
        required=['Unique disposable ArduCopter4.4.4 instance, network=none, same-container loopback MAVROS only',
                  'Verify patched factory loaded and source/binary hashes; keep system sys_time excluded',
                  'Review candidate ekf3_dry_run.parm, read back all actual noise/delay/default parameters after reboot',
                  'Explicit separate FCU/reference identity, disarmed readback, clock/reference origin and durable ResetSafety approval',
                  'One pose_cov and one world-ENU speed_twist_cov path; no duplicate routes; independent packet/DF witness',
                  'Fusion acceptance requires coherent FCU IMU and VIO frames/trajectory; recorded moving VIO versus stationary FCU is not a valid navigation experiment',
                  'Enable LOG_DISARMED=1 only on disposable FCU; preserve DataFlash logs and inspect actual log schema',
                  'No arm/takeoff; record VISP/VISV and actual EKF source/fusion/innovation evidence; reception alone is not fusion',
                  'Treat >=300ms receipt gaps as expected unhealthy/dropout diagnostic, not continuous navigation',
                  'On reset STOP forwarding, persist latch, never auto-increment/reset_counter or replay; new FCU/reference init approval required',
                  'Rollback closes sender first; archive logs; restore saved baseline only on disposable FCU; verify disarmed'],
        rate=dict(documented_guidance_hz=4,observed_nominal_hz=8,project_target_hz='10-20, unchanged separate requirement',
                  maximum_observed_gaps_s=[3.3,1.8],continuous_navigation='HOLD: gaps exceed freshness thresholds'),
        covariance='Conservative, not formally calibrated; record actual VISO noise values, no flight readiness claimed',
        flight_or_working_fcu='HOLD; not authorized by this dry-run')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
