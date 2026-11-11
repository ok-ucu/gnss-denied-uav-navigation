"""DISARMED profiling/fault cases on the existing phase1.compose.yaml stack.

Only own processes in four exact named services. No FCU vision route exists.
Each case has an explicit NEW capture reference; reset/restart within a case
cannot auto-authorize. Faults use real VIO with labelled local input injection.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time
import uuid
import re
import math

from launch_phase1 import validate_run_id


def fault_write_code(path,row):
    """Same synchronous fsync/atomic replace for either owned storage location."""
    return ('import json,os,time; from pathlib import Path; '
            'p=Path('+repr(path)+'); '
            'q=p.with_name("fault-'+row['generation']+'.writing"); '
            'f=q.open("x"); f.write('+repr(json.dumps(row))+'); f.flush(); os.fsync(f.fileno()); f.close(); '
            'q.replace(p); print(time.monotonic_ns())')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--run-id',type=validate_run_id,required=True)
    parser.add_argument('--phase',choices=['before','after'],required=True)
    parser.add_argument('--resource-suffix',choices=['','fine'],default='',help='One-off services on the SAME physics/FCU namespace')
    parser.add_argument('--transport-suffix',choices=['','store'],default='',help='Owned one-off transport from the SAME Compose stack; retain original container')
    parser.add_argument('--tag',help='NEW evidence basename for a repeated attempt')
    parser.add_argument('--cases',nargs='+',choices=['normal','camera_loss','imu_loss','tracking_loss','bad_covariance','stale','process_restart','mavros_reconnect','pause_resume','clock_reset'],help='Run only the cases needed for a specific regression')
    parser.add_argument('--normal-duration',type=float,default=35,help='Bounded normal observation window, wall seconds (10..120)')
    parser.add_argument('--camera-trace-library',help='Opt-in passive native timing interposer, absolute /opt/integration path in NEW own camera bridge')
    parser.add_argument('--linux-fault-control',action='store_true',help='Same synchronous fault protocol on the isolated shared Linux volume; no new cache or lease')
    args=parser.parse_args()
    if args.camera_trace_library and not args.camera_trace_library.startswith('/opt/integration/'):
        parser.error('Camera trace library must be under the read-only integration mount')
    if not math.isfinite(args.normal_duration) or not 10<=args.normal_duration<=120:parser.error('Normal duration must be finite, 10..120 seconds')
    prefix=args.tag or args.phase
    if not re.fullmatch('[a-z0-9_]+',prefix):raise ValueError('Invalid evidence tag')
    run=args.run.resolve()
    if not run.is_relative_to(Path(__file__).resolve().parent/'phase1_runs'):
        raise ValueError('Owned run required')
    names={kind:f'gnss-p1-{kind}-{args.run_id}' for kind in ('physics','fcu','vio','transport')}
    if args.resource_suffix:
        for kind in ('vio','transport'):names[kind]+='-'+args.resource_suffix
    if args.transport_suffix:
        if args.resource_suffix:raise ValueError('Choose one transport override')
        names['transport']+='-'+args.transport_suffix
    out=run/'evidence'/(prefix+'_matrix');out.mkdir(exist_ok=False)
    audit=(out/'commands.jsonl').open('x');handles=[];children=[]
    ros='source /opt/ros/jazzy/setup.bash; '
    patched=ros+'source /tmp/patch_install/setup.bash; '
    control_file='/var/lib/phase1-control/latency_fault.json' if args.linux_fault_control else '/run/integration/state/latency_fault.json'
    def command(kind,shell,check=True):
        cmd=['docker','exec',names[kind],'bash','-lc',shell]
        result=subprocess.run(cmd,capture_output=True,text=True,timeout=15)
        audit.write(json.dumps(dict(command=cmd,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr,host_utc_ns=time.time_ns()))+'\n');audit.flush()
        if check and result.returncode:raise RuntimeError(result.stdout+result.stderr)
        return result
    def start(label,kind,shell):
        f=(out/(label+'.console.txt')).open('x');handles.append(f)
        cmd=['docker','exec',names[kind],'bash','-lc',shell]
        process=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
        children.append(process);audit.write(json.dumps(dict(event='start',label=label,command=cmd,host_utc_ns=time.time_ns()))+'\n');audit.flush()
        return process
    def stop_pid(kind,path):
        pid=int(path.read_text())
        if pid<=1:raise ValueError('Invalid owned PID')
        command(kind,'kill -TERM '+str(pid),False)
    def write_fault(path,name):
        row=dict(fault=name,generation=str(uuid.uuid4()),host_utc_ns=time.time_ns())
        # Commit on the Linux side. Host-side rename/size invalidation across the
        # macOS bind mount produced a torn reader view in the retained attempt.
        command('vio','python3 -c '+shlex.quote(fault_write_code(control_file,row)))
        audit.write(json.dumps(dict(event='fault_control',path=control_file,**row))+'\n');audit.flush()
    def read_fault():
        if not args.linux_fault_control:return json.loads(fault_path.read_text())
        # Preserve reading the actual file, including errors; no writer-side cache.
        code='from pathlib import Path; print(Path('+repr(control_file)+').read_text())'
        return json.loads(command('vio','python3 -c '+shlex.quote(code)).stdout)
    def wire_count(label):
        path=run/'evidence'/label/'wire.jsonl'
        if not path.exists():return 0
        return sum(json.loads(line)['valid_capture'] for line in path.read_text().splitlines(keepends=True) if line.endswith('\n'))
    def transport(case,number,initialize,duration):
        label=prefix+'_'+case+'_transport'+str(number)
        extra='' if not initialize else ' --initialize-capture-reference '+str(uuid.uuid4())
        process=start(label,'transport',patched+'exec python3 -B /opt/integration/phase1_transport.py --run /run/integration --output-name '+label+
                      ' --duration '+str(duration)+' --fault-file '+shlex.quote(control_file)+
                      ' --mavros-pid-file /run/integration/state/'+prefix+'_mavros.pid'+extra)
        guard_labels.append(label)
        return label,process
    mavros_number=0
    def mavros():
        nonlocal mavros_number
        mavros_number+=1
        return start('mavros_'+str(mavros_number),'transport',patched+'echo $$ > /run/integration/state/'+prefix+'_mavros.pid; exec /opt/ros/jazzy/lib/mavros/mavros_node --ros-args --params-file /opt/integration/phase1_capture_mavros.yaml')
    fault_path=run/'state/latency_fault.json'
    if fault_path.exists():
        # This is a live control file, not recorded evidence. Prior commands have
        # immutable UUID/UTC audit entries; no experimental recording is replaced.
        write_fault(fault_path,None)
    else:write_fault(fault_path,None)
    results=[];fixture=False;guard_labels=[]
    try:
        mount=json.loads((run/'evidence/integrated/summary.json').read_text())
        if mount['mount_status']!='PASS' or max(mount['max_angles_deg'])>1:raise RuntimeError('Mount HOLD')
        fixture_dir='/run/integration/artifacts/fixture_'+prefix
        command('vio',ros+'python3 -B /opt/integration/vio_ground_fixture.py create --directory '+fixture_dir+' --size-m 1 --tile-size-m .02')
        fixture=True
        camera='/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
        camera_exec='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge '
        if args.camera_trace_library:
            camera_exec='env LD_PRELOAD='+shlex.quote(args.camera_trace_library)+' CAMERA_TRACE_PREFIX=/tmp/phase1_camera_boundary '+camera_exec
        start('camera_bridge','vio',ros+'echo $$ > /run/integration/state/'+prefix+'_camera.pid; exec '+camera_exec+camera+'@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r __node:=phase1_latency_camera_bridge')
        mavros()
        start('observer','vio',ros+'echo $$ > /run/integration/state/'+prefix+'_observer.pid; exec python3 -B /opt/integration/observe_frame_bridge.py --directory /run/integration/evidence/'+prefix+'_matrix/observer --duration 800 --output-topic /integration/phase1/odometry_enu')
        cases=args.cases or (['normal'] if args.phase=='before' else ['normal','camera_loss','imu_loss','tracking_loss','bad_covariance','stale','process_restart','mavros_reconnect','pause_resume','clock_reset'])
        if 'clock_reset' in cases and cases[-1]!='clock_reset':raise ValueError('World reset must be the final case; require a new lifecycle afterwards')
        for case in cases:
            write_fault(fault_path,None)
            duration=args.normal_duration if case=='normal' else 25
            label,guard=transport(case,1,True,duration+18)
            time.sleep(3)
            source_label=prefix+'_'+case+'_vio'
            source=start(source_label,'vio',ros+'exec python3 -B /opt/integration/run_vio_stream.py --vio-dir /opt/vio --output-dir /run/integration/evidence/'+source_label+
                         ' --duration '+str(duration)+' --ground-z .001 --mount-calibration /run/integration/artifacts/mount_calibration.json'+
                         ' --profile --async-diagnostics --reliable-camera --min-odom-subscribers 2 --min-epoch-subscribers 1 --fault-file '+shlex.quote(control_file))
            beginning=time.monotonic();injected=False;positive=0;injection_utc=None;restart_label=None
            while source.poll() is None:
                elapsed=time.monotonic()-beginning
                if elapsed>duration+20:raise RuntimeError('Bounded source timeout')
                count=wire_count(label)
                if case!='normal' and not injected and (count>=10 or elapsed>=15):
                    positive=count;injection_utc=time.time_ns()
                    if case in ('camera_loss','imu_loss','tracking_loss','bad_covariance','stale'):
                        write_fault(fault_path,case)
                    elif case=='process_restart':
                        stop_pid('transport',run/'evidence'/label/'pid');guard.wait(timeout=8)
                        restart_label,guard=transport(case,2,False,25)
                    elif case=='mavros_reconnect':
                        stop_pid('transport',run/'state'/(prefix+'_mavros.pid'))
                        time.sleep(1);mavros()
                    else:
                        action='reset' if case=='clock_reset' else 'pause'
                        target='/run/integration/evidence/'+prefix+'_matrix/'+case+'.json'
                        command('physics',ros+'python3 -B /opt/integration/phase1_control.py '+action+' --expected-partition gnss-p1-integrated-'+args.run_id+' --output '+target)
                        if case=='pause_resume':
                            time.sleep(1)
                            command('physics',ros+'python3 -B /opt/integration/phase1_control.py resume --expected-partition gnss-p1-integrated-'+args.run_id+' --output '+target.replace('.json','_resume.json'))
                    injected=True
                # Restore local sensor input, but never the transport reference.
                if injected and case in ('camera_loss','imu_loss','tracking_loss','bad_covariance','stale') and time.time_ns()-injection_utc>3_000_000_000:
                    if read_fault()['fault'] is not None:write_fault(fault_path,None)
                time.sleep(.1)
            if source.returncode:raise RuntimeError('Source failed; preserve case '+case)
            active=restart_label or label
            if (run/'evidence'/active/'pid').exists():stop_pid('transport',run/'evidence'/active/'pid')
            guard.wait(timeout=10)
            results.append(dict(case=case,source=source_label,transport=label,restarted_transport=restart_label,
                                positive_wire_packets_before_fault=positive,injected=injected,injection_utc_ns=injection_utc,
                                total_valid_wire_packets=wire_count(label),forwarding_enabled=False))
            with (out/'results.json').open('w') as f:json.dump(results,f,indent=2)
            print(json.dumps(results[-1]),flush=True)
    finally:
        write_fault(fault_path,None)
        for label in guard_labels:
            path=run/'evidence'/label/'pid'
            if path.exists():stop_pid('transport',path)
        if fixture:command('vio',ros+'python3 -B /opt/integration/vio_ground_fixture.py remove --directory '+fixture_dir,False)
        for kind,path in [('transport',run/'state'/(prefix+'_mavros.pid')),('vio',run/'state'/(prefix+'_camera.pid')),('vio',run/'state'/(prefix+'_observer.pid'))]:
            if path.exists():stop_pid(kind,path)
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.terminate()
        for f in handles:f.close()
        audit.close()


if __name__=='__main__':main()
