"""Own-stack validation only. No arm, takeoff, flight or vision FCU forwarding.

Requires newly created phase1.compose.yaml stack and successful measured mount.
Existing containers are read only; all exec/stop targets use a fixed allowlist.
"""
import json
import argparse
import os
from pathlib import Path
import subprocess
import time
from launch_phase1 import validate_run_id

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run',type=Path,required=True)
parser.add_argument('--run-id',required=True)
args=parser.parse_args()
RUN=args.run.resolve()
if not RUN.is_relative_to(Path(__file__).resolve().parent/'phase1_runs'):
    raise ValueError('Require an owned Phase1 run')
validate_run_id(args.run_id)
NAMES={kind:f'gnss-p1-{kind}-{args.run_id}' for kind in ('physics','fcu','vio','transport')}
PARTITION='gnss-p1-integrated-'+args.run_id
audit=(RUN/'evidence/commands.jsonl').open('x')
children=[];handles=[]

def call(args,check=True,timeout=30):
    if args[0]!='docker':raise ValueError('Only audited Docker commands')
    r=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
    audit.write(json.dumps(dict(utc_ns=time.time_ns(),host_clock='macOS host',args=args,
                               returncode=r.returncode,stdout=r.stdout,stderr=r.stderr))+'\n');audit.flush()
    if check and r.returncode:raise RuntimeError(str(args)+r.stderr+r.stdout)
    return r

def own_exec(kind,shell,timeout=30,check=True):
    return call(['docker','exec',NAMES[kind],'bash','-lc',shell],check,timeout)

def start(label,kind,shell):
    path=RUN/'evidence'/(label+'.console.txt');f=path.open('x');handles.append(f)
    args=['docker','exec',NAMES[kind],'bash','-lc',shell]
    child=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
    children.append((label,child));audit.write(json.dumps(dict(event='start',label=label,args=args,utc_ns=time.time_ns()))+'\n');audit.flush()
    return child

def stop_pid(kind,path):
    pid=int((RUN/path).read_text())
    if pid<=1:raise ValueError('Invalid owned process PID')
    own_exec(kind,'kill -TERM '+str(pid))

def inventory():
    names=call(['docker','ps','--format','{{.Names}}']).stdout.splitlines()
    names=[n for n in names if n not in NAMES.values()]
    fmt='{"name":{{json .Name}},"id":{{json .Id}},"start":{{json .State.StartedAt}},"image":{{json .Image}},"network":{{json .HostConfig.NetworkMode}}}'
    return [json.loads(s) for s in call(['docker','inspect','--format',fmt,*names]).stdout.splitlines()]

before=inventory();(RUN/'evidence/existing_before.json').write_text(json.dumps(before,indent=2))
ros='source /opt/ros/jazzy/setup.bash; '
patched=ros+'source /tmp/patch_install/setup.bash; '
camera='/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
fixture=False
try:
    mount=json.loads((RUN/'evidence/integrated/summary.json').read_text())
    if mount['mount_status']!='PASS' or max(mount['max_angles_deg'])>1:raise RuntimeError('Mount HOLD')
    own_exec('vio',ros+'python3 -B /opt/integration/vio_ground_fixture.py create --directory /run/integration/artifacts/fixture --size-m 1 --tile-size-m .02')
    fixture=True
    start('camera_bridge','vio',ros+f'exec ros2 run ros_gz_bridge parameter_bridge {camera}@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r __node:=phase1_camera_bridge')
    start('mavros_1','transport',patched+'echo $$ > /run/integration/state/mavros_1.pid; exec /opt/ros/jazzy/lib/mavros/mavros_node --ros-args --params-file /opt/integration/phase1_capture_mavros.yaml')
    gate=start('transport_1','transport',patched+'exec python3 -B /opt/integration/phase1_transport.py --run /run/integration --duration 150 --output-name transport_1')
    start('observer','vio',ros+'exec python3 -B /opt/integration/observe_frame_bridge.py --directory /run/integration/evidence/observer --duration 125 --output-topic /integration/phase1/odometry_enu')
    time.sleep(3)
    source=start('vio','vio',ros+'exec python3 -B /opt/integration/run_vio_stream.py --vio-dir /opt/vio --output-dir /run/integration/evidence/vio --duration 110 --ground-z .001 --mount-calibration /run/integration/artifacts/mount_calibration.json --profile --async-diagnostics --reliable-camera --min-odom-subscribers 2 --min-epoch-subscribers 1 --faults --fault-offset 60')
    beginning=time.monotonic();done=set()
    while source.poll() is None:
        elapsed=time.monotonic()-beginning
        if elapsed>145:raise RuntimeError('Producer did not stop within bounded duration')
        if elapsed>=60 and 'mavros' not in done:
            stop_pid('transport',Path('state/mavros_1.pid'))
            time.sleep(2)
            start('mavros_2','transport',patched+'echo $$ > /run/integration/state/mavros_2.pid; exec /opt/ros/jazzy/lib/mavros/mavros_node --ros-args --params-file /opt/integration/phase1_capture_mavros.yaml')
            done.add('mavros')
        if elapsed>=65 and 'guard' not in done:
            stop_pid('transport',Path('evidence/transport_1/pid'))
            gate.wait(timeout=12)
            start('transport_2','transport',patched+'exec python3 -B /opt/integration/phase1_transport.py --run /run/integration --duration 65 --output-name transport_2')
            done.add('guard')
        if elapsed>=82 and 'pause' not in done:
            own_exec('physics',ros+'python3 -B /opt/integration/phase1_control.py pause --expected-partition '+PARTITION+' --output /run/integration/evidence/pause.json')
            time.sleep(1)
            own_exec('physics',ros+'python3 -B /opt/integration/phase1_control.py resume --expected-partition '+PARTITION+' --output /run/integration/evidence/resume.json')
            done.add('pause')
        if elapsed>=100 and 'reset' not in done:
            own_exec('physics',ros+'python3 -B /opt/integration/phase1_control.py reset --expected-partition '+PARTITION+' --output /run/integration/evidence/reset.json')
            done.add('reset')
        time.sleep(.1)
    if source.returncode:raise RuntimeError('VIO producer failed; preserve unsuccessful run')
    # Graceful stop only our transport so the durable state is explicitly closed.
    if (RUN/'evidence/transport_2/pid').exists():stop_pid('transport',Path('evidence/transport_2/pid'))
    for label,child in children:
        if label in ('transport_1','transport_2','vio'):child.wait(timeout=15)
finally:
    if fixture:
        own_exec('vio',ros+'python3 -B /opt/integration/vio_ground_fixture.py remove --directory /run/integration/artifacts/fixture',check=False)
    # Capture logs/readback before stopping only newly owned containers.
    for kind,name in NAMES.items():
        r=call(['docker','logs',name],check=False)
        with (RUN/'evidence'/(kind+'_container.console.txt')).open('x') as out:out.write(r.stdout+r.stderr)
    for kind in ('transport','vio','fcu','physics'):
        call(['docker','stop','--time','8',NAMES[kind]],check=False)
    for label,child in children:
        try:child.wait(timeout=12)
        except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
    for f in handles:f.close()
    after=inventory();(RUN/'evidence/existing_after.json').write_text(json.dumps(after,indent=2))
    with (RUN/'evidence/isolation.json').open('x') as out:json.dump(dict(existing_services_unchanged=before==after,
        execution_allowed=False,forwarding_enabled=False),out,indent=2)
    audit.close()
