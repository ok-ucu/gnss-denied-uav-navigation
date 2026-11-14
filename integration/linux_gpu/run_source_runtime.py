"""NEW NVIDIA source-only runtime, then optionally the existing bounded probe.

Startup belongs here, never in run_vio_source_probe.sh. No MAVROS/ExternalNav,
flight, navigation changes, reuse, rename, image pull/build or container removal.
Always stop only this Compose project's recorded container IDs in finally.
"""
import argparse
import json
from pathlib import Path, PurePosixPath
import shlex
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from linux_gpu import run_camera, prepare_vio


def recorded_command(evidence, audit, cmd, label, timeout=30):
    """Keep separate streams, including partial output on timeout/launch failure."""
    stdout=label+'.txt';stderr=label+'.stderr.txt'
    record=dict(command=cmd,label=label,host_start=time.monotonic(),exit=None,
                stdout=stdout,stderr=stderr)
    try:
        with (evidence/stdout).open('x') as out, (evidence/stderr).open('x') as err:
            answer=subprocess.run(cmd,stdout=out,stderr=err,timeout=timeout)
            record['exit']=answer.returncode
    except Exception as exc:
        record['error']=type(exc).__name__+': '+str(exc)
        record['timed_out']=isinstance(exc,subprocess.TimeoutExpired)
        raise RuntimeError(label+' failed: '+record['error']+'; STOP, evidence retained') from exc
    finally:
        record['host_end']=time.monotonic()
        audit.write(json.dumps(record)+'\n');audit.flush()
    if answer.returncode:
        raise RuntimeError(f'{label} failed (exit {answer.returncode}); STOP, evidence retained')


def renderer_checks(egl_vendor):
    """One operation per label; file checks need no implicit ROS setup shell."""
    vendor_check='''import json, os, sys
from pathlib import Path
p=Path(sys.argv[1])
print(json.dumps(dict(path=str(p), symlink=p.is_symlink(),
    symlink_target=os.readlink(p) if p.is_symlink() else None,
    resolved=str(p.resolve()), is_file=p.is_file())), flush=True)
if not p.is_file():
    raise SystemExit('NVIDIA EGL vendor JSON missing or symlink target unavailable: '+str(p))
'''
    plugin_check='''import json, sys
from pathlib import Path
p=Path(sys.argv[1]);actual=p.read_bytes().rstrip(b'\\n').decode('ascii');expected=sys.argv[2]
print(json.dumps(dict(path=str(p), actual=actual, expected=expected)), flush=True)
if actual != expected:
    raise SystemExit('ArduPilot Gazebo plugin commit mismatch')
'''
    availability_check='''import json, sys, time
from pathlib import Path
p=Path(sys.argv[1]);deadline=time.monotonic()+float(sys.argv[2])
def ready():
    return p.is_file() and p.stat().st_size>0
while not ready() and time.monotonic()<deadline:
    time.sleep(.1)
print(json.dumps(dict(path=str(p), is_file=p.is_file(),
    bytes=p.stat().st_size if p.is_file() else None)), flush=True)
if not ready():
    raise SystemExit('Required renderer evidence missing or empty: '+str(p))
sys.stdout.flush()
sys.stdout.buffer.write(p.read_bytes())
'''
    return [
        ('source-renderer-ros-environment',['bash','-lc',
            'set -e; source /opt/ros/jazzy/setup.bash; printf "ROS environment setup completed\\n"']),
        ('source-renderer-egl-vendor',['python3','-B','-c',vendor_check,egl_vendor]),
        ('source-renderer-plugin-provenance',['python3','-B','-c',plugin_check,
            '/opt/phase1/plugin-commit.txt','f116b03552d05c8d0b0bed5ed67033a1183121d1']),
        ('source-renderer-gimbal-sdf',['cmp',
            '/opt/ardupilot_gazebo/models/gimbal_small_3d/model.sdf',
            '/run/integration/evidence/integrated_gimbal_original.sdf']),
        ('source-renderer-identity',['python3','-B','-c',availability_check,
            '/tmp/render_gl_identity.txt','10']),
        ('source-renderer-ogre-log',['python3','-B','-c',availability_check,
            '/root/.gz/rendering/ogre2.log','0']),
    ]


def renderer_preflight(physics, egl_vendor, call):
    # Keep the original 20s overall command budget and 10s identity wait.
    deadline=time.monotonic()+20
    for label,command in renderer_checks(egl_vendor):
        remaining=deadline-time.monotonic()
        if remaining<=0:
            raise RuntimeError(label+' blocked: renderer preflight 20s budget exhausted')
        call(['docker','exec',physics,*command],label,remaining)


def prepare(run_id, camera_image, fcu_image, egl_vendor, domain):
    if (not PurePosixPath(egl_vendor).is_absolute() or '..' in PurePosixPath(egl_vendor).parts
            or not egl_vendor.endswith('.json') or any(c not in '/._-abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789' for c in egl_vendor)):
        raise ValueError('Require explicit absolute NVIDIA EGL vendor JSON path inside container')
    if type(domain) is not int or not 1 <= domain <= 232:
        raise ValueError('Require dedicated ROS domain 1..232')
    run = run_camera.prepare(run_id, 'nvidia', camera_image, fcu_image, 30)
    prepare_vio.export(run/'artifacts/vio')  # Separate pinned dependency; never vio/.
    # Identity of this NEW FCU is explicit, not guessed from the first sender.
    with (run/'artifacts/camera.parm').open('a') as out:
        out.write('SYSID_THISMAV 1\n')
    override = dict(services=dict(physics=dict(gpus='all', environment={
        'LIBGL_ALWAYS_SOFTWARE':'0', 'NVIDIA_VISIBLE_DEVICES':'all',
        'NVIDIA_DRIVER_CAPABILITIES':'graphics,utility,compute',
        '__EGL_VENDOR_LIBRARY_FILENAMES':egl_vendor, '__GLX_VENDOR_LIBRARY_NAME':'nvidia',
        'ROS_DOMAIN_ID':str(domain), 'GNSS_EXPECTED_SYSTEM':'1', 'GNSS_EXPECTED_COMPONENT':'1'})))
    # JSON is valid YAML; no dependency on PyYAML for preparation.
    (run/'device.override.yaml').write_text(json.dumps(override, indent=2)+'\n')
    plan = dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,
        run_id=run_id,partition='gnss-gpu-'+run_id,ros_domain=domain,
        names=['gnss-gpu-physics-'+run_id,'gnss-gpu-fcu-'+run_id],
        expected_system=1,expected_component=1,egl_vendor=egl_vendor,
        vio_commit=prepare_vio.PIN,scope='DISARMED source-only; no transport/FCU vision route')
    (run/'SOURCE_RUNTIME.json').write_text(json.dumps(plan,indent=2)+'\n')
    preparation=json.loads((run/'PREPARATION.json').read_text())
    preparation.pop('duration_s',None)
    preparation.update(scope=plan['scope'],source_exec_bound_s=240)
    (run/'PREPARATION.json').write_text(json.dumps(preparation,indent=2)+'\n')
    return run,plan


def owned_ids(records, project, names):
    """All ownership checks before any stop; do not trust a name alone."""
    if len(records)!=len(names) or {r['Name'].lstrip('/') for r in records}!=set(names):
        raise RuntimeError('Unexpected runtime container set')
    ids=[]
    for row in records:
        if (row['Config'].get('Labels',{}).get('com.docker.compose.project')!=project or
                row['HostConfig']['RestartPolicy']['Name']!='no'):
            raise RuntimeError('Unexpected container owner/restart policy')
        ids.append(row['Id'])
    return ids


def validate_runtime(records, plan, run):
    ids=owned_ids(records,plan['partition'],plan['names'])
    physics,fcu=({r['Name'].lstrip('/'):r for r in records}[name] for name in plan['names'])
    if (physics['HostConfig']['NanoCpus']!=3_000_000_000 or
            set(physics['NetworkSettings']['Networks'])!={plan['partition']} or
            fcu['HostConfig']['NetworkMode']!='container:'+physics['Id']):
        raise RuntimeError('Wrong CPU/network namespace contract')
    for row in records:
        if row['HostConfig'].get('PortBindings') or row['HostConfig'].get('PublishAllPorts'):
            raise RuntimeError('Runtime must not publish host ports')
        mounts={m['Destination']:m for m in row['Mounts']}
        if (mounts['/opt/integration']['Source']!=str(ROOT) or mounts['/opt/integration']['RW'] or
                mounts['/run/integration']['Source']!=str(run) or not mounts['/run/integration']['RW']):
            raise RuntimeError('Wrong source/state mounts')
    env=dict(v.split('=',1) for v in physics['Config']['Env'])
    if any(env.get(k)!=v for k,v in {
            'GZ_PARTITION':plan['partition'],'ROS_DOMAIN_ID':str(plan['ros_domain']),
            '__EGL_VENDOR_LIBRARY_FILENAMES':plan['egl_vendor'],
            'GNSS_EXPECTED_SYSTEM':'1','GNSS_EXPECTED_COMPONENT':'1'}.items()):
        raise RuntimeError('Wrong partition/domain/EGL/expected-source environment')
    return ids


def stop_owned(names, project, evidence, call):
    """Collection errors cannot skip stop. Retain containers/network/evidence."""
    records=[];errors=[]
    for name in names:
        found=subprocess.run(['docker','inspect',name],capture_output=True,text=True,timeout=10)
        if found.returncode==0:records.extend(json.loads(found.stdout))
    # A foreign/mismatched record must never be stopped.
    ids=owned_ids(records,project,[r['Name'].lstrip('/') for r in records])
    if any(r['Name'].lstrip('/') not in names for r in records):
        raise RuntimeError('Unexpected owned name')
    for row in records:
        try:call(['docker','logs',row['Id']],'source-log-'+row['Name'].lstrip('/'))
        except Exception as exc:errors.append(str(exc))
    for container in reversed(ids):
        try:call(['docker','stop','--timeout','8',container],'source-stop-'+container[:12],25)
        except Exception as exc:
            errors.append(str(exc))
            # Even a full evidence filesystem must not prevent an owned stop.
            retry=subprocess.run(['docker','stop','--timeout','8',container],capture_output=True,timeout=25)
            if retry.returncode:errors.append('Fallback stop failed: '+container)
    final_records=[]
    for row in records:
        final=json.loads(subprocess.check_output(['docker','inspect',row['Id']],text=True))[0]
        final_records.append(final)
        if final['State']['Running']:errors.append('Still running: '+row['Id'])
    (evidence/'source-owned-final.json').write_text(json.dumps(final_records,indent=2)+'\n')
    return errors


def execute(run, plan, compose, run_probe):
    names=plan['names'];physics,fcu=names;project=plan['partition']
    # Pre-existing names/networks cause HOLD before any service is started.
    for kind,name in [('container',n) for n in names]+[('network',project)]:
        if subprocess.run(['docker',kind,'inspect',name],capture_output=True).returncode==0:
            raise RuntimeError('Refuse pre-existing resource: '+name)
    evidence=run/'evidence'
    result=dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,
                startup='BLOCKED',vio_source='NOT TESTED',cleanup='NOT TESTED')
    audit=(evidence/'source-runtime-commands.jsonl').open('x')
    def call(cmd,label,timeout=30):
        recorded_command(evidence,audit,cmd,label,timeout)
    def shell(code,label,timeout=30):
        call(['docker','exec',physics,'bash','-lc','set -e; source /opt/ros/jazzy/setup.bash; '+code],label,timeout)
    attempted=False
    try:
        images=[]
        for name in (json.loads((run/'PREPARATION.json').read_text())[k] for k in ('camera_image','fcu_image')):
            info=json.loads(subprocess.check_output(['docker','image','inspect',name],text=True))[0]
            if (info['Os'],info['Architecture'])!=('linux','amd64'):
                raise RuntimeError('Linux amd64 image required: '+name)
            images.append(info)
        (evidence/'source-images.json').write_text(json.dumps(images,indent=2)+'\n')
        attempted=True
        call(compose+['up','-d','--no-build','--pull','never'],'source-up',60)
        records=json.loads(subprocess.check_output(['docker','inspect',*names],text=True))
        validate_runtime(records,plan,run)
        network=json.loads(subprocess.check_output(['docker','network','inspect',project],text=True))[0]
        if not network['Internal']:raise RuntimeError('Internal Docker network required')
        (evidence/'source-owned-before.json').write_text(json.dumps(records,indent=2)+'\n')
        if any(not r['State']['Running'] for r in records):raise RuntimeError('Runtime exited at startup')
        imports=("import sys; sys.path.insert(0,'/run/integration/artifacts/vio'); "
                 "import cv2, numpy, rclpy, gz.transport13; "
                 "from gz.msgs10.image_pb2 import Image; "
                 "from sensor_msgs.msg import Imu; from rosgraph_msgs.msg import Clock; "
                 "from nav_msgs.msg import Odometry; "
                 "import vio_core, vio_frontend, vio_ros_source, vio_ros_out; print('Source imports available')")
        shell('command -v setsid; command -v ps; test -x /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge; '
              'python3 -B -c '+shlex.quote(imports),'source-dependencies')
        # Build identity, actual model and vendor path must match this experiment.
        call(['docker','exec',fcu,'bash','-lc',
            'set -e; test "$(cat /opt/phase1/ardupilot-commit.txt)" = 9279df6191a772c098b2e4797c95ac954f258d88; '
            'sha256sum -c /opt/phase1/firmware.sha256; cat /opt/phase1/parser-tests.txt; '
            "grep -F '[  PASSED  ] 5 tests.' /opt/phase1/parser-tests.txt"], 'source-firmware')
        renderer_preflight(physics,plan['egl_vendor'],call)
        # Compare the active context with its own Ogre log, not another diagnostic.
        code=("import sys; sys.path.insert(0,'/opt/integration'); from pathlib import Path; "
              "from vio_source_probe import hardware_renderer; "
              "s=Path('/tmp/render_gl_identity.txt').read_text(); r=hardware_renderer(s); "
              "assert 'vendor=NVIDIA Corporation' in s; "
              "assert r in Path('/root/.gz/rendering/ogre2.log').read_text(); print(r)")
        shell('python3 -B -c '+shlex.quote(code),'source-gpu-check')
        heartbeat='python3 -B /opt/integration/linux_gpu/verified_disarmed.py --expected-system 1 --expected-component 1 --directory '
        shell(heartbeat+'/run/integration/evidence/source-disarmed-before','source-disarmed-before',12)
        # Set only the reviewed gimbal topics in this NEW partition. No world pause.
        for axis,value in (('yaw',.000796326795),('roll',-.000796326795),('pitch',1.570796326795)):
            shell('gz topic -t /gimbal/cmd_'+axis+' -m gz.msgs.Double -p '+shlex.quote('data: '+str(value)), 'source-gimbal-'+axis)
        time.sleep(8)
        shell('gz topic -l','source-topic-list')
        advertised=set((evidence/'source-topic-list.txt').read_text().splitlines())
        for label,topic in (
                ('camera','/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'),
                ('imu','/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu'),
                ('clock','/clock'),('pose','/world/iris_runway/dynamic_pose/info')):
            if topic not in advertised:raise RuntimeError('Missing native topic: '+topic)
            shell('gz topic -i -t '+shlex.quote(topic),'source-topic-'+label)
        # No ROS bridges yet: probe requires an unused domain and owns /clock.
        shell('ros2 topic list -t','source-ros-before')
        shell('mkdir /run/integration/evidence/source-preflight; python3 -B /opt/integration/vio_source_probe.py preflight '
              '--directory /run/integration/evidence/source-preflight --vio-dir /run/integration/artifacts/vio','source-preflight',15)
        result['startup']='PASS'
        if run_probe:
            result['vio_source']='FAIL'
            try:
                # This script starts NOTHING; simulator is already verified above.
                shell('bash /opt/integration/run_vio_source_probe.sh '+shlex.quote(plan['run_id'])+
                      ' /run/integration/artifacts/vio','source-probe',240)
                result['vio_source']='PASS'
            finally:
                call(['docker','cp',physics+':/tmp/gnss-vio-source-'+plan['run_id'],str(evidence/'source-probe')], 'source-copy',30)
        shell(heartbeat+'/run/integration/evidence/source-disarmed-after','source-disarmed-after',12)
    except BaseException as exc:
        result['error']=str(exc)
        raise
    finally:
        errors=[]
        if attempted:
            try:
                errors=stop_owned(names,project,evidence,call)
            except Exception as exc:errors.append(str(exc))
        result['cleanup']='FAIL' if errors else 'PASS'
        result['cleanup_errors']=errors
        (evidence/'SOURCE_RUNTIME_RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
        audit.close()
        if errors:raise RuntimeError('Owned runtime cleanup failed: '+str(errors))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--camera-image',default='gnss-phase1-camera:linux-gpu')
    parser.add_argument('--fcu-image',default='gnss-phase1-sitl:linux-gpu')
    parser.add_argument('--egl-vendor-file',required=True)
    parser.add_argument('--ros-domain',type=int,default=123)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--probe',action='store_true',help='After startup PASS invoke the existing OFF/ON/OFF probe')
    args=parser.parse_args()
    if args.probe and not args.execute:parser.error('--probe requires --execute')
    if args.execute and sys.platform!='linux':parser.error('Linux NVIDIA runtime required')
    run,plan=prepare(args.run_id,args.camera_image,args.fcu_image,args.egl_vendor_file,args.ros_domain)
    compose=['docker','compose','--env-file',str(run/'compose.env'),'-f',str(ROOT/'linux_gpu/camera.compose.yaml'),'-f',str(run/'device.override.yaml')]
    with (run/'artifacts/source-resolved-compose.yaml').open('x') as out:
        subprocess.run(compose+['config'],stdout=out,check=True)
    print(str(run),flush=True)
    if args.execute:
        def interrupted(*_):raise KeyboardInterrupt('Interrupted')
        signal.signal(signal.SIGTERM,interrupted)
        execute(run,plan,compose,args.probe)
    else:print('PREPARED ONLY; execution_allowed=false; forwarding_enabled=false')


if __name__=='__main__':main()
