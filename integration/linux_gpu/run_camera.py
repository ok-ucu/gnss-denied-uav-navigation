"""Prepare, or explicitly execute, ONE new bounded DISARMED camera-only run.

No VIO, ROS bridge, MAVROS, ExternalNav or flight commands. Never reuses names,
old evidence or firmware authorization. Hardware identity is checked in Gazebo.
"""
import argparse
import json
import math
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys

INTEGRATION=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(INTEGRATION))
from launch_phase1 import validate_run_id, verified_file

def hardware_status(identity,ogre):
    fields=dict(line.split('=',1) for line in identity.splitlines() if '=' in line)
    renderer=fields.get('renderer','').strip()
    if not renderer or not fields.get('version'):return 'HOLD: missing actual GL identity'
    if any(token in renderer.lower() for token in ('llvmpipe','softpipe','swrast','swiftshader','lavapipe','software')):
        return 'HOLD: software rendering'
    if renderer not in ogre:return 'HOLD: GL identity not corroborated by Ogre log'
    return 'HARDWARE CANDIDATE: verify device/vendor against recorded device inventory'

def prepare(run_id,mode,camera_image,fcu_image,duration,perf=False):
    validate_run_id(run_id)
    if not math.isfinite(duration) or not 10<=duration<=60:raise ValueError('Duration must be finite, 10..60s')
    if mode not in ('software','dri','nvidia'):raise ValueError('Explicit supported device mode required')
    run=INTEGRATION/'linux_gpu_runs'/run_id
    if run.exists():raise ValueError('Refuse to overwrite a prepared or completed run')
    for rel in ('models/iris_with_gimbal','evidence','state/fcu','artifacts'):
        (run/rel).mkdir(parents=True,exist_ok=False)
    for name in ('model.sdf','model.config'):
        shutil.copyfile(verified_file('models/patched_c/iris_with_gimbal/'+name),run/'models/iris_with_gimbal'/name)
    shutil.copyfile(verified_file('models/patched_c_world.sdf'),run/'models/world.sdf')
    shutil.copyfile(verified_file('evidence/patched_c_gimbal_original.sdf'),run/'evidence/integrated_gimbal_original.sdf')
    # Camera-only physics exchange, no EKF/GPS/navigation-source override.
    (run/'artifacts/camera.parm').write_text('SERIAL0_PROTOCOL 2\nLOG_DISARMED 1\n')
    env=dict(GPU_RUN_ID=run_id,GPU_INTEGRATION_ROOT=str(INTEGRATION),GPU_RUN_ROOT=str(run),
             GPU_CAMERA_IMAGE=camera_image,GPU_FCU_IMAGE=fcu_image,GPU_SOFTWARE='1' if mode=='software' else '0')
    if any(any(c in str(value) for c in '\n\r$#') for value in env.values()):raise ValueError('Unsupported Compose value')
    (run/'compose.env').write_text(''.join(k+'='+v+'\n' for k,v in env.items()))
    override='services:\n  physics:\n'
    if mode=='dri':override+='    devices: ["/dev/dri:/dev/dri"]\n'
    elif mode=='nvidia':override+='    gpus: all\n    environment:\n      NVIDIA_DRIVER_CAPABILITIES: graphics,utility,compute\n'
    else:override+='    environment:\n      LIBGL_ALWAYS_SOFTWARE: "1"\n'
    if perf:override+='    cap_add: [PERFMON]\n    security_opt: [seccomp=unconfined]\n'
    (run/'device.override.yaml').write_text(override)
    (run/'PREPARATION.json').write_text(json.dumps(dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,
        mode=mode,duration_s=duration,perf=perf,camera_image=camera_image,fcu_image=fcu_image,
        historical_firmware_pin_unchanged=True,scope='camera-only; no navigation approval'),indent=2)+'\n')
    return run

def execute(run,args,compose):
    names=['gnss-gpu-physics-'+args.run_id,'gnss-gpu-fcu-'+args.run_id]
    for name in names:
        if subprocess.run(['docker','inspect',name],capture_output=True).returncode==0:
            raise RuntimeError('Refuse to reuse existing container '+name)
    if args.mode=='dri' and not Path('/dev/dri').is_dir():raise RuntimeError('No host /dev/dri; HOLD')
    physics,fcu=names
    processes=[];handles=[]
    audit=(run/'evidence/commands.jsonl').open('x')
    def call(cmd,label,timeout=30):
        audit.write(json.dumps(dict(command=cmd,label=label))+'\n');audit.flush()
        with (run/'evidence'/(label+'.txt')).open('x') as output:
            result=subprocess.run(cmd,stdout=output,stderr=subprocess.STDOUT,timeout=timeout)
        if result.returncode:raise RuntimeError(label+' failed; evidence retained')
    def command(shell,label,timeout=30):
        call(['docker','exec',physics,'bash','-lc','source /opt/ros/jazzy/setup.bash; '+shell],label,timeout)
    def start(shell,label):
        cmd=['docker','exec',physics,'bash','-lc','source /opt/ros/jazzy/setup.bash; '+shell]
        audit.write(json.dumps(dict(command=cmd,label=label))+'\n');audit.flush()
        output=(run/'evidence'/(label+'.txt')).open('x');handles.append(output)
        process=subprocess.Popen(cmd,stdout=output,stderr=subprocess.STDOUT);processes.append(process)
        return process
    try:
        call(['docker','image','inspect',args.camera_image,args.fcu_image],'images')
        call(compose+['up','-d','--no-build','--pull','never'],'up',60)
        call(['docker','inspect',*names],'owned-metadata')
        call(['docker','exec',fcu,'bash','-lc',
              'test "$(cat /opt/phase1/ardupilot-commit.txt)" = 9279df6191a772c098b2e4797c95ac954f258d88 && '
              'sha256sum -c /opt/phase1/firmware.sha256 && cat /opt/phase1/parser-tests.txt'],
             'firmware-build-record')
        source_check=('from pathlib import Path; '
            'assert Path("/opt/phase1/plugin-commit.txt").read_text().strip()=="f116b03552d05c8d0b0bed5ed67033a1183121d1"; '
            'assert Path("/opt/ardupilot_gazebo/models/gimbal_small_3d/model.sdf").read_bytes()=='
            'Path("/run/integration/evidence/integrated_gimbal_original.sdf").read_bytes(); '
            'print("Pinned plugin and actual camera SDF match")')
        command('python3 -c '+shlex.quote(source_check),'plugin-camera-provenance')
        command('python3 -B /opt/integration/linux_gpu/disarmed.py','disarmed-before',15)
        command('python3 -B /opt/integration/isolated_blockers/20261109_a/code/probe_experiment.py --run /run/integration --variant integrated --duration 30','mount',55)
        mount=json.loads((run/'evidence/integrated/summary.json').read_text())
        if mount['mount_status']!='PASS' or not mount['settled_snapshots'] or max(mount['max_angles_deg'])>1 or mount['unexpected_commands']:
            raise RuntimeError('Measured mount HOLD')
        command('ls -l /dev; cat /sys/fs/cgroup/cpu.max; cat /sys/fs/cgroup/cpuset.cpus.effective; cat /tmp/render_gl_identity.txt; cat /opt/phase1/packages.txt','device-versions')
        command('python3 -B /opt/integration/vio_ground_fixture.py create --directory /run/integration/artifacts/fixture --size-m 1 --tile-size-m .02','fixture',20)
        duration=str(args.duration)
        start('exec python3 -B /opt/integration/linux_gpu/sample_camera.py '+duration+' /tmp/camera_profile','camera')
        if args.perf:
            start('p=$(find /usr/lib/linux-tools-* -type f -name perf | sort | tail -1); test -n "$p"; exec "$p" record -e cpu-clock:u -F 99 --call-graph dwarf,8192 -p 1 -o /tmp/render.perf.data -- sleep '+duration,'perf')
        for process in processes:
            if process.wait(timeout=args.duration+20):raise RuntimeError('Camera/profiler failed')
        command('python3 -B /opt/integration/linux_gpu/disarmed.py','disarmed-after',15)
        if args.perf:
            command('p=$(find /usr/lib/linux-tools-* -type f -name perf | sort | tail -1); "$p" report --stdio --no-children -i /tmp/render.perf.data > /tmp/perf-report.txt; "$p" script -i /tmp/render.perf.data > /tmp/perf-stacks.txt','perf-analysis',60)
    finally:
        for process in processes:
            if process.poll() is None:process.terminate()
        for handle in handles:handle.close()
        # All finalization is best effort, but one failed collection cannot skip stop.
        try:
            command('python3 -B /opt/integration/vio_ground_fixture.py remove --directory /run/integration/artifacts/fixture','remove-fixture',15)
        except Exception as exc:audit.write(json.dumps(dict(cleanup_error=str(exc)))+'\n')
        code='import glob,shutil,pathlib; p=pathlib.Path("/run/integration/evidence/native_render"); p.mkdir(exist_ok=False); files=glob.glob("/tmp/phase1_camera_boundary_*.csv")+glob.glob("/tmp/phase1_native_stack_*.csv")+glob.glob("/tmp/render_gl_identity.txt")+glob.glob("/tmp/camera_profile.*")+glob.glob("/tmp/render.perf.data")+glob.glob("/tmp/perf-*.txt"); [shutil.copyfile(f,p/pathlib.Path(f).name) for f in files]; [shutil.copyfile(f,p/"ogre2.log") for f in glob.glob("/root/.gz/rendering/ogre2.log")]'
        for cmd,label in [(['docker','exec',physics,'python3','-c',code],'collect'),(['docker','logs',physics],'physics-log'),(['docker','logs',fcu],'fcu-log')]:
            try:call(cmd,label)
            except Exception as exc:audit.write(json.dumps(dict(cleanup_error=str(exc)))+'\n')
        for name in reversed(names):
            try:
                found=subprocess.run(['docker','inspect',name],capture_output=True,text=True,timeout=10)
                if found.returncode:continue
                record=json.loads(found.stdout)[0]
                if record['Config']['Labels'].get('com.docker.compose.project')!='gnss-gpu-'+args.run_id:
                    raise RuntimeError('Unexpected owner; refusing to stop '+name)
                call(['docker','stop','--timeout','8',record['Id']],'stop-'+name,25)
            except Exception as exc:audit.write(json.dumps(dict(cleanup_error=str(exc),container=name))+'\n')
        try:call(['docker','inspect',*names],'owned-final')
        finally:audit.close()
    capture=run/'evidence/native_render'
    from analyze_camera import analyze_capture
    metrics=analyze_capture(capture)
    (run/'evidence/METRICS.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    result=hardware_status((capture/'render_gl_identity.txt').read_text(),(capture/'ogre2.log').read_text())
    print(result+'; overall HOLD; no full normal or navigation approval')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',type=validate_run_id,required=True)
    parser.add_argument('--mode',choices=['software','dri','nvidia'],required=True)
    parser.add_argument('--camera-image',default='gnss-phase1-camera:linux-gpu')
    parser.add_argument('--fcu-image',default='gnss-phase1-sitl:linux-gpu')
    parser.add_argument('--duration',type=float,default=30)
    parser.add_argument('--perf',action='store_true')
    parser.add_argument('--execute',action='store_true',help='Explicitly start only new DISARMED camera/FCU services')
    args=parser.parse_args()
    run=prepare(args.run_id,args.mode,args.camera_image,args.fcu_image,args.duration,args.perf)
    compose=['docker','compose','--env-file',str(run/'compose.env'),'-f',str(INTEGRATION/'linux_gpu/camera.compose.yaml'),'-f',str(run/'device.override.yaml')]
    with (run/'artifacts/resolved-compose.yaml').open('x') as output:
        subprocess.run(compose+['config'],stdout=output,check=True)
    print(run,flush=True)
    if not args.execute:
        print('PREPARED ONLY; no containers started; use a fresh run-id for execution')
        return
    def stop(*_):raise KeyboardInterrupt('bounded camera-only run interrupted')
    signal.signal(signal.SIGTERM,stop)
    execute(run,args,compose)

if __name__=='__main__':main()
