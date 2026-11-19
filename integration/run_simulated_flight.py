"""Separate opt-in Linux NVIDIA scenario. Default is offline admission ONLY.

Never changes the DISARMED launcher or an existing stack. All flight commands
remain inside phase1_transport; no second/direct FCU connection is created.
"""
import argparse
import json
from pathlib import Path
import subprocess
import struct
import sys
import time

from launch_phase1 import validate_run_id, validate_vio_dir
from runtime_candidate import ROOT, load_candidate, sha
from simulation_vision import prepare_permit, origin_values
from simulation_flight import verify_proof, decode_dataflash, current_fusion_attestation, SCOPE, LIMITS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-id',required=True,type=validate_run_id)
    p.add_argument('--runtime-candidate',type=Path,required=True)
    p.add_argument('--vio-dir',type=Path,required=True)
    p.add_argument('--fusion-proof',type=Path,required=True)
    p.add_argument('--origin',required=True,help='Explicit simulation lat,lon,alt_m')
    p.add_argument('--execute-simulation-flight',action='store_true',help='SEPARATE APPROVAL REQUIRED: may ARM ONLY this new owned SITL')
    p.add_argument('--tolerate-single-rejection',action='store_true',
                   help='Opt-in (audit C2), SAFETY OWNER APPROVAL REQUIRED: passed to run_vio_stream.py; '
                        'recorded in evidence/flight-source/configuration.json')
    a=p.parse_args()
    baseline,permit=verify_proof(a.fusion_proof)
    candidate=load_candidate(a.runtime_candidate);vio=validate_vio_dir(a.vio_dir)
    origin=list(map(float,a.origin.split(',')))
    if candidate['firmware_sha256']!=permit['firmware_sha256'] or origin_values(origin)!=permit['origin']:
        raise ValueError('Flight firmware/origin differ from validated DISARMED baseline')
    if not a.execute_simulation_flight:
        print('OFFLINE ADMISSION ONLY; no containers/ARM/flight. Linux live preflight remains required.');return
    if sys.platform!='linux':raise ValueError('Execution requires the separately approved Linux NVIDIA host')
    # Preparation only; this never invokes the DISARMED validation runtime.
    subprocess.run([sys.executable,'-B',str(ROOT/'launch_phase1.py'),'--run-id',a.run_id,
        '--runtime-candidate',str(a.runtime_candidate),'--vio-dir',str(vio),'--gpu-mode','nvidia',
        '--isolated-imu','--simulation-disarmed-vision','--simulation-origin='+a.origin],check=True,timeout=30)
    run=ROOT/'phase1_runs'/a.run_id
    names={k:'gnss-p1-'+k+'-'+a.run_id for k in ('physics','fcu','vio','transport')}
    compose=['docker','compose','--env-file',str(run/'compose.env'),'-f',str(ROOT/'phase1.compose.yaml'),
             '-f',str(ROOT/'linux_gpu/phase1.gpu.nvidia.yaml')]
    owned=None;children=[];handles=[];fixture=False;errors=[];safe_landed=False
    audit=(run/'evidence/flight-host.jsonl').open('x')
    ros='source /opt/ros/jazzy/setup.bash; '
    def command(args,label,timeout=15):
        result=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
        audit.write(json.dumps(dict(command=args,exit=result.returncode,stdout=result.stdout,stderr=result.stderr,host_ns=time.monotonic_ns()))+'\n');audit.flush()
        (run/'evidence'/label).write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(label+' failed')
        return result.stdout
    def start(label,kind,code):
        f=(run/'evidence'/(label+'.console.txt')).open('x');handles.append(f)
        proc=subprocess.Popen(['docker','exec',names[kind],'bash','-lc',code],stdout=f,stderr=subprocess.STDOUT)
        children.append(proc);return proc
    try:
        command(compose+['up','-d','--no-build','--pull','never'],'flight-compose-up.txt',60)
        owned=json.loads(subprocess.check_output(['docker','inspect',*names.values()],text=True,timeout=10))
        project='gnss-phase1-integrated-'+a.run_id
        if len(owned)!=4 or any(r['Config'].get('Labels',{}).get('com.docker.compose.project')!=project for r in owned):
            owned=None;raise ValueError('Cannot establish owned container identities')
        (run/'evidence/flight-owned.json').write_text(json.dumps(owned,indent=2)+'\n')
        from linux_gpu.run_source_runtime import renderer_preflight
        renderer_preflight(names['physics'],'/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
            lambda cmd,label,timeout:command(cmd,label+'.txt',timeout))
        check=("from pathlib import Path; from vio_source_probe import hardware_renderer; "
               "s=Path('/tmp/render_gl_identity.txt').read_text(); r=hardware_renderer(s); "
               "assert 'vendor=NVIDIA Corporation' in s; assert r in Path('/root/.gz/rendering/ogre2.log').read_text()")
        command(['docker','exec',names['physics'],'python3','-B','-c',check],'flight-renderer.txt')
        command(['docker','exec',names['physics'],'bash','-lc',ros+
            'python3 -B /opt/integration/isolated_blockers/20261109_a/code/probe_experiment.py --run /run/integration --variant integrated --duration 30'],
            'flight-mount.txt',45)
        mount=json.loads((run/'evidence/integrated/summary.json').read_text())
        if mount['mount_status']!='PASS' or not mount['settled_snapshots'] or max(mount['max_angles_deg'])>1 or mount['unexpected_commands']:
            raise ValueError('Actual camera/gimbal mount failed')
        (run/'artifacts/mount_calibration.json').write_text(json.dumps(mount)+'\n')
        simpermit=prepare_permit(run,a.runtime_candidate,origin,approved=True)
        current=json.loads(simpermit.read_text())
        if current['source_hashes']!=permit['source_hashes']:raise ValueError('DISARMED fusion source stack changed')
        proof=a.fusion_proof.resolve()
        plan=dict(schema=1,scope=SCOPE,approved=True,limits=LIMITS,run_id=a.run_id,
            reference_nonce=current['reference_nonce'],fusion_proof=str(proof.relative_to(ROOT)),
            fusion_proof_sha256=sha(proof),source_sha256=sha(ROOT/'simulation_flight.py'),
            host_runner_sha256=sha(Path(__file__)),
            execution_allowed=False,forwarding_enabled=False)
        with (run/'artifacts/flight_plan.json').open('x') as f:json.dump(plan,f,allow_nan=False)
        # Integration-owned visual scene only; no sensor/physics/model change.
        command(['docker','exec',names['vio'],'bash','-lc',ros+
            'python3 -B /opt/integration/vio_ground_fixture.py create --directory /run/integration/artifacts/flight-fixture --size-m 16 --tile-size-m .02 --pattern noise'],
            'flight-fixture.txt',30);fixture=True
        camera='/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
        start('flight-camera','vio',ros+'exec ros2 run ros_gz_bridge parameter_bridge '+camera+'@sensor_msgs/msg/Image[gz.msgs.Image')
        start('flight-mavros','transport',ros+'source /tmp/patch_install/setup.bash; exec /opt/ros/jazzy/lib/mavros/mavros_node --ros-args --params-file /opt/integration/phase1_capture_mavros.yaml')
        start('flight-observer','vio',ros+'exec python3 -B /opt/integration/observe_frame_bridge.py --directory /run/integration/evidence/flight-observer --duration 145 --output-topic /integration/phase1/odometry_enu')
        transport=start('flight-transport','transport',ros+'source /tmp/patch_install/setup.bash; exec python3 -B /opt/integration/phase1_transport.py --run /run/integration --output-name flight-transport --duration 115 --simulation-vision-permit /run/integration/artifacts/simulation_vision_permit.json --simulation-flight-plan /run/integration/artifacts/flight_plan.json')
        transport_started=time.monotonic()
        source=start('flight-source','vio',ros+'exec python3 -B /opt/integration/run_vio_stream.py --vio-dir /opt/vio --output-dir /run/integration/evidence/flight-source --duration 110 --ground-z .001 --mount-calibration /run/integration/artifacts/mount_calibration.json --profile --async-diagnostics --reliable-camera --isolated-imu --min-odom-subscribers 2 --min-epoch-subscribers 1'
            +(' --tolerate-single-rejection' if a.tolerate_single_rejection else ''))
        source_started=time.monotonic()
        # Current-boot aiding is proven independently from the transport's wire
        # counters. No FCU commands/connections; bounded, immutable log snapshots.
        witness_end=time.monotonic()+35;attempt=0;ready=False
        while transport.poll() is None and time.monotonic()<witness_end:
            bins=sorted((run/'state/fcu/logs').glob('*.BIN'))
            if bins:
                attempt+=1;snapshot=run/'evidence'/('current-fusion-'+str(attempt)+'.BIN')
                with bins[-1].open('rb') as raw:data=raw.read(32*1024*1024+1)
                if len(data)>32*1024*1024:raise ValueError('Current DataFlash snapshot memory bound exceeded')
                with snapshot.open('xb') as f:f.write(data)
                try:
                    records=decode_dataflash(snapshot)
                    trace=run/'evidence/applied.csv'
                    with trace.open('rb') as raw:
                        raw.seek(max(0,trace.stat().st_size-8192));tail=raw.read()
                    lines=tail.split(b'\n')[:-1]
                    latest_applied=int(lines[-1].split(b',')[4])*1000
                    attestation=current_fusion_attestation(records,current,snapshot,latest_applied)
                except (ValueError,EOFError,struct.error) as exc:
                    audit.write(json.dumps(dict(current_fusion='NOT READY',snapshot=snapshot.name,reason=str(exc)))+'\n');audit.flush()
                else:
                    pending=run/'artifacts/current_fusion_ready.pending'
                    with pending.open('x') as f:json.dump(attestation,f)
                    pending.replace(run/'artifacts/current_fusion_ready.json')
                    ready=True;break
            time.sleep(.5)
        if not ready:raise RuntimeError('Current-boot live fusion NOT VERIFIED; ARM remains blocked')
        transport.wait(timeout=max(1,120-(time.monotonic()-transport_started)))
        result=json.loads((run/'evidence/flight-transport/flight_result.json').read_text())
        safe_landed=result.get('landed_disarmed_confirmed') is True
        if transport.returncode or result['status']!='PASS':raise RuntimeError('Flight failed; LAND/STOP evidence retained')
        # Allow the bounded source to finish/export normally while DISARMED.
        source.wait(timeout=max(1,125-(time.monotonic()-source_started)))
        if source.returncode:raise RuntimeError('Flight source export incomplete')
        print(json.dumps(result))
    finally:
        if fixture and safe_landed:
            try:command(['docker','exec',names['vio'],'bash','-lc',ros+'python3 -B /opt/integration/vio_ground_fixture.py remove --directory /run/integration/artifacts/flight-fixture'],'flight-fixture-remove.txt')
            except (OSError,RuntimeError,subprocess.SubprocessError) as exc:errors.append(str(exc))
        # STOP is a simulation containment action, never an airborne FCU DISARM.
        order=('transport','vio','fcu','physics') if safe_landed else ('fcu','physics','transport','vio')
        for kind in order:
            try:
                row=next(r for r in (owned or []) if r['Name'].lstrip('/')==names[kind])
                current=json.loads(subprocess.check_output(['docker','inspect',names[kind]],text=True,timeout=10))[0]
                if current['Id']!=row['Id']:raise ValueError('Container replaced; no signal sent')
                command(['docker','stop','--time','8' if safe_landed else '2',row['Id']],'flight-stop-'+kind+'.txt',15)
                final=json.loads(subprocess.check_output(['docker','inspect',row['Id']],text=True,timeout=10))[0]
                if final['State']['Running']:raise ValueError('Container still running')
            except (OSError,ValueError,StopIteration,RuntimeError,subprocess.SubprocessError) as exc:errors.append(str(exc))
        for proc in children:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:proc.wait(timeout=2)
                except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2)
        for f in handles:f.close()
        audit.write(json.dumps(dict(cleanup='INCOMPLETE' if errors else 'STOPPED_OWN_SIMULATION',errors=errors))+'\n');audit.close()
        if errors:raise RuntimeError('Simulation STOP/cleanup incomplete: '+repr(errors))


if __name__=='__main__':main()
