"""Prepare/run a NEW disarmed Phase 1 stack. Capture-only by default.

No builds, image pulls, flight commands, parameter writes to existing services,
container removal or reference reuse. --execute defaults to the fault matrix;
explicit --simulation-disarmed-vision selects only a normal SITL ingress test.
Omitting it only prepares files and validates the installed image/artifact IDs.
"""
import argparse
import hashlib
import json
import re
import shlex
import shutil
import subprocess
import sys
import uuid
from pathlib import Path
from linux_gpu.prepare_vio import FILES as VIO_FILES, PIN as VIO_PIN

INTEGRATION = Path(__file__).resolve().parent
VERIFIED = INTEGRATION / 'isolated_blockers/20261109_a'
IMAGES = {
    'physics': 'sha256:6a5215efd7798818523c309eb220753f5755115ed493cd70ada8ae4ed0b61e6b',
    'fcu': 'sha256:f5dffdb9f79ed8a7ff58aa7e6f6dfcbe2ebd2cead9fe7b20daea8d953b491540',
    'transport': 'sha256:acd5f652ca1e72744c6427ed63d7d0805412ead3b1b1da95f2a2f08dcbfa7698',
}


def validate_vio_dir(directory):
    """Mount an explicit pinned export; never fall back to another VIO copy."""
    directory = Path(directory).resolve()
    for name, expected in VIO_FILES.items():
        path = directory / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Missing or changed pinned VIO dependency: ' + str(path))
    return directory


def validate_branch(branch):
    if branch not in ('gazebo-sitl-integration', 'phase1-integration-merge'):
        raise ValueError('Unexpected branch; no switching performed')


def validate_run_id(value):
    if not re.fullmatch(r'[0-9]{8}-[a-z0-9]+', value):
        raise ValueError('Use a new ID such as 20261110-check1')
    return value


def verified_file(relative):
    """Reuse recorded bytes; never silently replace or rebuild a tested artifact."""
    manifest = json.loads((VERIFIED / 'SHA256SUMS.json').read_text())['files']
    path = VERIFIED / relative
    key = str(path.relative_to(INTEGRATION.parent))
    expected = manifest[key]
    data = path.read_bytes()
    if len(data) != expected['bytes'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
        raise ValueError('Historical artifact changed: ' + key)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True, type=validate_run_id)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--isolated-imu', action='store_true', help='Opt-in spawned IMU/clock receiver in capture validation')
    parser.add_argument('--vio-dir', type=Path, help='Pinned VIO export directory; defaults to repository vio/')
    parser.add_argument('--runtime-candidate',type=Path,help='Verified native export; PREPARE ONLY unless explicit DISARMED simulation mode is selected')
    parser.add_argument('--gpu-mode',choices=['software','dri','nvidia','wsl'],default='software')
    parser.add_argument('--simulation-disarmed-vision',action='store_true',help='Explicit approval: bounded DISARMED vision to owned Gazebo/SITL only, never flight')
    parser.add_argument('--simulation-origin',help='Explicit lat,lon,alt_m for the isolated simulation EKF origin')
    args = parser.parse_args()
    if args.simulation_disarmed_vision and (not args.runtime_candidate or not args.simulation_origin or args.gpu_mode!='nvidia'):
        parser.error('Simulation vision requires native candidate, explicit origin and NVIDIA profile')
    if args.simulation_origin and not args.simulation_disarmed_vision:
        parser.error('Simulation origin requires explicit simulation vision mode')
    origin=None
    if args.simulation_disarmed_vision:
        if sys.platform!='linux':parser.error('DISARMED simulation vision mode requires a Linux NVIDIA host')
        from simulation_vision import origin_values
        origin=list(map(float,args.simulation_origin.split(',')));origin_values(origin)
        from linux_gpu.run_source_runtime import check_host_egl_vendor
        check_host_egl_vendor('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    candidate=None
    if args.runtime_candidate:
        from runtime_candidate import load_candidate
        candidate=load_candidate(args.runtime_candidate)
        if args.execute and not args.simulation_disarmed_vision:
            raise ValueError('Native candidate is HOLD: no runtime/clock admission; --execute forbidden')
    images=IMAGES if candidate is None else candidate['images']
    architecture='arm64' if candidate is None else candidate['architecture']
    if candidate is None and args.gpu_mode!='software':
        raise ValueError('GPU preview requires an explicit runtime candidate; legacy profile unchanged')
    vio_dir = validate_vio_dir(args.vio_dir if args.vio_dir is not None else INTEGRATION.parent / 'vio')
    run = INTEGRATION / 'phase1_runs' / args.run_id
    names = {kind: f'gnss-p1-{kind}-{args.run_id}' for kind in ('physics', 'fcu', 'vio', 'transport')}
    network = 'gnss-p1-integrated-' + args.run_id
    if run.exists():
        raise ValueError('Refuse to overwrite a run, including a prepared-only run')
    validate_branch(subprocess.check_output(['git', 'branch', '--show-current'], cwd=INTEGRATION, text=True).strip())
    for name in names.values():
        result = subprocess.run(['docker', 'container', 'inspect', name], capture_output=True)
        if result.returncode == 0:
            raise ValueError('Container name already exists: ' + name)
    if subprocess.run(['docker', 'network', 'inspect', network], capture_output=True).returncode == 0:
        raise ValueError('Network name already exists: ' + network)
    for digest in images.values():
        platform = subprocess.check_output(['docker', 'image', 'inspect', '--format', '{{.Os}}/{{.Architecture}}', digest], text=True).strip()
        if platform != 'linux/'+architecture:
            raise ValueError('Image OS/architecture mismatch: ' + digest)
    files = {relative: verified_file(relative) for relative in (
        'models/patched_c/iris_with_gimbal/model.sdf',
        'models/patched_c/iris_with_gimbal/model.config', 'models/patched_c_world.sdf',
        'evidence/patched_c_gimbal_original.sdf', 'code/probe_experiment.py', 'code/mount_math.py')}
    from physics_clock_mapping import PATCHED_SHA
    firmware=verified_file('evidence/arducopter_patched') if candidate is None else candidate['firmware']
    firmware_sha=hashlib.sha256(firmware.read_bytes()).hexdigest()
    if candidate is None and firmware_sha != PATCHED_SHA:
        raise ValueError('Unreviewed FCU binary')
    overlay=INTEGRATION/'timesync_patch_runs/20261108/patch_install' if candidate is None else candidate['overlay']
    if not (overlay/'setup.bash').is_file():
        raise ValueError('Tested MAVROS overlay missing; no unpatched fallback')
    for subdir in ('models/iris_with_gimbal', 'artifacts', 'evidence', 'state/fcu'):
        (run / subdir).mkdir(parents=True, exist_ok=True)
    for filename in ('model.sdf', 'model.config'):
        shutil.copyfile(files['models/patched_c/iris_with_gimbal/' + filename], run / 'models/iris_with_gimbal' / filename)
    shutil.copyfile(files['models/patched_c_world.sdf'], run / 'models/world.sdf')
    shutil.copyfile(files['evidence/patched_c_gimbal_original.sdf'], run / 'evidence/integrated_gimbal_original.sdf')
    shutil.copy2(firmware,run/'artifacts/arducopter')
    (run / 'artifacts/run_identity.json').write_text(json.dumps(dict(
        run_id=args.run_id, clock_epoch=str(uuid.uuid4()), reference_nonce=str(uuid.uuid4()),
        execution_allowed=False, forwarding_enabled=False,firmware_sha256=firmware_sha,
        architecture=architecture,runtime_admitted=candidate is None), indent=2) + '\n')
    # This profile is loaded ONLY by the newly named FCU. No PARAM_SET traffic.
    (run / 'artifacts/externalnav.parm').write_text(
        (INTEGRATION / 'ekf3_dry_run.parm').read_text() + '\nSERIAL0_PROTOCOL 2\nLOG_DISARMED 1\n')
    env_values = dict(P1_RUN_ID=args.run_id, INTEGRATION_ROOT=str(INTEGRATION),
                      PHASE1_RUN_ROOT=str(run), VIO_ROOT=str(vio_dir),
                      P1_PHYSICS_IMAGE=images['physics'],P1_FCU_IMAGE=images['fcu'],
                      P1_TRANSPORT_IMAGE=images['transport'],P1_OVERLAY_ROOT=str(overlay))
    if any('\n' in v or '\r' in v or '$' in v or '#' in v for v in env_values.values()):
        raise ValueError('Unsupported Compose env path characters')
    env_file = run / 'compose.env'
    env_file.write_text(''.join(k + '=' + v + '\n' for k, v in env_values.items()))
    evidence = dict(images=images, names=names, network=network, partition=network,
                    architecture=architecture,runtime_admitted=candidate is None,
                    gpu_mode=args.gpu_mode,
                    decision='HOLD',candidate_manifest=None if candidate is None else str(args.runtime_candidate),
                    host_ports=[], execution_allowed=False, forwarding_enabled=False,
                    isolated_imu=args.isolated_imu,
                    vio_commit=VIO_PIN, vio_source_sha256=VIO_FILES,
                    simulation_disarmed_vision_requested=args.simulation_disarmed_vision,
                    mode='DISARMED; capture-only; no FCU vision route')
    (run / 'PREPARATION.json').write_text(json.dumps(evidence, indent=2) + '\n')
    compose = ['docker', 'compose', '--env-file', str(env_file), '-f', str(INTEGRATION / 'phase1.compose.yaml')]
    if args.gpu_mode!='software':
        compose+=['-f',str(INTEGRATION/'linux_gpu'/('phase1.gpu.'+args.gpu_mode+'.yaml'))]
    config = subprocess.check_output(compose + ['config'], text=True)
    (run / 'artifacts/resolved-compose.yaml').write_text(config)
    print(run, flush=True)
    if not args.execute:
        if candidate is not None:
            print('PREPARED CANDIDATE ONLY: HOLD, runtime NOT ADMITTED; '+
                  ('explicit DISARMED simulation mode requested; no services started.' if args.simulation_disarmed_vision else '--execute forbidden.'))
        else:
            print('PREPARED ONLY. For execution choose a fresh --run-id and add --execute.')
        return
    audit = (run / 'evidence/launcher.jsonl').open('x')
    def call(command, output,timeout=None):
        with output.open('x') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,timeout=timeout)
        audit.write(json.dumps(dict(command=command, returncode=result.returncode)) + '\n')
        audit.flush()
        if result.returncode:
            raise RuntimeError('Failed; preserved output: ' + str(output))
    owned=None
    try:
        call(compose + ['up', '-d', '--no-build', '--pull', 'never'], run / 'evidence/compose-up.txt',60 if args.simulation_disarmed_vision else None)
        if args.simulation_disarmed_vision:
            owned=json.loads(subprocess.check_output(['docker','inspect',*names.values()],text=True,timeout=10))
            if (len(owned)!=4 or any(r['Config'].get('Labels',{}).get('com.docker.compose.project')!=network
                    or not re.fullmatch('[0-9a-f]{64}',r['Id']) for r in owned)):
                owned=None
                raise ValueError('Cannot establish owned simulation container IDs')
            (run/'evidence/simulation-owned-initial.json').write_text(json.dumps(owned,indent=2)+'\n')
            from linux_gpu.run_source_runtime import renderer_preflight
            def renderer_call(command,label,timeout):
                call(command,run/'evidence'/(label+'.txt'),timeout)
            renderer_preflight(names['physics'],'/usr/share/glvnd/egl_vendor.d/10_nvidia.json',renderer_call)
            code=("from pathlib import Path; from vio_source_probe import hardware_renderer; "
                  "s=Path('/tmp/render_gl_identity.txt').read_text(); r=hardware_renderer(s); "
                  "assert 'vendor=NVIDIA Corporation' in s; "
                  "assert r in Path('/root/.gz/rendering/ogre2.log').read_text(); print(r)")
            call(['docker','exec',names['physics'],'python3','-B','-c',code],run/'evidence/simulation-renderer.txt')
        mount_command = ['docker', 'exec', names['physics'], 'bash', '-lc',
                         'source /opt/ros/jazzy/setup.bash; exec python3 -B ' +
                         shlex.quote('/opt/integration/isolated_blockers/20261109_a/code/probe_experiment.py') +
                         ' --run /run/integration --variant integrated --duration 30']
        call(mount_command, run / 'evidence/mount.console.txt')
        mount = json.loads((run / 'evidence/integrated/summary.json').read_text())
        if (mount['mount_status'] != 'PASS' or not mount['settled_snapshots']
                or max(mount['max_angles_deg']) > 1 or mount['unexpected_commands']):
            raise RuntimeError('Actual mount HOLD; no VIO started')
        shutil.copyfile(run / 'evidence/integrated/summary.json', run / 'artifacts/mount_calibration.json')
        extra=[]
        if args.simulation_disarmed_vision:
            from simulation_vision import prepare_permit
            permit=prepare_permit(run,args.runtime_candidate,origin,approved=True)
            extra=['--simulation-vision-permit',str(permit)]
        call([sys.executable, '-B', str(INTEGRATION / 'run_phase1_validation.py'), '--run', str(run),
              '--run-id', args.run_id]+(['--isolated-imu'] if args.isolated_imu else [])+extra, run / 'evidence/validation.console.txt',180 if args.simulation_disarmed_vision else None)
    finally:
        # Exact allowlist; never docker compose down, prune, stop-all or removal.
        cleanup_errors=[]
        for kind in ('transport', 'vio', 'fcu', 'physics'):
            if args.simulation_disarmed_vision:
                try:
                    initial=next(r for r in (owned or []) if r['Name'].lstrip('/')==names[kind])
                    current=json.loads(subprocess.check_output(['docker','inspect',names[kind]],text=True,timeout=10))[0]
                    if current['Id']!=initial['Id']:raise ValueError('Container name replaced; no signal sent')
                    result=subprocess.run(['docker','stop','--time','8',initial['Id']],capture_output=True,text=True,timeout=15)
                    final=json.loads(subprocess.check_output(['docker','inspect',initial['Id']],text=True,timeout=10))[0]
                    if result.returncode or final['State']['Running']:raise ValueError('Owned container not confirmed stopped')
                    audit.write(json.dumps(dict(cleanup_id=initial['Id'],stopped=True))+'\n')
                except (OSError,ValueError,StopIteration,subprocess.SubprocessError) as exc:
                    cleanup_errors.append(str(exc));audit.write(json.dumps(dict(cleanup=names[kind],error=str(exc)))+'\n')
                continue
            result = subprocess.run(['docker', 'stop', '--time', '8', names[kind]], capture_output=True, text=True)
            audit.write(json.dumps(dict(cleanup=names[kind], returncode=result.returncode,
                                        stdout=result.stdout, stderr=result.stderr)) + '\n')
        audit.close()
        if cleanup_errors:raise RuntimeError('Simulation cleanup INCOMPLETE: '+'; '.join(cleanup_errors))


if __name__ == '__main__':
    main()
