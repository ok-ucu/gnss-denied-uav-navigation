"""Prepare/run a NEW disarmed, capture-only Phase 1 stack. No vision FCU route.

No builds, image pulls, flight commands, parameter writes to existing services,
container removal or reference reuse. --execute runs the recorded fault matrix.
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

INTEGRATION = Path(__file__).resolve().parent
VERIFIED = INTEGRATION / 'isolated_blockers/20261109_a'
IMAGES = {
    'physics': 'sha256:6a5215efd7798818523c309eb220753f5755115ed493cd70ada8ae4ed0b61e6b',
    'fcu': 'sha256:f5dffdb9f79ed8a7ff58aa7e6f6dfcbe2ebd2cead9fe7b20daea8d953b491540',
    'transport': 'sha256:acd5f652ca1e72744c6427ed63d7d0805412ead3b1b1da95f2a2f08dcbfa7698',
}


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
    args = parser.parse_args()
    run = INTEGRATION / 'phase1_runs' / args.run_id
    names = {kind: f'gnss-p1-{kind}-{args.run_id}' for kind in ('physics', 'fcu', 'vio', 'transport')}
    network = 'gnss-p1-integrated-' + args.run_id
    if run.exists():
        raise ValueError('Refuse to overwrite a run, including a prepared-only run')
    if subprocess.check_output(['git', 'branch', '--show-current'], cwd=INTEGRATION, text=True).strip() != 'gazebo-sitl-integration':
        raise ValueError('Unexpected branch; no switching performed')
    for name in names.values():
        result = subprocess.run(['docker', 'container', 'inspect', name], capture_output=True)
        if result.returncode == 0:
            raise ValueError('Container name already exists: ' + name)
    if subprocess.run(['docker', 'network', 'inspect', network], capture_output=True).returncode == 0:
        raise ValueError('Network name already exists: ' + network)
    for digest in IMAGES.values():
        arch = subprocess.check_output(['docker', 'image', 'inspect', '--format', '{{.Architecture}}', digest], text=True).strip()
        if arch != 'arm64':
            raise ValueError('Require existing ARM64 image ' + digest)
    files = {relative: verified_file(relative) for relative in (
        'evidence/arducopter_patched', 'models/patched_c/iris_with_gimbal/model.sdf',
        'models/patched_c/iris_with_gimbal/model.config', 'models/patched_c_world.sdf',
        'evidence/patched_c_gimbal_original.sdf', 'code/probe_experiment.py', 'code/mount_math.py')}
    from physics_clock_mapping import PATCHED_SHA
    if hashlib.sha256(files['evidence/arducopter_patched'].read_bytes()).hexdigest() != PATCHED_SHA:
        raise ValueError('Unreviewed FCU binary')
    if not (INTEGRATION / 'timesync_patch_runs/20261108/patch_install/setup.bash').is_file():
        raise ValueError('Tested MAVROS overlay missing; no unpatched fallback')
    for subdir in ('models/iris_with_gimbal', 'artifacts', 'evidence', 'state/fcu'):
        (run / subdir).mkdir(parents=True, exist_ok=True)
    for filename in ('model.sdf', 'model.config'):
        shutil.copyfile(files['models/patched_c/iris_with_gimbal/' + filename], run / 'models/iris_with_gimbal' / filename)
    shutil.copyfile(files['models/patched_c_world.sdf'], run / 'models/world.sdf')
    shutil.copyfile(files['evidence/patched_c_gimbal_original.sdf'], run / 'evidence/integrated_gimbal_original.sdf')
    (run / 'artifacts/run_identity.json').write_text(json.dumps(dict(
        run_id=args.run_id, clock_epoch=str(uuid.uuid4()), reference_nonce=str(uuid.uuid4()),
        execution_allowed=False, forwarding_enabled=False), indent=2) + '\n')
    # This profile is loaded ONLY by the newly named FCU. No PARAM_SET traffic.
    (run / 'artifacts/externalnav.parm').write_text(
        (INTEGRATION / 'ekf3_dry_run.parm').read_text() + '\nSERIAL0_PROTOCOL 2\nLOG_DISARMED 1\n')
    env_values = dict(P1_RUN_ID=args.run_id, INTEGRATION_ROOT=str(INTEGRATION),
                      PHASE1_RUN_ROOT=str(run), VIO_ROOT=str(INTEGRATION.parent / 'vio'))
    if any('\n' in v or '\r' in v or '$' in v or '#' in v for v in env_values.values()):
        raise ValueError('Unsupported Compose env path characters')
    env_file = run / 'compose.env'
    env_file.write_text(''.join(k + '=' + v + '\n' for k, v in env_values.items()))
    evidence = dict(images=IMAGES, names=names, network=network, partition=network,
                    host_ports=[], execution_allowed=False, forwarding_enabled=False,
                    mode='DISARMED; capture-only; no FCU vision route')
    (run / 'PREPARATION.json').write_text(json.dumps(evidence, indent=2) + '\n')
    compose = ['docker', 'compose', '--env-file', str(env_file), '-f', str(INTEGRATION / 'phase1.compose.yaml')]
    config = subprocess.check_output(compose + ['config'], text=True)
    (run / 'artifacts/resolved-compose.yaml').write_text(config)
    print(run, flush=True)
    if not args.execute:
        print('PREPARED ONLY. For execution choose a fresh --run-id and add --execute.')
        return
    audit = (run / 'evidence/launcher.jsonl').open('x')
    def call(command, output):
        with output.open('x') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        audit.write(json.dumps(dict(command=command, returncode=result.returncode)) + '\n')
        audit.flush()
        if result.returncode:
            raise RuntimeError('Failed; preserved output: ' + str(output))
    try:
        call(compose + ['up', '-d', '--no-build', '--pull', 'never'], run / 'evidence/compose-up.txt')
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
        call([sys.executable, '-B', str(INTEGRATION / 'run_phase1_validation.py'), '--run', str(run),
              '--run-id', args.run_id], run / 'evidence/validation.console.txt')
    finally:
        # Exact allowlist; never docker compose down, prune, stop-all or removal.
        for kind in ('transport', 'vio', 'fcu', 'physics'):
            result = subprocess.run(['docker', 'stop', '--time', '8', names[kind]], capture_output=True, text=True)
            audit.write(json.dumps(dict(cleanup=names[kind], returncode=result.returncode,
                                        stdout=result.stdout, stderr=result.stderr)) + '\n')
        audit.close()


if __name__ == '__main__':
    main()
