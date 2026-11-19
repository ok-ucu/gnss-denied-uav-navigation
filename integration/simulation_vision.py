"""Opt-in DISARMED vision route for an owned, internal Gazebo/SITL namespace.

The host issues a run-bound permit after read-only Docker/build inspection.
This is operator approval, not cryptographic host attestation or flight approval.
No Docker API/socket is exposed to the transport. Default capture is unchanged.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

from runtime_candidate import ROOT, AP_COMMIT, load_candidate, sha
from linux_gpu.verified_disarmed import HeartbeatGuard
from vio_adapter import expand_upper_triangle, validate_covariance

KINDS = {'VISION_POSITION_ESTIMATE', 'VISION_SPEED_ESTIMATE'}
SCOPE = 'gazebo-sitl-disarmed-vision'
BOUND_SOURCES = ('phase1_transport.py', 'simulation_vision.py', 'physics_clock_mapping.py',
                 'navigation_lifecycle.py', 'navigation_egress.py', 'phase1_capture_mavros.yaml',
                 'launch_phase1.py', 'run_phase1_validation.py', 'preflight_ekf3.py',
                 'phase1.compose.yaml', 'linux_gpu/phase1.gpu.nvidia.yaml')


def record_local_pair_half(expected, kind, local_send):
    """Return one completion record after two validated local socket writes."""
    halves = expected.setdefault('local_wire_halves', {})
    if kind not in KINDS or kind in halves:
        return None
    halves[kind] = local_send['end_host_ns']
    if halves.keys() != KINDS:
        return None
    meta = expected['meta']
    return dict(event='simulation_local_pair_complete', acquisition_ns=meta['acquisition_ns'],
                fcu_ns=meta['fcu_ns'], host_ns=max(halves.values()), half_end_host_ns=dict(halves),
                reference=meta['reference'], producer=meta['producer'], epoch=meta['epoch'])



def process_identity():
    return dict(netns=os.readlink('/proc/self/ns/net'),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                init_starttime=Path('/proc/1/stat').read_text().rsplit(') ', 1)[1].split()[19],
                hostname=Path('/etc/hostname').read_text().strip())


def origin_values(values):
    values = list(map(float, values))
    if (len(values) != 3 or not all(map(math.isfinite, values)) or
            not -90 <= values[0] <= 90 or not -180 <= values[1] <= 180):
        raise ValueError('Explicit finite simulation origin lat/lon/alt required')
    return [round(values[0]*1e7), round(values[1]*1e7), round(values[2]*1000)]


def inspect_environment(records, network, run, candidate, processes):
    """Pure validation of actual Docker inspect/proc outputs; never names alone."""
    identity = json.loads((run/'artifacts/run_identity.json').read_text())
    run_id = identity['run_id']; project = 'gnss-p1-integrated-'+run_id
    names = {kind: 'gnss-p1-'+kind+'-'+run_id for kind in ('physics', 'fcu', 'vio', 'transport')}
    by_name = {r['Name'].lstrip('/'): r for r in records}
    if len(records) != 4 or set(by_name) != set(names.values()):
        raise ValueError('Require exact four owned simulation containers')
    if network.get('Name') != project or network.get('Internal') is not True:
        raise ValueError('Require internal run-specific network')
    physics = by_name[names['physics']]
    for kind, name in names.items():
        row = by_name[name]; h = row['HostConfig']
        if (not re.fullmatch('[0-9a-f]{64}', row['Id']) or not row['State']['Running'] or
                row['Config'].get('Labels', {}).get('com.docker.compose.project') != 'gnss-phase1-integrated-'+run_id or
                row['Image'] != candidate['images'][kind if kind != 'vio' else 'physics'] or
                h.get('Privileged') or h.get('CapAdd') or h.get('PidMode') or h.get('Devices') or
                h['RestartPolicy']['Name'] != 'no' or h.get('PortBindings') or
                any(row.get('NetworkSettings', {}).get('Ports', {}).values())):
            raise ValueError('Unsafe container identity/devices/ports: '+kind)
        if kind != 'physics' and h.get('DeviceRequests'):
            raise ValueError('Unexpected device request outside renderer')
        if processes is not None and processes[kind]['hostname'] != row['Config']['Hostname']:
            raise ValueError('Docker/process hostname identity mismatch')
        expected_mode = project if kind == 'physics' else 'container:'+physics['Id']
        if h['NetworkMode'] != expected_mode:
            raise ValueError('Wrong simulation network namespace: '+kind)
        mounts = {m['Destination']: m for m in row['Mounts']}
        if (mounts.get('/opt/integration', {}).get('Source') != str(ROOT.resolve()) or
                mounts['/opt/integration']['RW'] is not False or
                mounts.get('/run/integration', {}).get('Source') != str(run.resolve())):
            raise ValueError('Wrong source/run mount')
        allowed = {'/opt/integration', '/run/integration', '/opt/vio', '/tmp/patch_install',
                   '/var/lib/phase1-guard', '/var/lib/phase1-control'}
        if kind == 'physics':
            allowed.add('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
        if set(mounts)-allowed:
            raise ValueError('Unexpected mount/device/socket exposure: '+kind)
    if set(physics['NetworkSettings']['Networks']) != {project}:
        raise ValueError('Unexpected additional physics network')
    if processes is None:
        return {kind: by_name[name]['Id'] for kind, name in names.items()}
    ns = processes['transport']['netns']; boot = processes['transport']['boot_id']
    if any(p['netns'] != ns or p['boot_id'] != boot for p in processes.values()):
        raise ValueError('Kernel namespace/boot mismatch')
    physics_cmd=processes['physics']['cmdline']
    if len(physics_cmd) == 1:
        import shlex
        physics_cmd = shlex.split(physics_cmd[0])
    if not all(v in physics_cmd for v in ('sim','-s','-r','--headless-rendering','/run/integration/models/world.sdf')):
        raise ValueError('Physics process is not the owned headless Gazebo world; PID1 cmdline='+repr(physics_cmd))
    fcu = processes['fcu']
    cmd = fcu['cmdline']
    if (fcu['exe_sha256'] != candidate['firmware_sha256'] or
            cmd[0] != '/run/integration/artifacts/arducopter' or
            '--model' not in cmd or cmd[cmd.index('--model')+1] != 'JSON' or
            '--serial0' not in cmd or cmd[cmd.index('--serial0')+1] != 'tcp:5760' or
            '--sim-port-out' not in cmd or cmd[cmd.index('--sim-port-out')+1] != '9002'):
        raise ValueError('FCU is not the pinned matched-physics SITL process')
    return {kind: by_name[name]['Id'] for kind, name in names.items()}


def prepare_permit(run, candidate_path, origin, *, approved=False, execute=subprocess.check_output):
    """Only the explicit simulation launcher calls with approved=True; no startup."""
    if approved is not True:
        raise ValueError('Explicit DISARMED simulation vision approval required')
    run = Path(run).resolve(); candidate_path = Path(candidate_path).resolve()
    if not run.is_relative_to(ROOT/'phase1_runs'):
        raise ValueError('Require integration-owned run directory')
    candidate = load_candidate(candidate_path)  # ELF, source, patches, parser, MAVROS2.16 pin.
    identity = json.loads((run/'artifacts/run_identity.json').read_text())
    if (identity.get('execution_allowed') is not False or identity.get('forwarding_enabled') is not False or
            identity.get('firmware_sha256') != candidate['firmware_sha256'] or
            sha(run/'artifacts/arducopter') != candidate['firmware_sha256']):
        raise ValueError('Prepared firmware/safety identity mismatch')
    project = 'gnss-p1-integrated-'+identity['run_id']
    names = ['gnss-p1-'+kind+'-'+identity['run_id'] for kind in ('physics', 'fcu', 'vio', 'transport')]
    def call(args):
        return execute(args, text=True, timeout=10)
    records = json.loads(call(['docker', 'inspect', *names]))
    network = json.loads(call(['docker', 'network', 'inspect', project]))[0]
    inspect_environment(records, network, run, candidate, None)
    probe = ('import json,os,hashlib; from pathlib import Path; '
             'p=Path("/proc/1"); print(json.dumps(dict('
             'netns=os.readlink("/proc/self/ns/net"), '
             'boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(), '
             'init_starttime=(p/"stat").read_text().rsplit(") ",1)[1].split()[19], '
             'hostname=Path("/etc/hostname").read_text().strip(), '
             'exe_sha256=hashlib.sha256((p/"exe").read_bytes()).hexdigest(), '
             'cmdline=(p/"cmdline").read_bytes().decode().rstrip("\\0").split("\\0"))))')
    # Reject unsafe metadata before exec; process evidence is then checked again.
    processes = {}
    for kind, row in zip(('physics', 'fcu', 'vio', 'transport'), records):
        if row['Name'].lstrip('/') != names[len(processes)] or not re.fullmatch('[0-9a-f]{64}', row['Id']):
            raise ValueError('Unexpected Docker inspect ordering/identity')
        processes[kind] = json.loads(call(['docker', 'exec', row['Id'], 'python3', '-B', '-c', probe]))
    ids = inspect_environment(records, network, run, candidate, processes)
    after = json.loads(call(['docker', 'inspect', *names]))
    if [(r['Id'], r['State']['StartedAt'], r['State']['Running']) for r in records] != [
            (r['Id'], r['State']['StartedAt'], r['State']['Running']) for r in after]:
        raise ValueError('Container replaced/restarted during simulation admission')
    permit = dict(schema=1, scope=SCOPE, approved=True, run_id=identity['run_id'],
        reference_nonce=identity['reference_nonce'], clock_epoch=identity['clock_epoch'],
        candidate=str(candidate_path.relative_to(ROOT)), candidate_sha256=sha(candidate_path),
        firmware_sha256=candidate['firmware_sha256'], ap_commit=AP_COMMIT,
        origin=origin_values(origin), containers=ids, transport_identity=processes['transport'],
        source_hashes={n: sha(ROOT/n) for n in BOUND_SOURCES},
        execution_allowed=False, forwarding_enabled=False, native_verified=False,
        scope_notice='Only simulation vision; no ARM/mode/takeoff; not proof of FCU fusion')
    path = run/'artifacts/simulation_vision_permit.json'
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as f:
        json.dump(permit, f, indent=2); f.write('\n'); f.flush(); os.fsync(f.fileno())
    return path


class SimulationVisionRoute:
    def __init__(self, run, path, *, identity_reader=process_identity, heartbeat_factory=HeartbeatGuard):
        run = Path(run); self.permit = p = json.loads(Path(path).read_text())
        identity = json.loads((run/'artifacts/run_identity.json').read_text())
        if (p.get('schema') != 1 or p.get('scope') != SCOPE or p.get('approved') is not True or
                p.get('execution_allowed') is not False or p.get('forwarding_enabled') is not False or
                p.get('ap_commit') != AP_COMMIT or
                any(p.get(k) != identity.get(k) for k in ('run_id', 'reference_nonce', 'clock_epoch'))):
            raise ValueError('Missing/mismatched explicit simulation-only permit')
        if p['source_hashes'] != {n: sha(ROOT/n) for n in BOUND_SOURCES}:
            raise ValueError('Simulation boundary source changed after approval')
        actual = identity_reader()
        if (set(actual) != {'netns','boot_id','init_starttime','hostname'} or
                any(actual[k] != p['transport_identity'].get(k) for k in actual)):
            raise ValueError('Simulation namespace/process identity mismatch')
        if (set(p['containers']) != {'physics','fcu','vio','transport'} or
                any(not re.fullmatch('[0-9a-f]{64}',v) for v in p['containers'].values()) or
                len(p['origin']) != 3 or any(type(v) is not int for v in p['origin']) or
                origin_values([p['origin'][0]/1e7,p['origin'][1]/1e7,p['origin'][2]/1000]) != p['origin']):
            raise ValueError('Malformed simulation identity/origin')
        candidate_path = (ROOT/p['candidate']).resolve()
        if not candidate_path.is_relative_to(ROOT/'linux_gpu_runs'):
            raise ValueError('Simulation candidate must remain under linux_gpu_runs')
        if sha(candidate_path) != p['candidate_sha256']:
            raise ValueError('Candidate manifest changed')
        candidate = load_candidate(candidate_path)
        if (p['firmware_sha256'] != candidate['firmware_sha256'] or
                p['firmware_sha256'] != sha(run/'artifacts/arducopter')):
            raise ValueError('Simulation firmware hash mismatch')
        self.firmware_sha256 = p['firmware_sha256']
        self.heartbeat = heartbeat_factory(1, 1, time.monotonic())
        self.failure = None; self.writes = 0; self.last_send = None

    def accepts_firmware(self, digest):
        return digest == self.firmware_sha256

    def fail(self, reason):
        self.failure = reason
        raise ValueError(reason)

    def ready(self, now):
        if self.failure:
            raise ValueError(self.failure)
        return self.heartbeat.ready(now/1e9)

    def forward(self, msg, expected, gate, last_odom, send, *, clock=time.monotonic_ns):
        """Called ONLY after consuming a real final VisionEgress permission.

        A partial/late local socket write latches failure, never retries/replays.
        Successful local enqueue is NOT evidence of FCU receipt/fusion.
        """
        self.last_send = None
        try:
            now = clock(); kind = msg.get_type()
            if not self.ready(now) or kind not in KINDS or expected is None:
                raise ValueError('Simulation forwarding not ready/matched')
            if not gate.mapping.verified or gate.mapping.broken:
                raise ValueError('Simulation clock mapping unavailable')
            gate.mapping.check_freshness(now)
            if not gate.watchdog(now) or last_odom is None or not 0 <= now-last_odom <= 500_000_000:
                raise ValueError('Simulation VIO unavailable/clock blocked')
            if msg.get_msgbuf()[0] != 253 or msg.reset_counter != 0:
                raise ValueError('Require MAVLink2 epoch0')
            want = expected['wire'][kind]
            values = [msg.x, msg.y, msg.z]
            target = want['position'] if kind == 'VISION_POSITION_ESTIMATE' else want['speed']
            if any(not math.isfinite(a) or not math.isclose(a, b, rel_tol=1e-5, abs_tol=1e-6)
                   for a, b in zip(values, target)):
                raise ValueError('Wire position/velocity frame mismatch')
            cov = list(msg.covariance)
            validate_covariance(expand_upper_triangle(cov) if kind == 'VISION_POSITION_ESTIMATE' else cov,
                                6 if kind == 'VISION_POSITION_ESTIMATE' else 3)
            if len(cov) != len(want['covariance']) or any(
                    not math.isclose(a, b, rel_tol=1e-5, abs_tol=1e-7) for a, b in zip(cov, want['covariance'])):
                raise ValueError('Wire covariance mismatch')
            if kind == 'VISION_POSITION_ESTIMATE' and any(not math.isfinite(a) or abs(math.atan2(math.sin(a-b), math.cos(a-b))) > 1e-5
                    for a, b in zip((msg.roll, msg.pitch, msg.yaw), want['rpy'])):
                raise ValueError('Wire attitude mismatch')
            if msg.usec != expected['meta']['fcu_ns']//1000:
                raise ValueError('Wire acquisition timestamp mismatch')
            now = clock()
            if (not gate.watchdog(now) or not expected['meta']['approved_host_ns'] <= now <= expected['meta']['allowed_host_deadline_ns']):
                raise ValueError('Simulation final permission expired/revoked')
            packet = bytes(msg.get_msgbuf())
            count = send(packet); end = clock()
            self.last_send = dict(start_host_ns=now, end_host_ns=end, bytes=count,
                                  deadline_host_ns=expected['meta']['allowed_host_deadline_ns'])
            if count != len(packet):
                raise ValueError('Partial simulation socket write; no retry')
            self.writes += 1
            if end > expected['meta']['allowed_host_deadline_ns']:
                raise ValueError('Permission expired during socket enqueue; delivery NOT validated')
            return True
        except (ValueError, RuntimeError, OSError) as exc:
            gate.block(str(exc)); self.fail(str(exc))
