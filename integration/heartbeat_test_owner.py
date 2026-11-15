"""Stage 1 test-only ownership and cleanup. No FCU or workload launcher.

Portable policy is dependency-injected. Linux authority uses retained pidfds
and an open, verified cgroup v2 directory, NEVER a numeric group signal. The
host manager integration is deliberately unavailable until separately reviewed.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import sys
import time
import uuid


class Blocked(RuntimeError):
    """Admission/authority unavailable; do not release a workload barrier."""


def canonical(value):
    return (json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':'))+'\n').encode()


def durable_new(path, value):
    """Atomic no-overwrite JSON publication; fsync file AND directory.

    Requires a trusted, private parent directory (host-manager responsibility).
    An exception, including directory fsync failure, is not durability success.
    """
    path = Path(path)
    temporary = path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.pending')
    try:
        with temporary.open('xb') as out:
            out.write(canonical(value)); out.flush(); os.fsync(out.fileno())
        os.link(str(temporary), str(path))  # atomic publish, EEXIST never overwritten
        temporary.unlink()
        fd = os.open(str(path.parent), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try: os.fsync(fd)
        finally: os.close(fd)
    finally:
        if temporary.exists(): temporary.unlink()


@dataclass(frozen=True)
class Deadline:
    """Absolute HOST CLOCK_MONOTONIC nanoseconds; restored unchanged on restart."""
    start_ns: int
    end_ns: int
    term_ns: int
    kill_ns: int
    verify_ns: int
    export_ns: int
    boot_id: str
    time_namespace: str

    def __post_init__(self):
        values = (self.start_ns, self.end_ns, self.term_ns, self.kill_ns,
                  self.verify_ns, self.export_ns)
        if (any(type(v) is not int for v in values) or self.start_ns < 0 or
                any(v <= 0 for v in values[2:]) or
                self.end_ns-self.start_ns <= sum(values[2:]) or
                not self.boot_id or not self.time_namespace):
            raise Blocked('Invalid/unapproved absolute deadline or sub-budgets')

    def remaining(self, now, reserve_ns=0):
        if type(now) is not int or now < self.start_ns:
            raise Blocked('Host monotonic clock reversed or wrong domain')
        return max(0, self.end_ns-reserve_ns-now)/1e9

    def check_domain(self, boot_id, time_namespace):
        if (boot_id, time_namespace) != (self.boot_id, self.time_namespace):
            raise Blocked('Deadline boot/time namespace mismatch; never rebase')


@dataclass(frozen=True)
class ProcessIdentity:
    host_pid: int
    container_pid: int
    ppid: int
    pgid: int
    sid: int
    starttime: int
    boot_id: str
    namespaces: dict
    cgroup: str
    role: str

    def __post_init__(self):
        if (any(type(v) is not int for v in (self.host_pid,self.container_pid,self.ppid,
                self.pgid,self.sid,self.starttime)) or self.host_pid <= 1 or self.container_pid <= 0 or
                self.ppid < 0 or min(self.pgid,self.sid,self.starttime) <= 0 or not self.boot_id or
                not {'pid','mnt','net','time','user'}.issubset(self.namespaces) or
                any(not isinstance(v,str) or not v for v in self.namespaces.values()) or
                not self.cgroup.startswith('/') or '..' in Path(self.cgroup).parts or not self.role):
            raise Blocked('Incomplete Linux process identity')


def linux_process(pid, role, proc=Path('/proc')):
    """Read via host /proc; NSpid's final entry is the innermost namespace PID.

    Snapshot is evidence, not signal authority. bind_process() rechecks it around
    pidfd acquisition. PPID is recorded but can legitimately change on orphaning.
    """
    base = Path(proc)/str(pid)
    text = (base/'stat').read_text()
    fields = text[text.rfind(')')+2:].split()  # comm can contain spaces/parentheses
    namespaces = {n: os.readlink(str(base/'ns'/n)) for n in ('pid', 'mnt', 'net', 'time', 'user')}
    status = (base/'status').read_text().splitlines()
    nsrow = next((r for r in status if r.startswith('NSpid:')), None)
    if nsrow is None: raise Blocked('Host NSpid identity unavailable')
    nspids = [int(v) for v in nsrow.split()[1:]]
    if not nspids or nspids[0] != pid: raise Blocked('Not a verified host /proc PID view')
    groups = (base/'cgroup').read_text().splitlines()
    unified = [r[3:] for r in groups if r.startswith('0::')]
    if len(unified) != 1: raise Blocked('Require unified cgroup v2 membership')
    return ProcessIdentity(pid, nspids[-1], int(fields[1]), int(fields[2]), int(fields[3]),
                           int(fields[19]), (Path(proc)/'sys/kernel/random/boot_id').read_text().strip(),
                           namespaces, unified[0], role)


def same_process(expected, current):
    # Reparenting is allowed, but PID generation/session/namespace drift is not.
    return all(getattr(expected, k) == getattr(current, k) for k in (
        'host_pid', 'container_pid', 'pgid', 'sid', 'starttime', 'boot_id',
        'namespaces', 'cgroup', 'role'))


@dataclass(frozen=True)
class ContainerIdentity:
    full_id: str
    name: str
    created: str
    image_id: str

    def __post_init__(self):
        if (not re.fullmatch(r'[a-f0-9]{64}', self.full_id) or not self.created or
                not self.image_id or not self.name):
            raise Blocked('Require original full container ID/Created/image/name')

    @classmethod
    def capture(cls, row):
        return cls(row['Id'], row['Name'].lstrip('/'), row['Created'], row['Image'])

    def matches(self, row):
        return self == ContainerIdentity.capture(row)


@dataclass(frozen=True)
class CgroupIdentity:
    path: str
    device: int
    inode: int

    def __post_init__(self):
        if (not Path(self.path).is_absolute() or '..' in Path(self.path).parts or
                type(self.device) is not int or type(self.inode) is not int or
                self.device < 0 or self.inode <= 0): raise Blocked('Incomplete cgroup identity')


@dataclass(frozen=True)
class Ledger:
    test_id: str
    container: ContainerIdentity
    cgroup: CgroupIdentity
    processes: tuple
    deadline: Deadline

    def __post_init__(self):
        if str(uuid.UUID(self.test_id)) != self.test_id: raise Blocked('Require canonical unique UUID')
        if not self.processes or len({p.host_pid for p in self.processes}) != len(self.processes):
            raise Blocked('Require unique registered process identities')
        if any(p.host_pid <= 1 or p.starttime <= 0 or not p.role or not p.namespaces or
               p.boot_id != self.deadline.boot_id for p in self.processes):
            raise Blocked('Incomplete process ownership')
        allowed = {'test-init', 'coordinator', 'witness', 'collector', 'control', 'proxy', 'synthetic-child'}
        if any(p.role not in allowed for p in self.processes):
            raise Blocked('Not a permitted test-only workload role')

    def record(self):
        return dict(schema=1, **asdict(self), execution_allowed=False, forwarding_enabled=False)

    def digest(self):
        return hashlib.sha256(canonical(self.record())).hexdigest()

    def publish(self, directory):
        durable_new(Path(directory)/'ownership.json', self.record())

    @classmethod
    def restore(cls, row):
        if (row.get('schema') != 1 or row.get('execution_allowed') is not False or
                row.get('forwarding_enabled') is not False): raise Blocked('Invalid ledger safety/schema')
        return cls(row['test_id'], ContainerIdentity(**row['container']), CgroupIdentity(**row['cgroup']),
                   tuple(ProcessIdentity(**p) for p in row['processes']), Deadline(**row['deadline']))


class CleanupEvents:
    """One exclusive file per recovery attempt; no concurrent appenders."""
    def __init__(self, path, test_id, clock=time.monotonic_ns):
        self.path, self.test_id, self.clock = Path(path), test_id, clock
        self.file = self.path.open('xb'); self.sequence = 0

    def emit(self, action, **data):
        row = dict(test_id=self.test_id, sequence=self.sequence, host_monotonic_ns=self.clock(),
                   action=action, **data)
        self.file.write(canonical(row)); self.file.flush(); os.fsync(self.file.fileno())
        self.sequence += 1

    def close(self): self.file.close()


def verify_receipt(path, ledger):
    """No size-only evidence admission, and no absolute evidence-directory binding."""
    row = json.loads(Path(path).read_text())
    if (row.get('schema') != 1 or row.get('test_id') != ledger.test_id or
            row.get('ledger_sha256') != ledger.digest() or
            row.get('deadline_ns') != ledger.deadline.end_ns or
            row.get('execution_allowed') is not False or row.get('forwarding_enabled') is not False):
        raise Blocked('Incomplete/wrong cleanup receipt identity')
    for field in ('events', 'errors', 'terminal', 'start_ns', 'end_ns', 'status', 'deadline_met', 'foreign_preserved'):
        if field not in row: raise Blocked('Incomplete cleanup receipt: '+field)
    if (row.get('container') != asdict(ledger.container) or row.get('cgroup') != asdict(ledger.cgroup) or
            type(row['start_ns']) is not int or type(row['end_ns']) is not int or
            not ledger.deadline.start_ns <= row['start_ns'] <= row['end_ns']):
        raise Blocked('Invalid cleanup identity/timeline')
    event = row['events']; relative = Path(event['path'])
    if relative.is_absolute() or len(relative.parts) != 1: raise Blocked('Nonportable event path')
    raw = (Path(path).parent/relative).read_bytes()
    lines = [json.loads(line) for line in raw.splitlines()]
    if (not lines or len(lines) != event['count'] or len(raw) != event['bytes'] or
            hashlib.sha256(raw).hexdigest() != event['sha256'] or
            any(r['test_id'] != ledger.test_id or r['sequence'] != i for i, r in enumerate(lines))):
        raise Blocked('Corrupt/incomplete cleanup events')
    stamps = [r['host_monotonic_ns'] for r in lines]
    if (lines[0]['action'] != 'begin' or lines[-1]['action'] != 'finish' or
            lines[0].get('ledger_sha256') != ledger.digest() or
            any(type(t) is not int for t in stamps) or
            any(b < a for a,b in zip(stamps, stamps[1:])) or
            stamps[0] < row['start_ns'] or stamps[-1] > row['end_ns']):
        raise Blocked('Incomplete cleanup event lifecycle/timeline')
    if row['status'] not in ('PASS', 'FAIL', 'INCOMPLETE'):
        raise Blocked('Invalid cleanup receipt status')
    if row['status'] == 'PASS' and (row['errors'] or row['deadline_met'] is not True or
            row['end_ns'] >= ledger.deadline.end_ns or row['foreign_preserved'] is not True or
            row['terminal'] != dict(cgroup_empty=True, docker_stopped=True, processes_exited=True)):
        raise Blocked('Cleanup PASS missing terminal proof')
    return row


class CleanupController:
    """Backend calls MUST be bounded by the supplied absolute phase deadline.

    backend.verify/signal_term/kill/wait_empty/stop_container/terminal are trusted
    capabilities, not assertions supplied by the workload. See Stage 1 contract.
    """
    def __init__(self, ledger, backend, clock=time.monotonic_ns):
        self.ledger, self.backend, self.clock = ledger, backend, clock

    def run(self, directory):
        ledger, budget = self.ledger, self.ledger.deadline
        start = self.clock(); errors = []; violation = False; terminal = {}; foreign = None
        events = None; event_path = Path(directory)/('cleanup.'+uuid.uuid4().hex+'.jsonl')
        def record(action, **value):
            if events is not None:
                try: events.emit(action, **value)
                except Exception as exc: errors.append('evidence: '+str(exc))
        def operation(action, function, *args):
            try:
                result = function(*args); record(action, result=result); return result
            except Exception as exc:
                errors.append(action+': '+str(exc)); record(action, error=str(exc)); return None
        try: events = CleanupEvents(event_path, ledger.test_id, self.clock)
        except Exception as exc: errors.append('events unavailable: '+str(exc))
        record('begin', ledger_sha256=ledger.digest(), deadline_ns=budget.end_ns,
               expected_container=asdict(ledger.container), expected_cgroup=asdict(ledger.cgroup),
               expected_processes=[asdict(p) for p in ledger.processes])
        # Always revalidate kernel authority, even if Docker inspection is broken.
        verified = operation('ownership', self.backend.verify, ledger, budget.end_ns)
        if verified is True:
            term_end = min(self.clock()+budget.term_ns,
                           budget.end_ns-budget.kill_ns-budget.verify_ns-budget.export_ns)
            if self.clock() < term_end:
                operation('term', self.backend.signal_term, ledger, term_end)
                operation('term_wait', self.backend.wait_empty, term_end)
            kill_end = budget.end_ns-budget.verify_ns-budget.export_ns
            # Emergency containment is still attempted when the deadline is gone;
            # never turn a deadline violation into a fresh timeout budget.
            operation('kill', self.backend.kill, ledger, kill_end)
            if self.clock() < kill_end: operation('kill_wait', self.backend.wait_empty, kill_end)
            docker_end = budget.end_ns-budget.export_ns
            if self.clock() < docker_end:
                operation('docker_cleanup', self.backend.stop_container, ledger, docker_end)
                terminal = operation('terminal', self.backend.terminal, ledger, docker_end) or {}
                foreign = operation('foreign_preservation', self.backend.foreign_preservation, ledger, docker_end)
        if any(v is False for v in terminal.values()): violation = True
        if foreign is False: violation = True
        deadline_met = self.clock() < budget.end_ns
        if not deadline_met: violation = True; errors.append('absolute deadline exceeded')
        proven = terminal == dict(cgroup_empty=True, docker_stopped=True, processes_exited=True)
        status = 'FAIL' if violation else ('PASS' if proven and foreign is True and not errors else 'INCOMPLETE')
        record('finish', status=status, terminal=terminal, deadline_met=deadline_met)
        if events is not None:
            try: events.close()
            except Exception as exc: errors.append('events close: '+str(exc))
        try:
            raw = event_path.read_bytes()
            event_info = dict(path=event_path.name, bytes=len(raw), count=len(raw.splitlines()),
                              sha256=hashlib.sha256(raw).hexdigest())
            if errors and status == 'PASS': status = 'INCOMPLETE'
            receipt = dict(schema=1, test_id=ledger.test_id, ledger_sha256=ledger.digest(),
                container=asdict(ledger.container), cgroup=asdict(ledger.cgroup),
                start_ns=start, end_ns=self.clock(), deadline_ns=budget.end_ns,
                deadline_met=self.clock() < budget.end_ns, status=status, terminal=terminal,
                foreign_preserved=foreign, errors=errors, events=event_info,
                execution_allowed=False, forwarding_enabled=False)
            if not receipt['deadline_met']: receipt['status'] = 'FAIL'
            path = Path(directory)/'cleanup.receipt.json'
            durable_new(path, receipt)
            verify_receipt(path, ledger)
            # Export itself is in the SAME deadline. No final completion marker
            # is produced here; host manager must verify this post-write result.
            if self.clock() >= budget.end_ns:
                return dict(status='FAIL', error='deadline exceeded during receipt finalization',
                            receipt_complete=False)
            return receipt
        except Exception as exc:
            return dict(status='FAIL' if violation or self.clock() >= budget.end_ns else 'INCOMPLETE', receipt_complete=False,
                        error='cleanup receipt incomplete: '+str(exc), errors=errors)


class LinuxPidfdCgroup:
    """Kernel capability, acquired by a privileged reviewed HOST manager only.

    Explicit mount root + registered identity required. Never creates/migrates
    cgroups, discovers containers or signals a PGID. No systemd integration yet.
    """
    def __init__(self, identity, mount=Path('/sys/fs/cgroup')):
        if sys.platform != 'linux' or not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
            raise Blocked('Native Linux pidfd/cgroup v2 required')
        self.identity, self.mount, self.pidfds = identity, Path(mount).resolve(), {}
        path = Path(identity.path)
        if not path.is_absolute() or '..' in path.parts or path == self.mount or self.mount not in path.parents:
            raise Blocked('Refuse root/outside cgroup containment')
        # Confirm an actual cgroup2 mount, not ordinary files simulating one.
        mount_lines = Path('/proc/self/mountinfo').read_text().splitlines()
        if not any(line.split()[4] == str(self.mount) and line.split(' - ')[1].startswith('cgroup2 ')
                   for line in mount_lines): raise Blocked('Unverified cgroup v2 mount')
        flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        # Reject symlinks in ANY component beneath the mount.
        fd = os.open(str(self.mount), flags)
        try:
            for part in path.relative_to(self.mount).parts:
                child = os.open(part, flags, dir_fd=fd); os.close(fd); fd = child
            self.fd = fd; fd = None
            try: self.verify()
            except BaseException:
                os.close(self.fd)
                raise
        finally:
            if fd is not None: os.close(fd)

    def verify(self):
        info = os.fstat(self.fd)
        if (info.st_dev, info.st_ino) != (self.identity.device, self.identity.inode):
            raise Blocked('Cgroup handle identity mismatch')
        current = os.stat(self.identity.path, follow_symlinks=False)
        if not stat.S_ISDIR(current.st_mode) or (current.st_dev, current.st_ino) != (info.st_dev, info.st_ino):
            raise Blocked('Cgroup path replaced; no guessed-path fallback')

    def bind_process(self, identity):
        self.verify()
        before = linux_process(identity.host_pid, identity.role)
        if not same_process(identity, before): raise Blocked('PID/starttime/namespace mismatch')
        expected_path = self.mount/identity.cgroup.lstrip('/')
        if expected_path != Path(self.identity.path) and Path(self.identity.path) not in expected_path.parents:
            raise Blocked('Foreign process outside registered containment')
        descriptor = os.pidfd_open(identity.host_pid, 0)
        try:
            if not same_process(identity, linux_process(identity.host_pid, identity.role)):
                raise Blocked('Process changed during pidfd acquisition')
            if identity.host_pid in self.pidfds: raise Blocked('Process already bound')
            self.pidfds[identity.host_pid] = (identity, descriptor); descriptor = None
        finally:
            if descriptor is not None: os.close(descriptor)

    def term(self, identity):
        self.verify()
        registered, descriptor = self.pidfds[identity.host_pid]
        if not same_process(identity, registered): raise Blocked('Unregistered pidfd identity')
        # pidfd eliminates PID reuse between check and signal. Migration by a
        # privileged external actor remains a separately documented host risk.
        try:
            current = linux_process(identity.host_pid, identity.role)
        except FileNotFoundError:
            return  # Exited; the descriptor can never address a replacement PID.
        if not same_process(identity, current): raise Blocked('Ownership changed before TERM')
        try: signal.pidfd_send_signal(descriptor, signal.SIGTERM)
        except ProcessLookupError: pass

    def kill(self):
        self.verify()
        fd = os.open('cgroup.kill', os.O_WRONLY | os.O_NOFOLLOW, dir_fd=self.fd)
        try: os.write(fd, b'1')
        finally: os.close(fd)

    def empty(self):
        self.verify()
        fd = os.open('cgroup.events', os.O_RDONLY | os.O_NOFOLLOW, dir_fd=self.fd)
        try:
            with os.fdopen(fd) as stream: values = dict(line.split() for line in stream)
        except BaseException:
            raise
        if values.get('populated') not in ('0', '1'): raise Blocked('Missing cgroup population proof')
        return values['populated'] == '0'  # Zombie reaping must be verified separately.

    def close(self):
        for _, descriptor in self.pidfds.values(): os.close(descriptor)
        self.pidfds.clear(); os.close(self.fd)


class DockerByIdentity:
    """Bounded Docker adapter, no container creation/removal or name targeting."""
    def __init__(self, run=None, clock=time.monotonic_ns):
        self.run, self.clock = run if run is not None else subprocess.run, clock

    def inspect(self, selector, until_ns):
        remaining = (until_ns-self.clock())/1e9
        if remaining <= 0: raise Blocked('Docker inspect deadline expired')
        answer = self.run(['docker', 'inspect', selector], capture_output=True, text=True, timeout=remaining)
        if answer.returncode: raise Blocked('Docker inspect failed: '+selector+': '+answer.stderr)
        rows = json.loads(answer.stdout)
        if len(rows) != 1: raise Blocked('Docker inspect ambiguous/missing result')
        return rows[0]

    def verify(self, identity, until_ns):
        row = self.inspect(identity.full_id, until_ns)
        if not identity.matches(row): raise Blocked('Original container identity mismatch')
        # Names are cross-checks ONLY, never action targets. Replacement blocks.
        named = self.inspect(identity.name, until_ns)
        if not identity.matches(named): raise Blocked('Same-name replacement container')
        return row

    def stop(self, identity, until_ns):
        self.verify(identity, until_ns)
        remaining = (until_ns-self.clock())/1e9
        if remaining <= 0: raise Blocked('Docker stop deadline expired')
        # This is an administrative cleanup budget, not a navigation threshold.
        answer = self.run(['docker', 'stop', '--timeout', str(max(0, math.floor(remaining)-1)),
                           identity.full_id], capture_output=True, text=True, timeout=remaining)
        if answer.returncode: raise Blocked('Docker stop failed: '+answer.stderr)


class LinuxCleanupBackend:
    """Explicit backend integration contract; manager must provide exit/reap proof.

    Kernel and Docker checks are independent: Docker failure never cancels safe
    pre-registered kernel containment, but final cleanup cannot report PASS.
    """
    def __init__(self, kernel, docker, process_exit_proof, foreign_proof,
                 clock=time.monotonic_ns, sleep=time.sleep):
        self.kernel, self.docker, self.process_exit_proof = kernel, docker, process_exit_proof
        self.foreign_proof = foreign_proof
        self.clock, self.sleep = clock, sleep

    def verify(self, ledger, until):
        self.kernel.verify()
        if self.kernel.identity != ledger.cgroup: raise Blocked('Wrong containment authority')
        ledger.deadline.check_domain(Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                                    os.readlink('/proc/self/ns/time'))
        own = linux_process(os.getpid(), 'recovery')
        own_path = self.kernel.mount/own.cgroup.lstrip('/')
        if (own_path == Path(ledger.cgroup.path) or Path(ledger.cgroup.path) in own_path.parents or
                os.getpid() in {p.host_pid for p in ledger.processes}):
            raise Blocked('Recovery inside workload containment')
        for process in ledger.processes:
            registered = self.kernel.pidfds.get(process.host_pid)
            if not registered or not same_process(process, registered[0]):
                raise Blocked('Missing registered pidfd')
        return True

    def signal_term(self, ledger, until):
        for process in ledger.processes:
            if self.clock() >= until: raise Blocked('TERM phase budget expired')
            self.kernel.term(process)
        return True

    def wait_empty(self, until):
        while self.clock() < until:
            if self.kernel.empty(): return True
            self.sleep(min(.01, max(0, (until-self.clock())/1e9)))
        return self.kernel.empty()

    def kill(self, ledger, until): self.kernel.kill(); return True
    def stop_container(self, ledger, until): self.docker.stop(ledger.container, until); return True

    def terminal(self, ledger, until):
        empty = self.kernel.empty()
        row = self.docker.verify(ledger.container, until)
        state = row.get('State', {})
        stopped = (state.get('Running') is False and state.get('Paused') is False and
                   state.get('Restarting') is False and state.get('Pid') == 0)
        # Callback must distinguish exited vs zombie and prove manager reaping.
        exited = self.process_exit_proof(ledger)
        if type(exited) is not bool: raise Blocked('Missing independent exit/reap proof')
        return dict(cgroup_empty=empty, docker_stopped=stopped, processes_exited=exited)

    def foreign_preservation(self, ledger, until):
        # Independent protected-process/container snapshots, supplied by manager.
        # An unknown result remains INCOMPLETE, never implicitly True.
        return self.foreign_proof(ledger, until)
