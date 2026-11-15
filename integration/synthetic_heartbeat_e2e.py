"""Stage 3 OFFLINE synthetic workflow. No configured endpoint or native adapter.

TCP endpoints are freshly bound private loopback peers owned by this module.
Children inherit only private AF_UNIX control sockets and a read-only pipe.
Popen creation handles authorize fixture teardown, not Linux cgroup authority.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import select
import shutil
import signal
import socket
import stat
import subprocess
import sys
import threading
import time
import uuid

from heartbeat_fault_proxy import (FaultProxy,Pending,ReadinessAuthority,ReadPipe,
    ReceiveCollector,RxOnlyIngress,one_way_pipe)
from heartbeat_test_evidence import (EventJournal,finalize_bundle,inspect_file,
    parse_json,verify_bundle)
from heartbeat_test_owner import (Blocked,CgroupIdentity,CleanupController,ContainerIdentity,
    Deadline,Ledger,ProcessIdentity,canonical,durable_new)
from heartbeat_witness import HeartbeatConsumer
from linux_gpu.heartbeat_test import IndependentWatchdog,RecoveryRegistration,StartupBarrier
from linux_gpu.verified_disarmed import load_codec
from synthetic_heartbeat_ipc import AuthenticationFailure,TestControl

def host_now():
    # CPython 3.9 on macOS may give monotonic_ns() a per-process epoch.
    # Use the explicit kernel clock, unchanged across our same-host children.
    if not hasattr(time,'CLOCK_MONOTONIC') or not hasattr(time,'clock_gettime_ns'):
        raise Blocked('Shared host CLOCK_MONOTONIC unavailable; no fallback')
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


ROOT=Path(__file__).resolve().parent
SCENARIOS=('normal','missing_readiness','unauthorized_trigger','mismatched_run_id',
    'control_interruption','unexpected_upstream_write','launcher_hang','launcher_sigkill',
    'watchdog_crash','ownership_mismatch','cleanup_deadline_exceeded','incomplete_evidence',
    'corrupted_evidence','partial_export','surviving_owned_descendant')
NATIVE=dict(manager_owned_recovery='NOT TESTED',cgroup_containment='NOT TESTED',
    pidfd_and_namespace_cleanup='NOT TESTED',native_endpoint_noninterference='NOT TESTED',
    native_network_enforcement='NOT TESTED',live_heartbeat_loss='NOT TESTED')


class PrivatePeer:
    """Factory reserves its own listener; accepts NO host/port/configuration input."""
    def __init__(self):
        self.listener=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.listener.bind(('127.0.0.1',0));self.listener.listen(1)
        self.identity=dict(address=self.listener.getsockname(),nonce=uuid.uuid4().hex)
    def connect_owned(self,deadline):
        # The retained listener, not any configured or default FCU address.
        client=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        bound=min(1,deadline.remaining(host_now()))
        if bound<=0:client.close();raise Blocked('Peer startup deadline exceeded')
        client.settimeout(bound);self.listener.settimeout(bound);client.connect(self.listener.getsockname())
        server,peer=self.listener.accept()
        if peer!=client.getsockname() or client.getpeername()!=self.listener.getsockname():
            server.close();client.close();raise Blocked('Unowned local TCP peer; fixture admission refused')
        for stream in (client,server):stream.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);stream.setblocking(False)
        return server,client
    def close(self):self.listener.close()


def source_state(deadline):
    def git(*args):
        return subprocess.check_output(['git',*args],cwd=ROOT.parent,
            timeout=max(.001,deadline.remaining(host_now())))
    paths=(ROOT/'SOURCE_FILES.txt').read_text().splitlines()
    manifest={p:hashlib.sha256((ROOT.parent/p).read_bytes()).hexdigest() for p in paths}
    return dict(head=git('rev-parse','HEAD').decode().strip(),branch=git('branch','--show-current').decode().strip(),
        status_porcelain=git('status','--porcelain=v1','--untracked-files=all').decode().splitlines(),
        unstaged_diff_sha256=hashlib.sha256(git('diff','--binary','--no-ext-diff')).hexdigest(),
        staged_diff_sha256=hashlib.sha256(git('diff','--cached','--binary','--no-ext-diff')).hexdigest(),
        selected_source_sha256=manifest)


def reap_owned(children, deadline_ns):
    """Dispose retained fixture handles as ONE bounded batch, not 2s per child.

    Late emergency disposal never extends the workflow D or repairs a failed
    cleanup receipt. Interrupts are deferred until every handle has been tried.
    No PID lookup, process/group discovery or group signaling is permitted.
    """
    interrupted=[False];main=threading.current_thread() is threading.main_thread()
    if main:
        previous=signal.signal(signal.SIGINT,lambda *_:interrupted.__setitem__(0,True))
    try:
        result=_reap_owned(children,deadline_ns)
        if interrupted[0]:result.update(interrupted=True,status='FAIL')
        return result
    finally:
        if main:signal.signal(signal.SIGINT,previous)


def _reap_owned(children,deadline_ns):
    started=host_now();late=started>=deadline_ns
    until=started+2_000_000_000
    if not late:until=min(until,deadline_ns)
    pending=list(children);errors=[];interrupted=False
    while pending:
        for child in list(pending):
            try:
                child.signal(signal.SIGKILL)
                if child.process.poll() is not None:pending.remove(child)
            except KeyboardInterrupt:interrupted=True
            except Exception as exc:
                error=child.role+': '+str(exc)
                if error not in errors:errors.append(error)
        if not pending or host_now()>=until:break
        try:time.sleep(min(.005,max(0,(until-host_now())/1e9)))
        except KeyboardInterrupt:interrupted=True
    for child in children:
        try:child.close()
        except KeyboardInterrupt:interrupted=True
        except Exception as exc:errors.append(child.role+' close: '+str(exc))
    return dict(status='FAIL' if pending or errors or interrupted or late else 'PASS',
        start_ns=started,end_ns=host_now(),disposal_deadline_ns=until,
        workflow_deadline_ns=deadline_ns,late_emergency_disposal=late,
        interrupted=interrupted,errors=errors,survivors=[c.role for c in pending])


class OwnedChild:
    def __init__(self,role,run_id,deadline,directory,data_fd=None):
        self.role=role;self.key=secrets.token_bytes(32);self.nonce=uuid.uuid4().hex
        parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_STREAM)
        args=[sys.executable,'-B',str(Path(__file__).resolve()),'_worker','--role',role,
              '--control-fd',str(child.fileno())]
        inherited=[child.fileno()]
        if data_fd is not None:args+=['--data-fd',str(data_fd)];inherited.append(data_fd)
        try:
            self.process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,pass_fds=tuple(inherited),start_new_session=True)
        except BaseException:
            parent.close();child.close();raise
        child.close();self.channel=TestControl(parent,self.key,run_id,'supervisor',role,deadline.end_ns)
        self.stderr_text=''
        boot=dict(synthetic_only=True,test_id=run_id,deadline_ns=deadline.end_ns,
                  key=self.key.hex(),nonce=self.nonce,directory=str(directory))
        bootstrap_start=host_now()
        try:
            self.process.stdin.write(canonical(boot));self.process.stdin.close()
            os.set_blocking(self.process.stdout.fileno(),False);raw=bytearray()
            while b'\n' not in raw and deadline.remaining(host_now())>0:
                ready,_,_=select.select([self.process.stdout],[],[],min(.05,deadline.remaining(host_now())))
                if ready:
                    data=os.read(self.process.stdout.fileno(),4096)
                    if not data:raise Blocked('Synthetic bootstrap exited')
                    raw.extend(data)
                    if len(raw)>4096:raise Blocked('Synthetic bootstrap frame bound')
            if b'\n' not in raw:raise Blocked('Synthetic bootstrap deadline exceeded')
            self.identity=parse_json(raw)
            if (self.identity.get('pid')!=self.process.pid or self.identity.get('nonce')!=self.nonce or
                    self.identity.get('ppid')!=os.getpid() or
                    not bootstrap_start<=self.identity['bootstrap_ns']<=host_now()):
                raise Blocked('Child bootstrap identity/clock mismatch')
        except BaseException:
            reap_owned([self],deadline.end_ns)
            raise
    def signal(self,sig):
        # Retain/reap the direct creation child; never kill a PID from a ledger.
        if self.process.poll() is None:self.process.send_signal(sig)
    def close(self):
        self.channel.close()
        # An inherited pipe held by another process must not block disposal.
        if not self.process.stderr.closed:
            os.set_blocking(self.process.stderr.fileno(),False)
            try:raw=os.read(self.process.stderr.fileno(),65536)
            except BlockingIOError:raw=b''
            self.stderr_text=raw.decode(errors='replace')
        for stream in (self.process.stdin,self.process.stdout,self.process.stderr):stream.close()


class SyntheticRecovery:
    """TEST ONLY; bound to one coordinator's live creation handles/fixture nonce.

    No inheritance from WitnessAuthority, no native adapter selector or endpoint
    support. Verification here attests ONLY synthetic Popen fixture ownership.
    """
    synthetic_only=True
    def __init__(self,owner):self.owner=owner;self.registration=None;self.requests=[]
    def verify_endpoint(self,*_):raise Blocked('Synthetic adapter refuses ALL production endpoint admission')
    def register(self,ledger):
        self.owner.check_fixture(ledger)
        self.registration=RecoveryRegistration(ledger.test_id,ledger.digest(),ledger.deadline.end_ns,
                                               'synthetic:'+secrets.token_hex(16))
        return self.registration
    def verify(self,registration,ledger):
        self.owner.check_fixture(ledger)
        return registration==self.registration and registration.deadline_ns==self.owner.deadline.end_ns
    def recover(self,registration,ledger,reason):
        if not self.verify(registration,ledger):raise Blocked('Synthetic registration mismatch')
        self.requests.append(reason);self.owner.recovery_reason=reason
        self.owner.event('synthetic_recovery_requested',reason=reason,native_recovery='NOT TESTED')
        return dict(status='INCOMPLETE',reason='synthetic supervisor recovery requested')


class FixtureCleanup:
    def __init__(self,owner):self.owner=owner;self.kill_failed=False
    def verify(self,ledger,until):self.owner.check_fixture(ledger);return True
    def signal_term(self,ledger,until):
        for child in self.owner.children.values():child.signal(signal.SIGTERM)
        if self.owner.scenario=='cleanup_deadline_exceeded':
            while host_now()<ledger.deadline.end_ns:
                time.sleep(min(.01,max(0,(ledger.deadline.end_ns-host_now())/1e9)))
        return True
    def wait_empty(self,until):
        if self.kill_failed:return False
        while host_now()<until:
            if all(c.process.poll() is not None for c in self.owner.children.values()):return True
            time.sleep(.005)
        return all(c.process.poll() is not None for c in self.owner.children.values())
    def kill(self,ledger,until):
        self.owner.check_fixture(ledger)
        for role,child in self.owner.children.items():
            if role=='worker' and self.owner.scenario=='surviving_owned_descendant':self.kill_failed=True;continue
            child.signal(signal.SIGKILL)
        if self.kill_failed:raise Blocked('Injected owned-worker KILL failure')
        return True
    def stop_container(self,ledger,until):return True  # NO Docker exists; modeled test-only obligation.
    def terminal(self,ledger,until):
        self.owner.alive_at_terminal=[r for r,c in self.owner.children.items() if c.process.poll() is None]
        return dict(cgroup_empty=not self.owner.alive_at_terminal,docker_stopped=True,
                    processes_exited=not self.owner.alive_at_terminal)
    def foreign_preservation(self,ledger,until):return self.owner.foreign.process.poll() is None


class Coordinator:
    def __init__(self,directory,scenario,budget_seconds):
        if scenario not in SCENARIOS:raise ValueError('Unknown synthetic scenario')
        if not math.isfinite(budget_seconds) or not 12<=budget_seconds<=60:
            raise ValueError('Explicit synthetic fixture budget 12..60 seconds required')
        self.started=host_now();self.test_id=str(uuid.uuid4());self.root=Path(directory)
        self.deadline=Deadline(self.started,self.started+int(budget_seconds*1e9),500_000_000,
            500_000_000,1_000_000_000,2_000_000_000,'synthetic-boot:'+self.test_id,'synthetic-host-monotonic')
        self.root.mkdir(mode=0o700,parents=False,exist_ok=False)
        self.scenario=scenario;self.children={};self.foreign=None;self.ledger=None
        self.recovery_reason=None;self.failure=None;self.triggered=False;self.control_status={}
        self.hang_entered=False;self.alive_at_terminal=[];self.peer=None;self.downstream=None
        self.trace=EventJournal(self.root/'timeline.jsonl',self.test_id,clock=host_now)
        self.auth_log=EventJournal(self.root/'control_ipc.jsonl',self.test_id,clock=host_now)
        self.auth_log.emit('reserved',mechanism='per-run HMAC-SHA256 over private AF_UNIX socketpair')
        self.watchdog_log=EventJournal(self.root/'watchdog.jsonl',self.test_id,clock=host_now)
        self.timeline=[];self.logs=[];self.resources=[];self.witness=None;self.proxy=None
        self.manager=SyntheticRecovery(self)
    def event(self,kind,**data):
        self.trace.emit(kind,**data);self.timeline.append(dict(host_monotonic_ns=host_now(),kind=kind,**data))
    def check_fixture(self,ledger):
        if (ledger is not self.ledger or ledger.test_id!=self.test_id or
                ledger.container.image_id!='synthetic:NO_DOCKER' or
                ledger.cgroup.path!='/synthetic-only/'+self.test_id or
                ledger.deadline!=self.deadline):raise Blocked('Not this synthetic fixture ledger')
        if self.scenario=='ownership_mismatch':raise Blocked('Injected ownership/start-identity mismatch')
        for row,child in zip(ledger.processes,self.children.values()):
            if row.host_pid!=child.process.pid or child.identity['nonce']!=self.creation_nonces[child.role]:
                raise Blocked('Creation handle/nonce mismatch; no numeric signal fallback')
    def setup(self):
        state=source_state(self.deadline)
        durable_new(self.root/'source.json',dict(test_id=self.test_id,**state))
        self.source=state;self.event('reserved',deadline_ns=self.deadline.end_ns)
        self.peer=PrivatePeer();self.downstream=PrivatePeer()
        self.control_reader,self.control_writer=one_way_pipe();self.resources+=[self.control_reader,self.control_writer]
        for role in ('launcher','worker','watchdog','control'):
            self.children[role]=OwnedChild(role,self.test_id,self.deadline,self.root,
                self.control_reader.fd if role=='control' else None)
        self.foreign=OwnedChild('foreign',self.test_id,self.deadline,self.root)
        self.creation_nonces={role:c.nonce for role,c in self.children.items()}
        rows=[]
        for role,child in self.children.items():
            info=child.identity
            rows.append(ProcessIdentity(child.process.pid,child.process.pid,info['ppid'],info['pgid'],info['sid'],
                info['bootstrap_ns'],self.deadline.boot_id,{k:'synthetic-not-linux:'+self.test_id for k in
                ('pid','mnt','net','time','user')},'/synthetic-only/'+self.test_id,'synthetic-child'))
        self.ledger=Ledger(self.test_id,ContainerIdentity(hashlib.sha256(self.test_id.encode()).hexdigest(),
            'synthetic-only-'+self.test_id,'synthetic-created','synthetic:NO_DOCKER'),
            CgroupIdentity('/synthetic-only/'+self.test_id,0,1),tuple(rows),self.deadline)
        self.ledger.publish(self.root)
        durable_new(self.root/'fixture.json',dict(test_id=self.test_id,scope='synthetic-only',
            source_peer=self.peer.identity,witness_peer=self.downstream.identity,
            process_authority='retained unreaped Popen creation handles; NOT Linux pidfd/cgroup',
            processes={r:dict(c.identity) for r,c in self.children.items()},foreign=dict(self.foreign.identity),
            linux_native=NATIVE))
        barrier=StartupBarrier(self.ledger,self.manager,clock=host_now)
        self.registration=barrier.arm();barrier.release()
        self.watchdog_log.emit('synthetic_manager_armed',deadline_ns=self.deadline.end_ns)
        self.event('synthetic_barrier_released',native_recovery='NOT TESTED')
        for child in [*self.children.values(),self.foreign]:
            child.channel.queue('ACTIVATE',dict(ledger=self.ledger.record(),registration=asdict(self.registration),
                                               scenario=self.scenario));child.channel.flush()
        self.source_socket,ingress_socket=self.peer.connect_owned(self.deadline);self.resources+=[self.source_socket]
        proxy_socket,witness_socket=self.downstream.connect_owned(self.deadline)
        self.resources.extend((proxy_socket,witness_socket))
        proxy_reader,proxy_writer=one_way_pipe();self.resources.extend((proxy_reader,proxy_writer))
        self.proxy_log=EventJournal(self.root/'proxy.jsonl',self.test_id,clock=host_now);self.logs.append(self.proxy_log)
        self.collector_log=EventJournal(self.root/'collector.jsonl',self.test_id,clock=host_now);self.logs.append(self.collector_log)
        self.collector_log.emit('started',scope='synthetic-owned peer only')
        self.rx=RxOnlyIngress(ingress_socket,self.collector_log.emit);self.resources.append(self.rx)
        self.collector=ReceiveCollector(self.rx,self.control_writer,proxy_writer,65536,self.collector_log.emit)
        self.capability=object();issuer=ReadinessAuthority()
        self.proxy=FaultProxy(self.test_id,proxy_reader,proxy_socket,65536,self.capability,issuer,self.proxy_log.emit,clock=host_now)
        self.witness=HeartbeatConsumer(witness_socket,self.root,self.test_id,1,1,'witness',clock=host_now,readiness_issuer=issuer)
        wire=load_codec();encoder=wire.MAVLink(None,srcSystem=1,srcComponent=1)
        self.packet=wire.MAVLink_heartbeat_message(2,3,0,0,3,3).pack(encoder)
        self.source_pending=Pending(65536);self.next_packet=0;self.notified=False;self.forced_post_failure=False
    def rpc(self):
        for role in ('launcher','watchdog','control'):
            child=self.children[role]
            if getattr(child,'peer_eof',False):continue
            try:messages=child.channel.receive()
            except (EOFError,OSError):
                child.peer_eof=True
                if role=='watchdog':
                    self.failure=self.failure or 'WATCHDOG_CRASH'
                    self.manager.recover(self.registration,self.ledger,'watchdog control exited')
                elif role=='control':self.failure=self.failure or 'CONTROL_PROCESS_EXIT'
                continue
            except AuthenticationFailure as exc:
                self.auth_log.emit('rejected',peer=role,reason=str(exc));self.failure='CONTROL_AUTH_REJECTED';return
            for row in messages:
                command=row['command'];payload=row['payload']
                self.auth_log.emit('accepted',peer=role,command=command,command_sequence=row['sequence'])
                if role=='control' and command=='STATUS':
                    self.control_status=payload
                    if payload.get('failure_code'):self.failure='CONTROL_HEARTBEAT_INTERRUPTED'
                elif role=='launcher' and command=='SILENCE':
                    try:self.proxy.trigger(self.test_id,self.capability,self.witness.proof)
                    except Exception as exc:self.failure='READINESS_OR_TRIGGER_FAILED';self.event('trigger_rejected',reason=str(exc));return
                    self.triggered=True;self.event('authorized_silence')
                    child.channel.queue('ACK')
                elif role=='launcher' and command=='HANG_STARTED':self.hang_entered=True
                elif role=='watchdog' and command in ('VERIFY','HEALTH','RECOVER'):
                    if command=='VERIFY':answer=dict(verified=self.manager.verify(self.registration,self.ledger))
                    elif command=='HEALTH':
                        state='HUNG' if self.hang_entered else ('EXITED' if self.children['launcher'].process.poll() is not None else 'HEALTHY')
                        answer=dict(health=state)
                    else:
                        self.manager.recover(self.registration,self.ledger,payload['reason'])
                        answer=dict(status='INCOMPLETE',scope='synthetic recovery requested')
                        self.event('watchdog_recovery_requested',reason=payload['reason'])
                    child.channel.queue('REPLY',dict(request=command,**answer))
                else:self.failure='UNEXPECTED_CONTROL_COMMAND'
            try:child.channel.flush()
            except OSError:
                child.peer_eof=True
                if role=='watchdog':
                    self.failure=self.failure or 'WATCHDOG_CRASH'
                    self.manager.recover(self.registration,self.ledger,'watchdog reply disconnected')
    def source_tick(self,force=False):
        now=host_now()
        if self.scenario!='missing_readiness' and (force or now>=self.next_packet):
            self.source_pending.append(self.packet);self.next_packet=now+500_000_000
        self.source_pending.flush(self.source_socket.send)
        try:received=self.source_socket.recv(65536)
        except BlockingIOError:received=None
        if received:self.failure='UPSTREAM_WIRE_WRITE'
    def observe(self):
        cleanup_boundary=self.deadline.end_ns-sum((self.deadline.term_ns,self.deadline.kill_ns,
                                                 self.deadline.verify_ns,self.deadline.export_ns))
        while host_now()<cleanup_boundary:
            self.source_tick();self.collector.step();self.proxy.step();self.witness.step();self.rpc()
            if self.children['watchdog'].process.poll() is not None:
                self.failure='WATCHDOG_CRASH';self.manager.recover(self.registration,self.ledger,'watchdog exited')
            if self.failure or self.recovery_reason:break
            ready=self.witness.ready and self.control_status.get('ready')
            if not self.notified and (ready or self.scenario=='missing_readiness'):
                self.notified=True;self.event('readiness_decision',witness_ready=self.witness.ready,
                                             control_ready=bool(self.control_status.get('ready')))
                if self.scenario in ('launcher_sigkill','watchdog_crash'):
                    role='launcher' if self.scenario=='launcher_sigkill' else 'watchdog'
                    self.children[role].signal(signal.SIGKILL);self.event('injected_sigkill',role=role)
                elif self.scenario=='unexpected_upstream_write':
                    try:self.rx.send(b'NEVER_SENT')
                    except RuntimeError:self.failure='UPSTREAM_WRITE_ATTEMPT'
                else:self.children['launcher'].channel.queue('TRIGGER_NOW');self.children['launcher'].channel.flush()
            if self.triggered and self.scenario=='control_interruption':
                class Paused:
                    def send(self,data):raise BlockingIOError()
                self.collector.writers['control']=Paused()
            if self.witness.failure_code:
                if self.witness.failure_code!='HEARTBEAT_STALE' or not self.triggered:
                    self.failure=self.witness.failure_code;break
                # Control must receive a real synthetic packet AFTER witness FAIL.
                if not self.forced_post_failure:self.source_tick(force=True);self.forced_post_failure=True
                if (self.control_status.get('last_ns') or 0)>=self.witness.failure_ns:
                    self.event('expected_heartbeat_loss',code=self.witness.failure_code);return
            time.sleep(.01)
        if not self.failure:self.failure='WORKFLOW_DEADLINE_OR_RECOVERY' if self.recovery_reason else 'OBSERVATION_DEADLINE'
    def cleanup(self):
        if self.witness and not self.witness.failure_code:self.witness.fail(self.failure or 'ABORTED','Synthetic workflow aborted')
        if self.proxy:self.proxy.close()
        if self.witness:
            try:self.witness.finish()
            except Exception as exc:self.event('witness_export_incomplete',reason=str(exc))
        for log in self.logs:log.close()
        if self.proxy:durable_new(self.root/'proxy.json',self.proxy.summary(self.rx,self.collector))
        self.watchdog_log.emit('synthetic_cleanup_requested',reason=self.failure or 'expected heartbeat loss')
        result=CleanupController(self.ledger,FixtureCleanup(self),clock=host_now).run(self.root)
        if (self.root/'cleanup.receipt.json').exists():
            self.watchdog_log.emit('synthetic_cleanup_finished',status=result['status'],
                cleanup_receipt_sha256=hashlib.sha256((self.root/'cleanup.receipt.json').read_bytes()).hexdigest())
        self.watchdog_log.close()
        durable_new(self.root/'admission.json',dict(test_id=self.test_id,evidence_scope='synthetic',
            ledger_sha256=self.ledger.digest(),endpoint_noninterference_verified=True,
            manager_recovery_verified=True,network_isolation_verified=True,forbidden_actions=[],
            scope='synthetic fixture ownership only; NO Linux recovery or firewall proof',linux_native=NATIVE))
        return result
    def teardown(self):
        # Independent creator authority for test fixture disposal, including a
        # deliberate primary cleanup failure. Never changes the failed receipt.
        children=[*self.children.values(),*([self.foreign] if self.foreign else [])]
        self.teardown_result=reap_owned(children,self.deadline.end_ns)
        try:
            # Observation may have been interrupted before primary cleanup.
            # Finalize only genuine failure evidence; never create a PASS marker.
            try:
                if self.witness and not self.witness.closed:
                    self.witness.fail('WORKFLOW_ABORTED','Interrupted before primary cleanup completed')
                    self.witness.finish()
                if self.proxy:self.proxy.close()
            except KeyboardInterrupt:
                self.teardown_result.update(interrupted=True,status='FAIL')
            except Exception as exc:
                self.teardown_result['errors'].append('resource finalization: '+str(exc))
                self.teardown_result['status']='FAIL'
            finally:
                if self.witness:self.witness.journal.close()
            durable_new(self.root/'process_exit.json',dict(test_id=self.test_id,
                scope='actual local Popen fixture exits; not native containment',
                teardown=self.teardown_result,
                children={r:dict(pid=c.process.pid,exit_code=c.process.returncode,stderr=c.stderr_text)
                          for r,c in self.children.items()},
                foreign_fixture_exit_code=self.foreign.process.returncode if self.foreign else None))
        finally:
            for resource in self.resources:
                try:resource.close()
                except OSError:pass
            for peer in (self.peer,self.downstream):
                if peer:peer.close()
        if self.teardown_result['interrupted']:raise KeyboardInterrupt()


def run_synthetic(directory,scenario='normal',budget_seconds=20):
    """No endpoint argument, native adapter, environment selection or fallback."""
    owner=Coordinator(directory,scenario,budget_seconds);cleanup={'status':'INCOMPLETE'}
    evidence={'status':'INCOMPLETE'};foreign=False;failure=None
    try:
        try:
            try:
                owner.setup();owner.observe()
            except Exception as exc:
                owner.failure='OWNERSHIP_MISMATCH' if scenario=='ownership_mismatch' else 'WORKFLOW_EXCEPTION'
                owner.event('exception',reason=str(exc))
            if owner.ledger:cleanup=owner.cleanup()
            foreign=owner.foreign is not None and owner.foreign.process.poll() is None
            owner.event('cleanup_result',status=cleanup['status'],foreign_preserved=foreign,
                        alive_at_terminal=owner.alive_at_terminal)
        finally:
            owner.teardown()
    except BaseException:
        # Includes Ctrl+C during setup, observation, cleanup AND reaping. Never
        # export a positive result for an interrupted or exceptional workflow.
        for log in [*owner.logs,owner.watchdog_log,owner.auth_log,owner.trace]:log.close()
        raise
    try:
        publisher=None
        if scenario=='partial_export':
            def publisher(path,value):
                if path.name=='bundle.complete.json':raise OSError('Injected partial export')
                durable_new(path,value)
        evidence=finalize_bundle(owner.root,owner.ledger,clock=host_now,publisher=publisher) if owner.ledger else evidence
        if scenario in ('incomplete_evidence','corrupted_evidence') and evidence['status']=='PASS':
            copied=owner.root/'verification-copy';shutil.copytree(owner.root,copied,ignore=shutil.ignore_patterns('verification-copy'))
            target=copied/('control.json' if scenario=='incomplete_evidence' else 'native.jsonl')
            if scenario=='incomplete_evidence':target.unlink()  # owned fault-copy only
            else:
                raw=target.read_bytes();target.write_bytes(raw.replace(b'idle_open',b'idle_opem',1))
            evidence=verify_bundle(copied,owner.ledger)
    except Exception as exc:evidence=dict(status='INCOMPLETE',reason=str(exc))
    all_exited=all(c.process.poll() is not None for c in owner.children.values())
    heartbeat='PASS' if owner.witness and owner.witness.failure_code=='HEARTBEAT_STALE' and owner.triggered else 'FAIL'
    failures=[owner.failure] if owner.failure else []
    if owner.teardown_result['status']!='PASS':failures.append('Fixture teardown failed')
    if cleanup['status']=='FAIL' or not foreign or not all_exited:failures.append('Cleanup/foreign preservation violation')
    if host_now()>=owner.deadline.end_ns:failures.append('Immutable deadline exceeded')
    status='FAIL' if failures else ('PASS' if heartbeat=='PASS' and cleanup['status']=='PASS' and evidence['status']=='PASS' else 'INCOMPLETE')
    owner.event('final_verdict',status=status)
    owner.auth_log.close();owner.trace.close()
    report=dict(schema=3,test_id=owner.test_id,scope='synthetic_offline_expected_failure_only',scenario=scenario,
        source=owner.source if hasattr(owner,'source') else {},fixture=dict(kind='owned loopback + private Unix IPC + direct Popen children',
            upstream=owner.peer.identity if owner.peer else None,witness=owner.downstream.identity if owner.downstream else None),
        deadline=asdict(owner.deadline),end_ns=host_now(),timeline=owner.timeline,
        heartbeat_loss=dict(status=heartbeat,observed_code=owner.witness.failure_code if owner.witness else None,
            witness_component_exit_code=1 if owner.witness and owner.witness.failure_code else None,
            source_continuity='NOT TESTED'),cleanup=dict(status=cleanup['status'],alive_at_terminal=owner.alive_at_terminal,
            foreign_preserved_through_cleanup=foreign,all_fixture_children_reaped=all_exited),
        teardown=owner.teardown_result,
        evidence=dict(status=evidence['status'],evaluated_copy=scenario in ('incomplete_evidence','corrupted_evidence')),
        supervision=dict(recovery_requests=owner.manager.requests,authority='synthetic Popen fixture only',
                         native_manager_owned_recovery='NOT TESTED'),
        status=status,failures=failures,linux_native=NATIVE,
        decision='HOLD',execution_allowed=False,forwarding_enabled=False)
    report['clock_domain']='shared host POSIX CLOCK_MONOTONIC nanoseconds; no per-process monotonic epoch'
    report['test_control_authentication']=dict(method='per-run HMAC-SHA256/private AF_UNIX',secrets_persisted=False)
    report['message_counts']=dict(witness=owner.witness.guard.count if owner.witness else 0,
        control=owner.control_status.get('heartbeats',0),
        upstream_application_writes=owner.rx.application_writes if hasattr(owner,'rx') else 0,
        rejected_upstream_write_attempts=owner.rx.write_attempts if hasattr(owner,'rx') else 0)
    report['evidence'].update(errors=evidence.get('incomplete',[]),violations=evidence.get('failures',[]))
    report['cleanup']['errors']=cleanup.get('errors',[])
    durable_new(owner.root/'SYNTHETIC_E2E_REPORT.json',report)
    try:export_report(owner.root,report)
    except Exception as exc:
        status='FAIL' if host_now()>=owner.deadline.end_ns or report['status']=='FAIL' else 'INCOMPLETE'
        failed=dict(test_id=owner.test_id,status=status,reason=str(exc),host_monotonic_ns=host_now(),
                    scope='synthetic-only finalization failure',execution_allowed=False,forwarding_enabled=False)
        # Retain provisional report/manifest without overwriting. Native external
        # durability acknowledgement remains NOT TESTED; this is failure evidence.
        durable_new(owner.root/'SYNTHETIC_FINALIZATION_FAILURE.json',failed)
        report=dict(report,status=status,finalization_failure=str(exc))
    return report


def export_report(root,report):
    def budget():
        if report['status']=='PASS' and host_now()>=report['deadline']['end_ns']:
            raise TimeoutError('Immutable deadline exceeded during final export')
    budget()
    files={}
    for path in sorted(Path(root).rglob('*')):
        if path.is_file() and path.name not in ('synthetic.manifest.json','synthetic.complete.json'):
            name=path.relative_to(root).as_posix()
            try:files[name]=inspect_file(root,name,report['test_id'])[0]
            except Exception as exc:
                if report['status']=='PASS':raise
                raw=path.read_bytes()
                files[name]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),
                    invalid_evidence=True,parse_error=str(exc))
    manifest=dict(test_id=report['test_id'],scope='synthetic-only',files=files)
    budget()
    durable_new(Path(root)/'synthetic.manifest.json',manifest)
    if report['status']=='PASS':
        budget()
        durable_new(Path(root)/'synthetic.complete.json',dict(test_id=report['test_id'],status='PASS',
            manifest_sha256=hashlib.sha256(canonical(manifest)).hexdigest(),scope='synthetic-only',
            decision='HOLD',execution_allowed=False,forwarding_enabled=False))
        budget()


def verify_synthetic(root):
    try:
        root=Path(root);report=parse_json((root/'SYNTHETIC_E2E_REPORT.json').read_bytes())
        if (root/'SYNTHETIC_FINALIZATION_FAILURE.json').exists():
            failed=parse_json((root/'SYNTHETIC_FINALIZATION_FAILURE.json').read_bytes())
            if failed.get('test_id')!=report['test_id'] or failed.get('status') not in ('FAIL','INCOMPLETE'):
                raise ValueError('Invalid finalization failure receipt')
            return dict(status=failed['status'],test_id=report['test_id'],reason=failed['reason'],
                scope=report['scope'],decision='HOLD',execution_allowed=False,forwarding_enabled=False)
        manifest=parse_json((root/'synthetic.manifest.json').read_bytes());test_id=report['test_id']
        required={'SYNTHETIC_E2E_REPORT.json','source.json','fixture.json','timeline.jsonl',
                  'control_ipc.jsonl','watchdog.jsonl','ownership.json','cleanup.receipt.json','process_exit.json'}
        if (manifest.get('test_id')!=test_id or manifest.get('scope')!='synthetic-only' or
                not required<=set(manifest['files']) or report['status'] not in ('PASS','FAIL','INCOMPLETE')):
            raise ValueError('Incomplete synthetic export manifest')
        for name,description in manifest['files'].items():
            if description.get('invalid_evidence'):
                from heartbeat_test_evidence import local_file
                raw=local_file(root,name).read_bytes()
                if report['status']=='PASS' or len(raw)!=description['bytes'] or hashlib.sha256(raw).hexdigest()!=description['sha256']:
                    raise ValueError('Invalid negative-evidence inventory: '+name)
            elif inspect_file(root,name,test_id)[0]!=description:raise ValueError('Outer hash/count mismatch: '+name)
        source=parse_json((root/'source.json').read_bytes());source.pop('test_id')
        if source!=report['source']:raise ValueError('Source revision/working-tree state mismatch')
        if report['status']=='PASS':
            ledger=Ledger.restore(parse_json((root/'ownership.json').read_bytes()))
            complete=parse_json((root/'synthetic.complete.json').read_bytes())
            exits=parse_json((root/'process_exit.json').read_bytes())
            if (verify_bundle(root,ledger)['status']!='PASS' or complete.get('test_id')!=test_id or
                    complete.get('status')!='PASS' or complete.get('scope')!='synthetic-only' or
                    complete.get('execution_allowed') is not False or complete.get('forwarding_enabled') is not False or
                    any(row['exit_code'] is None for row in exits['children'].values()) or
                    complete['manifest_sha256']!=hashlib.sha256(canonical(manifest)).hexdigest() or
                    report['heartbeat_loss']['status']!='PASS' or report['cleanup']['status']!='PASS' or
                    report['evidence']['status']!='PASS' or not report['cleanup']['all_fixture_children_reaped'] or
                    not report['cleanup']['foreign_preserved_through_cleanup'] or report['end_ns']>=ledger.deadline.end_ns):
                raise ValueError('Missing integrated positive proof')
        if (report['scope']!='synthetic_offline_expected_failure_only' or report['linux_native']!=NATIVE or
                report['execution_allowed'] is not False or report['forwarding_enabled'] is not False):
            raise ValueError('Not synthetic disabled-safety report')
        return dict(status=report['status'],scope=report['scope'],test_id=test_id,decision='HOLD',
                    execution_allowed=False,forwarding_enabled=False)
    except Exception as exc:return dict(status='INCOMPLETE',reason=str(exc),decision='HOLD',execution_allowed=False,forwarding_enabled=False)


def worker(role,control_fd,data_fd):
    boot=parse_json(sys.stdin.buffer.readline())
    if boot.get('synthetic_only') is not True:raise Blocked('Synthetic bootstrap required')
    channel=TestControl(socket.socket(fileno=control_fd),bytes.fromhex(boot['key']),boot['test_id'],role,'supervisor',boot['deadline_ns'])
    print(json.dumps(dict(pid=os.getpid(),ppid=os.getppid(),pgid=os.getpgrp(),sid=os.getsid(0),
        bootstrap_ns=host_now(),nonce=boot['nonce'])),flush=True)
    stop=[False]
    signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True))
    activation=None
    while not stop[0] and host_now()<boot['deadline_ns']:
        for message in channel.receive():
            if message['command']!='ACTIVATE':raise AuthenticationFailure('Startup barrier not released')
            activation=message['payload']
        if activation:break
        time.sleep(.005)
    if not activation:return 2
    scenario=activation['scenario'];consumer=None
    if role=='worker':signal.signal(signal.SIGTERM,signal.SIG_IGN)
    if role=='control':
        if data_fd is None or not stat.S_ISFIFO(os.fstat(data_fd).st_mode):raise Blocked('Read-only fixture pipe required')
        consumer=HeartbeatConsumer(ReadPipe(data_fd),boot['directory'],boot['test_id'],1,1,'control',clock=host_now)
    if role=='watchdog':
        ledger=Ledger.restore(activation['ledger']);registration=RecoveryRegistration(**activation['registration'])
        def request(command,payload=None):
            channel.queue(command,payload)
            while not stop[0] and host_now()<boot['deadline_ns']:
                for message in channel.receive():
                    if message['command']=='REPLY' and message['payload'].get('request')==command:return message['payload']
                time.sleep(.005)
            raise InterruptedError('Synthetic watchdog deadline/shutdown')
        class RPC:
            def verify(self,r,l):return request('VERIFY',dict(ledger_sha256=l.digest(),deadline_ns=l.deadline.end_ns))['verified']
            def recover(self,r,l,reason):return request('RECOVER',dict(reason=reason))
        policy=IndependentWatchdog(ledger,registration,RPC(),lambda:request('HEALTH')['health'],clock=host_now)
    last_count=-1
    try:
        while not stop[0] and host_now()<boot['deadline_ns']:
            if role=='watchdog':
                if policy.tick() is not None:break
            else:
                for message in channel.receive():
                    if role=='launcher' and message['command']=='TRIGGER_NOW':
                        if scenario=='launcher_hang':channel.queue('HANG_STARTED')
                        else:channel.queue('SILENCE',corrupt_mac=scenario=='unauthorized_trigger',wrong_run=scenario=='mismatched_run_id')
                if consumer:
                    consumer.step()
                    if consumer.guard.count!=last_count or consumer.failure_code:
                        channel.queue('STATUS',dict(ready=consumer.ready,last_ns=consumer.hb_times[-1] if consumer.hb_times else None,
                            heartbeats=consumer.guard.count,failure_code=consumer.failure_code))
                        last_count=consumer.guard.count
                        if consumer.failure_code:channel.flush();break
                channel.flush()
            time.sleep(.02 if role=='watchdog' else .005)
    except (InterruptedError,EOFError):pass
    finally:
        if consumer:consumer.finish()
        channel.close()
    return 0


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='action',required=True)
    run=sub.add_parser('run');run.add_argument('--synthetic-offline',action='store_true',required=True)
    run.add_argument('--directory',type=Path,required=True);run.add_argument('--budget-seconds',type=float,required=True)
    run.add_argument('--scenario',choices=SCENARIOS,default='normal')
    verify=sub.add_parser('verify');verify.add_argument('--directory',type=Path,required=True)
    internal=sub.add_parser('_worker');internal.add_argument('--role',choices=['launcher','worker','watchdog','control','foreign'],required=True)
    internal.add_argument('--control-fd',type=int,required=True);internal.add_argument('--data-fd',type=int)
    args=parser.parse_args()
    if args.action=='_worker':return worker(args.role,args.control_fd,args.data_fd)
    report=run_synthetic(args.directory,args.scenario,args.budget_seconds) if args.action=='run' else verify_synthetic(args.directory)
    print(json.dumps({k:report[k] for k in ('test_id','status','decision','execution_allowed','forwarding_enabled') if k in report},allow_nan=False))
    return {'PASS':0,'FAIL':1,'INCOMPLETE':2}[report['status']]


if __name__=='__main__':raise SystemExit(main())
