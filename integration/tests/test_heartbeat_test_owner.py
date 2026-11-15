"""Synthetic policy/adapter tests. No Docker, cgroup mutation or FCU connection."""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import uuid

from heartbeat_test_owner import (Blocked, CgroupIdentity, CleanupController,
    ContainerIdentity, Deadline, DockerByIdentity, Ledger, LinuxPidfdCgroup,
    ProcessIdentity, durable_new, same_process, verify_receipt)


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now


def fixture():
    clock = Clock()
    deadline = Deadline(clock.now, clock.now+20_000_000_000, 2_000_000_000,
                        2_000_000_000, 2_000_000_000, 2_000_000_000, 'boot-fixture', 'time:[1]')
    process = ProcessIdentity(101, 2, 100, 101, 101, 123, 'boot-fixture',
        dict(pid='pid:[2]', net='net:[3]', mnt='mnt:[4]', time='time:[1]', user='user:[5]'),
        '/test-only-fixture', 'coordinator')
    ledger = Ledger(str(uuid.uuid4()), ContainerIdentity('a'*64, 'test-only', 'created-fixture', 'sha256:fixture'),
                    CgroupIdentity('/sys/fs/cgroup/test-only-fixture', 1, 2), (process,), deadline)
    return clock, ledger


class FakeBackend:
    """Models native authority; never asserts real Linux containment."""
    def __init__(self, clock):
        self.clock = clock; self.calls = []; self.refuse = None; self.fail = None
        self.terminal_value = dict(cgroup_empty=True, docker_stopped=True, processes_exited=True)

    def step(self, name):
        self.calls.append(name)
        if self.fail == name: raise Blocked('injected '+name)

    def verify(self, ledger, until):
        self.step('verify')
        if self.refuse: raise Blocked(self.refuse)
        return True

    def signal_term(self, ledger, until): self.step('term'); return True
    def wait_empty(self, until): self.step('wait'); self.clock.now = min(until, self.clock.now+1_000_000); return True
    def kill(self, ledger, until): self.step('kill'); return True
    def stop_container(self, ledger, until): self.step('docker'); return True
    def terminal(self, ledger, until): self.step('terminal'); return self.terminal_value
    def foreign_preservation(self, ledger, until): self.step('foreign'); return True


class OwnershipTests(unittest.TestCase):
    def test_ledger_durable_roundtrip_and_no_overwrite(self):
        clock, ledger = fixture()
        with tempfile.TemporaryDirectory() as temp:
            ledger.publish(temp); path=Path(temp)/'ownership.json'
            restored=Ledger.restore(json.loads(path.read_text()))
            self.assertEqual(restored,ledger); self.assertEqual(restored.deadline.end_ns,ledger.deadline.end_ns)
            with self.assertRaises(FileExistsError): ledger.publish(temp)
            self.assertEqual(ledger.digest(), restored.digest())

    def test_pid_starttime_namespace_session_cgroup_reuse_mismatch(self):
        _, ledger=fixture(); p=ledger.processes[0]
        for field,value in [('host_pid',202),('starttime',999),('namespaces',dict(p.namespaces,pid='reused')),
                            ('boot_id','newboot'),('sid',222),('pgid',222),('cgroup','/foreign')]:
            with self.subTest(field=field): self.assertFalse(same_process(p,replace(p,**{field:value})))
        self.assertTrue(same_process(p,replace(p,ppid=1)))  # real orphan reparenting

    def test_full_container_id_required(self):
        for invalid in ('short-id', 'a'*63, 'z'*64):
            with self.assertRaises(Blocked): ContainerIdentity(invalid,'fixture','created','image')

    def test_deadline_no_rebase_and_explicit_host_domain(self):
        clock, ledger=fixture(); d=Deadline(**asdict(ledger.deadline))
        clock.now+=4_000_000_000
        self.assertEqual(d.remaining(clock()),16)
        with self.assertRaises(Blocked): d.check_domain('other-boot',d.time_namespace)
        with self.assertRaises(Blocked): d.check_domain(d.boot_id,'other-time-ns')
        with self.assertRaises(Blocked): d.remaining(d.start_ns-1)
        for field in ('end_ns','term_ns'):
            with self.assertRaises(Blocked): replace(d,**{field:float('inf')})

    def test_partial_ledger_or_enabled_safety_denied(self):
        _,ledger=fixture();row=ledger.record();row['execution_allowed']=True
        with self.assertRaises(Blocked): Ledger.restore(row)
        with self.assertRaises(Blocked): replace(ledger,processes=())


class CleanupTests(unittest.TestCase):
    def run_cleanup(self, backend_change=None):
        clock, ledger=fixture(); backend=FakeBackend(clock)
        if backend_change: backend_change(backend,clock,ledger)
        with tempfile.TemporaryDirectory() as temp:
            result=CleanupController(ledger,backend,clock).run(temp)
            self.assertEqual(result['status'],verify_receipt(Path(temp)/'cleanup.receipt.json',ledger)['status'])
            return result,backend

    def test_normal_cleanup_and_durable_receipt(self):
        result,backend=self.run_cleanup()
        self.assertEqual(result['status'],'PASS')  # cleanup policy only
        self.assertEqual(backend.calls,['verify','term','wait','kill','wait','docker','terminal','foreign'])
        self.assertFalse(result['execution_allowed']);self.assertFalse(result['forwarding_enabled'])

    def test_unknown_authority_never_signals(self):
        for mismatch in ('PID reuse','namespace mismatch','cgroup replaced','permission denied'):
            with self.subTest(mismatch=mismatch):
                result,backend=self.run_cleanup(lambda b,c,l:setattr(b,'refuse',mismatch))
                self.assertEqual(result['status'],'INCOMPLETE');self.assertEqual(backend.calls,['verify'])

    def test_term_resistant_and_orphan_setsid_descendants_use_containment(self):
        def change(backend,clock,ledger):
            backend.children={'term-resistant':True,'orphan-setsid':True,'foreign-control':True}
            def term(*_): backend.step('term'); return True  # resistant/orphan stay alive
            def kill(*_):
                backend.step('kill');backend.children['term-resistant']=False
                backend.children['orphan-setsid']=False;return True
            backend.signal_term=term;backend.kill=kill
        result,backend=self.run_cleanup(change)
        self.assertEqual(result['status'],'PASS');self.assertTrue(backend.children['foreign-control'])
        self.assertFalse(backend.children['orphan-setsid']);self.assertFalse(backend.children['term-resistant'])

    def test_docker_error_does_not_prevent_kernel_cleanup_or_become_pass(self):
        result,backend=self.run_cleanup(lambda b,c,l:setattr(b,'fail','docker'))
        self.assertIn('kill',backend.calls);self.assertIn('terminal',backend.calls)
        self.assertEqual(result['status'],'INCOMPLETE')

    def test_unknown_terminal_is_incomplete_but_proven_survivor_is_fail(self):
        for terminal,status in [({},'INCOMPLETE'),(dict(cgroup_empty=False),'FAIL')]:
            result,_=self.run_cleanup(lambda b,c,l:setattr(b,'terminal_value',terminal))
            self.assertEqual(result['status'],status)

    def test_foreign_state_unknown_or_affected_never_passes(self):
        for value,status in ((None,'INCOMPLETE'),(False,'FAIL')):
            def change(b,c,l):b.foreign_preservation=lambda *_:value
            result,_=self.run_cleanup(change)
            self.assertEqual(result['status'],status)

    def test_deadline_expiry_does_not_create_new_budget(self):
        def change(b,c,l):
            def wait(until): c.now=l.deadline.end_ns+1;return False
            b.wait_empty=wait
        result,backend=self.run_cleanup(change)
        self.assertEqual(result['status'],'FAIL');self.assertIn('kill',backend.calls)
        self.assertNotIn('docker',backend.calls)

    def test_docker_hang_timeout_is_incomplete_not_pass(self):
        def change(b,c,l):
            def hang(ledger,until):
                c.now=until
                raise subprocess.TimeoutExpired('synthetic docker',1)
            b.stop_container=hang
        result,_=self.run_cleanup(change)
        self.assertEqual(result['status'],'INCOMPLETE')

    def test_partial_receipt_never_overwritten_or_accepted(self):
        clock,ledger=fixture();backend=FakeBackend(clock)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'cleanup.receipt.json';path.write_text('{"schema":')
            result=CleanupController(ledger,backend,clock).run(temp)
            self.assertEqual(result['status'],'INCOMPLETE');self.assertIn('kill',backend.calls)
            self.assertEqual(path.read_text(),'{"schema":')
            with self.assertRaises(ValueError):verify_receipt(path,ledger)

    def test_receipt_events_detect_same_size_corruption_and_copy_portably(self):
        import shutil
        clock,ledger=fixture()
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'source';source.mkdir()
            CleanupController(ledger,FakeBackend(clock),clock).run(source)
            target=Path(temp)/'copied';shutil.copytree(source,target)
            self.assertEqual(verify_receipt(target/'cleanup.receipt.json',ledger)['status'],'PASS')
            events=next(target.glob('cleanup.*.jsonl'));raw=events.read_bytes()
            events.write_bytes(raw.replace(b'begin',b'begix',1))
            with self.assertRaises(Blocked):verify_receipt(target/'cleanup.receipt.json',ledger)

    def test_receipt_export_exceeding_deadline_cannot_return_pass(self):
        import heartbeat_test_owner as owner
        clock,ledger=fixture();original=owner.durable_new
        def delayed(path,value):
            original(path,value);clock.now=ledger.deadline.end_ns+1
        with tempfile.TemporaryDirectory() as temp,patch.object(owner,'durable_new',side_effect=delayed):
            result=CleanupController(ledger,FakeBackend(clock),clock).run(temp)
            self.assertEqual(result['status'],'FAIL');self.assertFalse(result['receipt_complete'])

    def test_export_error_after_deadline_remains_fail(self):
        import heartbeat_test_owner as owner
        clock,ledger=fixture()
        def delayed(*_):
            clock.now=ledger.deadline.end_ns+1;raise OSError('injected fsync failure')
        with tempfile.TemporaryDirectory() as temp,patch.object(owner,'durable_new',side_effect=delayed):
            result=CleanupController(ledger,FakeBackend(clock),clock).run(temp)
            self.assertEqual(result['status'],'FAIL');self.assertFalse(result['receipt_complete'])


class KernelAdapterPolicyTests(unittest.TestCase):
    """Mocked syscalls validate policy only, not native kernel guarantees."""
    def kernel(self):
        _,ledger=fixture();kernel=LinuxPidfdCgroup.__new__(LinuxPidfdCgroup)
        kernel.identity=ledger.cgroup;kernel.mount=Path('/sys/fs/cgroup');kernel.fd=501
        kernel.pidfds={101:(ledger.processes[0],601)}
        return kernel,ledger

    def test_identity_checked_immediately_before_pidfd_term(self):
        kernel,ledger=self.kernel();process=ledger.processes[0]
        for change in ('starttime','namespaces'):
            current=replace(process,**{change:999 if change=='starttime' else dict(process.namespaces,pid='foreign')})
            with self.subTest(change=change),patch.object(kernel,'verify'), \
                 patch('heartbeat_test_owner.linux_process',return_value=current), \
                 patch.object(signal,'pidfd_send_signal',create=True) as send:
                with self.assertRaises(Blocked):kernel.term(process)
                send.assert_not_called()
        with patch.object(kernel,'verify'),patch('heartbeat_test_owner.linux_process',return_value=process), \
             patch.object(signal,'pidfd_send_signal',create=True) as send:
            kernel.term(process);send.assert_called_once_with(601,signal.SIGTERM)

    def test_containment_kill_uses_verified_directory_descriptor(self):
        kernel,_=self.kernel()
        with patch.object(kernel,'verify') as verify,patch('heartbeat_test_owner.os.open',return_value=701) as op, \
             patch('heartbeat_test_owner.os.write') as write,patch('heartbeat_test_owner.os.close'):
            kernel.kill();verify.assert_called_once()
            self.assertEqual(op.call_args.kwargs['dir_fd'],501)
            self.assertEqual(op.call_args.args[0],'cgroup.kill');write.assert_called_once_with(701,b'1')
        with patch.object(kernel,'verify',side_effect=Blocked('replaced cgroup')), \
             patch('heartbeat_test_owner.os.open') as op:
            with self.assertRaises(Blocked):kernel.kill()
            op.assert_not_called()


class DockerAdapterTests(unittest.TestCase):
    def setUp(self):
        self.clock,self.ledger=fixture();self.calls=[];self.mode='ok'

    def fake_run(self,command,**kwargs):
        self.calls.append((command,kwargs));identity=self.ledger.container
        if command[1]=='stop':return subprocess.CompletedProcess(command,0,'','')
        if self.mode=='inspect-error':return subprocess.CompletedProcess(command,1,'','daemon unavailable')
        if self.mode=='timeout':raise subprocess.TimeoutExpired(command,kwargs['timeout'])
        row=dict(Id=identity.full_id,Name='/'+identity.name,Created=identity.created,Image=identity.image_id)
        if self.mode=='replacement' and command[-1]==identity.name:row['Id']='b'*64
        return subprocess.CompletedProcess(command,0,json.dumps([row]),'')

    def test_uses_original_full_id_and_all_calls_bounded(self):
        adapter=DockerByIdentity(self.fake_run,self.clock)
        adapter.stop(self.ledger.container,self.ledger.deadline.end_ns)
        self.assertEqual(self.calls[-1][0][-1],self.ledger.container.full_id)
        self.assertTrue(all(0<kwargs['timeout']<=20 for _,kwargs in self.calls))

    def test_inspect_errors_replacement_and_timeout_never_stop(self):
        for mode in ('inspect-error','replacement','timeout'):
            self.mode=mode;self.calls=[]
            with self.subTest(mode=mode),self.assertRaises((Blocked,subprocess.TimeoutExpired)):
                DockerByIdentity(self.fake_run,self.clock).stop(self.ledger.container,self.ledger.deadline.end_ns)
            self.assertFalse(any(command[1]=='stop' for command,_ in self.calls))


class LinuxNativeTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform=='linux','NOT TESTED: Linux pidfd unavailable on Mac')
    def test_native_pidfd_survives_process_exit_without_retargeting(self):
        # Pure synthetic child; no cgroup/systemd/Docker setup or FCU.
        if not hasattr(os,'pidfd_open') or not hasattr(signal,'pidfd_send_signal'):
            self.skipTest('NOT TESTED: pidfd permission/API unavailable')
        child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(10)'])
        descriptor=None
        try:
            descriptor=os.pidfd_open(child.pid,0)
            signal.pidfd_send_signal(descriptor,signal.SIGTERM);child.wait(timeout=3)
            with self.assertRaises(ProcessLookupError):signal.pidfd_send_signal(descriptor,signal.SIGTERM)
        finally:
            if child.poll() is None:child.kill();child.wait(timeout=3)
            if descriptor is not None:os.close(descriptor)

    @unittest.skip('BLOCKED: reviewed manager backend + separately approved delegated cgroup required')
    def test_native_manager_kill_recovery_and_cgroup_orphan_reaping(self):
        # Native acceptance specification: manager arms immutable deadline, holds
        # startup barrier; SIGKILL launcher then watchdog in separate runs; kill
        # TERM-resistant setsid descendants via retained cgroup.kill handle;
        # verify empty subtree AND reaping, foreign process alive, durable receipt.
        # Deliberately not implemented as a fake native PASS.
        raise Blocked('Native manager integration not implemented')

    def test_mac_cannot_construct_linux_authority(self):
        if sys.platform=='linux':self.skipTest('Mac-specific admission check')
        _,ledger=fixture()
        with self.assertRaises(Blocked):LinuxPidfdCgroup(ledger.cgroup)


if __name__=='__main__':unittest.main()
