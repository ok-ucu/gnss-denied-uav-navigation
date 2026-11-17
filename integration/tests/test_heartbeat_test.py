"""Offline supervision with fake authority; real local fixture subprocesses.

These tests do NOT prove Linux cgroup/systemd integration. No Docker/FCU/runtime.
"""
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from heartbeat_test_owner import Blocked
from linux_gpu.heartbeat_test import (IndependentWatchdog, ManagerRecovery,
                                    RecoveryRegistration, StartupBarrier)
from test_heartbeat_test_owner import fixture


class FakeManager(ManagerRecovery):
    def __init__(self): self.calls=[]; self.valid=True
    def register(self,ledger):
        self.calls.append('register')
        return RecoveryRegistration(ledger.test_id,ledger.digest(),ledger.deadline.end_ns,'fixture-token')
    def verify(self,registration,ledger): self.calls.append('verify');return self.valid
    def recover(self,registration,ledger,reason):
        self.calls.append(('recover',reason,ledger.deadline.end_ns))
        return dict(status='INCOMPLETE',reason='synthetic recovery requested')


class WatchdogPolicyTests(unittest.TestCase):
    def test_no_default_manager_or_unarmed_barrier_can_start(self):
        clock,ledger=fixture();barrier=StartupBarrier(ledger,clock=clock)
        with self.assertRaisesRegex(Blocked,'not integrated'):barrier.arm()
        with self.assertRaises(Blocked):barrier.release()

    def test_arm_recheck_before_release_and_no_restart_extension(self):
        clock,ledger=fixture();manager=FakeManager();barrier=StartupBarrier(ledger,manager,clock)
        registration=barrier.arm();self.assertEqual(registration.deadline_ns,ledger.deadline.end_ns)
        clock.now+=1_000_000_000
        self.assertTrue(barrier.release())
        with self.assertRaises(Blocked):barrier.release()
        restarted=StartupBarrier(ledger,manager,clock)
        self.assertEqual(restarted.arm().deadline_ns,registration.deadline_ns)

    def test_registration_permissions_and_recheck_failure_block_workload(self):
        clock,ledger=fixture();manager=FakeManager();barrier=StartupBarrier(ledger,manager,clock)
        manager.valid=False
        with self.assertRaises(Blocked):barrier.arm()
        self.assertFalse(barrier.released)
        manager.valid=True;barrier.arm();manager.valid=False
        with self.assertRaises(Blocked):barrier.release()
        self.assertFalse(barrier.released)

    def test_changed_deadline_or_ledger_cannot_authorize(self):
        clock,ledger=fixture();manager=FakeManager();original=manager.register
        for field,value in [('deadline_ns',ledger.deadline.end_ns+1),('ledger_sha256','wrong'),('test_id','other')]:
            manager.register=lambda l,field=field,value=value:replace(original(l),**{field:value})
            with self.subTest(field=field),self.assertRaises(Blocked):StartupBarrier(ledger,manager,clock).arm()

    def test_watchdog_requests_recovery_for_crash_hang_unknown_or_deadline(self):
        for state in ('EXITED','HUNG','UNKNOWN','HEALTHY'):
            clock,ledger=fixture();manager=FakeManager();registration=manager.register(ledger)
            if state=='HEALTHY':clock.now=ledger.deadline.end_ns-ledger.deadline.term_ns-ledger.deadline.kill_ns-ledger.deadline.verify_ns-ledger.deadline.export_ns
            watchdog=IndependentWatchdog(ledger,registration,manager,lambda:state,clock)
            self.assertEqual(watchdog.tick()['status'],'INCOMPLETE')
            self.assertEqual(watchdog.state,'INCOMPLETE')
            self.assertEqual(manager.calls[-1][0],'recover')
            self.assertEqual(manager.calls[-1][2],ledger.deadline.end_ns)

    def test_unknown_recovery_authority_is_not_silent_success(self):
        clock,ledger=fixture();manager=FakeManager();registration=manager.register(ledger);manager.valid=False
        watchdog=IndependentWatchdog(ledger,registration,manager,lambda:'HEALTHY',clock)
        self.assertEqual(watchdog.tick()['status'],'INCOMPLETE')
        self.assertIn('authority',manager.calls[-1][1])

    def test_large_poll_does_not_sleep_through_cleanup_reserve(self):
        clock,ledger=fixture();manager=FakeManager();registration=manager.register(ledger)
        watchdog=IndependentWatchdog(ledger,registration,manager,lambda:'HEALTHY',clock)
        sleeps=[]
        def sleep(seconds):sleeps.append(seconds);clock.now+=int(seconds*1e9)
        result=watchdog.run(100,sleep)
        self.assertEqual(sleeps,[12]);self.assertEqual(result['status'],'INCOMPLETE')


# Synthetic manager implementation IN THE TEST ONLY. It owns direct Popen
# children until wait/reap, never uses process names, kill-0 or group signals.
# Cgroup/Docker results are modeled; only independent OS-process behavior is real.
MANAGER_FIXTURE = r'''
from dataclasses import replace
import json, os, pathlib, signal, subprocess, sys, time
from heartbeat_test_owner import CleanupController, Deadline, durable_new
from test_heartbeat_test_owner import fixture, FakeBackend
from test_heartbeat_test import FakeManager
from linux_gpu.heartbeat_test import StartupBarrier
directory=pathlib.Path(sys.argv[1]);scenario=sys.argv[2]
_,ledger=fixture();start=time.monotonic_ns()
ledger=replace(ledger,deadline=Deadline(start,start+5_000_000_000,
    400_000_000,400_000_000,400_000_000,400_000_000,'boot-fixture','time:[1]'))
ledger.publish(directory)
health=directory/'health';health.write_text('HEALTHY')
watchcode="""
import json, pathlib, sys, time
from heartbeat_test_owner import Ledger
from linux_gpu.heartbeat_test import IndependentWatchdog
from test_heartbeat_test import FakeManager
p=pathlib.Path(sys.argv[1]);ledger=Ledger.restore(json.loads((p/'ownership.json').read_text()))
class RequestManager(FakeManager):
    def recover(self,registration,ledger,reason):
        (p/'recovery.request').write_text(reason)
        return {'status':'INCOMPLETE'}
m=RequestManager();r=m.register(ledger)
IndependentWatchdog(ledger,r,m,lambda:(p/'health').read_text()).run(.01)
"""
children=[];watchdog=None;foreign=None
try:
    manager=FakeManager();barrier=StartupBarrier(ledger,manager);barrier.arm();barrier.release()
    # Bootstrap barrier above is real policy; manager attestation is synthetic.
    launcher=subprocess.Popen([sys.executable,'-c','import time; time.sleep(20)']);children.append(launcher)
    resistant=subprocess.Popen([sys.executable,'-c',
        "import os,signal,time; os.setsid(); signal.signal(signal.SIGTERM,signal.SIG_IGN); print('ready',flush=True); time.sleep(20)"],stdout=subprocess.PIPE,text=True)
    children.append(resistant)
    if resistant.stdout.readline().strip()!='ready':raise RuntimeError('fixture failed')
    foreign=subprocess.Popen([sys.executable,'-c','import time; time.sleep(20)'])
    watchdog=subprocess.Popen([sys.executable,'-c',watchcode,str(directory)])
    durable_new(directory/'fixture.identities.json',dict(launcher=launcher.pid,watchdog=watchdog.pid,
        manager=os.getpid(),resistant=resistant.pid,foreign=foreign.pid))
    if scenario=='launcher-sigkill':launcher.kill();launcher.wait(timeout=1);health.write_text('EXITED')
    elif scenario=='watchdog-sigkill':watchdog.kill();watchdog.wait(timeout=1)
    elif scenario=='normal':launcher.terminate();launcher.wait(timeout=1);health.write_text('EXITED')
    # 'hang': launcher never reports completion; original absolute deadline wins.
    while not (directory/'recovery.request').exists() and watchdog.poll() is None:
        if time.monotonic_ns()>=ledger.deadline.end_ns:break
        time.sleep(.01)
    reason='watchdog exited: independent manager recovery' if watchdog.poll() is not None else 'request'
    class LocalChildren(FakeBackend):
        def signal_term(self,ledger,until):
            for child in children:
                if child.poll() is None:child.terminate()
            return True
        def wait_empty(self,until):
            while time.monotonic_ns()<until:
                if all(p.poll() is not None for p in children):return True
                time.sleep(.005)
            return all(p.poll() is not None for p in children)
        def kill(self,ledger,until):
            for child in children:
                if child.poll() is None:child.kill()
            return True
        def terminal(self,ledger,until):
            for child in children:child.wait(timeout=max(.001,(until-time.monotonic_ns())/1e9))
            return dict(cgroup_empty=True,docker_stopped=True,processes_exited=all(p.poll() is not None for p in children))
        def foreign_preservation(self,ledger,until):return foreign.poll() is None
    result=CleanupController(ledger,LocalChildren(time.monotonic_ns)).run(directory)
    durable_new(directory/'fixture.result.json',dict(status=result['status'],reason=reason,
        foreign_alive=foreign.poll() is None,all_owned_exited=all(p.poll() is not None for p in children),
        end_ns=time.monotonic_ns(),deadline_ns=ledger.deadline.end_ns,
        evidence_scope='synthetic authority / real local children only'))
finally:
    for child in children+[watchdog,foreign]:
        if child is not None:
            if child.poll() is None:child.kill()
            child.wait(timeout=2)
            if child.stdout:child.stdout.close()
'''


class RealPortableSupervisorTests(unittest.TestCase):
    def test_local_manager_survives_launcher_and_watchdog_sigkill_hang_and_normal(self):
        for scenario in ('normal','launcher-sigkill','watchdog-sigkill','hang'):
            with self.subTest(scenario=scenario),tempfile.TemporaryDirectory() as temp:
                # subprocess.run here is a TEST timeout, not a production watchdog.
                answer=subprocess.run([sys.executable,'-B','-c',MANAGER_FIXTURE,temp,scenario],
                    capture_output=True,text=True,timeout=12)
                self.assertEqual(answer.returncode,0,answer.stderr)
                result=json.loads((Path(temp)/'fixture.result.json').read_text())
                self.assertEqual(result['status'],'PASS',result)
                self.assertTrue(result['all_owned_exited']);self.assertTrue(result['foreign_alive'])
                self.assertLess(result['end_ns'],result['deadline_ns'])
                self.assertTrue((Path(temp)/'cleanup.receipt.json').is_file())


if __name__=='__main__':unittest.main()
