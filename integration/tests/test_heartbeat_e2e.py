"""Stage 3 actual loopback/subprocess workflow; NO FCU/native runtime tests."""
import json
from pathlib import Path
import os
import subprocess
import sys
import shutil
import signal
import socket
import tempfile
import time
import unittest
from unittest.mock import patch

from heartbeat_test_owner import Blocked
from heartbeat_witness import witness_only
from linux_gpu.heartbeat_test import StartupBarrier
from synthetic_heartbeat_e2e import (Coordinator,NATIVE,SCENARIOS,OwnedChild,export_report,
    host_now,reap_owned,run_synthetic,verify_synthetic)
from synthetic_heartbeat_ipc import AuthenticationFailure,TestControl
from test_heartbeat_test_owner import fixture
from test_heartbeat_witness import plan_for


class AuthenticatedIPCTests(unittest.TestCase):
    def channels(self):
        left,right=socket.socketpair(socket.AF_UNIX,socket.SOCK_STREAM)
        self.addCleanup(left.close);self.addCleanup(right.close)
        key=b'x'*32
        return (TestControl(left,key,'synthetic-run','launcher','supervisor',123),
                TestControl(right,key,'synthetic-run','supervisor','launcher',123))
    def transfer(self,sender,receiver):
        sender.flush()
        for _ in range(50):
            answer=receiver.receive()
            if answer:return answer
            time.sleep(.001)
        self.fail('Private IPC fixture did not deliver')
    def test_authenticated_exchange_and_replay_rejected(self):
        child,parent=self.channels();child.queue('SILENCE');rows=self.transfer(child,parent)
        self.assertEqual(rows[0]['command'],'SILENCE')
        child.sent=0;child.queue('SILENCE')
        with self.assertRaises(AuthenticationFailure):self.transfer(child,parent)
    def test_wrong_secret_run_id_and_deadline_are_rejected(self):
        for mode in ('secret','run','deadline'):
            with self.subTest(mode=mode):
                child,parent=self.channels()
                if mode=='secret':child.key=b'y'*32
                if mode=='deadline':child.deadline_ns=124
                child.queue('SILENCE',wrong_run=mode=='run')
                with self.assertRaises(AuthenticationFailure):self.transfer(child,parent)


class SyntheticBoundaryTests(unittest.TestCase):
    def test_shared_posix_clock_has_cross_process_epoch(self):
        before=host_now()
        child=int(subprocess.check_output([sys.executable,'-B','-c',
            'from synthetic_heartbeat_e2e import host_now; print(host_now())'],text=True))
        self.assertLessEqual(before,child);self.assertLessEqual(child,host_now())
    def test_export_cannot_publish_positive_marker_after_deadline(self):
        with tempfile.TemporaryDirectory() as temp,patch('synthetic_heartbeat_e2e.host_now',return_value=11):
            with self.assertRaises(TimeoutError):export_report(Path(temp),dict(status='PASS',deadline=dict(end_ns=10)))
            self.assertFalse((Path(temp)/'synthetic.complete.json').exists())
    def test_native_barrier_remains_closed_and_adapter_refuses_native_endpoint(self):
        clock,ledger=fixture()
        with self.assertRaises(Blocked):StartupBarrier(ledger,clock=clock).arm()
        with tempfile.TemporaryDirectory() as temp:
            owner=Coordinator(Path(temp)/'run','normal',20)
            try:
                with self.assertRaisesRegex(Blocked,'refuses ALL production'):
                    witness_only(plan_for(ledger),ledger,owner.manager,clock)
            finally:owner.trace.close();owner.auth_log.close();owner.watchdog_log.close()
        with self.assertRaises(Blocked):StartupBarrier(ledger,clock=clock).arm()
    def test_unknown_scenario_and_nonfinite_or_unbounded_budget_refused(self):
        for scenario,budget in [('not-real',20),('normal',float('nan')),('normal',float('inf')),('normal',1),('normal',1000)]:
            with tempfile.TemporaryDirectory() as temp,self.assertRaises(ValueError):Coordinator(Path(temp)/'run',scenario,budget)


class IntegratedSyntheticTests(unittest.TestCase):
    def run_case(self,root,scenario,budget=20):
        started=host_now();report=None
        print('\n  START scenario='+scenario+' budget='+str(budget)+'s',file=sys.stderr,flush=True)
        try:
            report=run_synthetic(root,scenario,budget)
            return report
        finally:
            exits=root/'process_exit.json'
            codes={role:row['exit_code'] for role,row in json.loads(exits.read_text())['children'].items()} if exits.exists() else {}
            print('  END '+json.dumps(dict(scenario=scenario,elapsed_seconds=round((host_now()-started)/1e9,3),
                status=report['status'] if report else 'EXCEPTION',exit_codes=codes,
                teardown=report.get('teardown') if report else None)),file=sys.stderr,flush=True)
    def test_complete_expected_failure_and_portable_copy(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'normal'
            report=self.run_case(root,'normal')
            self.assertEqual(report['status'],'PASS',report)
            self.assertEqual(report['heartbeat_loss']['observed_code'],'HEARTBEAT_STALE')
            self.assertEqual(report['heartbeat_loss']['witness_component_exit_code'],1)
            self.assertEqual(report['heartbeat_loss']['source_continuity'],'NOT TESTED')
            self.assertEqual(report['cleanup']['status'],'PASS');self.assertTrue(report['cleanup']['foreign_preserved_through_cleanup'])
            self.assertTrue(report['cleanup']['all_fixture_children_reaped'])
            self.assertEqual(report['linux_native'],NATIVE)
            self.assertFalse(report['execution_allowed']);self.assertFalse(report['forwarding_enabled'])
            self.assertIn('head',report['source']);self.assertIn('selected_source_sha256',report['source'])
            self.assertLess(report['end_ns'],report['deadline']['end_ns'])
            self.assertEqual(verify_synthetic(root)['status'],'PASS')
            copied=Path(temp)/'copy';shutil.copytree(root,copied)
            self.assertEqual(verify_synthetic(copied)['status'],'PASS')
            original=(copied/'SYNTHETIC_E2E_REPORT.json').read_bytes()
            (copied/'SYNTHETIC_E2E_REPORT.json').write_bytes(original.replace(b'normal',b'normam',1))
            self.assertEqual(verify_synthetic(copied)['status'],'INCOMPLETE')
    def test_fail_closed_scenario_matrix(self):
        expected_incomplete={'incomplete_evidence','corrupted_evidence','partial_export'}
        for scenario in SCENARIOS:
            if scenario=='normal':continue
            with self.subTest(scenario=scenario),tempfile.TemporaryDirectory() as temp:
                root=Path(temp)/scenario
                report=self.run_case(root,scenario,12 if scenario=='cleanup_deadline_exceeded' else 20)
                wanted='INCOMPLETE' if scenario in expected_incomplete else 'FAIL'
                compact={k:report[k] for k in ('scenario','status','failures','heartbeat_loss','cleanup','evidence','supervision','timeline')}
                self.assertEqual(report['status'],wanted,compact)
                self.assertTrue(report['cleanup']['all_fixture_children_reaped'],report)
                self.assertFalse(report['teardown']['survivors'],report)
                self.assertTrue(report['cleanup']['foreign_preserved_through_cleanup'],report)
                self.assertEqual(report['linux_native'],NATIVE)
                self.assertFalse((root/'synthetic.complete.json').exists())
                self.assertEqual(verify_synthetic(root)['status'],wanted,report)
                if scenario in ('launcher_hang','launcher_sigkill','watchdog_crash'):
                    self.assertTrue(report['supervision']['recovery_requests'],report)
                    self.assertEqual(report['cleanup']['status'],'PASS')
                if scenario=='surviving_owned_descendant':
                    self.assertIn('worker',report['cleanup']['alive_at_terminal'])
                    self.assertEqual(report['cleanup']['status'],'FAIL')
                if scenario=='cleanup_deadline_exceeded':
                    self.assertEqual(report['cleanup']['status'],'FAIL')
                    self.assertGreaterEqual(report['end_ns'],report['deadline']['end_ns'])
                if scenario=='ownership_mismatch':
                    receipt=json.loads((root/'cleanup.receipt.json').read_text())
                    events=[json.loads(line) for line in (root/receipt['events']['path']).read_text().splitlines()]
                    self.assertFalse(any(row['action'] in ('term','kill') for row in events))


class TeardownTests(unittest.TestCase):
    def assert_reaped(self,owner):
        for child in [*owner.children.values(),*([owner.foreign] if owner.foreign else [])]:
            self.assertIsNotNone(child.process.returncode,child.role)
            self.assertIsNotNone(child.process.poll(),child.role)
            self.assertTrue(child.process.stdout.closed,child.role)
            self.assertTrue(child.process.stderr.closed,child.role)

    def test_observation_interrupt_timeout_and_cleanup_exception_reap_every_child(self):
        # The unrelated control is NOT in any coordinator's creation ledger.
        foreign=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])
        try:
            for mode in ('interrupt','timeout','cleanup_exception'):
                with self.subTest(mode=mode),tempfile.TemporaryDirectory() as temp:
                    seen=[];setup=Coordinator.setup
                    def started(owner):
                        seen.append(owner);setup(owner)
                    def observe(owner):
                        if mode=='interrupt':raise KeyboardInterrupt()
                        if mode=='timeout':owner.children['worker'].process.wait(timeout=.01)
                    with patch.object(Coordinator,'setup',started),patch.object(Coordinator,'observe',observe):
                        if mode=='cleanup_exception':
                            with patch.object(Coordinator,'cleanup',side_effect=RuntimeError('Injected cleanup error')):
                                with self.assertRaises(RuntimeError):run_synthetic(Path(temp)/mode)
                        elif mode=='interrupt':
                            with self.assertRaises(KeyboardInterrupt):run_synthetic(Path(temp)/mode)
                        else:
                            report=run_synthetic(Path(temp)/mode)
                            self.assertEqual(report['status'],'FAIL')
                            self.assertIn('WORKFLOW_EXCEPTION',report['failures'])
                    owner=seen[0];self.assert_reaped(owner)
                    self.assertFalse((owner.root/'synthetic.complete.json').exists())
                    self.assertIsNone(foreign.poll(),'Unrelated process must survive')
        finally:
            foreign.kill();foreign.wait(timeout=2)

    def test_interrupt_while_reaping_is_deferred_until_siblings_are_reaped(self):
        with tempfile.TemporaryDirectory() as temp:
            owner=Coordinator(Path(temp)/'interrupt','normal',20)
            try:
                for role in ('launcher','worker','control'):
                    owner.children[role]=OwnedChild(role,owner.test_id,owner.deadline,owner.root)
                process=owner.children['launcher'].process;poll=process.poll;first=[True]
                previous=signal.getsignal(signal.SIGINT)
                def interrupted_poll():
                    if first[0]:first[0]=False;os.kill(os.getpid(),signal.SIGINT)
                    return poll()
                with patch.object(process,'poll',interrupted_poll):
                    with self.assertRaises(KeyboardInterrupt):owner.teardown()
                self.assert_reaped(owner)
                self.assertEqual(signal.getsignal(signal.SIGINT),previous)
                receipt=json.loads((owner.root/'process_exit.json').read_text())
                self.assertTrue(receipt['teardown']['interrupted'])
                self.assertEqual(receipt['teardown']['status'],'FAIL')
                self.assertEqual(receipt['teardown']['workflow_deadline_ns'],owner.deadline.end_ns)
            finally:
                reap_owned(list(owner.children.values()),owner.deadline.end_ns)
                for log in (owner.trace,owner.auth_log,owner.watchdog_log):log.close()

    def test_delayed_reap_and_timeout_are_bounded_without_per_child_windows(self):
        with tempfile.TemporaryDirectory() as temp:
            owner=Coordinator(Path(temp)/'delayed','normal',20)
            try:
                children=[OwnedChild(role,owner.test_id,owner.deadline,owner.root) for role in ('launcher','worker')]
                owner.children={c.role:c for c in children}
                delayed=children[0].process;poll=delayed.poll;ready=host_now()+150_000_000
                with patch.object(delayed,'poll',lambda:None if host_now()<ready else poll()):
                    result=reap_owned(children,owner.deadline.end_ns)
                self.assertEqual(result['status'],'PASS',result);self.assert_reaped(owner)
                self.assertLess(result['end_ns']-result['start_ns'],2_000_000_000)
            finally:
                reap_owned(list(owner.children.values()),owner.deadline.end_ns)
                for log in (owner.trace,owner.auth_log,owner.watchdog_log):log.close()

    def test_surviving_child_timeout_is_fail_and_later_disposal_does_not_rewrite_it(self):
        with tempfile.TemporaryDirectory() as temp:
            owner=Coordinator(Path(temp)/'survivor','normal',20)
            child=OwnedChild('worker',owner.test_id,owner.deadline,owner.root)
            try:
                with patch.object(child,'signal',return_value=None):
                    result=reap_owned([child],host_now()+50_000_000)
                self.assertEqual(result['status'],'FAIL')
                self.assertEqual(result['survivors'],['worker'])
            finally:
                disposal=reap_owned([child],owner.deadline.end_ns)
                self.assertEqual(disposal['status'],'PASS',disposal)
                for log in (owner.trace,owner.auth_log,owner.watchdog_log):log.close()
            self.assertEqual(result['status'],'FAIL')
            self.assertIsNotNone(child.process.returncode)

    def test_bootstrap_exception_reaps_partially_constructed_child(self):
        with tempfile.TemporaryDirectory() as temp:
            owner=Coordinator(Path(temp)/'bootstrap','normal',20);created=[];popen=subprocess.Popen
            def spawn(*args,**kwargs):
                child=popen(*args,**kwargs);created.append(child);return child
            try:
                with patch('synthetic_heartbeat_e2e.subprocess.Popen',spawn), \
                     patch('synthetic_heartbeat_e2e.parse_json',side_effect=ValueError('Injected malformed bootstrap')):
                    with self.assertRaises(ValueError):OwnedChild('worker',owner.test_id,owner.deadline,owner.root)
                self.assertEqual(len(created),1);self.assertIsNotNone(created[0].returncode)
            finally:
                for log in (owner.trace,owner.auth_log,owner.watchdog_log):log.close()


if __name__=='__main__':unittest.main()
