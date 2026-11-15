"""Stage 3 actual loopback/subprocess workflow; NO FCU/native runtime tests."""
import json
from pathlib import Path
import os
import subprocess
import sys
import shutil
import socket
import tempfile
import time
import unittest
from unittest.mock import patch

from heartbeat_test_owner import Blocked
from heartbeat_witness import witness_only
from linux_gpu.heartbeat_test import StartupBarrier
from synthetic_heartbeat_e2e import Coordinator,NATIVE,SCENARIOS,export_report,host_now,run_synthetic,verify_synthetic
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
    def test_complete_expected_failure_and_portable_copy(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'normal'
            report=run_synthetic(root,'normal',20)
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
                report=run_synthetic(root,scenario,12 if scenario=='cleanup_deadline_exceeded' else 20)
                wanted='INCOMPLETE' if scenario in expected_incomplete else 'FAIL'
                compact={k:report[k] for k in ('scenario','status','failures','heartbeat_loss','cleanup','evidence','supervision','timeline')}
                self.assertEqual(report['status'],wanted,compact)
                self.assertTrue(report['cleanup']['all_fixture_children_reaped'],report)
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


if __name__=='__main__':unittest.main()
