"""Offline clock/fault fixtures; never live synchronization/fusion evidence."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace as S
from navigation_clock import ClockMapping
from navigation_lifecycle import NavigationLifecycle
from run_navigation_preflight import evaluate, REQUIRED
from test_vio_frame_bridge import message


def mapping(verified=True):
    return ClockMapping([dict(source_ns=s,target_ns=s+100*10**9,host_ns=s-90*10**9) for s in [99*10**9,100*10**9,100_200_000_000]],
                        source_epoch='fixture-A',verified=verified,uncertainty_ns=1000)


def token(ns=100_110_000_000,epoch=0,session='fixture-producer'):
    return dict(session=session,epoch=epoch,stamp_ns=ns,epoch_start_ns=100_000_000_000,ready=True)


class ClockMappingTests(unittest.TestCase):
    def test_affine_mapping_and_original_stamp_metadata(self):
        m=mapping();value=m.map(100_123_456_789,10_200_000_000,'fixture-A')
        self.assertEqual(value['fcu_ns'],200_123_456_789)
        self.assertEqual(value['acquisition_ns'],100_123_456_789)
    def test_regression_without_time_bound_cannot_authorize(self):
        with self.assertRaises(ValueError):mapping(False).map(100_000_000_000,10_200_000_000,'fixture-A')
    def test_reset_pause_stale_future_and_epoch_rejected(self):
        m=mapping()
        for stamp,host,epoch in [(100_000_000_000,12_000_000_000,'fixture-A'),(102_000_000_000,10_200_000_000,'fixture-A'),(100_000_000_000,10_200_000_000,'other'),(float('nan'),10_200_000_000,'fixture-A')]:
            with self.subTest(stamp=stamp,epoch=epoch),self.assertRaises(ValueError):m.map(stamp,host,epoch)
        for anchors in [[dict(source_ns=1,target_ns=1,host_ns=1)]*3,
                        [dict(source_ns=i,target_ns=i,host_ns=i) for i in [3,2,1]]]:
            with self.assertRaises(ValueError):ClockMapping(anchors,source_epoch='fixture')
    def test_drift_jitter_and_large_epoch_numeric_precision(self):
        anchors=[dict(source_ns=10**18+i*10**9,target_ns=5*10**9+i*1_001_000_000+(i%2)*1000,host_ns=i*10**9) for i in range(10)]
        m=ClockMapping(anchors,source_epoch='unit',verified=True,uncertainty_ns=1000)
        self.assertAlmostEqual(m.slope,1.001,places=6)
        self.assertLess(abs(m.map(10**18+9*10**9,9*10**9,'unit')['fcu_ns']-(5*10**9+9*1_001_000_000)),2000)


class LifecycleTests(unittest.TestCase):
    def make_gate(self,folder):
        g=NavigationLifecycle(Path(folder)/'authorization.json',mapping())
        g.clock_tick(100_200_000_000,10_200_000_000)
        g.authorize('explicit-fixture-reference',token(),'PASS',10_200_000_000)
        return g
    def test_real_conversion_composition_preserves_input_and_maps_pair_once(self):
        with tempfile.TemporaryDirectory() as folder:
            g=self.make_gate(folder);m=message();before=copy.deepcopy(m)
            result=g.pair(m,token(100_123_456_789),10_200_000_000,True,S,S)
            self.assertIsNotNone(result);pose,speed,meta=result
            self.assertEqual(m,before)
            self.assertEqual(pose.header.stamp.sec,200)
            self.assertEqual(speed.header.stamp.nanosec,123456789)
            self.assertEqual(meta['acquisition_ns'],100_123_456_789)
    def test_faults_latch_even_after_clock_or_quality_recovers(self):
        for fault in ['quality','clock_loss','clock_reset','stale','future','epoch','producer','missing_token','covariance']:
            with self.subTest(fault=fault),tempfile.TemporaryDirectory() as folder:
                g=self.make_gate(folder);m=message();t=token(100_123_456_789);quality=True
                if fault=='quality':quality=False
                if fault=='clock_loss':g.watchdog(12_000_000_000)
                if fault=='clock_reset':g.clock_tick(1,10_210_000_000)
                if fault=='stale':m.header.stamp.sec=99
                if fault=='future':m.header.stamp.sec=102
                if fault=='epoch':t['epoch']=1
                if fault=='producer':t['session']='new-producer'
                if fault=='missing_token':t['stamp_ns']+=1
                if fault=='covariance':m.pose.covariance[0]=-1
                self.assertIsNone(g.pair(m,t,10_200_000_000,quality,S,S))
                self.assertIsNone(g.pair(message(),token(100_123_456_789),10_200_000_000,True,S,S))
    def test_actual_process_restart_stays_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'authorization.json'
            env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]))
            script1='from test_navigation_preflight import LifecycleTests; import sys,os; g=LifecycleTests().make_gate(sys.argv[1]); os._exit(0)'
            subprocess.run([sys.executable,'-B','-c',script1,folder],env=env,check=True)
            self.assertFalse(json.loads(path.read_text())['blocked'])
            script='from navigation_lifecycle import NavigationLifecycle; from test_navigation_preflight import mapping; import sys; print(NavigationLifecycle(sys.argv[1],mapping()).durable.state["blocked"])'
            result=subprocess.run([sys.executable,'-B','-c',script,str(path)],capture_output=True,text=True,env=env,check=True)
            self.assertEqual(result.stdout.strip(),'True')
            restored=NavigationLifecycle(path,mapping());restored.clock_tick(100_200_000_000,10_200_000_000)
            self.assertIsNone(restored.pair(message(),token(100_123_456_789),10_200_000_000,True,S,S))
            with self.assertRaises(ValueError):restored.authorize('explicit-fixture-reference',token(),'PASS',10_200_000_000)
            restored.authorize('explicit-NEW-fixture-reference',token(),'PASS',10_200_000_000)
            self.assertIsNotNone(restored.pair(message(),token(100_123_456_789),10_200_000_000,True,S,S))
    def test_hold_preflight_or_unverified_mapping_cannot_initialize(self):
        with tempfile.TemporaryDirectory() as folder:
            g=NavigationLifecycle(Path(folder)/'state.json',mapping(False));g.clock_tick(100_200_000_000,10_200_000_000)
            with self.assertRaises(ValueError):g.authorize('new',token(),'PASS',10_200_000_000)


class EvidenceTests(unittest.TestCase):
    def test_missing_or_fixture_evidence_never_passes_live_preflight(self):
        self.assertEqual(evaluate({},Path('.'))['decision'],'HOLD')
        self.assertEqual(evaluate({name:dict(status='PASS',scope='unit',artifacts=[]) for name in REQUIRED},Path('.'))['decision'],'HOLD')


class EgressTests(unittest.TestCase):
    def test_queued_mavros_packet_revoked_on_latch_and_egress_restart(self):
        from navigation_egress import VisionEgress
        with tempfile.TemporaryDirectory() as folder:
            gate=LifecycleTests().make_gate(folder)
            pair=gate.pair(message(),token(100_123_456_789),10_200_000_000,True,S,S)
            egress=VisionEgress(gate.durable.path);egress.authorize('explicit-fixture-reference','fixture-producer','PASS');self.assertTrue(egress.approve(pair[2]))
            usec=pair[2]['fcu_ns']//1000
            self.assertTrue(egress.check('VISION_POSITION_ESTIMATE',usec,0,10_200_000_000))
            self.assertFalse(egress.check('VISION_POSITION_ESTIMATE',usec,0,10_200_000_000))
            egress.close()
            restarted=VisionEgress(gate.durable.path)
            self.assertFalse(restarted.check('VISION_SPEED_ESTIMATE',usec,0,10_200_000_000))
            self.assertFalse(restarted.approve(pair[2]))
            with self.assertRaises(ValueError):restarted.authorize('explicit-fixture-reference','fixture-producer','PASS')
            gate.block('reset after MAVROS enqueue')
            self.assertFalse(egress.check('VISION_SPEED_ESTIMATE',usec,0,10_200_000_000))
    def test_expiry_and_unapproved_reset_counter_fail_closed(self):
        from navigation_egress import VisionEgress
        with tempfile.TemporaryDirectory() as folder:
            gate=LifecycleTests().make_gate(folder)
            pair=gate.pair(message(),token(100_123_456_789),10_200_000_000,True,S,S)
            egress=VisionEgress(gate.durable.path);egress.authorize('explicit-fixture-reference','fixture-producer','PASS');egress.approve(pair[2]);usec=pair[2]['fcu_ns']//1000
            self.assertFalse(egress.check('VISION_SPEED_ESTIMATE',usec,1,10_200_000_000))
            self.assertFalse(egress.check('VISION_SPEED_ESTIMATE',usec,0,pair[2]['allowed_host_deadline_ns']+1))
            self.assertFalse(egress.check('ODOMETRY',usec,0,10_200_000_000))


class StoreOwnershipTests(unittest.TestCase):
    def test_second_writer_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=LifecycleTests().make_gate(folder)
            with self.assertRaises(ValueError):NavigationLifecycle(gate.durable.path,mapping())
            gate.close()
            restarted=NavigationLifecycle(gate.durable.path,mapping())
            self.assertTrue(restarted.durable.state['blocked'])
