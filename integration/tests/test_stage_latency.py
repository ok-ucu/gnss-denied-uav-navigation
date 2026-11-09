"""Offline regressions; these are not live timing or flight evidence."""
import json
from pathlib import Path
import tempfile
import unittest
import threading

import stage_timing
from stage_timing import Recorder,FaultFile,AsyncAudit
import test_navigation_preflight as fixtures
from navigation_egress import VisionEgress
from run_vio_stream import recovery_deadline
from navigation_lifecycle import NavigationLifecycle
from test_phase1_pipeline import prepared
from analyze_stage_latency import fault_verdict,quantiles


class FinalBoundaryTests(unittest.TestCase):
    def test_common_clock_event_keeps_age_but_checks_mapping_at_now(self):
        mapping=prepared()
        with tempfile.TemporaryDirectory() as folder:
            gate=NavigationLifecycle(Path(folder)/'store.json',mapping)
            # Common ROS watermark is one applied anchor behind. Its event is
            # real, but is earlier than the latest anchor's host timestamp.
            with self.assertRaises(ValueError):mapping.check_clock(240_000_000,10_720_000_000)
            gate.clock_tick(240_000_000,10_720_000_000,validation_host_ns=10_850_000_000)
            self.assertEqual(gate.clock,240_000_000)
            self.assertEqual(gate.clock_host,10_720_000_000)
            self.assertFalse(gate.watchdog(11_000_000_000))  # age not refreshed to NOW
            gate.close()
    def test_deadline_is_rechecked_after_state_read(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=fixtures.LifecycleTests().make_gate(folder)
            pair=gate.pair(fixtures.message(),fixtures.token(100_123_456_789),10_200_000_000,True,fixtures.S,fixtures.S)
            egress=VisionEgress(gate.durable.path)
            egress.authorize('explicit-fixture-reference','fixture-producer','PASS')
            self.assertTrue(egress.approve(pair[2]))
            usec=pair[2]['fcu_ns']//1000
            # The arrival is timely, but a blocking persistent-state read crossed
            # the deadline. A pre-read timestamp must not produce admission.
            self.assertFalse(egress.check('VISION_POSITION_ESTIMATE',usec,0,10_200_000_000,
                                          clock=lambda:pair[2]['allowed_host_deadline_ns']+1))
            self.assertTrue(egress.authorization.state['blocked'])
            gate.close();egress.close()

    def test_original_deadline_is_not_extended(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=fixtures.LifecycleTests().make_gate(folder)
            pair=gate.pair(fixtures.message(),fixtures.token(100_123_456_789),10_200_000_000,True,fixtures.S,fixtures.S)
            meta=pair[2]
            expected=10_200_000_000+200_000_000-(100_200_000_000-meta['acquisition_ns'])-meta['uncertainty_ns']
            self.assertEqual(meta['allowed_host_deadline_ns'],expected)
            gate.close()


class ProfilingTests(unittest.TestCase):
    def test_closed_before_injection_is_not_a_fault_pass(self):
        self.assertEqual(fault_verdict(True,True,True,0,False),'NOT TESTED')
        self.assertEqual(fault_verdict(False,False,True,0,False),'NOT TESTED')
        self.assertEqual(fault_verdict(True,False,True,0,False),'PASS')
        self.assertEqual(fault_verdict(True,False,True,1,False),'FAIL')
        self.assertEqual(fault_verdict(True,False,True,0,True),'FAIL')
    def test_latency_quantiles_keep_count_and_nonfinite_fails(self):
        self.assertEqual(quantiles([])['n'],0)
        self.assertEqual(quantiles([1,2,3])['p50'],2)
        with self.assertRaises(ValueError):quantiles([float('nan')])
    def test_diagnostic_writer_drains_and_preserves_order(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'audit.jsonl'
            with path.open('x') as file:
                audit=AsyncAudit({'rows':file})
                for i in range(10):audit.put('rows',dict(i=i))
                audit.close()
            self.assertEqual([json.loads(line)['i'] for line in path.read_text().splitlines()],list(range(10)))

    def test_diagnostic_overflow_is_error_not_silent_drop(self):
        entered=threading.Event();release=threading.Event()
        class SlowFile:
            def write(self,value):entered.set();release.wait(2)
            def flush(self):pass
        audit=AsyncAudit({'rows':SlowFile()},capacity=1)
        audit.put('rows',dict(i=0));self.assertTrue(entered.wait(1))
        audit.put('rows',dict(i=1))
        with self.assertRaises(RuntimeError):audit.put('rows',dict(i=2))
        release.set();audit.close()
    def test_no_reset_arithmetic_before_first_camera_timestamp(self):
        self.assertIsNone(recovery_deadline(None))
        self.assertEqual(recovery_deadline(10.),10.25)
    def test_profile_cannot_overwrite_and_reports_overflow(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'stages.jsonl'
            recorder=Recorder(path,capacity=2)
            recorder.add(dict(stage='one'));recorder.add(dict(stage='two'));recorder.add(dict(stage='three'))
            recorder.close()
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertGreaterEqual(rows[-1]['dropped'],1)
            with self.assertRaises(ValueError):Recorder(path)

    def test_fault_control_is_explicit_and_unknown_value_fails(self):
        self.assertEqual(FaultFile(None).read(),{})
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'fault.json';path.write_text('{"fault":"stale"}')
            self.assertEqual(FaultFile(path).read()['fault'],'stale')
            path.write_text('{"fault":"invent-measurements"}')
            with self.assertRaises(ValueError):FaultFile(path).read()


if __name__=='__main__':unittest.main()
