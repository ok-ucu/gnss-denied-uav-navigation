"""Offline regressions; these are not live timing or flight evidence."""
import json
from pathlib import Path
import tempfile
import unittest
import threading
import io
from contextlib import redirect_stdout
from unittest.mock import patch

import stage_timing
from stage_timing import Recorder,FaultFile,AsyncAudit
import test_navigation_preflight as fixtures
from navigation_egress import VisionEgress
from run_vio_stream import recovery_deadline
from navigation_lifecycle import NavigationLifecycle
from test_phase1_pipeline import prepared
from analyze_stage_latency import fault_verdict,quantiles,normal_verdict,analyze,clock_path_stats,stage_summary
from phase1_transport import heartbeat_fresh,ready_datagrams


class FinalBoundaryTests(unittest.TestCase):
    def clock_gate(self,folder,ros_stamp=280_000_000,ros_host=10_840_000_000):
        gate=NavigationLifecycle(Path(folder)/'clock.json',prepared())
        gate.clock_tick(240_000_000,10_720_000_000,validation_host_ns=10_850_000_000)
        gate.observe_clock(ros_stamp,ros_host)
        token=dict(session='clock-fixture',epoch=0,stamp_ns=220_000_000,epoch_start_ns=1,ready=True)
        gate.authorize('explicit-clock-fixture',token,'PASS',max(10_850_000_000,ros_host))
        return gate

    def test_sparse_common_watermark_is_not_clock_liveness(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=self.clock_gate(folder,ros_host=10_950_000_000)
            # Analogue of timing2: common anchor >200ms old, but real ROS
            # progress and latest actual AP production are both <200ms old.
            self.assertGreater(11_000_000_000-gate.clock_host,200_000_000)
            self.assertTrue(gate.watchdog(11_000_000_000))
            self.assertEqual(gate.clock_host,10_720_000_000)  # never relabel anchor
            gate.close()

    def test_fresh_ros_cannot_hide_stale_ap_production(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=self.clock_gate(folder)
            gate.observe_clock(300_000_000,11_040_000_000)
            self.assertFalse(gate.watchdog(11_050_000_000))
            self.assertEqual(gate.reason,'applied clock expired/paused')
            gate.close()

    def test_fresh_ap_cannot_hide_ros_pause_or_auto_reopen(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=self.clock_gate(folder,ros_host=10_720_000_000)
            self.assertFalse(gate.watchdog(11_000_000_000))
            gate.observe_clock(290_000_000,11_000_000_000)
            self.assertFalse(gate.watchdog(11_000_000_000))
            gate.close()

    def test_duplicate_and_reset_observations_do_not_refresh_liveness(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=self.clock_gate(folder)
            gate.observe_clock(280_000_000,10_950_000_000)
            self.assertEqual(gate.ros_clock_host,10_840_000_000)
            gate.observe_clock(1,10_960_000_000)
            self.assertTrue(gate.durable.state['blocked'])
            gate.close()

    def test_permission_uses_actual_ros_age_not_older_common_label(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=self.clock_gate(folder)
            msg=fixtures.message();msg.header.stamp.sec=0;msg.header.stamp.nanosec=230_000_000
            token=dict(session='clock-fixture',epoch=0,stamp_ns=230_000_000,epoch_start_ns=1,ready=True)
            pair=gate.pair(msg,token,10_850_000_000,True,fixtures.S,fixtures.S)
            self.assertIsNotNone(pair)
            metadata=pair[2]
            self.assertEqual(metadata['acquisition_ns'],230_000_000)
            self.assertEqual(metadata['allowed_host_deadline_ns'],10_850_000_000+150_000_000-metadata['uncertainty_ns'])
            gate.close()

    def test_ros_progress_does_not_create_an_unbracketed_anchor(self):
        with tempfile.TemporaryDirectory() as folder:
            gate=self.clock_gate(folder)
            gate.observe_clock(310_000_000,10_860_000_000)
            msg=fixtures.message();msg.header.stamp.sec=0;msg.header.stamp.nanosec=300_000_000
            token=dict(session='clock-fixture',epoch=0,stamp_ns=300_000_000,epoch_start_ns=1,ready=True)
            self.assertIsNone(gate.pair(msg,token,10_860_000_000,True,fixtures.S,fixtures.S))
            self.assertIn('never extrapolate',gate.reason)
            gate.close()

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
    def test_clock_production_and_read_age_are_separate(self):
        stages=[dict(stage='clock_anchor_received',source_ns=1,produced_host_ns=10,received_host_ns=30),
                dict(stage='clock_anchor_validated',source_ns=1,host_ns=35)]
        wire=[dict(valid_capture=True,clock=dict(latest_applied_age_ns=201_000_000,ros_age_ns=1))]
        result=clock_path_stats(stages,wire)
        self.assertEqual(result['production_to_read_wall_ms']['max'],20/1e6)
        self.assertEqual(result['read_to_validated_wall_ms']['max'],5/1e6)
        self.assertEqual(result['valid_wire_freshness_violations'],1)

    def test_normal_case_without_injection_timestamp_can_be_analyzed(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);matrix=root/'evidence/test_matrix';matrix.mkdir(parents=True)
            (matrix/'results.json').write_text(json.dumps([dict(case='normal',transport='t',source='s',injected=False,injection_utc_ns=None)]))
            (matrix/'commands.jsonl').write_text(json.dumps(dict(command=['docker','exec'],host_utc_ns=10))+'\n')
            metric=dict(first_block_after_reference=None,source_summary=dict(gaps=0,recoveries=0),
                        wire_valid=10,wire_invalid=0,mavros_pairs_enqueued=5)
            with patch('analyze_stage_latency.case_metrics',return_value=(metric,[],[])),redirect_stdout(io.StringIO()):
                result=analyze(root,'test',root/'analysis')
            self.assertEqual(result['cases'][0]['status'],'PASS')
            self.assertEqual(result['decision'],'HOLD')

    def test_normal_capture_missing_wire_packet_is_not_success(self):
        metric=dict(source_summary=dict(gaps=0,recoveries=0),wire_valid=10,
                    wire_invalid=0,mavros_pairs_enqueued=5)
        self.assertEqual(normal_verdict(metric),'PASS')
        metric['mavros_pairs_enqueued']=6
        self.assertEqual(normal_verdict(metric),'FAIL')
        metric['mavros_pairs_enqueued']=5
        metric['first_block_after_reference']=dict(reason='clock lost/stalled')
        self.assertEqual(normal_verdict(metric),'FAIL')
        metric['first_block_after_reference']=dict(reason='owner shutdown')
        self.assertEqual(normal_verdict(metric),'PASS')
        metric['missing_eligible_source']=[123]
        self.assertEqual(normal_verdict(metric),'FAIL')
        metric['missing_eligible_source']=[]
        metric['source_capture_end_host_ns']=20
        metric['first_block_after_reference']=dict(reason='clock lost/stalled',host_ns=21)
        self.assertEqual(normal_verdict(metric),'PASS')  # after measured capture ends
        metric['first_block_after_reference']['host_ns']=19
        self.assertEqual(normal_verdict(metric),'FAIL')
        self.assertEqual(quantiles([-10,3,4])['min'],-10)

    def test_missing_future_and_expired_heartbeat_cannot_pass(self):
        self.assertFalse(heartbeat_fresh(None,4_000_000_000))
        self.assertFalse(heartbeat_fresh(5,4))
        self.assertFalse(heartbeat_fresh(1,3_000_000_002))
        self.assertFalse(heartbeat_fresh(True,4))
        self.assertTrue(heartbeat_fresh(1,3_000_000_001))

    def test_heartbeat_period_is_fcu_time_not_slow_simulator_host_time(self):
        # A four-second host interval at RTF=.25 contains one FCU second.
        # A reset goes backwards and cannot inherit the old heartbeat witness.
        self.assertTrue(heartbeat_fresh(10_000_000_000,11_000_000_000))
        self.assertFalse(heartbeat_fresh(10_000_000_000,1_000_000_000))

    def test_wire_batch_drains_until_would_block_and_stays_bounded(self):
        class Socket:
            def __init__(self):self.reads=0
            def recvfrom(self,size):
                self.reads+=1
                if self.reads>3:raise BlockingIOError()
                return bytes([self.reads]),('127.0.0.1',1)
        sock=Socket()
        batch=list(ready_datagrams(sock,limit=2))
        self.assertEqual([v[0] for v in batch],[b'\x01',b'\x02'])
        self.assertEqual(sock.reads,2)
        rest=list(ready_datagrams(sock))
        self.assertEqual([v[0] for v in rest],[b'\x03'])
        self.assertLessEqual(batch[0][1],rest[0][1])

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

class CriticalStageTests(unittest.TestCase):
    def test_summary_accepts_both_cpu_schemas_and_missing_cpu(self):
        result=stage_summary([
            dict(stage='old',start_host_ns=10,end_host_ns=20,cpu_ns=4),
            dict(stage='critical',start_host_ns=30,end_host_ns=50,process_cpu_ns=7,thread_cpu_ns=3),
            dict(stage='unknown',start_host_ns=60,end_host_ns=70)])
        self.assertEqual(result['process_cpu_ms']['old']['max'],4e-6)
        self.assertEqual(result['process_cpu_ms']['critical']['max'],7e-6)
        self.assertNotIn('unknown',result['process_cpu_ms'])
        self.assertEqual(result['wall_ms']['unknown']['n'],1)

    def test_disabled_probe_does_not_read_scheduler(self):
        with patch.object(stage_timing,'_recorder',None),patch.object(stage_timing,'thread_schedule',side_effect=AssertionError('disabled probe')):
            with stage_timing.critical_stage('disabled') as fields:
                self.assertEqual(fields,{})

    def test_thread_cpu_process_cpu_and_runqueue_are_distinct(self):
        rows=[];recorder=type('Sink',(),{'add':staticmethod(rows.append)})()
        with patch.object(stage_timing,'_recorder',recorder), \
             patch.object(stage_timing,'thread_schedule',side_effect=[(11,20,1),(51,120,3)]), \
             patch.object(stage_timing.time,'monotonic_ns',side_effect=[100,110,10110,10120]), \
             patch.object(stage_timing.time,'thread_time_ns',side_effect=[10,1010]), \
             patch.object(stage_timing.time,'process_time_ns',side_effect=[100,5100]):
            with stage_timing.sample(42),stage_timing.critical_stage('actual-source') as fields:
                fields['samples']=7
        row=rows[0]
        self.assertEqual(row['acquisition_ns'],42)
        self.assertEqual(row['end_host_ns']-row['start_host_ns'],10000)
        self.assertEqual(row['thread_cpu_ns'],1000)
        self.assertEqual(row['process_cpu_ns'],5000)
        self.assertEqual(row['schedule_delta'],[40,100,2])
        self.assertEqual(row['samples'],7)

    def test_unavailable_schedule_is_not_zero_and_error_propagates(self):
        rows=[];recorder=type('Sink',(),{'add':staticmethod(rows.append)})()
        with patch.object(stage_timing,'_recorder',recorder),patch.object(stage_timing,'thread_schedule',return_value=None):
            with self.assertRaisesRegex(ValueError,'source invalid'):
                with stage_timing.critical_stage('failed-source'):
                    raise ValueError('source invalid')
        self.assertIsNone(rows[0]['schedule_delta'])
        self.assertGreaterEqual(rows[0]['thread_cpu_ns'],0)

class CameraQueueTimingTests(unittest.TestCase):
    def test_latest_slot_overwrite_dequeue_and_stamp_are_preserved(self):
        from types import SimpleNamespace
        rows=[];owner=SimpleNamespace(_latest=None)
        sink=type('Sink',(),{'add':staticmethod(rows.append)})()
        lock=stage_timing.FrameQueueLock(owner)
        first=(object(),1.2,99.,.1);second=(object(),1.3,100.,.1)
        with patch.object(stage_timing,'_recorder',sink):
            with lock:owner._latest=first
            with lock:owner._latest=second
            with lock:item,owner._latest=owner._latest,None
            with lock:self.assertIsNone(owner._latest)
        self.assertIs(item,second)
        self.assertEqual([r['stage'] for r in rows],['camera_queue_insert','camera_queue_insert','camera_queue_dequeue'])
        self.assertEqual(rows[1]['replaced_source_ns'],1_200_000_000)
        self.assertEqual(rows[2]['source_ns'],1_300_000_000)
        self.assertGreaterEqual(rows[2]['queue_wait_ns'],0)
        self.assertEqual((rows[2]['depth_before'],rows[2]['depth_after']),(1,0))

    def test_slot_lock_releases_and_original_exception_propagates(self):
        from types import SimpleNamespace
        lock=stage_timing.FrameQueueLock(SimpleNamespace(_latest=None))
        with self.assertRaisesRegex(ValueError,'bad frame'):
            with lock:raise ValueError('bad frame')
        self.assertTrue(lock.lock.acquire(blocking=False));lock.lock.release()

    def test_disabled_loop_probe_has_no_timing_reads(self):
        with patch.object(stage_timing,'_recorder',None),patch.object(stage_timing.time,'monotonic_ns',side_effect=AssertionError('disabled')):
            with stage_timing.trace_span('poll') as info:self.assertEqual(info,{})

    def test_short_frame_call_is_retained_but_fast_empty_poll_is_not(self):
        rows=[];sink=type('Sink',(),{'add':staticmethod(rows.append)})()
        with patch.object(stage_timing,'_recorder',sink):
            with stage_timing.trace_span('poll',slow_ns=10**12):pass
            with stage_timing.trace_span('poll',slow_ns=10**12) as info:info['source_ns']=42
        self.assertEqual(len(rows),1);self.assertEqual(rows[0]['source_ns'],42)
        self.assertGreaterEqual(rows[0]['thread_cpu_ns'],0)
        self.assertIsNone(rows[0]['schedule_delta'])
