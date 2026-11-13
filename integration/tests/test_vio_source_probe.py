"""Synthetic OFF/ON/OFF evidence regressions; NOT a live ROS/GPU verification."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from vio_source_probe import analyze, stats, hardware_renderer
from run_vio_stream import clock_status


class SourceProbeTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)
        (self.root/'collection.exit').write_text('0\n')
        self.source=self.root/'source';self.source.mkdir()
        (self.root/'observer').mkdir()
        self.write('phases.jsonl',[dict(phase=k,host=v) for k,v in
            [('before',80),('on',99),('after',161),('end',171)]],True)
        self.write('witness.json',dict(start=75,end=172,failure=[]))
        native=[];resources=[]
        for start in (80,99,161):
            for i in range(3):
                host=start+.1+i*.1
                native.extend(dict(kind=kind,host=host,sim=host*.7,width=640,height=480,bytes=640*480*3) for kind in ('camera','imu','clock'))
                resources.append(dict(host=host,cpu_stat=f'usage_usec {i}\nnr_throttled 0',cpu_max='300000 100000'))
        self.write('native.jsonl',native,True)
        self.write('resources.json',resources)
        self.write('source/configuration.json',dict(isolated_imu=True,profile=True,min_odom_subscribers=1,publication_max_age_sim_s=.18))
        self.write('source/capture_meta.json',dict(
            isolated_imu=dict(failure=None,worker_fault=False,worker_stopped=True,metric_overflows=0,
                ipc_metrics=[dict(ipc_delivery_s=.002)]),overflows=dict(imu=0,clocks=0),
            camera_profile=dict(ros_record_overflows=0,native_record_overflows=0),
            qos=dict(clock_publishers=[dict(publishers=[dict(gid='offline-fixture')])])) )
        self.write('source/summary.json',dict(gaps=0,recoveries=0))
        self.write('source/stages.jsonl',[
            dict(stage='sensor_capture_started',host_ns=100*10**9),
            dict(stage='sensor_capture_finished',host_ns=160*10**9),
            dict(stage='profile_summary',dropped=0)],True)
        publications=[dict(t=10+i*.1,publish_monotonic_s=101+i*.1) for i in range(590)]
        self.write('source/publications.jsonl',publications,True)
        stream=[dict(t=r['t'],published=True,ready=True,wall=r['publish_monotonic_s']-.02,
            publication_clock_check=dict(valid=True,simulation_age_s=.05,callback_age_wall_s=.001),
            odom_subscribers=1,processing_s=.02,processing_thread_cpu_s=.015) for r in publications]
        self.write('source/stream.jsonl',stream,True)
        observed=[dict(t=r['t'],arrival_monotonic_s=r['publish_monotonic_s']+.01,valid=True) for r in publications]
        self.write('observer/input.jsonl',observed,True)
        self.write('source/gate_transitions.json',[dict(wall=100.9,ready=True,reason='readiness passed')])
        sensor_rows=[dict(t=10+i*.001,arrival_monotonic_s=100+i*.001,
            ipc_received_monotonic_s=100.005,ipc_sequence=0) for i in range(2)]
        for name in ('imu','clocks','native_imu'):
            self.write('source/'+name+'.jsonl',sensor_rows,True)
        for name in ('ros_camera','native_camera'):
            self.write('source/'+name+'.jsonl',[dict(t=r['t'],arrival_monotonic_s=r['wall']-.001) for r in stream],True)
        self.write('source/process_load.json',[dict(arrival_monotonic_s=101)])

    def write(self,name,value,jsonl=False):
        (self.root/name).write_text(''.join(json.dumps(r)+'\n' for r in value) if jsonl else json.dumps(value))

    def read(self,name,jsonl=False):
        text=(self.root/name).read_text()
        return [json.loads(line) for line in text.splitlines()] if jsonl else json.loads(text)

    def failed_checks(self):
        result=analyze(self.root)
        self.assertEqual(result['source_continuity'],'FAIL')
        self.assertEqual(result['overall_decision'],'HOLD')
        return [r['name'] for r in result['checks'] if r['status']=='FAIL']

    def test_complete_capture_passes_only_source_gate(self):
        result=analyze(self.root)
        self.assertEqual(result['source_continuity'],'PASS',result['checks'])
        self.assertEqual(result['overall_decision'],'HOLD')
        self.assertFalse(result['execution_allowed'])
        self.assertFalse(result['forwarding_enabled'])
        self.assertIn('NOT TESTED',result['fault_injection'])
        self.assertEqual(result['measurements']['counts']['independent'],590)

    def test_final_silence_cannot_hide_behind_good_pair_gaps(self):
        # Every observed pair is100ms apart; only the FINAL silence exceeds500ms.
        pubs=self.read('source/publications.jsonl',True)[:-10]
        self.write('source/publications.jsonl',pubs,True)
        self.write('source/stream.jsonl',self.read('source/stream.jsonl',True)[:-10],True)
        self.write('observer/input.jsonl',self.read('observer/input.jsonl',True)[:-10],True)
        failures=self.failed_checks()
        self.assertIn('publication_availability',failures)
        self.assertIn('subscriber_availability',failures)
        self.assertEqual(analyze(self.root)['measurements']['publication_first_outage']['last_host'],158.9)

    def test_independent_subscriber_required_loss_duplicate_and_invalid_fail(self):
        original=self.read('observer/input.jsonl',True)
        for mode in ('absent','loss','duplicate','invalid'):
            with self.subTest(mode=mode):
                observed=json.loads(json.dumps(original))
                if mode=='absent':observed=[]
                elif mode=='loss':observed.pop(10)
                elif mode=='duplicate':observed.insert(10,observed[10])
                else:observed[10]['valid']=False
                self.write('observer/input.jsonl',observed,True)
                self.failed_checks()

    def test_worker_failure_and_missing_stage_evidence_fail(self):
        meta=self.read('source/capture_meta.json')
        meta['isolated_imu']['worker_fault']=True
        self.write('source/capture_meta.json',meta)
        self.assertIn('worker_health',self.failed_checks())
        (self.source/'stages.jsonl').unlink()
        self.assertIn('complete_parseable_evidence',self.failed_checks())

    def test_blocked_gate_never_qualifies_as_normal_publication(self):
        self.write('source/gate_transitions.json',[dict(wall=100.9,ready=False,reason='worker failed')])
        self.assertIn('no_publication_while_gate_blocked',self.failed_checks())

    def test_stale_publication_imu_gap_and_competing_clocks_fail(self):
        stream=self.read('source/stream.jsonl',True)
        stream[10]['publication_clock_check']['simulation_age_s']=.181
        self.write('source/stream.jsonl',stream,True)
        self.write('source/imu.jsonl',[dict(t=10),dict(t=10.021)],True)
        meta=self.read('source/capture_meta.json')
        meta['qos']['clock_publishers'][0]['publishers'].append(dict(gid='competing'))
        self.write('source/capture_meta.json',meta)
        failures=self.failed_checks()
        for name in ('publication_checks','imu_order_and_max_gap','one_clock_publisher'):
            self.assertIn(name,failures)

    def test_incomplete_phases_cannot_pass(self):
        self.write('phases.jsonl',[dict(phase='before',host=80)],True)
        self.assertIn('phases_complete',self.failed_checks())

    def test_stats_empty_and_nonfinite(self):
        self.assertIsNone(stats([])['p99'])
        self.assertEqual(stats([1,2,3])['p50'],2)
        for value in (float('nan'),float('inf'),float('-inf')):
            with self.assertRaises(ValueError):stats([value])

    def test_clock_receipt_between_frame_entry_and_snapshot_is_not_future(self):
        # Frame entry10.0 -> real callback10.001 -> snapshot/validation10.002.
        clock=dict(t=1.02,arrival_monotonic_s=10.001)
        self.assertFalse(clock_status(1.,clock,now=10.)['valid'])
        with patch('run_vio_stream.time.monotonic',return_value=10.002):
            result=clock_status(1.,clock)
        self.assertTrue(result['valid'])
        self.assertEqual(result['validation_host_s'],10.002)
        self.assertAlmostEqual(result['callback_age_wall_s'],.001)

    def test_actual_renderer_file_formats_and_cpu_fallbacks(self):
        renderer='D3D12 (NVIDIA GeForce RTX 5060 Laptop GPU)'
        self.assertEqual(hardware_renderer('vendor=Microsoft Corporation\nrenderer='+renderer),renderer)
        self.assertEqual(hardware_renderer('GL_RENDERER=NVIDIA GeForce RTX 4090'),'NVIDIA GeForce RTX 4090')
        for value in ('llvmpipe (LLVM 15)','D3D12 (Microsoft Basic Render Driver)','SwiftShader','unknown'):
            with self.assertRaises(ValueError):hardware_renderer('renderer='+value)


if __name__=='__main__':unittest.main()
