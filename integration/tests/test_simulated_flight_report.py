"""SYNTHETIC regression fixtures only. Injected decoder can NEVER give overall
PASS/native decoding proof. No simulator, FCU connection or demo PNG is made.
"""
import copy
import io
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import analyze_simulated_flight as a
from analyze_ekf3_fusion import analyze,analyze_records
from analyze_phase1_run import evaluate
from physics_clock_mapping import AppliedClockMapping
from runtime_candidate import AP_COMMIT
from linux_gpu.verified_disarmed import load_codec


class FlightReportTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.run=Path(self.tmp.name)/'run';self.run.mkdir();self.H=10_000_000_000
        self.bin=self.run/'state/fcu/logs/00000001.BIN';self.bin.parent.mkdir(parents=True)
        self.bin.write_bytes(b'\xa3\x95SYNTHETIC UNIT TEST: NOT A REAL LOG')
        import hashlib
        digest=hashlib.sha256(b'SYNTHETIC FIRMWARE FIXTURE').hexdigest()
        self.write('artifacts/arducopter',b'SYNTHETIC FIRMWARE FIXTURE')
        identity=dict(run_id='fixture',clock_epoch='test-epoch',reference_nonce='test-reference',firmware_sha256=digest,
                      execution_allowed=False,forwarding_enabled=False)
        self.write('artifacts/run_identity.json',identity)
        self.write('artifacts/simulation_vision_permit.json',dict(identity,ap_commit=AP_COMMIT,approved=True))
        self.write('artifacts/flight_plan.json',dict(limits=a.LIMITS,execution_allowed=False,forwarding_enabled=False))
        self.events=[dict(event='flight_transition',state=state,host_ns=self.host(t)) for state,t in
                     [('GUIDED',1.7),('ARM',1.9),('CLIMB',2.),('HOVER',4.),('LAND',9.),('DONE',11.)]]
        self.write('evidence/flight-transport/flight.jsonl',self.events)
        self.write('evidence/flight-transport/flight_result.json',dict(status='PASS',state='DONE',reason=None,landed_disarmed_confirmed=True))
        self.write('evidence/flight-transport/summary.json',dict(clock_mapping_verified=True,clock_latched=False))
        self.write('evidence/flight-source/configuration.json',dict(horizontal_gt_input=False,
            world_gt_attitude_used_for_camera_depth=False,source_sha256=a.FILES,synthetic=False,
            vertical_aid='Gazebo GT Z only',ground_z=.001))
        anchors=[];stages=[];witnesses=[];mappings=[];csv=[]
        class Firmware:
            def accepts_firmware(_,value):return value==digest
        mapper=AppliedClockMapping('test-epoch',digest,simulation_route=Firmware())
        for i in range(1221):
            ns=i*10_000_000;target=ns+5_000_000_000;host=self.H+ns
            row=dict(source_ns=ns,target_ns=target,host_ns=host,parsed_no_time_sync=1,use_time_sync=0)
            anchors.append(row);mapper.feed(**row)
            csv.append(f'{ns/1e9:.9f},1,0,0,{target//1000},0,{host},0,0\n')
            stages.extend([dict(stage='clock_anchor_received',source_ns=ns,produced_host_ns=host,received_host_ns=host+500),
                           dict(stage='clock_anchor_validated',source_ns=ns,produced_host_ns=host,host_ns=host+1000)])
            if 11<=i<=21:
                sent=host-10_000_000+2000;received=sent+1000
                fcu=target-10_000_000
                self.assertTrue(mapper.crosscheck(fcu,sent,received))
                witnesses.append(dict(kind='SYSTEM_TIME',fcu_ns=fcu,sent_ns=sent,received_ns=received,
                                      host_ns=host+2000,within_applied_bracket=True))
            if 101<=i<=1151 and i%10==1:
                stamp=ns-10_000_000;approved=host+5000
                mappings.append(dict(mapper.map(stamp,approved,'test-epoch'),approved_host_ns=approved,
                                     allowed_host_deadline_ns=approved+100_000_000))
        self.write('evidence/applied.csv',''.join(csv).encode())
        self.write('evidence/flight-source/clocks.jsonl',[dict(t=i*.01,arrival_monotonic_s=self.host(i*.01)/1e9) for i in range(1221)])
        self.write('evidence/flight-transport/stages.jsonl',stages)
        self.write('evidence/flight-transport/fcuwitness.jsonl',witnesses)
        self.write('evidence/flight-transport/mapping.jsonl',mappings)
        self.frames=[];pubs=[];obs=[];camera=[];wire=[]
        self.records=[dict(mavpackettype='PARM',TimeUS=0,Name=k,Value=v) for k,v in a.EXPECTED.items()]
        for core in (0,1):self.records.append(dict(mavpackettype='MSG',TimeUS=5_500_000,Message=f'EKF3 IMU{core} is using external nav data'))
        for m in mappings:
            t=m['acquisition_ns']/1e9;height,v=self.motion(t);state=[0.,0.,-height,0.,0.,-v,0.,0.,0.]
            self.frames.append(dict(t=t,wall=self.host(t)/1e9,state=state,covariance=(np.eye(9)*.01).tolist(),
                published=True,flow_accepted=True,epoch=0,ready=True,
                publication_clock_check=dict(valid=True,simulation_age_s=.01)))
            pubs.append(dict(t=t,publish_monotonic_s=self.host(t)/1e9+.01))
            stamp=m['acquisition_ns'];obs.append(dict(sec=stamp//10**9,nanosec=stamp%10**9,
                arrival_monotonic_s=self.host(t)/1e9+.012,valid=True))
            camera.append(dict(t=t))
            for kind,log,size in [('VISION_POSITION_ESTIMATE','VISP',21),('VISION_SPEED_ESTIMATE','VISV',9)]:
                cov=(np.eye(3)*.01).ravel().tolist() if size==9 else [float(i==j)*.01 for i in range(6) for j in range(i,6)]
                xyz=state[:3] if size==21 else state[3:6]
                wire.append(dict(message=dict(mavpackettype=kind,usec=m['fcu_ns']//1000,covariance=cov,x=xyz[0],y=xyz[1],z=xyz[2]),
                    forwarded_to_fcu=True,valid_capture=True,checked_host_ns=m['approved_host_ns']+1_000_000,
                    clock=dict(latest_applied_age_ns=1_005_000,ros_age_ns=1_005_000)))
                self.records.append(dict(mavpackettype=log,TimeUS=m['fcu_ns']//1000+12000,
                    RTimeUS=m['fcu_ns']//1000,CTimeMS=m['fcu_ns']//1_000_000+12,Ign=0,Rst=0))
        self.write('evidence/flight-source/stream.jsonl',self.frames)
        self.write('evidence/flight-source/summary.json',dict(gaps=0,recoveries=0,emitted=len(self.frames)))
        self.write('evidence/flight-source/publications.jsonl',pubs)
        self.write('evidence/flight-observer/input.jsonl',obs)
        self.write('evidence/flight-source/ros_camera.jsonl',camera)
        imu=[dict(t=1+i*.01) for i in range(1051)]
        self.write('evidence/flight-source/imu.jsonl',imu)
        poses=[]
        for i in range(611):
            t=i*.02;height,_=self.motion(t)
            poses.append(dict(t=t,poses=[dict(name='iris_with_gimbal',p=[0,0,height+.001])]))
        self.write('evidence/flight-source/poses.jsonl',poses)
        self.write('evidence/flight-transport/wire.jsonl',wire)
        # The transport logs every received /vio/odom sample (phase1_transport on_odom).
        self.write('evidence/flight-transport/odom.jsonl',[dict(acquisition_ns=m['acquisition_ns'],
            callback_entry_host_ns=self.host(m['acquisition_ns']/1e9)+12_000_000) for m in mappings])
        for i in range(123):
            us=5_000_000+i*100000
            for core in (0,1):
                self.records.extend([dict(mavpackettype='XKF3',TimeUS=us,C=core,IVN=0.,IVE=0.,IPN=0.,IPE=0.),
                    dict(mavpackettype='XKF4',TimeUS=us,C=core,FS=0,TS=0,SS=0,SV=.1,SP=.1,SH=.1),
                    dict(mavpackettype='XKFS',TimeUS=us,C=core,SS=0)])
        codec=load_codec();encoder=codec.MAVLink(None,srcSystem=1,srcComponent=1)
        hb=lambda armed,mode:codec.MAVLink_heartbeat_message(2,3,128 if armed else 0,mode,4,3).pack(encoder)
        tcp=hb(False,4)+hb(True,4)+hb(True,9)+codec.MAVLink_extended_sys_state_message(0,1).pack(encoder)
        tcp+=codec.MAVLink_command_ack_message(21,0).pack(encoder)+hb(False,9)
        for w in witnesses:tcp+=codec.MAVLink_system_time_message(0,w['fcu_ns']//1_000_000).pack(encoder)
        self.write('evidence/flight-transport/fcu_tcp.bin',tcp)

    def host(self,t):return self.H+round(t*1e9)

    @staticmethod
    def motion(t):
        if t<2:return .1,0.
        if t<4:
            u=(t-2)/2;return .1+.45*(1-math.cos(math.pi*u)),.225*math.pi*math.sin(math.pi*u)
        if t<9:return 1.,0.
        if t<11:
            u=(t-9)/2;return 1.-.45*(1-math.cos(math.pi*u)),-.225*math.pi*math.sin(math.pi*u)
        return .1,0.

    def write(self,name,value):
        path=self.run/name;path.parent.mkdir(parents=True,exist_ok=True)
        if isinstance(value,bytes):path.write_bytes(value)
        elif name.endswith('.jsonl'):path.write_text(''.join(json.dumps(r)+'\n' for r in value))
        else:path.write_text(json.dumps(value)+'\n')

    def result(self):return a.analyze(self.run,self.bin,decoder=lambda path:self.records)

    def test_complete_logical_fixture_cannot_claim_native_pass(self):
        report,data=self.result()
        self.assertEqual(report['status'],a.NV,report['checks'])
        self.assertEqual(report['checks']['raw_dataflash']['status'],a.NV)
        for name,check in report['checks'].items():
            if name!='raw_dataflash':self.assertEqual(check['status'],'PASS',(name,check))
        self.assertIsNotNone(data['plot_data']);self.assertFalse(report['execution_allowed'])

    def test_climb_odometry_gap_fails_transport_continuity(self):
        path=self.run/'evidence/flight-transport/odom.jsonl'
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        climb=self.host(2.)
        kept=[r for r in rows if not climb<r['callback_entry_host_ns']<=climb+500_000_000]
        self.assertGreater(len(rows)-len(kept),2)
        path.write_text(''.join(json.dumps(r)+'\n' for r in kept))
        report,_=self.result()
        check=report['checks']['transport_odometry_continuity']
        self.assertEqual(check['status'],'FAIL',check)
        self.assertIn('CLIMB',check['reason'])
        self.assertEqual(report['status'],'FAIL')

    def test_missing_critical_files_never_pass(self):
        for name in ['state/fcu/logs/00000001.BIN','evidence/applied.csv','evidence/flight-source/poses.jsonl',
                     'evidence/flight-source/ros_camera.jsonl','evidence/flight-transport/fcuwitness.jsonl',
                     'evidence/flight-transport/flight.jsonl','evidence/flight-transport/flight_result.json',
                     'evidence/flight-transport/odom.jsonl']:
            path=self.run/name;raw=path.read_bytes();path.unlink()
            report,_=self.result();self.assertNotEqual(report['status'],'PASS',name)
            path.write_bytes(raw)

    def test_corrupt_json_partial_jsonl_and_nonfinite_fail(self):
        path=self.run/'evidence/flight-source/stream.jsonl';raw=path.read_bytes()
        for broken in [raw[:-2],b'{broken}\n',raw.replace(b'0.0',b'NaN',1),raw.replace(b'0.0',b'1e400',1)]:
            path.write_bytes(broken);report,_=self.result();self.assertEqual(report['status'],'FAIL')
        path.write_bytes(raw)

    def test_clock_reset_not_repaired_by_sorting(self):
        path=self.run/'evidence/applied.csv';lines=path.read_text().splitlines()
        lines[300]=lines[299];path.write_text('\n'.join(lines)+'\n')
        report,_=self.result();self.assertEqual(report['checks']['clock_mapping']['status'],'FAIL')

    def test_mapping_flag_without_witnesses_is_not_proof(self):
        path=self.run/'evidence/flight-transport/fcuwitness.jsonl';path.unlink()
        report,_=self.result();self.assertEqual(report['checks']['clock_mapping']['status'],a.NV)

    def test_timestamp_units_mismatch_and_contradictory_mapping(self):
        path=self.run/'evidence/flight-transport/mapping.jsonl';original=path.read_bytes()
        for factor in (.001,1000):
            rows=[json.loads(line) for line in original.splitlines()]
            for row in rows:row['fcu_ns']=round(row['fcu_ns']*factor)
            self.write(str(path.relative_to(self.run)),rows)
            report,_=self.result();self.assertEqual(report['checks']['clock_mapping']['status'],'FAIL')
        path.write_bytes(original)

    def test_vio_dropout_remains_failure_despite_good_rmse(self):
        # Missing camera frames/publications, not a fabricated low-quality state.
        for name in ['stream.jsonl','publications.jsonl','ros_camera.jsonl']:
            path=self.run/'evidence/flight-source'/name
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.write(str(path.relative_to(self.run)),[r for r in rows if not 5<r['t']<5.7])
        path=self.run/'evidence/flight-observer/input.jsonl'
        obs=[json.loads(line) for line in path.read_text().splitlines()]
        self.write(str(path.relative_to(self.run)),[r for r in obs if not 5<r['sec']+r['nanosec']*1e-9<5.7])
        pubs=[json.loads(line) for line in (self.run/'evidence/flight-source/publications.jsonl').read_text().splitlines()]
        self.write('evidence/flight-source/summary.json',dict(gaps=0,recoveries=0,emitted=len(pubs)))
        report,_=self.result();self.assertEqual(report['checks']['vio_continuity']['status'],'FAIL')
        self.assertEqual(report['status'],'FAIL')

    def test_synthetic_fallback_or_gt_horizontal_source_refused(self):
        path=self.run/'evidence/flight-source/configuration.json';c=json.loads(path.read_text())
        for change in [dict(synthetic=True),dict(horizontal_gt_input=True),dict(source_sha256={})]:
            self.write(str(path.relative_to(self.run)),dict(c,**change))
            report,_=self.result();self.assertEqual(report['checks']['camera_vio_evidence']['status'],'FAIL')

    def test_landing_marker_does_not_mask_failed_controller(self):
        self.write('evidence/flight-transport/flight_result.json',dict(status='FAIL',state='DONE',reason='VIO lost',landed_disarmed_confirmed=True))
        report,_=self.result();self.assertEqual(report['status'],'FAIL')
        self.assertEqual(report['checks']['confirmed_land_disarmed']['status'],'PASS')

    def test_armed_final_heartbeat_and_wrong_source_refused(self):
        path=self.run/'evidence/flight-transport/fcu_tcp.bin';original=path.read_bytes();codec=load_codec()
        for system,base in [(1,128),(42,0)]:
            enc=codec.MAVLink(None,srcSystem=system,srcComponent=1)
            path.write_bytes(original+codec.MAVLink_heartbeat_message(2,3,base,9,4,3).pack(enc))
            report,_=self.result();self.assertEqual(report['checks']['confirmed_land_disarmed']['status'],'FAIL')

    def test_gps_change_and_aiding_dropout_fail_independently(self):
        original=copy.deepcopy(self.records)
        self.records.append(dict(mavpackettype='PARM',TimeUS=7_000_000,Name='GPS_TYPE',Value=1))
        report,_=self.result();self.assertEqual(report['checks']['gps_disabled']['status'],'FAIL')
        self.records=original+[dict(mavpackettype='MSG',TimeUS=9_000_000,Message='EKF3 stopped aiding')]
        report,_=self.result();self.assertEqual(report['checks']['externalnav_fusion']['status'],'FAIL')

    def test_corrupt_bin_is_failure_not_injected_decoder_success(self):
        self.bin.write_bytes(b'not a BIN')
        report,_=self.result();self.assertEqual(report['checks']['raw_dataflash']['status'],'FAIL')

    def test_unbracketed_gt_and_missing_core_never_pass(self):
        path=self.run/'evidence/flight-source/poses.jsonl'
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        self.write(str(path.relative_to(self.run)),[r for r in rows if r['t']>3])
        report,_=self.result();self.assertEqual(report['checks']['trajectory_accuracy']['status'],a.NV)
        self.records=[r for r in self.records if r.get('C')!=1]
        report,_=self.result();self.assertEqual(report['checks']['externalnav_fusion']['status'],a.NV)

    def test_summary_json_markdown_and_manifest_without_demo_artifacts(self):
        report,_=self.result();out=Path(self.tmp.name)/'output';out.mkdir();a.write_report(report,out)
        decoded=json.loads((out/'report.json').read_text())
        self.assertEqual(decoded['status'],a.NV);self.assertIn('SHA256',(out/'REPORT.md').read_text())
        self.assertIn('state/fcu/logs/00000001.BIN',decoded['manifest'])

    def test_ros_clock_reset_and_wrong_fcu_witness_rejected(self):
        path=self.run/'evidence/flight-source/clocks.jsonl';original=path.read_bytes()
        rows=[json.loads(s) for s in original.splitlines()];rows[500]['t']=0
        self.write(str(path.relative_to(self.run)),rows)
        report,_=self.result();self.assertEqual(report['checks']['clock_mapping']['status'],'FAIL')
        path.write_bytes(original)
        path=self.run/'evidence/flight-transport/fcuwitness.jsonl'
        rows=[json.loads(s) for s in path.read_text().splitlines()];rows[0]['fcu_ns']+=10**9
        self.write(str(path.relative_to(self.run)),rows)
        report,_=self.result();self.assertEqual(report['checks']['clock_mapping']['status'],'FAIL')

    def test_false_positive_prevention_for_missing_innovations_and_substituted_position(self):
        original=copy.deepcopy(self.records)
        self.records=[r for r in self.records if r['mavpackettype']!='XKF3']
        report,_=self.result();self.assertEqual(report['checks']['externalnav_fusion']['status'],a.NV)
        self.records=original
        path=self.run/'evidence/flight-transport/wire.jsonl'
        rows=[json.loads(s) for s in path.read_text().splitlines()];rows[10]['message']['x']=42.
        self.write(str(path.relative_to(self.run)),rows)
        report,_=self.result();self.assertEqual(report['checks']['externalnav_fusion']['status'],'FAIL')

    def test_ground_static_gt_never_qualifies_as_takeoff(self):
        path=self.run/'evidence/flight-source/poses.jsonl'
        rows=[json.loads(s) for s in path.read_text().splitlines()]
        for r in rows:r['poses'][0]['p'][2]=.101
        self.write(str(path.relative_to(self.run)),rows)
        report,_=self.result();self.assertEqual(report['checks']['actual_gazebo_flight_motion']['status'],'FAIL')

    def test_wrong_acquisition_units_and_final_deadline_expiry_fail(self):
        path=self.run/'evidence/flight-source/ros_camera.jsonl';original=path.read_bytes()
        rows=[json.loads(s) for s in original.splitlines()]
        for r in rows:r['t']*=1000
        self.write(str(path.relative_to(self.run)),rows)
        report,_=self.result();self.assertEqual(report['checks']['camera_vio_evidence']['status'],'FAIL')
        path.write_bytes(original)
        path=self.run/'evidence/flight-transport/wire.jsonl';rows=[json.loads(s) for s in path.read_text().splitlines()]
        rows[0]['checked_host_ns']+=10**9;self.write(str(path.relative_to(self.run)),rows)
        report,_=self.result();self.assertEqual(report['checks']['externalnav_fusion']['status'],'FAIL')

    def test_imu_dropout_and_covariance_corruption_do_not_pass(self):
        path=self.run/'evidence/flight-source/imu.jsonl'
        rows=[json.loads(s) for s in path.read_text().splitlines()]
        self.write(str(path.relative_to(self.run)),[r for r in rows if not 5<r['t']<5.1])
        report,_=self.result();self.assertEqual(report['checks']['vio_continuity']['status'],'FAIL')
        path=self.run/'evidence/flight-source/stream.jsonl';rows=[json.loads(s) for s in path.read_text().splitlines()]
        rows[20]['covariance'][0][0]=-1;self.write(str(path.relative_to(self.run)),rows)
        report,_=self.result();self.assertEqual(report['checks']['trajectory_accuracy']['status'],'FAIL')

    def test_evidence_change_detected_and_existing_output_not_overwritten(self):
        report,data=self.result()
        (self.run/'evidence/flight-source/summary.json').write_text('{}\n')
        with self.assertRaisesRegex(ValueError,'changed'):data['evidence'].unchanged()
        output=Path(self.tmp.name)/'existing';output.mkdir();(output/'keep').write_text('preserve')
        with patch('sys.argv',['analyze_simulated_flight',str(self.run),'--bin',str(self.bin),'--output',str(output)]):
            with self.assertRaises(FileExistsError):a.main()
        self.assertEqual((output/'keep').read_text(),'preserve')

    def test_cli_missing_evidence_writes_not_verified_report_without_plots(self):
        run=Path(self.tmp.name)/'absent';output=Path(self.tmp.name)/'missing-report'
        with patch('sys.argv',['analyze_simulated_flight',str(run),'--bin',str(run/'state/fcu/logs/missing.BIN'),'--output',str(output)]),patch('sys.stdout',new_callable=io.StringIO):
            self.assertEqual(a.main(),2)
        self.assertEqual(json.loads((output/'report.json').read_text())['status'],a.NV)
        self.assertEqual(list(output.glob('*.png')),[])

    def test_file_decoder_refuses_endpoints_without_importing_pymavlink(self):
        for endpoint in ['tcp:127.0.0.1:5760','udpin:127.0.0.1:14551']:
            with self.assertRaises(ValueError):a.decode_bin(endpoint)

    def test_transient_source_change_is_not_hidden_by_final_profile(self):
        self.records.extend([dict(mavpackettype='PARM',TimeUS=8_000_000,Name='EK3_SRC1_POSXY',Value=3),
                             dict(mavpackettype='PARM',TimeUS=10_000_000,Name='EK3_SRC1_POSXY',Value=6)])
        report,_=self.result();self.assertEqual(report['checks']['externalnav_fusion']['status'],'FAIL')

    def test_late_fcu_receipts_are_not_freshness_pass(self):
        for r in self.records:
            if r['mavpackettype'] in ('VISP','VISV'):r['TimeUS']+=300_000
        report,_=self.result();self.assertEqual(report['checks']['externalnav_fusion']['status'],a.NV)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'),'Matplotlib unavailable; actual plot rendering NOT TESTED')
    def test_plot_construction_without_saving_synthetic_demo_images(self):
        import matplotlib.pyplot  # Initialize pyplot before mocking Figure.savefig.

        report,data=self.result()
        # Exercise plotting arrays/phase decorations only. No fictional PNGs.
        with patch('matplotlib.figure.Figure.savefig') as save:
            names=a.plots(Path(self.tmp.name),report,data)
        self.assertEqual(len(names),4);self.assertEqual(save.call_count,4)

    def test_old_bench_wrapper_matches_extracted_records_api(self):
        root=Path(self.tmp.name)/'bench';(root/'fcu').mkdir(parents=True);(root/'attempt02').mkdir()
        wire=[dict(r,direction='to_fcu') for r in [json.loads(s) for s in (self.run/'evidence/flight-transport/wire.jsonl').read_text().splitlines()]]
        params={r['Name']:r['Value'] for r in self.records if r['mavpackettype']=='PARM'}
        (root/'fcu/decoded.json').write_text(json.dumps(dict(records=self.records,params=params)))
        (root/'attempt02/wire.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in wire));(root/'attempt02/decisions.json').write_text('[]')
        self.assertEqual(analyze(root),analyze_records(self.records,params,wire))


if __name__=='__main__':unittest.main()
