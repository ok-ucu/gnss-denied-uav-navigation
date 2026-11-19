"""Synthetic policy/evidence fixtures ONLY. Never send a command or open an FCU."""
import ast
import copy
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import simulation_flight as f
import run_simulated_flight as runner
import phase1_transport
from linux_gpu.verified_disarmed import load_codec,HeartbeatGuard


class FlightPolicyTests(unittest.TestCase):
    def setUp(self):
        self.w=load_codec();self.commands=[];self.now=0
        self.origin=[-353632620,1491652370,584000]
        self.c=f.FlightController(self.w,0,lambda cmd,args:self.commands.append((cmd,args)),global_origin=self.origin)
        self.params=dict(f.EXPECTED,ARMING_CHECK=1)

    def message(self,kind,**fields):
        return S(get_type=lambda:kind,get_srcSystem=lambda:1,get_srcComponent=lambda:1,**fields)

    def feed(self,t,*,armed=False,mode=4,alt=0.,landed=1,vision=True,mutate=None):
        self.now=round(t*1e9)
        messages={
            'HEARTBEAT':dict(base_mode=128 if armed else 0,custom_mode=mode),
            'EKF_STATUS_REPORT':dict(flags=43,velocity_variance=.1,pos_horiz_variance=.1,pos_vert_variance=.1,compass_variance=.1),
            'GLOBAL_POSITION_INT':dict(relative_alt=round(alt*1000)),
            'LOCAL_POSITION_NED':dict(x=0.,y=0.,z=-alt,vx=0.,vy=0.,vz=0.),
            'HOME_POSITION':dict(latitude=self.origin[0],longitude=self.origin[1],altitude=self.origin[2]),
            'SYS_STATUS':dict(onboard_control_sensors_present=268435456,onboard_control_sensors_enabled=268435456,onboard_control_sensors_health=268435456),
            'EXTENDED_SYS_STATE':dict(landed_state=landed)}
        if mutate:mutate(messages)
        for k,v in messages.items():self.c.observe(self.message(k,**v),self.now)
        self.c.tick(self.now,self.params,True,vision,parameters_fresh=True)

    def ack(self,cmd,result=0):
        self.c.observe(self.message('COMMAND_ACK',command=cmd,result=result,
                                    target_system=255,target_component=191),self.now+1)

    def preflight(self):
        self.feed(0);self.ack(401)
        self.c.tick(2,self.params,True,True,parameters_fresh=True)
        self.now=2;self.ack(179)
        self.c.tick(4,self.params,True,True,parameters_fresh=True)
        for i in range(1,51):self.feed(i*.1)
        self.c.tick(5_000_000_004,self.params,True,True,parameters_fresh=True)
        self.now=5_000_000_004
        self.assertEqual(self.c.state,'GUIDED')

    def climb(self):
        self.preflight();self.ack(176);self.feed(5.1)
        self.assertEqual(self.c.state,'ARM');self.ack(400);self.feed(5.2,armed=True)
        self.assertEqual(self.c.state,'CLIMB');self.ack(22)
        for i in range(53,63):self.feed(i*.1,armed=True,alt=1.,landed=2)
        self.assertEqual(self.c.state,'HOVER')

    def test_takeoff_hover_land_requires_ack_and_confirmed_disarmed(self):
        self.climb()
        for i in range(63,104):self.feed(i*.1,armed=True,alt=1.,landed=2)
        self.assertEqual(self.c.state,'LAND');self.ack(21)
        self.feed(10.4,armed=True,mode=9,alt=.3,landed=2)
        self.assertFalse(self.c.done)
        self.feed(10.5,mode=9,alt=0.,landed=1)
        self.assertTrue(self.c.done);self.assertEqual(self.c.result,'PASS')
        self.assertEqual([cmd for cmd,_ in self.commands],[401,179,176,400,22,21])
        self.assertEqual(next(args for cmd,args in self.commands if cmd==400),[1,0,0.,0.,0.,0.,0.])

    def test_actual_transport_logger_preserves_flight_event_timestamp(self):
        # Compile ONLY the real nested logger, not main(): no ROS, sockets or
        # authorization. Exercise its actual FlightController callback path.
        source=Path(phase1_transport.__file__)
        main=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
        logger=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='log')
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'flight.jsonl'
            with path.open('x') as output:
                audit=phase1_transport.AsyncAudit({'flight':output})
                namespace=dict(audit_writer=audit,time=S(monotonic_ns=lambda:6_000_000_000))
                exec(compile(ast.Module(body=[logger],type_ignores=[]),str(source),'exec'),namespace)
                self.c.record=lambda **row:namespace['log']('flight',**row)
                try:
                    self.preflight()  # prearm_snapshot, then transition GUIDED
                    namespace['log']('flight',event='simulation_command_sent',command=176,params=[1,4])
                finally:audit.close()
            rows=[json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([r['event'] for r in rows],['prearm_snapshot','flight_transition','simulation_command_sent'])
        self.assertEqual([r['host_ns'] for r in rows],[5_000_000_004,5_000_000_004,6_000_000_000])
        self.assertEqual(rows[0]['parameters'],self.params)
        self.assertEqual(rows[0]['home'],self.origin)
        self.assertEqual((rows[1]['previous'],rows[1]['state']),('PREFLIGHT','GUIDED'))
        self.assertEqual((rows[2]['command'],rows[2]['params']),(176,[1,4]))

    def test_stale_live_witness_blocks_arm_at_socket(self):
        source = Path(phase1_transport.__file__)
        sender = next(node for node in ast.walk(ast.parse(source.read_text()))
                      if isinstance(node, ast.FunctionDef) and node.name == 'flight_command')
        params = [1,0,0,0,0,0,0]
        ns = dict(allowed={400:params}, wire=self.w, enc=self.w.MAVLink(None),
                  fresh_fusion_witness=lambda **kwargs: False,
                  tcp=S(send=lambda _: self.fail('ARM reached the fake socket')))
        exec(compile(ast.Module(body=[sender],type_ignores=[]),str(source),'exec'),ns)
        with self.assertRaisesRegex(ValueError,'Fresh current DataFlash'):
            ns['flight_command'](400,params)

    def test_ack_must_follow_matching_request_and_cannot_overlap_or_repeat(self):
        self.ack(401)
        self.feed(0)
        self.assertEqual([cmd for cmd,_ in self.commands], [401])
        self.c.observe(self.message('COMMAND_ACK',command=401,result=0,
                                    target_system=255,target_component=191),0)
        self.c.observe(self.message('COMMAND_ACK',command=401,result=0,
                                    target_system=1,target_component=1),1)
        self.feed(.1)
        self.assertNotIn(401,self.c.acks)
        with self.assertRaisesRegex(ValueError,'awaiting'):
            self.c.command(179,[1],now=self.now)
        self.ack(401)
        self.feed(.2)
        self.assertEqual([cmd for cmd,_ in self.commands], [401,179])
        with self.assertRaisesRegex(ValueError,'Duplicate'):
            self.c.command(401,now=self.now)

    def test_missing_vision_or_prearm_checks_never_arm(self):
        for i in range(451):self.feed(i*.1,vision=False)
        self.assertEqual(self.c.state,'STOP');self.assertNotIn(400,[cmd for cmd,_ in self.commands])

    def test_prearm_flag_and_gps_profile_are_not_bypassed(self):
        for i in range(451):self.feed(i*.1,mutate=lambda m:m['SYS_STATUS'].update(onboard_control_sensors_health=0))
        self.assertNotIn(400,[cmd for cmd,_ in self.commands]);self.assertEqual(self.c.result,'FAIL')

    def test_wrong_gps_or_arming_mask_refuses_guided(self):
        self.params['GPS_TYPE']=1
        for i in range(451):self.feed(i*.1)
        self.assertEqual(self.commands,[])

    def test_home_reference_and_innovations_refuse_arm(self):
        for i in range(451):self.feed(i*.1,mutate=lambda m:m['EKF_STATUS_REPORT'].update(pos_horiz_variance=1.01))
        self.assertEqual(self.commands,[])

    def test_prearm_rechecked_immediately_before_arm(self):
        self.preflight();self.ack(176)
        self.feed(5.1,mutate=lambda m:m['SYS_STATUS'].update(onboard_control_sensors_health=0))
        self.assertNotIn(400,[cmd for cmd,_ in self.commands]);self.assertEqual(self.c.state,'STOP')

    def test_parameters_must_be_reread_after_guided_before_arm(self):
        self.preflight();self.ack(176)
        self.c.tick(5_100_000_000,self.params,True,True,parameters_fresh=False)
        self.assertEqual(self.c.state,'GUIDED');self.assertNotIn(400,[cmd for cmd,_ in self.commands])
        self.feed(5.2);self.assertEqual(self.c.state,'ARM')

    def test_wrong_home_and_unexpected_arm_do_not_initiate_takeoff(self):
        for i in range(51):self.feed(i*.1,mutate=lambda m:m['HOME_POSITION'].update(latitude=self.origin[0]+1000))
        self.assertNotIn(400,[cmd for cmd,_ in self.commands])
        self.feed(5.1,armed=True)
        self.assertEqual(self.c.state,'EMERGENCY_LAND');self.assertEqual(self.commands[-1][0],21)
        self.assertNotIn(22,[cmd for cmd,_ in self.commands])

    def test_land_ack_rejection_and_missing_disarm_never_pass(self):
        self.climb();self.c.emergency('unit VIO loss',self.now);self.ack(21,2)
        self.assertEqual(self.c.result,'FAIL');self.assertEqual(self.c.state,'STOP')

    def test_vio_loss_lands_without_disarm_or_recovery(self):
        self.climb();deadline=self.c.deadline
        self.feed(6.3,armed=True,alt=1.,landed=2,vision=False)
        self.assertEqual(self.c.state,'EMERGENCY_LAND');self.assertEqual(self.commands[-1][0],21)
        self.ack(21)
        self.feed(6.4,mode=9,landed=1)
        self.assertEqual(self.c.result,'FAIL');self.assertEqual(self.c.deadline,deadline)
        self.assertEqual(sum(cmd==400 for cmd,_ in self.commands),1)

    def test_altitude_limit_and_land_rejection_stop_the_simulation(self):
        self.climb();self.feed(6.3,armed=True,alt=1.6,landed=2)
        self.assertEqual(self.c.state,'EMERGENCY_LAND')
        self.feed(6.4,armed=True,mode=9,alt=1.6,landed=2)
        self.assertEqual(self.c.state,'STOP');self.assertEqual(self.c.result,'FAIL')

    def test_arm_denied_does_not_trigger_takeoff(self):
        self.preflight();self.ack(176);self.feed(5.1);self.ack(400,2)
        self.assertNotIn(22,[cmd for cmd,_ in self.commands]);self.assertEqual(self.c.state,'EMERGENCY_LAND')

    def test_takeoff_and_land_timeouts_do_not_extend_global_deadline(self):
        self.climb();self.c.emergency('injected unit failure',self.now)
        original=self.c.land_start;deadline=self.c.deadline
        self.c.emergency('second failure',self.now+100)
        self.assertEqual(self.c.land_start,original)
        self.c.tick(deadline,self.params,True,False)
        self.assertEqual(self.c.state,'STOP');self.assertEqual(self.c.deadline,deadline)

    def test_controller_descheduling_is_not_counted_as_stable_preflight(self):
        self.feed(0);self.feed(5)
        self.assertNotIn(176,[cmd for cmd,_ in self.commands])

    def test_disarmed_guard_still_rejects_arm_flight_guard_checks_source(self):
        enc=self.w.MAVLink(None,srcSystem=1,srcComponent=1)
        data=self.w.MAVLink_heartbeat_message(2,3,128,0,3,3).pack(enc,force_mavlink1=False)
        with self.assertRaisesRegex(RuntimeError,'ARMED'):HeartbeatGuard(1,1,0).feed(data,1)
        guard=f.FlightHeartbeatGuard(1,1,0);guard.feed(data,1)
        self.assertEqual(guard.count,1)
        enc=self.w.MAVLink(None,srcSystem=2,srcComponent=1)
        bad=self.w.MAVLink_heartbeat_message(2,3,0,0,3,3).pack(enc,force_mavlink1=False)
        with self.assertRaises(RuntimeError):guard.feed(bad,2)


class FusionProofTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.root=Path(tmp.name).resolve();self.run=self.root/'phase1_runs/fixture'
        (self.root/'run_simulated_flight.py').write_bytes(Path(runner.__file__).read_bytes())
        self.root_patch=patch.object(f,'ROOT',self.root);self.root_patch.start();self.addCleanup(self.root_patch.stop)
        self.records=[dict(mavpackettype='PARM',Name=k,Value=v,TimeUS=0) for k,v in dict(f.EXPECTED,ARMING_CHECK=1).items()]
        self.records.extend([dict(mavpackettype='MSG',TimeUS=1_000_000,Message='EKF3 IMU0 is using external nav data'),
            dict(mavpackettype='XKFS',TimeUS=1_010_000,SS=0)])
        pubs=[];obs=[];stream=[];packets=[]
        for i in range(51):
            t=1+i*.1;ns=round(t*1e9);usec=ns//1000
            pubs.append(dict(t=t,publish_monotonic_s=100+t))
            obs.append(dict(sec=ns//10**9,nanosec=ns%10**9,valid=True,arrival_monotonic_s=100+t+.01))
            stream.append(dict(t=t,published=True,ready=True,epoch=0,publication_clock_check=dict(valid=True,simulation_age_s=.02)))
            self.records.append(dict(mavpackettype='XKF4',TimeUS=usec+10000,FS=0,TS=48,SV=.1,SP=.1))
            for kind,df in [('VISION_POSITION_ESTIMATE','VISP'),('VISION_SPEED_ESTIMATE','VISV')]:
                packets.append(dict(valid_capture=True,forwarded_to_fcu=True,message=dict(mavpackettype=kind,usec=usec)))
                self.records.append(dict(mavpackettype=df,TimeUS=usec+10000,RTimeUS=usec,Ign=0,Rst=0))
        self.write('artifacts/run_identity.json',{})
        self.write('artifacts/simulation_vision_permit.json',dict(firmware_sha256='fixture-pin',source_hashes={'test':'fixture'}))
        p=self.run/'artifacts/arducopter';p.write_bytes(b'UNIT FIXTURE ONLY, NEVER EXECUTED')
        self.write('evidence/vio/configuration.json',dict(horizontal_gt_input=False,faults=[],fault_file=None,
            world_gt_attitude_used_for_camera_depth=False,source_sha256=f.VIO_FILES,vertical_aid='Gazebo GT Z only'))
        self.write('evidence/vio/summary.json',dict(gaps=0,recoveries=0))
        for name,rows in [('publications',pubs),('stream',stream),('ros_camera',[dict(t=r['t']) for r in pubs]),
                           ('imu',[dict(t=1+i*.001) for i in range(5001)])]:self.write('evidence/vio/'+name+'.jsonl',rows,True)
        self.write('evidence/observer/input.jsonl',obs,True)
        self.write('evidence/transport_1/wire.jsonl',packets,True)
        self.write('evidence/transport_1/summary.json',dict(clock_mapping_verified=True,clock_latched=False))
        codec=load_codec();enc=codec.MAVLink(None,srcSystem=1,srcComponent=1)
        hb=codec.MAVLink_heartbeat_message(2,3,0,0,3,3)
        (self.run/'evidence/transport_1/fcu_tcp.bin').write_bytes(b''.join(hb.pack(enc,force_mavlink1=False) for _ in range(3)))
        self.write('evidence/transport_1/simulation_result.json',dict(failure=None,origin_confirmed=True,complete_forwarded_pairs=51))
        self.bin=self.run/'state/fcu/logs/00000001.BIN';self.bin.parent.mkdir(parents=True);self.bin.write_bytes(b'NOT REAL DATAFLASH: OFFLINE FIXTURE')
        self.proof=self.run/'evidence/flight_fusion_proof.json'

    def write(self,name,data,lines=False):
        p=self.run/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(''.join(json.dumps(x)+'\n' for x in data) if lines else json.dumps(data)+'\n')

    def build(self):
        # Dependency injection is explicitly synthetic, not native parser PASS.
        with patch.object(f,'decode_dataflash',return_value=self.records):f.build_proof(self.run,self.bin,self.proof)

    def test_missing_fusion_proof_refuses_before_any_docker_or_socket(self):
        args=['run_simulated_flight.py','--run-id','20261117-unit1','--runtime-candidate','missing-candidate',
              '--vio-dir','missing-vio','--fusion-proof',str(self.proof),'--origin=-35,149,584','--execute-simulation-flight']
        with patch('sys.argv',args),patch.object(runner.subprocess,'run',side_effect=AssertionError('Docker forbidden')):
            with self.assertRaises(FileNotFoundError):runner.main()

    def test_fixture_analysis_and_host_redecode_are_required(self):
        self.build()
        with patch.object(f,'decode_dataflash',return_value=self.records) as decode:
            f.verify_proof(self.proof);decode.assert_called_once_with(self.bin)
        with patch.object(f,'decode_dataflash',return_value=[]):
            with self.assertRaisesRegex(ValueError,'decoder output'):f.verify_proof(self.proof)

    def test_synthetic_or_self_referential_source_is_not_live_camera_proof(self):
        config=f.read_json(self.run/'evidence/vio/configuration.json');config['horizontal_gt_input']=True
        self.write('evidence/vio/configuration.json',config)
        with self.assertRaises(ValueError):f.evidence_analysis(self.run,self.records)

    def test_backend_receipt_alone_is_not_fusion(self):
        rows=[r for r in self.records if r['mavpackettype']!='MSG']
        with self.assertRaisesRegex(ValueError,'aiding'):f.evidence_analysis(self.run,rows)

    def test_armed_or_missing_baseline_heartbeat_is_not_disarmed_proof(self):
        path=self.run/'evidence/transport_1/fcu_tcp.bin';path.write_bytes(b'BAD_DATA')
        with self.assertRaisesRegex(ValueError,'heartbeat'):f.evidence_analysis(self.run,self.records)
        codec=load_codec();enc=codec.MAVLink(None,srcSystem=1,srcComponent=1)
        armed=codec.MAVLink_heartbeat_message(2,3,128,0,3,3).pack(enc,force_mavlink1=False)
        path.write_bytes(armed*3)
        with self.assertRaisesRegex(ValueError,'heartbeat'):f.evidence_analysis(self.run,self.records)

    def test_innovation_rejection_wrong_source_and_timestamp_mismatch_fail(self):
        for case in ('ratio','source','timestamp','arming'):
            rows=copy.deepcopy(self.records)
            if case=='ratio':next(r for r in rows if r['mavpackettype']=='XKF4')['SP']=1.1
            if case=='source':next(r for r in rows if r['mavpackettype']=='XKFS')['SS']=1
            if case=='timestamp':next(r for r in rows if r['mavpackettype']=='VISP')['RTimeUS']+=1
            if case=='arming':next(r for r in rows if r.get('Name')=='ARMING_CHECK')['Value']=0
            with self.subTest(case=case),self.assertRaises(ValueError):f.evidence_analysis(self.run,rows)

    def test_imu_gap_independent_delivery_and_stale_publication_are_rejected(self):
        for path,rows in [('evidence/vio/imu.jsonl',[dict(t=1),dict(t=1.03)]),
                          ('evidence/observer/input.jsonl',[dict(sec=1,nanosec=0,valid=False)])]:
            previous=(self.run/path).read_text();self.write(path,rows,True)
            with self.assertRaises(ValueError):f.evidence_analysis(self.run,self.records)
            (self.run/path).write_text(previous)

    def test_expected_post_stop_dropout_is_labelled_not_hidden(self):
        rows=self.records+[dict(mavpackettype='MSG',TimeUS=7_000_000,Message='stopped aiding')]
        self.assertTrue(f.evidence_analysis(self.run,rows)['post_stream_dropout'])
        rows[-1]['TimeUS']=4_000_000
        with self.assertRaises(ValueError):f.evidence_analysis(self.run,rows)

    def test_same_size_corruption_cannot_reuse_approved_proof(self):
        self.build();data=self.bin.read_bytes();self.bin.write_bytes(b'X'+data[1:])
        with self.assertRaisesRegex(ValueError,'evidence changed'):f.verify_proof(self.proof,redecode=False)

    def test_plan_cannot_change_firmware_limits_or_approval(self):
        self.build()
        route=S(permit=dict(run_id='new-flight',reference_nonce='new-ref',source_hashes={'test':'fixture'}),firmware_sha256='fixture-pin')
        plan=dict(schema=1,scope=f.SCOPE,approved=True,execution_allowed=False,forwarding_enabled=False,
            limits=f.LIMITS,run_id='new-flight',reference_nonce='new-ref',source_sha256=f.sha(Path(f.__file__)),
            host_runner_sha256=f.sha(Path(runner.__file__)),
            fusion_proof=str(self.proof.relative_to(self.root)),fusion_proof_sha256=f.sha(self.proof))
        path=self.root/'plan.json';path.write_text(json.dumps(plan))
        self.assertEqual(f.load_plan(path,route)['run_id'],'new-flight')
        for key,value in [('approved',False),('limits',dict(f.LIMITS,max_alt_m=100)),('reference_nonce','old')]:
            path.write_text(json.dumps(dict(plan,**{key:value})))
            with self.subTest(key=key),self.assertRaises(ValueError):f.load_plan(path,route)
        path.write_text(json.dumps(plan));route.firmware_sha256='wrong'
        with self.assertRaisesRegex(ValueError,'does not match'):f.load_plan(path,route)

    def test_focused_decoder_preserves_selected_records_and_exposes_truncated_tail(self):
        import struct

        def fmt(kind, name, layout, columns, size):
            return b'\xa3\x95\x80' + f.DATAFLASH_FORMAT.pack(kind,size,name,layout,columns)

        msg_format = fmt(42,b'MSG',b'QZ',b'TimeUS,Message',75)
        att_format = fmt(43,b'ATT',b'Qf',b'TimeUS,Roll',15)
        message = b'\xa3\x95\x2a' + struct.pack('<Q64s',1_000_000,b'ExternalNav fixture')
        unrelated = b'\xa3\x95\x2b' + struct.pack('<Qf',1_000_000,0.5)
        full = msg_format+att_format+message+unrelated+message
        path = self.root/'snapshot.BIN'
        path.write_bytes(full)
        expected = f.decode_dataflash(path)
        focused, tail = f.decode_live_dataflash(path)
        self.assertEqual(tail,0)
        self.assertEqual(focused,expected)
        path.write_bytes(full + message[:11])
        focused, tail = f.decode_live_dataflash(path)
        self.assertEqual(tail,11)
        self.assertEqual(focused,expected)
        for data in (full + b'bad', msg_format+att_format+b'\x00'+message,
                     full+fmt(42,b'MSG',b'Qf',b'TimeUS,Message',15),
                     full + b'\xa3\x95\xff'):
            with self.subTest(data=data[-12:]), self.assertRaises(ValueError):
                f.select_witness_records(data)

    def test_current_witness_must_still_be_fresh_after_slow_decode(self):
        permit=dict(run_id='current',reference_nonce='ref',clock_epoch='clock',
                    firmware_sha256='pin',containers={'fcu':'1'*64})
        fcu_at_copy=6_050_000_000
        fcu_after_decode=fcu_at_copy+500_000_000
        f.current_fusion_attestation(self.records,permit,self.bin,fcu_at_copy)
        with self.assertRaisesRegex(ValueError,'Buffered DataFlash'):
            f.current_fusion_attestation(self.records,permit,self.bin,fcu_after_decode)

    def test_current_boot_source_selection_requires_fresh_raw_fcu_evidence(self):
        permit=dict(run_id='current',reference_nonce='ref',clock_epoch='clock',firmware_sha256='pin',containers={'fcu':'1'*64})
        now=10_000_000_000
        with patch.object(f.time,'monotonic_ns',return_value=now):
            row=f.current_fusion_attestation(self.records,permit,self.bin,6_100_000_000)
        self.assertTrue(f.verify_current_attestation(row,permit,now,6_100_000_000))
        with self.assertRaises(ValueError):f.verify_current_attestation(row,permit,now+500_000_001,6_100_000_000)
        with self.assertRaises(ValueError):f.verify_current_attestation(row,dict(permit,clock_epoch='old'),now,6_100_000_000)
        with self.assertRaises(ValueError):f.current_fusion_attestation(self.records,permit,self.bin,9_000_000_000)

    def test_explicit_simulation_subnet_requires_aligned_private_ipv4_28(self):
        from launch_phase1 import private_simulation_subnet
        self.assertEqual(private_simulation_subnet('10.249.241.0/28'),'10.249.241.0/28')
        for value in ('10.249.241.1/28','10.0.0.0/8','8.8.8.0/28','fd00::/64'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                private_simulation_subnet(value)

    def test_launcher_reserves_same_run_id_once_under_contention(self):
        from concurrent.futures import ThreadPoolExecutor
        from threading import Barrier
        source = Path(runner.__file__).with_name('launch_phase1.py')
        statements = [node for node in ast.walk(ast.parse(source.read_text()))
                      if isinstance(node, ast.Expr) and ast.unparse(node.value)
                      == 'run.mkdir(parents=True, exist_ok=False)']
        claim = compile(ast.Module(body=statements,type_ignores=[]),str(source),'exec')
        self.assertEqual(len(statements),1)
        path = self.root/'reserved'
        start = Barrier(2)

        def reserve():
            start.wait(timeout=1)
            try:
                exec(claim,dict(run=path))
            except FileExistsError:
                return False
            return True

        with ThreadPoolExecutor(2) as workers:
            attempts = list(workers.map(lambda _: reserve(),range(2)))
        self.assertEqual(sorted(attempts), [False,True])

    def test_transport_lock_excludes_second_controller_for_same_run(self):
        with tempfile.TemporaryDirectory() as temp:
            first = Path(temp)/'first'
            other = Path(temp)/'other'
            (first/'state').mkdir(parents=True)
            (other/'state').mkdir(parents=True)
            with phase1_transport.acquire_transport_lock(first):
                with self.assertRaises(BlockingIOError):
                    phase1_transport.acquire_transport_lock(first)
                with phase1_transport.acquire_transport_lock(other):
                    pass
            with phase1_transport.acquire_transport_lock(first):
                pass

    def test_cleanup_never_accepts_foreign_or_replaced_container(self):
        name = 'gnss-p1-physics-20261119-unit'
        project = 'gnss-phase1-integrated-20261119-unit'
        network = 'gnss-p1-integrated-20261119-unit'
        row = dict(Id='a'*64, Name='/'+name,
                   Config={'Labels': {'com.docker.compose.project': project}},
                   Mounts=[dict(Destination='/run/integration',Source=str(self.run.resolve()))])
        self.assertEqual(runner.cleanup_container_id(row,name,project,self.run),row['Id'])
        with self.assertRaises(ValueError):
            runner.cleanup_container_id(row,name,project,self.run,'b'*64)
        with self.assertRaises(ValueError):
            runner.cleanup_container_id(row,name,project,self.root)
        with self.assertRaises(ValueError):
            runner.cleanup_container_id(row,name,network,self.run)

    def test_transport_flight_flag_alone_cannot_open_fcu(self):
        with patch('sys.argv',['phase1_transport.py','--run','unused','--simulation-flight-plan','unused']), \
             patch.object(phase1_transport.socket,'create_connection',side_effect=AssertionError('No socket')), \
             patch('sys.stderr',io.StringIO()):
            with self.assertRaises(SystemExit):phase1_transport.main()


if __name__=='__main__':unittest.main()
