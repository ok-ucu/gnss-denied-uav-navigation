"""Synthetic packets/build/namespace fixtures. No Docker, ROS or FCU connection."""
import copy
import ast
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as S

import simulation_vision as sim
from physics_clock_mapping import AppliedClockMapping
from phase1_transport import fcu_allowed, prepared_firmware_digest
from navigation_lifecycle import NavigationLifecycle
from navigation_egress import VisionEgress
from linux_gpu.verified_disarmed import HeartbeatGuard, load_codec
from analyze_mavros_wire import expected as expected_wire
from test_phase1_pipeline import prepared
from test_vio_frame_bridge import message


def token(ns):
    return dict(session='fixture-producer',epoch=0,stamp_ns=ns,epoch_start_ns=100_000_000,ready=True)


class SimulationVisionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name).resolve()
        self.run=self.root/'phase1_runs/20261116-unit1'
        (self.run/'artifacts').mkdir(parents=True)
        binary=self.run/'artifacts/arducopter';binary.write_bytes(b'synthetic binary fixture, never executed')
        self.digest=sim.sha(binary)
        self.identity=dict(run_id='20261116-unit1',clock_epoch='fixture-epoch',reference_nonce='explicit-fixture',
            firmware_sha256=self.digest,runtime_admitted=False,execution_allowed=False,forwarding_enabled=False)
        (self.run/'artifacts/run_identity.json').write_text(json.dumps(self.identity))
        self.candidate=self.root/'linux_gpu_runs/candidate.json';self.candidate.parent.mkdir()
        self.candidate.write_text('{"synthetic_fixture":true}')
        self.build=dict(firmware_sha256=self.digest,images={k:'sha256:'+str(i)*64 for i,k in enumerate(('physics','fcu','transport'),1)})
        self.process=dict(netns='net:[12345]',boot_id='synthetic-boot',init_starttime='123',hostname='owned-transport')
        self.permit=dict(schema=1,scope=sim.SCOPE,approved=True,
            **{k:self.identity[k] for k in ('run_id','clock_epoch','reference_nonce')},
            firmware_sha256=self.digest,ap_commit=sim.AP_COMMIT,candidate='linux_gpu_runs/candidate.json',
            candidate_sha256=sim.sha(self.candidate),transport_identity=self.process,
            containers={k:str(i)*64 for i,k in enumerate(('physics','fcu','vio','transport'),1)},
            source_hashes={n:sim.sha(sim.ROOT/n) for n in sim.BOUND_SOURCES},
            origin=[-353632620,1491652370,584000],execution_allowed=False,forwarding_enabled=False)
        # Source bytes remain actual; only the external Linux build/identity are fixtures.
        for n in sim.BOUND_SOURCES:
            p=self.root/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((sim.ROOT/n).read_bytes())
        self.patch_root=patch.object(sim,'ROOT',self.root);self.patch_root.start();self.addCleanup(self.patch_root.stop)
        self.patch_candidate=patch.object(sim,'load_candidate',return_value=self.build)
        self.patch_candidate.start();self.addCleanup(self.patch_candidate.stop)
        self.path=self.run/'artifacts/simulation_vision_permit.json'
        self.now=10_850_000_000
        self.codec=load_codec()

    def route(self):
        self.path.write_text(json.dumps(self.permit))
        route=sim.SimulationVisionRoute(self.run,self.path,identity_reader=lambda:self.process,
            heartbeat_factory=lambda system,component,started:HeartbeatGuard(system,component,self.now/1e9-3))
        encoder=self.codec.MAVLink(None,srcSystem=1,srcComponent=1)
        hb=self.codec.MAVLink_heartbeat_message(2,3,0,0,3,3)
        for dt in (-2,-1,0):route.heartbeat.feed(hb.pack(encoder,force_mavlink1=False),self.now/1e9+dt)
        return route

    def packet(self,route,kind='VISION_POSITION_ESTIMATE'):
        mapping=prepared()
        gate=NavigationLifecycle(self.root/'guard.json',mapping);self.addCleanup(gate.close)
        gate.observe_clock(250_000_000,self.now-10_000_000)
        gate.clock_tick(250_000_000,self.now-10_000_000,validation_host_ns=self.now)
        gate.authorize('SIMULATION_DISARMED:fixture',token(200_000_000),'PASS',self.now)
        egress=VisionEgress(self.root/'guard.json');self.addCleanup(egress.close)
        egress.authorize('SIMULATION_DISARMED:fixture','fixture-producer','PASS')
        msg=message();msg.header.stamp.sec=0;msg.header.stamp.nanosec=250_000_000
        pose,speed,meta=gate.pair(msg,token(250_000_000),self.now,True,S,S)
        self.assertTrue(egress.approve(meta))
        p=pose.pose.pose.position;q=pose.pose.pose.orientation;v=speed.twist.twist.linear
        wants={'VISION_POSITION_ESTIMATE':expected_wire(dict(kind='pose',values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w],covariance=pose.pose.covariance)),
               'VISION_SPEED_ESTIMATE':expected_wire(dict(kind='speed',values=[v.x,v.y,v.z],covariance=speed.twist.covariance))}
        enc=self.codec.MAVLink(None,srcSystem=255,srcComponent=191);usec=meta['fcu_ns']//1000
        w=wants[kind]
        if kind=='VISION_POSITION_ESTIMATE':
            packet=self.codec.MAVLink_vision_position_estimate_message(usec,*w['position'],*w['rpy'],w['covariance'],0)
        else:packet=self.codec.MAVLink_vision_speed_estimate_message(usec,*w['speed'],w['covariance'],0)
        packet.pack(enc,force_mavlink1=False)
        # Exercise the actual durable final interlock before simulation dispatch.
        self.assertTrue(egress.check(kind,usec,0,self.now,clock=lambda:self.now))
        return packet,dict(meta=meta,wire=wants),gate

    def test_default_policy_still_has_no_vision_route(self):
        self.assertTrue(fcu_allowed('TIMESYNC'))
        for kind in (*sim.KINDS,'ODOMETRY','COMMAND_LONG','SET_MODE','PARAM_SET'):
            self.assertFalse(fcu_allowed(kind))

    def test_empty_foreign_inventory_never_calls_docker_inspect_without_ids(self):
        source=Path(__file__).resolve().parents[1]/'run_phase1_validation.py'
        tree=ast.parse(source.read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='inventory')
        calls=[]
        def call(command):
            calls.append(command)
            if command[1]!='ps':raise AssertionError('No foreign containers to inspect')
            return S(stdout='owned\n')
        namespace=dict(call=call,NAMES={'fcu':'owned'},json=json)
        exec(compile(ast.Module(body=[function],type_ignores=[]),'inventory-fixture','exec'),namespace)
        self.assertEqual(namespace['inventory'](),[]);self.assertEqual(len(calls),1)

    def test_allowed_simulation_position_route_after_real_durable_permission(self):
        route=self.route();packet,expected,gate=self.packet(route)
        writes=[]
        self.assertTrue(route.forward(packet,expected,gate,self.now,lambda b:writes.append(b) or len(b),clock=lambda:self.now))
        self.assertEqual(len(writes),1);self.assertEqual(route.writes,1)
        self.assertEqual(writes[0],bytes(packet.get_msgbuf()))

    def test_allowed_simulation_velocity_route(self):
        route=self.route();packet,expected,gate=self.packet(route,'VISION_SPEED_ESTIMATE')
        self.assertTrue(route.forward(packet,expected,gate,self.now,len,clock=lambda:self.now))

    def test_stale_permission_never_writes(self):
        route=self.route();packet,expected,gate=self.packet(route);writes=[]
        expected['meta']['allowed_host_deadline_ns']=self.now-1
        with self.assertRaisesRegex(ValueError,'permission expired'):
            route.forward(packet,expected,gate,self.now,writes.append,clock=lambda:self.now)
        self.assertEqual(writes,[]);self.assertTrue(gate.durable.state['blocked'])

    def test_wrong_firmware_hash_and_changed_binary_fail(self):
        route=self.route()
        with self.assertRaises(ValueError):AppliedClockMapping('x','wrong',simulation_route=route)
        with self.assertRaises(ValueError):AppliedClockMapping('x',self.digest)
        mapping=AppliedClockMapping('x',self.digest,simulation_route=route)
        self.assertFalse(mapping.verified)  # eligibility is not native clock PASS
        (self.run/'artifacts/arducopter').write_bytes(b'changed')
        with self.assertRaises(ValueError):prepared_firmware_digest(self.run,self.identity,route)
        with self.assertRaisesRegex(ValueError,'firmware hash'):self.route()

    def test_clock_mismatch_and_loss_of_vio_fail_closed(self):
        for case in ('clock','vio','covariance','namespace','epoch','attitude'):
            with self.subTest(case=case):
                # Independent stores per scenario; never reopen a failed reference.
                old_root=self.root;self.root=old_root/case;self.root.mkdir()
                route=self.route();packet,expected,gate=self.packet(route);writes=[]
                if case=='clock':gate.mapping.broken=True
                if case=='covariance':packet.covariance[0]=-1
                if case=='epoch':packet.reset_counter=1
                if case=='attitude':packet.yaw=float('nan')
                if case=='namespace':route.failure='namespace authority revoked'
                last=self.now-500_000_001 if case=='vio' else self.now
                with self.assertRaises(ValueError):route.forward(packet,expected,gate,last,writes.append,clock=lambda:self.now)
                self.assertEqual(writes,[]);self.assertTrue(gate.durable.state['blocked'])
                self.root=old_root

    def test_lost_or_armed_heartbeat_cannot_authorize_a_send(self):
        route=self.route();packet,expected,gate=self.packet(route);writes=[]
        with self.assertRaisesRegex(ValueError,'heartbeat stale'):
            route.forward(packet,expected,gate,self.now,writes.append,clock=lambda:self.now+3_000_000_001)
        self.assertEqual(writes,[])
        encoder=self.codec.MAVLink(None,srcSystem=1,srcComponent=1)
        armed=self.codec.MAVLink_heartbeat_message(2,3,128,0,3,3)
        guard=HeartbeatGuard(1,1,0)
        with self.assertRaisesRegex(RuntimeError,'ARMED'):
            guard.feed(armed.pack(encoder,force_mavlink1=False),1)

    def test_namespace_permit_and_source_changes_are_rejected(self):
        self.permit['approved']=False
        with self.assertRaises(ValueError):self.route()
        self.permit['approved']=True;self.permit['transport_identity']=dict(self.process,netns='net:[foreign]')
        with self.assertRaisesRegex(ValueError,'namespace'):self.route()
        self.permit['transport_identity']=self.process
        (self.root/'phase1_transport.py').write_text('changed boundary')
        with self.assertRaisesRegex(ValueError,'source changed'):self.route()

    def test_partial_write_is_not_retried_or_reported_success(self):
        route=self.route();packet,expected,gate=self.packet(route)
        with self.assertRaisesRegex(ValueError,'Partial'):
            route.forward(packet,expected,gate,self.now,lambda b:1,clock=lambda:self.now)
        self.assertEqual(route.writes,0);self.assertTrue(gate.durable.state['blocked'])

    def test_actual_socket_completion_after_deadline_is_failed_and_counted(self):
        route=self.route();packet,expected,gate=self.packet(route)
        times=iter([self.now,self.now,expected['meta']['allowed_host_deadline_ns']+1])
        with self.assertRaisesRegex(ValueError,'during socket enqueue'):
            route.forward(packet,expected,gate,self.now,len,clock=lambda:next(times))
        self.assertEqual(route.writes,1)  # do not conceal an already-enqueued packet
        self.assertTrue(gate.durable.state['blocked'])

    def environment(self):
        names={k:'gnss-p1-'+k+'-'+self.identity['run_id'] for k in self.permit['containers']}
        project='gnss-p1-integrated-'+self.identity['run_id']
        rows=[];processes={}
        for k,name in names.items():
            processes[k]=dict(self.process,exe_sha256=self.digest,
                cmdline=['/run/integration/artifacts/arducopter','--model','JSON','--serial0','tcp:5760','--sim-port-out','9002'])
            if k=='physics':processes[k]['cmdline']=['gz','sim','-s','-r','--headless-rendering','/run/integration/models/world.sdf']
            rows.append(dict(Name='/'+name,Id=self.permit['containers'][k],
                Image=self.build['images'][k if k!='vio' else 'physics'],State=dict(Running=True,StartedAt='synthetic'),
                Config=dict(Hostname=self.process['hostname'],Labels={'com.docker.compose.project':'gnss-phase1-integrated-'+self.identity['run_id']}),
                HostConfig=dict(Privileged=False,CapAdd=None,PidMode='',Devices=[],DeviceRequests=[],
                    RestartPolicy=dict(Name='no'),PortBindings={},NetworkMode=project if k=='physics' else 'container:'+self.permit['containers']['physics']),
                NetworkSettings=dict(Ports={},Networks={project:{}}),
                Mounts=[dict(Destination='/opt/integration',Source=str(self.root),RW=False),
                        dict(Destination='/run/integration',Source=str(self.run),RW=True)]))
        return rows,dict(Name=project,Internal=True),processes

    def test_owned_internal_environment_and_explicit_approval(self):
        rows,network,processes=self.environment()
        self.assertEqual(sim.inspect_environment(rows,network,self.run,self.build,processes),self.permit['containers'])
        with self.assertRaisesRegex(ValueError,'Explicit'):sim.prepare_permit(self.run,self.candidate,[0,0,0])

    def test_external_network_devices_mounts_and_wrong_fcu_are_rejected(self):
        for case in ('external','privileged','port','device','hostnet','socket','wrongexe','namespace','image'):
            with self.subTest(case=case):
                rows,net,procs=self.environment()
                if case=='external':net['Internal']=False
                if case=='privileged':rows[1]['HostConfig']['Privileged']=True
                if case=='port':rows[1]['HostConfig']['PortBindings']={'5760/tcp':[{'HostPort':'5760'}]}
                if case=='device':rows[1]['HostConfig']['Devices']=[{'PathOnHost':'/dev/ttyACM0'}]
                if case=='hostnet':rows[1]['HostConfig']['NetworkMode']='host'
                if case=='socket':rows[1]['Mounts'].append(dict(Destination='/run/docker.sock',Source='/run/docker.sock',RW=True))
                if case=='wrongexe':procs['fcu']['exe_sha256']='wrong'
                if case=='namespace':procs['fcu']['netns']='net:[foreign]'
                if case=='image':rows[1]['Image']='sha256:'+'0'*64
                with self.assertRaises(ValueError):sim.inspect_environment(rows,net,self.run,self.build,procs)

    def test_host_permit_inspection_is_bounded_and_replacement_fails(self):
        rows,network,processes=self.environment();calls=[];inspections=[0]
        def execute(cmd,**kwargs):
            self.assertEqual(kwargs['timeout'],10);calls.append(cmd)
            if cmd[1]=='inspect':
                inspections[0]+=1;result=copy.deepcopy(rows)
                if inspections[0]>1:result[1]['Id']='f'*64
                return json.dumps(result)
            if cmd[1]=='network':return json.dumps([network])
            if cmd[1]=='exec':
                kind=next(k for k,v in self.permit['containers'].items() if v==cmd[2])
                return json.dumps(processes[kind])
            raise AssertionError(cmd)
        with self.assertRaisesRegex(ValueError,'replaced/restarted'):
            sim.prepare_permit(self.run,self.candidate,[0,0,0],approved=True,execute=execute)
        self.assertFalse(self.path.exists())
        self.assertTrue(all(cmd[1] in ('inspect','network','exec') for cmd in calls))

    def test_successful_fixture_permit_is_private_bound_and_cannot_be_overwritten(self):
        rows,network,processes=self.environment()
        def execute(cmd,**kwargs):
            if cmd[1]=='inspect':return json.dumps(rows)
            if cmd[1]=='network':return json.dumps([network])
            if cmd[1]=='exec':
                kind=next(k for k,v in self.permit['containers'].items() if v==cmd[2])
                return json.dumps(processes[kind])
            raise AssertionError(cmd)
        path=sim.prepare_permit(self.run,self.candidate,[0,0,0],approved=True,execute=execute)
        p=json.loads(path.read_text())
        self.assertEqual(p['containers'],self.permit['containers'])
        self.assertEqual(path.stat().st_mode & 0o777,0o600)
        self.assertIs(p['execution_allowed'],False);self.assertIs(p['forwarding_enabled'],False)
        self.assertIs(p['native_verified'],False)
        original=path.read_bytes()
        with self.assertRaises(FileExistsError):
            sim.prepare_permit(self.run,self.candidate,[0,0,0],approved=True,execute=execute)
        self.assertEqual(path.read_bytes(),original)


if __name__=='__main__':unittest.main()
