"""Offline fusion admission + bounded simulation-flight controller. No sockets.

Host decoding is an operator-reviewed attestation, not cryptographic evidence.
Transport rechecks its hashes and analysis, and independently checks live health.
The only sender is phase1_transport's existing inspected SITL TCP boundary.
"""
import argparse
import json
import math
from pathlib import Path
import time

from runtime_candidate import ROOT, sha
from preflight_ekf3 import EXPECTED
from linux_gpu.prepare_vio import FILES as VIO_FILES
from linux_gpu.verified_disarmed import HeartbeatGuard,load_codec

SCOPE='gazebo-sitl-takeoff-hover-land'
LIMITS=dict(target_alt_m=1.0,max_alt_m=1.5,max_xy_m=0.4,hover_s=5.0,
            total_s=105.0,preflight_s=45.0,command_s=5.0,climb_s=15.0,land_s=30.0)
PROOF_INPUTS=('artifacts/run_identity.json','artifacts/simulation_vision_permit.json','artifacts/arducopter',
              'evidence/vio/configuration.json','evidence/vio/summary.json',
              'evidence/vio/stream.jsonl','evidence/vio/ros_camera.jsonl','evidence/vio/imu.jsonl',
              'evidence/vio/publications.jsonl','evidence/observer/input.jsonl',
              'evidence/transport_1/wire.jsonl','evidence/transport_1/summary.json',
              'evidence/transport_1/fcu_tcp.bin',
              'evidence/transport_1/simulation_result.json')


def read_json(path):
    def bad(value):raise ValueError('Nonfinite JSON: '+value)
    return json.loads(Path(path).read_text(),parse_constant=bad)


def jsonl(path):
    text=Path(path).read_text()
    if not text or not text.endswith('\n'):raise ValueError('Incomplete/empty evidence: '+str(path))
    return [json.loads(s,parse_constant=lambda s:(_ for _ in ()).throw(ValueError(s))) for s in text.splitlines()]


def evidence_analysis(run,records):
    """Closed, real-camera DISARMED baseline; no static synthetic substitute."""
    config=read_json(run/'evidence/vio/configuration.json')
    summary=read_json(run/'evidence/vio/summary.json')
    received=read_json(run/'evidence/transport_1/simulation_result.json')
    transport=read_json(run/'evidence/transport_1/summary.json')
    codec=load_codec();parser=codec.MAVLink(None);parser.robust_parsing=True
    heartbeats=[m for m in parser.parse_buffer((run/'evidence/transport_1/fcu_tcp.bin').read_bytes()) or [] if m.get_type()=='HEARTBEAT']
    if len(heartbeats)<3 or any((m.get_srcSystem(),m.get_srcComponent())!=(1,1) or m.get_msgbuf()[0]!=253 or
            m.mavlink_version!=3 or m.autopilot!=3 or m.type!=2 or m.base_mode&128 for m in heartbeats):
        raise ValueError('DISARMED expected-source baseline heartbeat proof missing')
    if (config.get('horizontal_gt_input') is not False or config.get('faults') or config.get('fault_file') or
            config.get('world_gt_attitude_used_for_camera_depth') is not False or
            any(config.get('source_sha256',{}).get(n)!=h for n,h in VIO_FILES.items()) or
            summary.get('gaps')!=0 or summary.get('recoveries')!=0 or received.get('failure') or
            transport.get('clock_mapping_verified') is not True or transport.get('clock_latched') is not False or
            received.get('origin_confirmed') is not True or not received.get('complete_forwarded_pairs')):
        raise ValueError('Missing clean, pinned live-camera fusion baseline')
    pubs=jsonl(run/'evidence/vio/publications.jsonl');observed=jsonl(run/'evidence/observer/input.jsonl')
    stamps=[round(r['t']*1e9) for r in pubs]
    if (len(stamps)<2 or any(b<=a for a,b in zip(stamps,stamps[1:])) or
            stamps!=[r['sec']*10**9+r['nanosec'] for r in observed] or not all(r['valid'] for r in observed)):
        raise ValueError('Independent camera-VIO delivery proof missing/invalid')
    for rows,key in ((pubs,'publish_monotonic_s'),(observed,'arrival_monotonic_s')):
        times=[r[key] for r in rows]
        if any(not math.isfinite(v) for v in times) or any(not 0<b-a<=.5 for a,b in zip(times,times[1:])):
            raise ValueError('Baseline source/subscriber continuity failed')
    camera={round(r['t']*1e9) for r in jsonl(run/'evidence/vio/ros_camera.jsonl')}
    emitted={round(r['t']*1e9):r for r in jsonl(run/'evidence/vio/stream.jsonl') if r.get('published')}
    if set(stamps)-camera or set(stamps)!=set(emitted):raise ValueError('No matching live camera acquisitions')
    for r in emitted.values():
        check=r.get('publication_clock_check',{})
        if (r.get('ready') is not True or check.get('valid') is not True or
                not 0<=check.get('simulation_age_s',math.inf)<=.18 or r.get('epoch')!=0):
            raise ValueError('Invalid/stale/reset source publication')
    imu=[r['t'] for r in jsonl(run/'evidence/vio/imu.jsonl')]
    if len(imu)<2 or any(not math.isfinite(t) for t in imu) or any(not 0<b-a<=.02+1e-12 for a,b in zip(imu,imu[1:])):
        raise ValueError('Baseline IMU gap/reset')
    packets=[r for r in jsonl(run/'evidence/transport_1/wire.jsonl') if r.get('forwarded_to_fcu')]
    params={r['Name']:r['Value'] for r in records if r['mavpackettype']=='PARM'}
    if any(params.get(k)!=v for k,v in EXPECTED.items()) or params.get('ARMING_CHECK')!=1:
        raise ValueError('DataFlash ExternalNav/GPS/arming parameters not proven')
    messages=[r for r in records if r['mavpackettype']=='MSG']
    received_rows=[r for r in records if r['mavpackettype'] in ('VISP','VISV')]
    if not received_rows:raise ValueError('No FCU vision reception records')
    lo=min(r['TimeUS'] for r in received_rows);hi=max(r['TimeUS'] for r in received_rows)
    aiding=[r['TimeUS'] for r in messages if r['TimeUS']<=hi and 'is using external nav data' in r['Message']]
    stopped=[r['TimeUS'] for r in messages if 'stopped aiding' in r['Message']]
    if not aiding or any(lo<=t<=hi and t>min(aiding) for t in stopped):
        raise ValueError('No sustained DataFlash ExternalNav aiding proof')
    sources=[r for r in records if r['mavpackettype']=='XKFS' and max(lo,max(aiding))<=r['TimeUS']<=hi]
    if not sources or any(r.get('SS')!=0 for r in sources):
        raise ValueError('Missing active EKF3 source-set proof')
    healthy=[r for r in records if r['mavpackettype']=='XKF4' and max(lo,max(aiding))<=r['TimeUS']<=hi]
    if hi-max(lo,max(aiding))<5_000_000:raise ValueError('Need at least 5s of live aided baseline')
    if not healthy or any(r['FS']!=0 or int(r['TS'])&3 or not all(math.isfinite(r[k]) and 0<=r[k]<=1 for k in ('SV','SP')) for r in healthy):
        raise ValueError('EKF fusion faults/timeouts/innovation rejection')
    channels={}
    for kind,log in [('VISION_POSITION_ESTIMATE','VISP'),('VISION_SPEED_ESTIMATE','VISV')]:
        wire=[r['message']['usec'] for r in packets if r['message']['mavpackettype']==kind and r['valid_capture']]
        rows=[r for r in records if r['mavpackettype']==log]
        actual=[r['RTimeUS'] for r in rows]
        if (len(wire)<2 or wire!=actual or len(set(wire))!=len(wire) or
                any(r['Ign']!=0 or r['Rst']!=0 for r in rows)):
            raise ValueError('FCU reception/timestamp/rejection mismatch: '+kind)
        channels[kind]=len(wire)
    return dict(kind='LIVE_CAMERA_DISARMED_FUSION',channels=channels,disarmed_heartbeats=len(heartbeats),
                fcu_window_us=[lo,hi],post_stream_dropout=any(t>hi for t in stopped),
                gt_vertical_aid=config.get('vertical_aid'),synthetic=False)


def decode_dataflash(bin_path):
    """A file decoder only; this API must never receive a connection string."""
    bin_path=Path(bin_path)
    if not bin_path.is_file() or bin_path.suffix!='.BIN':raise ValueError('Require an actual DataFlash BIN file')
    from pymavlink import mavutil
    connection=mavutil.mavlink_connection(str(bin_path));records=[]
    wanted={'PARM','MSG','XKFS','XKF4','VISP','VISV'}
    try:
        while True:
            m=connection.recv_match()
            if m is None:break
            if m.get_type()=='BAD_DATA':raise ValueError('Corrupt DataFlash evidence')
            if m.get_type() in wanted:records.append(m.to_dict())
    finally:connection.close()
    return records


def build_proof(run,bin_path,output):
    """Offline only. Requires pymavlink on the Linux host; never opens an FCU."""
    run=Path(run).resolve();bin_path=Path(bin_path).resolve();output=Path(output).resolve()
    if not run.is_relative_to(ROOT/'phase1_runs') or not bin_path.is_relative_to(run/'state/fcu/logs'):
        raise ValueError('Require the closed owned DISARMED run and its DataFlash BIN')
    if output!=run/'evidence/flight_fusion_proof.json':raise ValueError('Use the exact new proof path')
    records=decode_dataflash(bin_path)
    analysis=evidence_analysis(run,records)
    files=(*PROOF_INPUTS,str(bin_path.relative_to(run)))
    proof=dict(schema=1,analysis=analysis,decoded_records=records,decoder='pymavlink DataFlash',
               source_sha256=sha(Path(__file__)),manifest={p:sha(run/p) for p in files})
    with output.open('x') as f:json.dump(proof,f,allow_nan=False)
    return output


def verify_proof(path,*,redecode=True):
    """Verify the operator/host decoder attestation and re-run all semantic checks."""
    path=Path(path).resolve();run=path.parent.parent
    if path.name!='flight_fusion_proof.json' or not run.is_relative_to(ROOT/'phase1_runs'):
        raise ValueError('Missing owned fusion proof')
    p=read_json(path)
    if (p.get('schema')!=1 or p.get('decoder')!='pymavlink DataFlash' or
            p.get('source_sha256')!=sha(Path(__file__)) or not set(PROOF_INPUTS)<=set(p['manifest']) or
            not any(n.startswith('state/fcu/logs/') and n.endswith('.BIN') for n in p['manifest'])):
        raise ValueError('Incomplete/source-mismatched fusion proof')
    for name,digest in p['manifest'].items():
        file=(run/name).resolve()
        if not file.is_relative_to(run) or sha(file)!=digest:raise ValueError('Fusion evidence changed: '+name)
    bins=[n for n in p['manifest'] if n.startswith('state/fcu/logs/') and n.endswith('.BIN')]
    if len(bins)!=1:raise ValueError('Require one exact baseline DataFlash file')
    if redecode and decode_dataflash(run/bins[0])!=p['decoded_records']:
        raise ValueError('DataFlash decoder output does not match the claimed fusion proof')
    if evidence_analysis(run,p['decoded_records'])!=p['analysis']:raise ValueError('Contradictory fusion proof')
    return run,read_json(run/'artifacts/simulation_vision_permit.json')


def load_plan(path,route):
    p=read_json(path)
    if (p.get('schema')!=1 or p.get('scope')!=SCOPE or p.get('approved') is not True or
            p.get('execution_allowed') is not False or p.get('forwarding_enabled') is not False or
            p.get('limits')!=LIMITS or p.get('run_id')!=route.permit['run_id'] or
            p.get('reference_nonce')!=route.permit['reference_nonce'] or
            p.get('source_sha256')!=sha(Path(__file__)) or
            p.get('host_runner_sha256')!=sha(ROOT/'run_simulated_flight.py')):
        raise ValueError('Explicit run-bound simulation flight plan required')
    proof=ROOT/p['fusion_proof']
    if sha(proof)!=p['fusion_proof_sha256']:raise ValueError('Fusion proof digest changed')
    # Trusted host launcher re-decodes before granting this run-specific plan.
    # The transport has no DataFlash dependency; it rechecks immutable byte
    # digests and semantic analysis of that approved host attestation.
    _,baseline=verify_proof(proof,redecode=False)
    if (baseline['firmware_sha256']!=route.firmware_sha256 or baseline['source_hashes']!=route.permit['source_hashes']):
        raise ValueError('Flight does not match the validated DISARMED stack')
    return p


def current_fusion_attestation(records,permit,snapshot,latest_applied_ns):
    """Read-only host DataFlash witness for THIS boot/run, not prior reception."""
    params={r['Name']:r['Value'] for r in records if r['mavpackettype']=='PARM'}
    if any(params.get(k)!=v for k,v in dict(EXPECTED,ARMING_CHECK=1).items()):
        raise ValueError('Current DataFlash parameters not ready')
    visits=[r for r in records if r['mavpackettype'] in ('VISP','VISV')]
    if not visits or {r['mavpackettype'] for r in visits}!={'VISP','VISV'} or any(r['Ign'] or r['Rst'] for r in visits):
        raise ValueError('Current FCU vision reception not ready')
    lo=min(r['TimeUS'] for r in visits);hi=max(r['TimeUS'] for r in visits)
    if hi-lo<5_000_000:raise ValueError('Current aided window shorter than 5s')
    if type(latest_applied_ns) is not int or not -20_000_000<=latest_applied_ns-hi*1000<=500_000_000:
        raise ValueError('Buffered DataFlash witness is stale/future in FCU time')
    messages=[r for r in records if r['mavpackettype']=='MSG']
    aiding=[r['TimeUS'] for r in messages if 'is using external nav data' in r['Message']]
    if not aiding or any(r['TimeUS']>max(aiding) and 'stopped aiding' in r['Message'] for r in messages):
        raise ValueError('Current ExternalNav aiding not proven')
    source=[r for r in records if r['mavpackettype']=='XKFS' and r['TimeUS']>=max(aiding)]
    status=[r for r in records if r['mavpackettype']=='XKF4' and r['TimeUS']>=max(aiding)]
    if not source or any(r['SS']!=0 for r in source) or not status or any(
            r['FS'] or int(r['TS'])&3 or any(not math.isfinite(r[k]) or not 0<=r[k]<=1 for k in ('SP','SV')) for r in status):
        raise ValueError('Current source/innovation/timeout proof failed')
    return dict(schema=1,scope='current-simulation-fusion',run_id=permit['run_id'],
                reference_nonce=permit['reference_nonce'],clock_epoch=permit['clock_epoch'],
                firmware_sha256=permit['firmware_sha256'],fcu_container_id=permit['containers']['fcu'],
                snapshot_sha256=sha(snapshot),last_fcu_time_ns=hi*1000,produced_host_ns=time.monotonic_ns())


def verify_current_attestation(row,permit,now,latest_fcu_ns):
    if (row.get('schema')!=1 or row.get('scope')!='current-simulation-fusion' or
            any(row.get(k)!=permit[k] for k in ('run_id','reference_nonce','clock_epoch','firmware_sha256')) or
            row.get('fcu_container_id')!=permit['containers']['fcu'] or
            type(row.get('last_fcu_time_ns')) is not int or
            not -20_000_000<=latest_fcu_ns-row['last_fcu_time_ns']<=500_000_000 or
            type(row.get('produced_host_ns')) is not int or not 0<=now-row['produced_host_ns']<=500_000_000):
        raise ValueError('Missing/stale/mismatched current FCU fusion witness')
    return True


class FlightHeartbeatGuard(HeartbeatGuard):
    """Only selected by a verified flight plan. Unexpected ARM closes vision."""
    def feed(self,data,now):
        self.assert_fresh(now)
        for m in self.parser.parse_buffer(data) or []:
            if m.get_type()=='BAD_DATA':self.bad_data+=1;continue
            if m.get_type()!='HEARTBEAT':continue
            if ((m.get_srcSystem(),m.get_srcComponent())!=(self.system,self.component) or
                    m.get_msgbuf()[0]!=253 or m.mavlink_version!=3 or
                    m.autopilot!=self.wire.MAV_AUTOPILOT_ARDUPILOTMEGA or m.type!=self.wire.MAV_TYPE_QUADROTOR):
                self.fail('Wrong simulation FCU heartbeat protocol/source/role')
            if self.first is None:self.first=now
            self.last=now;self.count+=1


class FlightController:
    """Pure FSM, immutable monotonic deadline. Sender accepts only fixed commands."""
    def __init__(self,wire,started_ns,send_command,record=lambda **r:None,*,global_origin=None):
        self.wire=wire;self.started=started_ns;self.deadline=started_ns+int(LIMITS['total_s']*1e9)
        self.send=send_command;self.record=record;self.state='PREFLIGHT';self.since=started_ns
        self.failure=None;self.telemetry={};self.acks={};self.stable=None;self.land_start=None
        self.arm_expected=False;self.armed_seen=False;self.last_pair=None;self.home_requested=False
        self.prearm_requested=False;self.done=False;self.result='NOT TESTED';self.origin=None
        self.global_origin=global_origin
        self.last_tick=None

    def transition(self,state,now):
        self.record(event='flight_transition',previous=self.state,state=state,host_ns=now)
        self.state=state;self.since=now

    def command(self,command,params=()):
        self.acks.pop(command,None)
        self.pending_command=command
        self.send(command,list(params)+[0.]*(7-len(params)))

    def observe(self,msg,now):
        if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):return
        kind=msg.get_type();self.telemetry[kind]=(msg,now)
        if kind=='COMMAND_ACK':
            self.acks[msg.command]=msg.result
            if msg.command==self.wire.MAV_CMD_NAV_LAND and msg.result not in (0,5) and self.state in ('LAND','EMERGENCY_LAND'):
                self.failure=self.failure or 'LAND command rejected';self.done=True;self.result='FAIL';self.transition('STOP',now)
            if msg.command==getattr(self,'pending_command',None) and msg.result not in (0,5) and self.state not in ('LAND','EMERGENCY_LAND'):
                self.emergency('Command rejected: '+str(msg.command),now)
        if kind=='HEARTBEAT':
            armed=bool(msg.base_mode&128)
            self.armed_seen|=armed
            if armed and not self.arm_expected:self.emergency('Unexpected ARMED heartbeat',now)

    def fresh(self,kind,now,age=500_000_000):
        pair=self.telemetry.get(kind)
        if pair is None or not 0<=now-pair[1]<=age:raise ValueError('Stale/missing '+kind)
        return pair[0]

    def healthy(self,now,parameters,origin_confirmed):
        if any(parameters.get(k)!=v for k,v in EXPECTED.items()) or parameters.get('ARMING_CHECK')!=1:
            raise ValueError('ExternalNav/GPS/arming-check parameter mismatch')
        if not origin_confirmed or self.global_origin is None:raise ValueError('EKF origin unavailable')
        hb=self.fresh('HEARTBEAT',now,3_000_000_000)
        status=self.fresh('EKF_STATUS_REPORT',now)
        required=1|2|32
        if (status.flags&required!=required or not status.flags&(8|16) or status.flags&(128|1024) or
                any(not math.isfinite(getattr(status,k)) or not 0<=getattr(status,k)<=1 for k in
                    ('velocity_variance','pos_horiz_variance','pos_vert_variance','compass_variance'))):
            raise ValueError('Unhealthy EKF/innovation test ratios')
        pos=self.fresh('GLOBAL_POSITION_INT',now)
        local=self.fresh('LOCAL_POSITION_NED',now)
        altitude=pos.relative_alt/1000.
        if (not all(math.isfinite(v) for v in (altitude,local.x,local.y,local.z,local.vx,local.vy,local.vz)) or
                altitude>LIMITS['max_alt_m'] or altitude<-.2):
            raise ValueError('Altitude/state envelope')
        if self.origin is not None and math.hypot(local.x-self.origin[0],local.y-self.origin[1])>LIMITS['max_xy_m']:
            raise ValueError('Horizontal drift envelope')
        if self.state in ('ARM','CLIMB','HOVER') and hb.custom_mode!=4:raise ValueError('GUIDED mode lost')
        if self.state in ('CLIMB','HOVER') and not hb.base_mode&128:raise ValueError('Unexpected DISARM during flight')
        return hb,altitude,local

    def emergency(self,reason,now):
        if self.failure is None:self.failure=reason
        if self.state not in ('EMERGENCY_LAND','DONE','STOP'):
            # ARM may be in flight even before the first armed heartbeat.
            if self.arm_expected or self.armed_seen:
                try:
                    if self.land_start is None:self.command(self.wire.MAV_CMD_NAV_LAND)
                except OSError:
                    self.done=True;self.result='FAIL';self.transition('STOP',now);return
                if self.land_start is None:self.land_start=now
                self.transition('EMERGENCY_LAND',now)
            else:self.done=True;self.result='FAIL';self.transition('STOP',now)

    def tick(self,now,parameters,origin_confirmed,vision_valid,*,parameters_fresh=False):
        if self.done:return
        if type(now) is not int or now<self.started or now>=self.deadline:
            self.failure=self.failure or 'Immutable flight deadline exceeded';self.result='FAIL';self.done=True
            self.transition('STOP',now);return
        if self.state=='PREFLIGHT' and now-self.started>=int(LIMITS['preflight_s']*1e9):
            self.emergency('Preflight timeout',now);return
        if self.state in ('GUIDED','ARM') and now-self.since>=int(LIMITS['command_s']*1e9):
            self.emergency('Command ACK/state timeout',now);return
        if self.state=='CLIMB' and now-self.since>=int(LIMITS['climb_s']*1e9):
            self.emergency('Takeoff timeout',now);return
        if self.last_tick is not None and now-self.last_tick>500_000_000:
            self.stable=None
            if self.arm_expected:self.emergency('Flight controller scheduling gap >500ms',now)
        self.last_tick=now
        if self.arm_expected and now>=self.deadline-int(LIMITS['land_s']*1e9) and self.state not in ('LAND','EMERGENCY_LAND'):
            self.emergency('Immutable LAND reserve reached',now)
        if self.state in ('LAND','EMERGENCY_LAND'):
            try:
                pos=self.fresh('GLOBAL_POSITION_INT',now)
                if pos.relative_alt/1000.>LIMITS['max_alt_m']:
                    self.failure=self.failure or 'Altitude limit persists during LAND';self.result='FAIL';self.done=True;self.transition('STOP',now);return
                hb=self.fresh('HEARTBEAT',now,3_000_000_000)
                landed=self.fresh('EXTENDED_SYS_STATE',now)
                if not hb.base_mode&128 and hb.custom_mode==9 and landed.landed_state==1 and self.acks.get(self.wire.MAV_CMD_NAV_LAND)==0:
                    self.result='FAIL' if self.failure else 'PASS';self.done=True;self.transition('DONE',now);return
            except ValueError:pass
            if now-self.land_start>=int(LIMITS['land_s']*1e9):
                self.failure=self.failure or 'LAND/disarmed not confirmed';self.result='FAIL';self.done=True;self.transition('STOP',now)
            return  # never force DISARM or replay stale pose
        if self.state!='PREFLIGHT' and not vision_valid:self.emergency('VIO/clock/permission lost',now);return
        try:
            hb,alt,local=self.healthy(now,parameters,origin_confirmed)
            if not vision_valid:raise ValueError('Waiting for live camera VIO/clock/permission')
            if self.state=='PREFLIGHT' and not self.prearm_requested:
                self.command(self.wire.MAV_CMD_RUN_PREARM_CHECKS);self.prearm_requested=True
            if self.state=='PREFLIGHT' and not self.home_requested:
                self.command(self.wire.MAV_CMD_DO_SET_HOME,[1]);self.home_requested=True
            home=self.fresh('HOME_POSITION',now,5_000_000_000)
            if (not -900000000<=home.latitude<=900000000 or not -1800000000<=home.longitude<=1800000000 or
                    (home.latitude,home.longitude)==(0,0)):raise ValueError('Invalid HOME_POSITION')
            north=(home.latitude-self.global_origin[0])/1e7*111319.5
            east=(home.longitude-self.global_origin[1])/1e7*111319.5*math.cos(math.radians(self.global_origin[0]/1e7))
            if math.hypot(north,east)>LIMITS['max_xy_m'] or abs(home.altitude-self.global_origin[2])/1000>LIMITS['max_alt_m']:
                raise ValueError('HOME/EKF origin reference mismatch')
            if self.state in ('PREFLIGHT','GUIDED'):
                sys=self.fresh('SYS_STATUS',now)
                bit=self.wire.MAV_SYS_STATUS_PREARM_CHECK
                if (sys.onboard_control_sensors_present&bit==0 or sys.onboard_control_sensors_enabled&bit==0 or
                        sys.onboard_control_sensors_health&bit==0):raise ValueError('Pre-arm checks not healthy')
                if hb.base_mode&128:raise ValueError('Must start DISARMED')
                if abs(alt)>.15:raise ValueError('Pre-arm altitude not on ground')
        except ValueError as exc:
            if self.state=='PREFLIGHT':
                self.stable=None
                if now-self.started>=int(LIMITS['preflight_s']*1e9):self.emergency(str(exc),now)
            else:self.emergency(str(exc),now)
            return
        if self.state=='PREFLIGHT':
            if self.stable is None:self.stable=now
            if now-self.stable>=5_000_000_000:
                self.origin=(local.x,local.y)
                status=self.telemetry['EKF_STATUS_REPORT'][0]
                self.record(event='prearm_snapshot',host_ns=now,parameters=dict(parameters),vision_valid=True,
                    ekf_flags=status.flags,velocity_ratio=status.velocity_variance,position_ratio=status.pos_horiz_variance,
                    relative_alt_m=alt,home=[home.latitude,home.longitude,home.altitude])
                self.command(self.wire.MAV_CMD_DO_SET_MODE,[1,4]);self.transition('GUIDED',now)
        elif self.state=='GUIDED':
            if hb.custom_mode==4 and self.acks.get(self.wire.MAV_CMD_DO_SET_MODE)==0 and parameters_fresh:
                self.arm_expected=True;self.command(self.wire.MAV_CMD_COMPONENT_ARM_DISARM,[1,0]);self.transition('ARM',now)
        elif self.state=='ARM':
            if hb.base_mode&128 and self.acks.get(self.wire.MAV_CMD_COMPONENT_ARM_DISARM)==0:
                self.command(self.wire.MAV_CMD_NAV_TAKEOFF,[0,0,0,0,0,0,LIMITS['target_alt_m']]);self.transition('CLIMB',now)
        elif self.state=='CLIMB':
            if self.acks.get(self.wire.MAV_CMD_NAV_TAKEOFF)==0 and abs(alt-LIMITS['target_alt_m'])<=.15 and abs(local.vz)<=.2:
                self.transition('HOVER',now)
        elif self.state=='HOVER':
            if abs(alt-LIMITS['target_alt_m'])>.2:self.emergency('Hover altitude envelope',now);return
            if now-self.since>=int(LIMITS['hover_s']*1e9):
                self.command(self.wire.MAV_CMD_NAV_LAND);self.land_start=now;self.transition('LAND',now)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='OFFLINE: decode/review a CLOSED DISARMED camera-fusion baseline. No FCU connection.')
    parser.add_argument('--run',type=Path,required=True);parser.add_argument('--bin',type=Path,required=True)
    args=parser.parse_args()
    print(build_proof(args.run,args.bin,args.run/'evidence/flight_fusion_proof.json'))
