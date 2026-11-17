"""Offline, independent report for a CLOSED simulation-only flight run.

No sockets, ROS, commands, authorization writes or estimate modification. Uses
existing GT evaluation, EKF diagnostics and applied-clock verification. CLI has
no synthetic decoder switch. PASS describes recorded evidence, never flight GO.
"""
import argparse
from bisect import bisect_left
from collections import Counter
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

from analyze_phase1_run import evaluate, plot_trajectory, intervals
from analyze_ekf3_fusion import analyze_records
from analyze_stage_latency import quantiles
from linux_gpu.prepare_vio import FILES
from linux_gpu.verified_disarmed import load_codec
from physics_clock_mapping import AppliedClockMapping
from preflight_ekf3 import EXPECTED
from runtime_candidate import AP_COMMIT
from simulation_flight import LIMITS
from vio_adapter import validate_covariance, expand_upper_triangle

NV='NOT VERIFIED'


class Missing(ValueError):pass


def number(value):
    if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('Nonfinite/non-numeric value')
    return value


def integer(value):
    if type(value) is not int or value<0:raise ValueError('Expected nonnegative raw integer timestamp/flag')
    return value


def verdict(status,reason,**detail):return dict(status=status,reason=reason,**detail)


def overall(checks):
    states=[c['status'] for c in checks.values()]
    return 'FAIL' if 'FAIL' in states else ('PASS' if states and all(s=='PASS' for s in states) else NV)


class Evidence:
    def __init__(self,run):self.run=Path(run).resolve();self.manifest={}
    def raw(self,name):
        path=(self.run/name).resolve()
        if not path.is_relative_to(self.run):raise ValueError('Evidence path escapes run')
        if not path.is_file():raise Missing('Missing evidence: '+str(name))
        raw=path.read_bytes()
        if not raw:raise ValueError('Empty evidence: '+str(name))
        name=str(path.relative_to(self.run));info=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
        if name in self.manifest and self.manifest[name]!=info:raise ValueError('Evidence changed between reads: '+name)
        self.manifest[name]=info
        return raw
    def json(self,name,lines=False):
        raw=self.raw(name)
        def floating(s):
            value=float(s);number(value);return value
        def bad(s):raise ValueError('Nonfinite JSON: '+s)
        def parse(s):return json.loads(s,parse_float=floating,parse_constant=bad)
        if lines:
            if not raw.endswith(b'\n') or any(not line.strip() for line in raw.splitlines()):raise ValueError('Partial/blank JSONL: '+str(name))
            return [parse(s) for s in raw.splitlines()]
        return parse(raw)
    def unchanged(self):
        for name,info in list(self.manifest.items()):
            if hashlib.sha256((self.run/name).read_bytes()).hexdigest()!=info['sha256']:raise ValueError('Evidence changed during analysis: '+name)


def ordered(rows,key,*,strict=True):
    values=[number(r[key]) for r in rows]
    if not values:raise Missing('No records: '+key)
    if any(b<a or (strict and b==a) for a,b in zip(values,values[1:])):raise ValueError('Reset/duplicate/unordered '+key)
    return values


def decode_bin(path):
    """Actual local BIN decoder. Never accepts an FCU endpoint/connection URL."""
    path=Path(path).resolve()
    if path.suffix!='.BIN' or not path.is_file():raise ValueError('Require existing raw DataFlash .BIN')
    from pymavlink import mavutil
    decoder=mavutil.mavlink_connection(str(path));rows=[]
    wanted={'PARM','MSG','VISP','VISV','XKF1','XKF3','XKF4','XKFS','XKV1'}
    try:
        while True:
            msg=decoder.recv_match()
            if msg is None:break
            if msg.get_type()=='BAD_DATA':raise ValueError('Corrupt raw DataFlash')
            if msg.get_type() in wanted:rows.append(msg.to_dict())
    finally:decoder.close()
    if not rows:raise ValueError('No decodable DataFlash records')
    return rows


class Timeline:
    """Replay actual guard math, not a new fitted time-alignment algorithm."""
    def __init__(self,e,transitions):
        identity=e.json('artifacts/run_identity.json');permit=e.json('artifacts/simulation_vision_permit.json')
        digest=permit['firmware_sha256']
        if (permit.get('ap_commit')!=AP_COMMIT or permit.get('approved') is not True or
                any(r.get(k) is not False for r in (identity,permit) for k in ('execution_allowed','forwarding_enabled')) or
                any(identity[k]!=permit[k] for k in ('run_id','reference_nonce','clock_epoch','firmware_sha256')) or
                hashlib.sha256(e.raw('artifacts/arducopter')).hexdigest()!=digest):
            raise ValueError('Unbound clock epoch/firmware provenance')
        class RecordedFirmware:
            def accepts_firmware(_,candidate):return candidate==digest
        self.epoch=identity['clock_epoch'];self.anchors=[]
        clocks=e.json('evidence/flight-source/clocks.jsonl',True)
        clock_values=ordered(clocks,'t',strict=False)
        ordered(clocks,'arrival_monotonic_s',strict=False)
        if clock_values[0]==clock_values[-1]:raise Missing('No advancing Gazebo/ROS clock evidence')
        raw=e.raw('evidence/applied.csv')
        if not raw.endswith(b'\n'):raise ValueError('Partial applied-clock CSV')
        for line in raw.decode().splitlines():
            fields=line.split(',')
            if len(fields)!=9:raise ValueError('Unexpected applied-clock schema/units')
            source=Decimal(fields[0])*10**9
            if not source.is_finite() or source!=int(source):raise ValueError('Invalid simulation seconds/nanoseconds')
            self.anchors.append(dict(source_ns=int(source),target_ns=int(fields[4])*1000,host_ns=int(fields[6]),
                                     parsed_no_time_sync=int(fields[1]),use_time_sync=int(fields[2])))
        if len(self.anchors)<3:raise Missing('Insufficient applied anchors')
        mapper=AppliedClockMapping(self.epoch,digest,simulation_route=RecordedFirmware())
        witnesses=e.json('evidence/flight-transport/fcuwitness.jsonl',True)
        mappings=e.json('evidence/flight-transport/mapping.jsonl',True)
        ordered(witnesses,'host_ns',strict=False);ordered(mappings,'approved_host_ns')
        codec=load_codec();parser=codec.MAVLink(None);parser.robust_parsing=True
        packets=parser.parse_buffer(e.raw('evidence/flight-transport/fcu_tcp.bin')) or []
        labels=Counter(m.time_boot_ms*1_000_000 for m in packets if m.get_type()=='SYSTEM_TIME'
                       and (m.get_srcSystem(),m.get_srcComponent())==(1,1) and m.get_msgbuf()[0]==253)
        for w in witnesses:
            if w.get('kind')=='SYSTEM_TIME':
                if labels[w['fcu_ns']]<=0:raise ValueError('Clock witness absent from actual FCU TCP packets')
                labels[w['fcu_ns']]-=1
        stages=e.json('evidence/flight-transport/stages.jsonl',True)
        if any(r.get('stage')=='profile_summary' and r.get('dropped',0) for r in stages):raise Missing('Dropped clock-profile evidence')
        by_source={r['source_ns']:r for r in self.anchors}
        if len(by_source)!=len(self.anchors):raise ValueError('Duplicate/reset applied source timestamps')
        # Validate the whole raw trace separately. Reception/validation below
        # uses the ACTUAL read/validate events, not retrospectively available anchors.
        raw_mapper=AppliedClockMapping(self.epoch,digest,simulation_route=RecordedFirmware())
        for anchor in self.anchors:raw_mapper.feed(**anchor)
        validated=[r for r in stages if r.get('stage')=='clock_anchor_validated']
        received={r['source_ns']:r for r in stages if r.get('stage')=='clock_anchor_received'}
        if not validated:raise Missing('No actual clock-anchor validation stages')
        ordered(validated,'source_ns');ordered(validated,'host_ns')
        events=[]
        for stage in validated:
            a=by_source[integer(stage['source_ns'])];receipt=received[a['source_ns']]
            if (stage['produced_host_ns']!=a['host_ns'] or receipt['produced_host_ns']!=a['host_ns'] or
                    not a['host_ns']<=receipt['received_host_ns']<=stage['host_ns']):
                raise ValueError('Contradictory anchor production/receipt/validation')
            events.append((stage['host_ns'],0,a))
        events += [(integer(r['host_ns']),1,r) for r in witnesses if r.get('kind')=='SYSTEM_TIME']
        events += [(integer(r['approved_host_ns']),2,r) for r in mappings]
        self.mapped={};self.witnesses=0
        # Merge DIFFERENT streams only. Raw per-stream order is validated first.
        for host,kind,r in sorted(events,key=lambda event:(event[0],event[1])):
            if kind==0:mapper.feed(**r)
            elif kind==1:
                if r.get('within_applied_bracket') is not True or not mapper.crosscheck(r['fcu_ns'],r['sent_ns'],r['received_ns']):
                    raise ValueError('FCU clock cross-check missing/outside applied bracket')
                self.witnesses+=1
            else:
                actual=mapper.map(integer(r['acquisition_ns']),host,self.epoch)
                for key in ('fcu_ns','lower_fcu_ns','upper_fcu_ns','uncertainty_ns'):
                    if integer(r[key])!=actual[key]:raise ValueError('Recorded mapping contradicts raw anchors/units: '+key)
                self.mapped[r['acquisition_ns']]=r
        if self.witnesses<10 or not mapper.verified:raise Missing('Need ten actual FCU clock witnesses')
        summary=e.json('evidence/flight-transport/summary.json')
        if summary.get('clock_mapping_verified') is not True or summary.get('clock_latched') is not False:
            raise ValueError('Transport clock was unverified/latched')
        self.phases={}
        for state in ('CLIMB','HOVER','LAND','DONE'):
            records=[r for r in transitions if r.get('event')=='flight_transition' and r.get('state')==state]
            if len(records)!=1:raise Missing('No unique nominal phase: '+state)
            self.phases[state]=self.bound(integer(records[0]['host_ns']),'host_ns')
        times=[self.phases[s]['host_ns'] for s in self.phases]
        if any(b<=a for a,b in zip(times,times[1:])):raise ValueError('Flight phase clock/order mismatch')
        self.start=self.phases['CLIMB']['source_lower_ns']*1e-9
        self.end=self.phases['DONE']['source_upper_ns']*1e-9
        if not clock_values[0]<=self.start<self.end<=clock_values[-1]:raise Missing('ROS clock does not cover mission')
        self.fcu_start=self.phases['CLIMB']['target_lower_ns']//1000
        self.fcu_end=self.phases['DONE']['target_upper_ns']//1000
        begin=self.phases['CLIMB']['host_ns'];end=self.phases['DONE']['host_ns']
        for left,right in zip(self.anchors,self.anchors[1:]):
            if left['host_ns']<end and right['host_ns']>begin and right['host_ns']-left['host_ns']>200_000_000:
                raise ValueError('Applied source stalled beyond200ms during mission')
        advancing=[r for i,r in enumerate(clocks) if i==0 or r['t']>clocks[i-1]['t']]
        for left,right in zip(advancing,advancing[1:]):
            if left['arrival_monotonic_s']*1e9<end and right['arrival_monotonic_s']*1e9>begin and right['arrival_monotonic_s']-left['arrival_monotonic_s']>.200000001:
                raise ValueError('Advancing ROS clock stalled beyond200ms during mission')

    def bound(self,value,key):
        index=bisect_left([r[key] for r in self.anchors],value)
        if index==len(self.anchors) or (index==0 and value<self.anchors[0][key]):raise Missing('No clock bracket; refusing extrapolation')
        right=self.anchors[index];left=right if right[key]==value else self.anchors[index-1]
        if right['host_ns']-left['host_ns']>200_000_000:raise ValueError('Clock boundary crosses200ms watchdog gap')
        return dict(host_ns=value if key=='host_ns' else None,source_lower_ns=left['source_ns'],source_upper_ns=right['source_ns'],
                    target_lower_ns=left['target_ns'],target_upper_ns=right['target_ns'])

    def fcu_seconds(self,usec):
        b=self.bound(integer(usec)*1000,'target_ns')
        return (b['source_lower_ns']+b['source_upper_ns'])/2e9


def heartbeat_records(e):
    wire=load_codec();parser=wire.MAVLink(None);parser.robust_parsing=True
    messages=parser.parse_buffer(e.raw('evidence/flight-transport/fcu_tcp.bin')) or []
    hb=[m for m in messages if m.get_type()=='HEARTBEAT']
    if len(hb)<3:raise Missing('No complete expected-source HEARTBEAT sequence')
    if any((m.get_srcSystem(),m.get_srcComponent())!=(1,1) or m.get_msgbuf()[0]!=253 or
           m.autopilot!=3 or m.type!=2 or m.mavlink_version!=3 for m in hb):raise ValueError('Wrong FCU identity/protocol')
    landed=[m for m in messages if m.get_type()=='EXTENDED_SYS_STATE' and (m.get_srcSystem(),m.get_srcComponent())==(1,1)]
    if not landed:raise Missing('No independent landed-state telemetry')
    last_armed=max((i for i,m in enumerate(messages) if m.get_type()=='HEARTBEAT' and m.base_mode&128),default=-1)
    if not any(i>last_armed and m in landed and m.landed_state==1 for i,m in enumerate(messages)):
        raise ValueError('No on-ground telemetry after armed heartbeat')
    if not any(m.get_type()=='COMMAND_ACK' and (m.get_srcSystem(),m.get_srcComponent())==(1,1) and m.command==21 and m.result==0 for m in messages):
        raise Missing('No independent LAND ACK')
    return hb,landed


def source_review(e,clock):
    config=e.json('evidence/flight-source/configuration.json')
    if (config.get('horizontal_gt_input') is not False or config.get('world_gt_attitude_used_for_camera_depth') is not False or
            config.get('faults') or config.get('fault_file') or config.get('synthetic') or
            any(config.get('source_sha256',{}).get(k)!=v for k,v in FILES.items())):
        raise ValueError('Not pinned live-camera VIO; synthetic/fallback/GT XY rejected')
    stream=e.json('evidence/flight-source/stream.jsonl',True)
    pubs=e.json('evidence/flight-source/publications.jsonl',True)
    obs=e.json('evidence/flight-observer/input.jsonl',True)
    camera=e.json('evidence/flight-source/ros_camera.jsonl',True)
    imu=e.json('evidence/flight-source/imu.jsonl',True)
    for rows,key in ((stream,'t'),(pubs,'t'),(camera,'t'),(imu,'t'),(obs,'arrival_monotonic_s')):ordered(rows,key)
    observed=[integer(r['sec'])*10**9+integer(r['nanosec']) for r in obs]
    stamps=[round(r['t']*1e9) for r in pubs]
    if stamps!=observed or any(not r['valid'] or r['nanosec']>=10**9 for r in obs):raise ValueError('Independent subscriber mismatch/invalid Odometry')
    acquired={round(r['t']*1e9) for r in camera}
    published={round(r['t']*1e9):r for r in stream if r.get('published')}
    if set(stamps)!=set(published) or not set(stamps)<=acquired:raise ValueError('No matching camera acquisitions/source publication')
    for r in published.values():
        check=r.get('publication_clock_check',{})
        if r.get('ready') is not True or r.get('epoch')!=0 or check.get('valid') is not True or not 0<=number(check['simulation_age_s'])<=.18:
            raise ValueError('Stale/reset/unready source publication')
    return dict(config=config,window=[r for r in stream if clock.start<=r['t']<=clock.end],pubs=pubs,imu=imu,published=published)


def continuity(e,source,clock):
    summary=e.json('evidence/flight-source/summary.json')
    if summary['gaps'] or summary['recoveries']:raise ValueError('Source IMU gap/reinitialization')
    if summary.get('emitted')!=len(source['pubs']):raise ValueError('Source publication count mismatch')
    imu=source['imu'];window=source['window']
    if any(not 0<b['t']-a['t']<=.020000001 for a,b in zip(imu,imu[1:])):raise ValueError('Raw IMU gap >20ms')
    sent=[r for r in source['pubs'] if clock.start<=r['t']<=clock.end]
    if len(sent)<3:raise Missing('Insufficient mission Odometry')
    for rows,key,bound in ((sent,'t',.200000001),(sent,'publish_monotonic_s',.5)):
        times=ordered(rows,key)
        if max(np.diff(times))>bound:raise ValueError('Mission VIO dropout: '+key)
    if sent[0]['t']-clock.start>.2 or clock.end-sent[-1]['t']>.2:raise ValueError('VIO missing at mission boundary')
    if any(r.get('epoch',0)!=0 for r in window):raise ValueError('Reset frame inside complete mission')
    for r in sent:
        ns=round(r['t']*1e9)
        if ns not in clock.mapped:raise ValueError('Published mission sample missing final transport mapping')
    return dict(samples=len(window),published=len(sent),blocked_frames=sum(not r.get('published') for r in window),
                reasons=[r['error'] for r in window if r.get('error')],
                intervals=intervals(sent,'publish_monotonic_s'),
                sim_gaps_s=quantiles(np.diff([r['t'] for r in sent])),
                wall_gaps_s=quantiles(np.diff([r['publish_monotonic_s'] for r in sent])))


def fcu_review(rows,wire,clock,source):
    params={r['Name']:r['Value'] for r in rows if r['mavpackettype']=='PARM'}
    lo,hi=clock.fcu_start,clock.fcu_end
    for name,value in EXPECTED.items():
        recorded=[r for r in rows if r['mavpackettype']=='PARM' and r['Name']==name]
        before=[r for r in recorded if r['TimeUS']<=lo]
        if not before:raise Missing('No preflight parameter proof: '+name)
        if before[-1]['Value']!=value or any(r['Value']!=value for r in recorded if lo<=r['TimeUS']<=hi):
            raise ValueError('ExternalNav parameter changed/invalid during mission: '+name)
    selected=[r for r in rows if lo<=r['TimeUS']<=hi]
    messages=[r for r in rows if r['mavpackettype']=='MSG']
    aiding=[r for r in messages if 'is using external nav data' in r['Message'] and r['TimeUS']<=lo]
    if not aiding:raise Missing('No DataFlash camera ExternalNav aiding before TAKEOFF')
    if any(lo<=r['TimeUS']<=hi and 'stopped aiding' in r['Message'] for r in messages):raise ValueError('EKF stopped aiding during mission')
    for core in (0,1):
        if not any(re.search(r'\bIMU'+str(core)+r'\b',r['Message']) for r in aiding):
            raise Missing('No per-core ExternalNav aiding message: IMU'+str(core))
        for typ in ('XKF3','XKF4','XKFS'):
            values=[r for r in rows if r['mavpackettype']==typ and r.get('C')==core]
            if not values or min(r['TimeUS'] for r in values)>lo or max(r['TimeUS'] for r in values)<hi:
                raise Missing('Unbracketed per-core fusion evidence: '+typ+' core'+str(core))
        status=[r for r in selected if r['mavpackettype']=='XKF4' and r.get('C')==core]
        sources=[r for r in selected if r['mavpackettype']=='XKFS' and r.get('C')==core]
        if not status or not sources:raise Missing('No mission EKF status/source records')
        if any(r['FS']!=0 or integer(r['TS'])&3 or any(not 0<=number(r[k])<=1 for k in ('SP','SV')) for r in status):
            raise ValueError('EKF fault/timeout/innovation rejection')
        if any(r['SS']!=0 for r in sources):raise ValueError('Active source set changed')
    receipt_bounds={}
    for name,typ,size in [('VISION_POSITION_ESTIMATE','VISP',21),('VISION_SPEED_ESTIMATE','VISV',9)]:
        incoming=[r for r in wire if r.get('forwarded_to_fcu') and r['message']['mavpackettype']==name]
        received=[r for r in rows if r['mavpackettype']==typ]
        if len(incoming)<2 or len(received)<2:raise Missing('No complete FCU receipt channel: '+typ)
        if [r['message']['usec'] for r in incoming]!=[r['RTimeUS'] for r in received]:raise ValueError('Wire/FCU acquisition timestamps mismatch')
        ordered([dict(usec=r['message']['usec']) for r in incoming],'usec')
        receipt_ages=[]
        for r,receipt in zip(incoming,received):
            if not r['valid_capture']:raise ValueError('Invalid forwarded vision')
            c=r['message']['covariance']
            if len(c)!=size:raise ValueError('Malformed covariance')
            validate_covariance(expand_upper_triangle(c) if size==21 else c,6 if size==21 else 3)
            mapping=next((m for m in clock.mapped.values() if m['fcu_ns']//1000==r['message']['usec']),None)
            if mapping is None or not mapping['approved_host_ns']<=r['checked_host_ns']<=mapping['allowed_host_deadline_ns']:
                raise ValueError('Final wire permission/mapping invalid')
            state=source['published'].get(mapping['acquisition_ns'])
            if state is None:raise ValueError('Wire vision has no matching camera VIO state')
            expected=state['state'][:3] if size==21 else state['state'][3:6]
            actual=[number(r['message'][key]) for key in ('x','y','z')]
            if not np.allclose(actual,expected,rtol=1e-5,atol=1e-6):raise ValueError('Wire pose/speed differs from camera VIO state')
            age_lo=receipt['TimeUS']*1000-mapping['upper_fcu_ns']
            age_hi=receipt['TimeUS']*1000-mapping['lower_fcu_ns']
            if age_hi<0:raise ValueError('FCU received future-dated acquisition')
            if age_lo<0 or age_hi>200_000_000:raise Missing('FCU acquisition freshness not bounded within existing200ms criterion')
            receipt_ages.append([age_lo,age_hi])
            if r['clock'].get('latest_applied_age_ns') is None or not 0<=r['clock']['latest_applied_age_ns']<=200_000_000:
                raise ValueError('Wire anchor freshness failed')
            if r['clock'].get('ros_age_ns') is None or not 0<=r['clock']['ros_age_ns']<=200_000_000:raise ValueError('Final ROS clock freshness failed')
        receipt_bounds[name]=dict(lower_ns=quantiles([x[0] for x in receipt_ages]),upper_ns=quantiles([x[1] for x in receipt_ages]),
            meaning='FCU receipt minus bounded mapped acquisition label; not wall exposure latency')
        mission=[r for r in received if lo<=r['TimeUS']<=hi]
        if len(mission)<2:raise Missing('No FCU mission vision receipts')
        if any(r['Ign'] or r['Rst'] for r in mission):raise ValueError('FCU ignored/reset vision')
        acquisition=[r['RTimeUS'] for r in mission]
        if any(not 0<b-a<=200001 for a,b in zip(acquisition,acquisition[1:])):raise ValueError('FCU vision acquisition dropout/units')
    adapted=[dict(r,direction='to_fcu') for r in wire if r.get('forwarded_to_fcu')]
    windows=[]
    states=list(clock.phases)
    for i,state in enumerate(states[:-1]):
        windows.append((state,clock.phases[state]['target_lower_ns']/1000,clock.phases[states[i+1]]['target_upper_ns']/1000))
    diagnostics=analyze_records(rows,params,adapted,phase_windows=windows,
        scope='Recorded flight diagnostics; origin XY RMS is NOT GT error; sampled aiding, not per-message acceptance')
    diagnostics['fcu_receipt_acquisition_bounds']=receipt_bounds
    return diagnostics


def analyze(run,bin_path,*,decoder=None):
    e=Evidence(run);checks={};context={};metrics={};plot_data=None
    def check(name,fn):
        try:
            value=fn();checks[name]=verdict('PASS','Recorded evidence checks passed');return value
        except (Missing,ImportError) as exc:checks[name]=verdict(NV,str(exc))
        except (ValueError,KeyError,TypeError,IndexError,OSError,ArithmeticError,np.linalg.LinAlgError) as exc:
            checks[name]=verdict('FAIL',str(exc))
    def scenario():
        result=e.json('evidence/flight-transport/flight_result.json')
        events=e.json('evidence/flight-transport/flight.jsonl',True)
        ordered(events,'host_ns',strict=False)
        if result.get('status')!='PASS' or result.get('state')!='DONE' or result.get('reason'):
            raise ValueError('Controller flight FAIL/INCOMPLETE: '+str(result))
        phases=[r['state'] for r in events if r.get('event')=='flight_transition']
        if phases!=['GUIDED','ARM','CLIMB','HOVER','LAND','DONE']:raise ValueError('Not nominal TAKEOFF/HOVER/LAND sequence')
        times={r['state']:r['host_ns'] for r in events if r.get('event')=='flight_transition'}
        if (times['LAND']-times['HOVER'])*1e-9<LIMITS['hover_s']:raise ValueError('Hover shorter than approved scenario')
        plan=e.json('artifacts/flight_plan.json')
        if plan.get('limits')!=LIMITS or plan.get('execution_allowed') is not False or plan.get('forwarding_enabled') is not False:
            raise ValueError('Scenario plan/safety mismatch')
        context['flight_result']=result;return events
    events=check('flight_scenario_completion',scenario)
    def landing():
        result=e.json('evidence/flight-transport/flight_result.json');hb,land=heartbeat_records(e)
        if (not result.get('landed_disarmed_confirmed') or not any(m.base_mode&128 for m in hb) or
                hb[-1].base_mode&128 or hb[-1].custom_mode!=9 or land[-1].landed_state!=1):
            raise ValueError('LAND/disarmed not independently confirmed')
    check('confirmed_land_disarmed',landing)
    clock=check('clock_mapping',lambda:Timeline(e,events)) if events else None
    if not events:checks['clock_mapping']=verdict(NV,'Missing valid nominal phase window')
    if clock:
        context['timeline']=dict(phases=clock.phases,simulation_window_s=[clock.start,clock.end],
            fcu_window_us=[clock.fcu_start,clock.fcu_end],witnesses=clock.witnesses,
            note='Phase/FCU markers are applied-anchor brackets, not exposure/one-way latency proof')
    records=None
    def dataflash():
        path=Path(bin_path).resolve()
        if not path.is_relative_to(e.run/'state/fcu/logs') or path.suffix!='.BIN':raise ValueError('Require exact owned raw DataFlash BIN path')
        raw=e.raw(path.relative_to(e.run))
        if raw[:2]!=b'\xa3\x95':raise ValueError('Not a raw DataFlash BIN')
        rows=(decoder or decode_bin)(path)
        if not rows:raise Missing('No DataFlash records')
        grouped={}
        for r in rows:
            for value in r.values():
                if isinstance(value,float):number(value)
            integer(r['TimeUS']);key=(r['mavpackettype'],r.get('C'),r.get('Name') if r['mavpackettype']=='PARM' else None)
            last=grouped.get(key)
            if last is not None and r['TimeUS']<last:raise ValueError('DataFlash timestamp reset/order/units')
            grouped[key]=r['TimeUS']
        return rows
    records=check('raw_dataflash',dataflash)
    if decoder is not None and records is not None:checks['raw_dataflash']=verdict(NV,'TEST-ONLY injected decoder; actual BIN decoding NOT VERIFIED')
    def gps():
        if records is None:raise Missing('No raw DataFlash parameter proof')
        for name in ('GPS_TYPE','GPS_TYPE2'):
            values=[r['Value'] for r in records if r['mavpackettype']=='PARM' and r['Name']==name]
            if not values:raise Missing('Missing GPS disable parameter: '+name)
            if any(v!=0 for v in values):raise ValueError('GPS enabled in recorded log')
    check('gps_disabled',gps)
    source=check('camera_vio_evidence',lambda:source_review(e,clock)) if clock else None
    if not clock:checks['camera_vio_evidence']=verdict(NV,'No verified common timeline')
    if source:metrics['availability']=check('vio_continuity',lambda:continuity(e,source,clock))
    else:checks['vio_continuity']=verdict(NV,'No verified camera source/delivery inputs')
    def transport_continuity():
        # Transport-only evidence: runs on failed flights too and names the phase.
        import odom_continuity
        report=odom_continuity.analyze(e.json('evidence/flight-transport/odom.jsonl',True),
                                       e.json('evidence/flight-transport/flight.jsonl',True))
        metrics['transport_odometry_continuity']=report
        if report['status']!='PASS':
            first=report['gaps'][0] if report['gaps'] else None
            raise ValueError('Odometry gap > %d ns during %s: %s'%(report['max_allowed_gap_ns'],
                first and first['phase'],first or 'duplicate acquisition stamps'))
    check('transport_odometry_continuity',transport_continuity)
    if source:
        config=source['config'];window=source['window']
        context['vertical_aid']=config.get('vertical_aid','UNKNOWN')
        def accuracy():
            nonlocal plot_data
            poses=e.json('evidence/flight-source/poses.jsonl',True)
            ordered(poses,'t')
            for r in window:
                if 'state' not in r:continue
                if len(r['state'])!=9:raise ValueError('Invalid VIO state dimension')
                for value in r['state']:number(value)
                cov=np.asarray(r['covariance'],float)
                if cov.shape!=(9,9):raise ValueError('Invalid state covariance dimensions')
                validate_covariance(cov.ravel().tolist(),9)
            for r in poses:
                matched=[p for p in r['poses'] if p['name']=='iris_with_gimbal']
                if len(matched)!=1:raise Missing('Missing/ambiguous Gazebo body GT pose')
                xyz=matched[0]['p']
                if len(xyz)!=3:raise ValueError('Invalid GT position')
                for value in xyz:number(value)
            result=evaluate(window,poses,number(config['ground_z']))
            if not isinstance(result,tuple):raise Missing(result['reason'])
            values,plot_data=result
            if not values['continuous_epoch']:raise ValueError('Trajectory contains reset epochs')
            metrics['trajectory']=values
            # Existing preflight_ekf3 motion accuracy/coverage limits. No XY
            # movement minimum: vertical TAKEOFF/HOVER is a different scenario.
            limits=dict(xy_rmse_m=.2,velocity_xy_rmse_m_s=.15,max_spatial_error_m=.4)
            if values['coverage']<.98 or any(values[k]>v for k,v in limits.items()):raise ValueError('Existing trajectory accuracy/coverage criteria failed')
            metrics['trajectory_criteria']=dict(limits,min_state_coverage=.98)
        check('trajectory_accuracy',accuracy)
        if plot_data is not None:
            def actual_motion():
                t,state,gt,gv,error,cov=plot_data
                hover=(t>=clock.phases['HOVER']['source_upper_ns']/1e9)&(t<=clock.phases['LAND']['source_lower_ns']/1e9)
                if np.count_nonzero(hover)<2:raise Missing('No bracketed GT hover interval')
                drift=error[hover,:2]-error[hover,:2][0]
                metrics['hover']=dict(samples=int(hover.sum()),max_xy_error_change_m=float(np.linalg.norm(drift,axis=1).max()))
                height=-gt[:,2]
                if (height.max()>LIMITS['max_alt_m'] or height.max()<LIMITS['target_alt_m']-.15 or
                        np.max(np.abs(height[hover]-LIMITS['target_alt_m']))>.2 or height[-1]>.15 or
                        np.linalg.norm(gt[:,:2]-gt[0,:2],axis=1).max()>LIMITS['max_xy_m']):
                    raise ValueError('GT does not confirm takeoff/hover/landing within existing controller envelopes')
            check('actual_gazebo_flight_motion',actual_motion)
    else:checks['trajectory_accuracy']=verdict(NV,'No complete camera VIO/GT evaluation inputs')
    if 'actual_gazebo_flight_motion' not in checks:checks['actual_gazebo_flight_motion']=verdict(NV,'No verified full GT trajectory')
    def fusion():
        if clock is None or records is None or source is None:raise Missing('Need clock, live-camera provenance/continuity and raw BIN')
        wire=e.json('evidence/flight-transport/wire.jsonl',True)
        metrics['ekf3']=fcu_review(records,wire,clock,source)
    check('externalnav_fusion',fusion)
    check('evidence_integrity',e.unchanged)
    result=dict(schema=1,status=overall(checks),checks=checks,metrics=metrics,context=context,
        manifest=e.manifest,analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Independent recorded Gazebo camera flight analysis; GT Z aided; not real hardware autonomy or flight authorization',
        decision='HOLD',execution_allowed=False,forwarding_enabled=False,
        capture_to_fcu_latency=NV+'; no verified wall-time exposure witness',
        covariance_calibration=NV+'; empirical diagnostics do not prove statistical calibration')
    result['units']=dict(source_time='Gazebo simulation seconds',fcu_time='FCU boot microseconds',host_time='Linux CLOCK_MONOTONIC nanoseconds')
    result['analysis_sources']={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                for name in ('analyze_phase1_run.py','analyze_ekf3_fusion.py','physics_clock_mapping.py')}
    return result,dict(plot_data=plot_data,clock=clock,records=records,source=source,evidence=e)


def plots(output,result,data):
    """Only real input arrays; unavailable panels are omitted, never filled in."""
    clock=data['clock'];arrays=data['plot_data'];created=[]
    if clock is None:return created
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    def decorate(ax,origin):
        for name,phase in clock.phases.items():
            if name=='DONE':continue
            left,right=phase['source_lower_ns']/1e9,phase['source_upper_ns']/1e9
            ax.axvspan(left-origin,right-origin,color='grey',alpha=.2)
            ax.axvline((left+right)/2-origin,ls=':',label={'CLIMB':'TAKEOFF'}.get(name,name))
    if arrays is not None:
        plot_trajectory(arrays,output/'position_velocity_error.png',dimensions=3,
            title='Recorded flight: N/E/D position, velocity and error; GT Z aid',xlabel='simulation seconds',decorate=decorate,velocity_error=True)
        created.append('position_velocity_error.png')
        t,state,gt,gv,error,cov=arrays
        fig,axes=plt.subplots(1,3,figsize=(14,4))
        axes[0].plot(state[:,1]-state[0,1],state[:,0]-state[0,0],label='VIO')
        axes[0].plot(gt[:,1]-gt[0,1],gt[:,0]-gt[0,0],label='Gazebo GT');axes[0].set(xlabel='East m',ylabel='North m');axes[0].axis('equal')
        for name,phase in clock.phases.items():
            stamp=(phase['source_lower_ns']+phase['source_upper_ns'])/2e9
            index=int(np.searchsorted(t,stamp).clip(0,len(t)-1))
            axes[0].scatter(state[index,1]-state[0,1],state[index,0]-state[0,0],label={'CLIMB':'TAKEOFF'}.get(name,name))
        axes[1].plot(t-t[0],-state[:,2],label='VIO height');axes[1].plot(t-t[0],-gt[:,2],label='GT height');axes[1].set_ylabel('height above ground m');decorate(axes[1],t[0])
        hover=(t>=clock.phases['HOVER']['source_upper_ns']/1e9)&(t<=clock.phases['LAND']['source_lower_ns']/1e9)
        if np.count_nonzero(hover)>=2:
            drift=error[hover,:2]-error[hover,:2][0]
            axes[2].plot(t[hover]-t[0],np.linalg.norm(drift,axis=1),label='XY error change from hover start')
        else:axes[2].text(.05,.5,'NOT VERIFIED: insufficient hover data',transform=axes[2].transAxes)
        axes[2].set_ylabel('hover drift m');decorate(axes[2],t[0])
        for ax in axes:ax.grid();ax.legend()
        fig.tight_layout();fig.savefig(output/'trajectory_altitude_hover.png');plt.close(fig);created.append('trajectory_altitude_hover.png')
    if data['source'] and data['source']['window']:
        rows=data['source']['window'];fig,ax=plt.subplots(figsize=(10,3))
        ax.step([r['t']-clock.start for r in rows],[int(bool(r.get('published'))) for r in rows],where='post',label='Published valid VIO')
        for left,right in zip(rows,rows[1:]):
            if right['t']-left['t']>.200000001:
                ax.axvspan(left['t']-clock.start,right['t']-clock.start,color='red',alpha=.3,label='No source samples / gap')
        ax.set(xlabel='simulation seconds',ylabel='availability',yticks=[0,1]);decorate(ax,clock.start);ax.legend();fig.tight_layout();fig.savefig(output/'vio_availability.png');plt.close(fig);created.append('vio_availability.png')
    rows=data['records']
    if rows:
        fig,axes=plt.subplots(4,1,figsize=(10,10),sharex=True);count=0
        for core in (0,1):
            for typ,axis,keys in [('XKF3',0,('IVN','IVE','IPN','IPE')),('XKF4',1,('SP','SV')),('XKF4',2,('FS','TS','SS')),('XKFS',3,('SS',))]:
                subset=[r for r in rows if r['mavpackettype']==typ and r.get('C')==core and clock.fcu_start<=r['TimeUS']<=clock.fcu_end]
                if not subset:continue
                times=[clock.fcu_seconds(r['TimeUS'])-clock.start for r in subset]
                for key in keys:axes[axis].plot(times,[number(r[key]) for r in subset],label=f'{key} core{core}');count+=1
        if count:
            for ax in axes:decorate(ax,clock.start);ax.grid();ax.legend(ncol=3)
            for r in rows:
                if r['mavpackettype']=='MSG' and clock.fcu_start<=r['TimeUS']<=clock.fcu_end and 'aiding' in r['Message']:
                    axes[3].axvline(clock.fcu_seconds(r['TimeUS'])-clock.start,color='red',label=r['Message'])
            axes[0].set_ylabel('velocity m/s; position m');axes[1].set_ylabel('innovation test ratios');axes[2].set_ylabel('status bitfields')
            axes[3].set(xlabel='simulation seconds (bounded FCU mapping)',ylabel='active source set index')
            fig.tight_layout();fig.savefig(output/'ekf3_innovations_status.png');created.append('ekf3_innovations_status.png')
        plt.close(fig)
    return created


def write_report(result,output):
    (output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# Simulation flight post-analysis',f"\n**{result['status']}** — {result['scope']}",
        '\n| Check | Verdict | Finding |','|---|---|---|']
    for name,check in result['checks'].items():lines.append('| '+name+' | '+check['status']+' | '+check['reason'].replace('|','/').replace('\n',' ')+' |')
    trajectory=result['metrics'].get('trajectory',{})
    for key in ('xy_rmse_m','velocity_xy_rmse_m_s','max_spatial_error_m','final_xy_drift_m','coverage'):
        if key in trajectory:lines.append(f'\n- {key}: {trajectory[key]:.6g}')
    lines+=['\nClock/phase details and SHA256 input manifest: `report.json`.',
        '\nGT height is a declared estimator aid. Missing plots mean missing evidence, not zero error.',
        '\nCapture-to-FCU wall latency and covariance calibration: NOT VERIFIED.',
        '\nPhase 1 HOLD. execution_allowed=false; forwarding_enabled=false. No runtime commands executed.']
    for name in result.get('plots',[]):lines.append(f'\n![{name}]({name})')
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    p.add_argument('--bin',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run=a.run.resolve();output=a.output.resolve()
    if any(output.is_relative_to(run/name) for name in ('evidence','artifacts','state')):p.error('Output must not modify captured evidence/state/artifacts')
    output.mkdir(parents=True,exist_ok=False)
    result,data=analyze(run,a.bin)
    try:result['plots']=plots(output,result,data)
    except (ValueError,KeyError,TypeError,IndexError,ImportError,OSError,ArithmeticError,RuntimeError) as exc:
        result['checks']['plot_generation']=verdict(NV,str(exc));result['status']=overall(result['checks'])
    try:data['evidence'].unchanged()
    except (OSError,ValueError) as exc:
        result['checks']['evidence_integrity']=verdict('FAIL',str(exc));result['status']=overall(result['checks'])
    write_report(result,output)
    print(json.dumps(dict(status=result['status'],report=str(output/'REPORT.md'))))
    return {'PASS':0,'FAIL':1,NV:2}[result['status']]


if __name__=='__main__':raise SystemExit(main())
