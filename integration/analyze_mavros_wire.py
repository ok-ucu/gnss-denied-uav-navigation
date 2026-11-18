"""Compare actual isolated MAVROS vision packets against real ROS inputs."""
import argparse
import json
import importlib.util
from collections import Counter
import math
from pathlib import Path


def expected(row):
    import numpy as np
    c=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    v=row['values'];cov=np.reshape(row['covariance'],(6,6))
    if row['kind']=='speed':
        return dict(speed=(c@v[:3]).tolist(),covariance=(c@cov[:3,:3]@c.T).ravel().tolist())
    j=np.zeros((6,6));j[:3,:3]=c;j[3:,3:]=c
    pc=j@cov@j.T;x,y,z,w=v[3:]
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    r=c@r@np.diag([1,-1,-1])
    euler=[math.atan2(r[2,1],r[2,2]),math.asin(max(-1,min(1,-r[2,0]))),math.atan2(r[1,0],r[0,0])]
    return dict(position=(c@v[:3]).tolist(),rpy=euler,covariance=[pc[i,k] for i in range(6) for k in range(i,6)])


def analyze(root):
    import numpy as np
    def read(p):return [json.loads(s) for s in p.read_text().splitlines()]
    ingress=read(root/'ingress.jsonl');packets=read(root/'wire.jsonl')
    rows=[r for r in ingress if r.get('published') and r['kind']!='clock']
    checks={};errors=[]
    clocks=[r for r in ingress if r.get('published') and r['kind']=='clock']
    source=root/'source_trial/source/stream'
    camera={round(r['t']*1e6):r for r in read(source/'ros_camera.jsonl')} if (source/'ros_camera.jsonl').exists() else {}
    epochs=read(root/'source_trial/adapter/adapter.jsonl') if (root/'source_trial/adapter/adapter.jsonl').exists() else []
    reset=min((r['stamp_ns']//1000 for r in epochs if r.get('event')=='epoch_token' and r['epoch']>0),default=None)
    def stats(v):return dict(mean=float(np.mean(v)),p95=float(np.percentile(v,95)),max=float(max(v))) if v else None
    for kind,typ in [('pose','VISION_POSITION_ESTIMATE'),('speed','VISION_SPEED_ESTIMATE')]:
        inputs={r['stamp_ns']//1000:r for r in rows if r['kind']==kind}
        got=[r for r in packets if r['message']['mavpackettype']==typ]
        times=[r['message']['usec'] for r in got];arrivals=[r['arrival_wall'] for r in got]
        for r in got:
            m=r['message'];t=m['usec']
            if t not in inputs:errors.append(dict(type=typ,reason='unmatched usec',stamp=t));continue
            want=expected(inputs[t]);good=r['mavlink2']
            keys=['x','y','z'];actual=[m[k] for k in keys]
            good=good and np.allclose(actual,want.get('position',want.get('speed')),rtol=1e-5,atol=1e-6)
            good=good and len(m['covariance'])==(21 if kind=='pose' else 9) and np.allclose(m['covariance'],want['covariance'],rtol=1e-5,atol=1e-7)
            if kind=='pose':good=good and all(abs(math.atan2(math.sin(m[k]-v),math.cos(m[k]-v)))<1e-5 for k,v in zip(['roll','pitch','yaw'],want['rpy']))
            if not good:errors.append(dict(type=typ,reason='payload/covariance/protocol',stamp=t))
        unique=set(times)
        ages=[]
        for r in got:
            previous=[c for c in clocks if c['ingress_wall']<=r['arrival_wall']]
            if previous:ages.append(previous[-1]['stamp_ns']/1e9-r['message']['usec']/1e6)
        checks[kind]=dict(input_count=len(inputs),packet_count=len(got),unique=len(unique),duplicates=len(times)-len(unique),
            missing_input_stamps=sorted(set(inputs)-unique),delivery_ratio=len(unique&inputs.keys())/len(inputs) if inputs else None,
            sim_hz=(len(times)-1)/((times[-1]-times[0])/1e6) if len(times)>1 else None,
            arrival_wall_hz=(len(times)-1)/(arrivals[-1]-arrivals[0]) if len(times)>1 else None,
            max_sender_gap_s=max(np.diff(times),default=0)/1e6,max_arrival_gap_s=float(max(np.diff(arrivals),default=0)),
            receiver_age_relayed_sim_clock_s=stats(ages),
            ros_camera_receipt_to_wire_wall_s=stats([r['arrival_wall']-camera[r['message']['usec']]['arrival_monotonic_s'] for r in got if r['message']['usec'] in camera]),
            post_reset_packets=sum(t>=reset for t in times) if reset is not None else None,
            reset_counters=sorted({r['message'].get('reset_counter') for r in got}),
            ingress_to_wire_wall_s=stats([r['arrival_wall']-inputs[r['message']['usec']]['ingress_wall'] for r in got if r['message']['usec'] in inputs]),
            export_to_wire_wall_s=stats([r['arrival_wall']-inputs[r['message']['usec']]['export_wall'] for r in got if r['message']['usec'] in inputs]))
    spec=importlib.util.spec_from_file_location('capture_wire',root.parent/'mavlink_v20.py')
    wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(wire)
    decoder=wire.MAVLink(None);decoder.robust_parsing=True
    raw_messages=decoder.parse_buffer((root/'wire.mavlink').read_bytes()) or []
    raw_counts=Counter(m.get_type() for m in raw_messages)
    json_counts=Counter(r['message']['mavpackettype'] for r in packets)
    raw_complete=sum(json_counts[t] for t in ['VISION_POSITION_ESTIMATE','VISION_SPEED_ESTIMATE'])>0 and all(raw_counts[t]==json_counts[t] for t in ['VISION_POSITION_ESTIMATE','VISION_SPEED_ESTIMATE'])
    source_publications=read(source/'publications.jsonl') if (source/'publications.jsonl').exists() else []
    result=dict(upstream_source_count=len(source_publications),
                guard_rejections=[r for r in epochs if r.get('event')=='rejected'],
                source_to_wire_fraction={k:checks[k]['packet_count']/len(source_publications) if source_publications else None for k in checks},
                raw_capture=dict(complete_vision=raw_complete,raw_counts=raw_counts,json_counts=json_counts),channels=checks,errors=errors,ingress_rejected=[r for r in ingress if r.get('rejected')],
                actual_mavros='2.16.0, real ROS and real loopback MAVLink; NOT ArduPilot fusion',
                latency_note='Monotonic wall receipt endpoints; acquisition is simulation header time, not wall capture time')
    (root/'wire_metrics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);analyze(p.parse_args().directory)
