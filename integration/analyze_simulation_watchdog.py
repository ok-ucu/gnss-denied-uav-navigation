"""Read-only JSONL evidence audit. Does not run Gazebo or authorize flight.

Usage: python3 integration/analyze_simulation_watchdog.py integration/phase1_runs/RUN --output report.json
"""
import argparse
import json
from pathlib import Path
from statistics import median


def rows(path):
    if not path.exists(): return []
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def gaps(items, key, scale=1):
    times=[r[key]/scale for r in items if isinstance(r.get(key),(int,float))]
    return [b-a for a,b in zip(times,times[1:]) if b>=a]


def summary(run):
    run=Path(run)
    sources=sorted((run/'evidence').glob('*/stream.jsonl'))
    source=next((s for s in sources if 'flight-source' in str(s)), sources[0] if sources else None)
    sample=rows(source) if source else []
    sampled=[r for r in sample if isinstance(r.get('t'),(int,float)) and isinstance(r.get('wall'),(int,float))]
    publications=[r for r in sampled if r.get('published') is True]
    rtf=None
    if len(sampled)>1:
        dw=sampled[-1]['wall']-sampled[0]['wall']
        if dw>0:rtf=(sampled[-1]['t']-sampled[0]['t'])/dw
    publish_gaps=gaps(publications,'wall')
    output={'run':run.name,'source':str(source) if source else None,
            'sample_count':len(sampled),'vio_publications':len(publications),
            'observed_average_rtf':rtf,'max_vio_publish_gap_host_s':max(publish_gaps,default=None),
            'median_vio_publish_gap_host_s':median(publish_gaps) if publish_gaps else None,
            'simulation_rate_attribution':'historical aggregate; not a controlled RTF setpoint'}
    transport=next((p for p in sorted((run/'evidence').glob('*/odom.jsonl')) if 'transport' in str(p)),None)
    if transport:
        odom=rows(transport)
        gs=gaps(odom,'callback_entry_host_ns',1e9)
        output.update(transport_odom_count=len(odom),max_transport_odom_gap_host_s=max(gs,default=None))
    for pattern, field in [('*/events.jsonl','watchdog_event_counts'),('*/wire.jsonl','wire_packet_count')]:
        paths=sorted((run/'evidence').glob(pattern))
        if field=='watchdog_event_counts':
            matches=[]
            for path in paths:
                for row in rows(path):
                    if any(word in str(row).lower() for word in ('watchdog','clock lost','stale','paused','expired')):
                        matches.append({'file':str(path),'event':row.get('event'),'reason':row.get('reason')})
            output[field]={'count':len(matches),'examples':matches[:12]}
        else:output[field]=sum(len(rows(path)) for path in paths)
    output['camera_hz']='not established from this VIO stream alone'
    output['imu_hz']='not established from this VIO stream alone'
    output['shadows_verified_live']=False
    output['flight_authorized']=False
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    result=summary(a.run);render=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if a.output:a.output.write_text(render)
    print(render,end='')


if __name__=='__main__':main()
