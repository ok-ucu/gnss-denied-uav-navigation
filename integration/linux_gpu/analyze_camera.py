"""Offline statistics for the two bounded camera-only captures."""
import collections
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def quantiles(values):
    return dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100])))) if len(values) else None

def cpu_stat(text):
    return {line.split()[0]:int(line.split()[1]) for line in text.splitlines()}

def union(intervals):
    end=-1
    total=0
    for start,stop in sorted(intervals):
        total+=max(0,stop-max(start,end))
        end=max(end,stop)
    return total

def analyze_capture(root):
    root=Path(root)
    capture=json.loads((root/'camera_profile.json').read_text())
    lo,hi=capture['start_ns'],capture['end_ns']
    rows=[]
    for line in (root/'phase1_camera_boundary_1.csv').read_text().splitlines():
        f=line.split(',')
        if len(f)==8:
            rows.append((f[0],*map(int,f[1:])))
    groups=collections.defaultdict(list)
    for row in rows:
        if lo<=row[2]<hi:groups[row[0]].append(row)
    native={kind:dict(count=len(data),wall_ms=quantiles([(r[3]-r[2])/1e6 for r in data]),
                           thread_cpu_ms=quantiles([r[4]/1e6 for r in data])) for kind,data in groups.items()}
    fractions=[]
    for render in groups['k']:
        children=[r for r in rows if r[0]=='X' and render[2]<=r[2]<r[3]<=render[3]]
        fractions.append(union([(r[2],r[3]) for r in children])/(render[3]-render[2]))
    resources=capture['resources']
    first,last=resources[0],resources[-1]
    delta={k:cpu_stat(last['cpu'])[k]-v for k,v in cpu_stat(first['cpu']).items()}
    threads=[]
    for thread in first['tasks']:
        later=next((t for t in last['tasks'] if t['tid']==thread['tid']),None)
        if later and thread['sched'] and later['sched']:
            d=[int(b)-int(a) for a,b in zip(thread['sched'].split(),later['sched'].split())]
            threads.append(dict(tid=thread['tid'],name=thread['name'],runtime_s=d[0]/1e9,runqueue_s=d[1]/1e9))
    perf=collections.Counter()
    names=collections.Counter()
    render_leaves=collections.Counter()
    render_tid=groups['k'][0][5]
    stack_file=root/'perf-stacks.txt'
    for block in (stack_file.read_text() if stack_file.exists() else '').split('\n\n'):
        lines=block.strip().splitlines()
        if len(lines)<2 or 'cpu-clock:u:' not in lines[0]:continue
        header=lines[0].split()
        module=lines[1].rsplit(' (',1)[-1].rstrip(')')
        perf[module]+=1
        names[header[0]]+=1
        if int(header[1])==render_tid:render_leaves[module]+=1
    images=capture['images']
    return dict(duration_s=(hi-lo)/1e9,native=native,frames=len(images),
                wall_hz=len(images)/((hi-lo)/1e9),
                sim_hz=(len(images)-1)/((images[-1]['sim_ns']-images[0]['sim_ns'])/1e9),
                image_formats=list({(i['width'],i['height'],i['pixel_format'],i['bytes']) for i in images}),
                arrival_gaps_ms=quantiles(np.diff([i['host_ns'] for i in images])/1e6),
                publication_gaps_ms=quantiles(np.diff([i[2] for i in groups['P']])/1e6),
                sim_intervals_ns=dict(collections.Counter(map(str,np.diff([i['sim_ns'] for i in images])))),
                scene_pass_union_fraction=quantiles(fractions),
                cgroup_delta=delta,resources_duration_s=(last['host_ns']-first['host_ns'])/1e9,
                llvmpipe_workers=len([t for t in first['tasks'] if t['name'].startswith('llvmpipe-')]),
                render_tid=render_tid,threads=threads,perf_status='MEASURED' if stack_file.exists() else 'NOT TESTED',perf_samples=sum(perf.values()),
                perf_leaf_modules=perf,perf_thread_names=names,render_thread_leaf_modules=render_leaves)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=analyze_capture(args.capture)
    with args.output.open('x') as output:
        json.dump(result,output,indent=2,allow_nan=False)
        output.write('\n')
    print(json.dumps(result,indent=2,allow_nan=False))
