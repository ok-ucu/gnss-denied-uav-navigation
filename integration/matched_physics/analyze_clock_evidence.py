"""Review existing clock/pause records, without controlling any simulator."""
import json
from pathlib import Path


def progress(rows,lo,hi):
    rows=[r for r in rows if 'source_ns' in r]
    if not rows:return dict(count=0,progress_s=None)
    origin=rows[0]['host_ns']
    window=[r for r in rows if lo<=(r['host_ns']-origin)/1e9<hi]
    return dict(count=len(window),progress_s=(window[-1]['source_ns']-window[0]['source_ns'])/1e9 if len(window)>1 else None,
                time_basis='relative to each independent probe start; approximate control-event window')


def review(directory):
    p=Path(directory)
    read=lambda name:[json.loads(s) for s in (p/name).read_text().splitlines()]
    gz,fcu=read('gazebo_pause_v3.jsonl'),read('fcu_pause_v3.jsonl')
    return dict(clock_mapping='HOLD',uncertainty_bound_verified=False,
                pause_resume_scope='Previously recorded pause/resume response; no new pause issued',
                gazebo={f'{lo}-{hi}s':progress(gz,lo,hi) for lo,hi in [(0,6),(6,9),(9,20)]},
                fcu={f'{lo}-{hi}s':progress(fcu,lo,hi) for lo,hi in [(0,6),(6,9),(9,20)]},
                execution_allowed=False,forwarding_enabled=False,
                limit='Separate probe starts, unknown transport delays, no exact common control-event markers; not a bounded acquisition mapping')
