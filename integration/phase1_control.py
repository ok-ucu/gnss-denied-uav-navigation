"""Only pause/resume/reset of our NEW named partition. Never a flight command."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('action',choices=['pause','resume','reset'])
p.add_argument('--output',type=Path,required=True)
p.add_argument('--expected-partition',required=True)
a=p.parse_args()
if not a.expected_partition.startswith('gnss-p1-integrated-') or os.environ.get('GZ_PARTITION')!=a.expected_partition:
    raise RuntimeError('Refuse other partitions')
payload='reset: {all: true}' if a.action=='reset' else 'pause: '+str(a.action=='pause').lower()
command=['gz','service','-s','/world/iris_runway/control','--reqtype','gz.msgs.WorldControl',
         '--reptype','gz.msgs.Boolean','--timeout','5000','--req',payload]
start=time.monotonic_ns();r=subprocess.run(command,text=True,capture_output=True,timeout=8)
record=dict(action=a.action,command=command,start_host_ns=start,end_host_ns=time.monotonic_ns(),
            stdout=r.stdout,stderr=r.stderr,returncode=r.returncode,success=r.returncode==0 and 'data: true' in r.stdout)
with a.output.open('x') as out:json.dump(record,out,indent=2)
if not record['success']:raise SystemExit(2)
