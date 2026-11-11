"""Export pinned teammate dependency to a NEW ignored directory, never vio/."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

PIN='d33f66008fd7b6de332c96dd324b8ddbd320c194'
FILES={
    'vio_core.py':'1bddfbbcf12fad27bf5c769a1718f11d9592f2bee54bdb238781b11f5dc40efa',
    'vio_frontend.py':'db69d2c5a11ce2ad20cc3f94d78f35aa2b07b6755931372b7e5f3fc0a3d11682',
    'vio_ros_out.py':'2356fdb2e75652e08fcefdeaf97cc6cb9b79ade710117cc9aeaaea94cb35d3b3',
    'vio_ros_source.py':'a7957f2c8973227cd0754a549d1c9ad403e1c3f7a49e24aa1a092d2d4d984bbe',
}

def export(output):
    root=Path(__file__).resolve().parents[2]
    output=Path(output).resolve()
    if not output.is_relative_to(root/'integration/linux_gpu_runs') or output.exists():
        raise ValueError('Require NEW output below integration/linux_gpu_runs')
    data={name:subprocess.check_output(['git','show',PIN+':vio/'+name],cwd=root) for name in FILES}
    for name,content in data.items():
        if hashlib.sha256(content).hexdigest()!=FILES[name]:raise ValueError('VIO source hash mismatch')
    output.mkdir(parents=True,exist_ok=False)
    for name,content in data.items():(output/name).write_bytes(content)
    (output/'DEPENDENCY.json').write_text(json.dumps(dict(commit=PIN,files=FILES),indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    export(p.parse_args().output)
