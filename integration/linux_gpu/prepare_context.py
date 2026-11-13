"""Create a NEW tiny build context; never send historical runs to Docker build."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

INPUTS={
    'camera':['linux_gpu/Camera.Dockerfile','linux_gpu/native_parts_render.cc','linux_gpu/build_native_probe.sh'],
    'sitl':['linux_gpu/SITL.Dockerfile','isolated_blockers/20261109_a/evidence/instrumentation.patch',
            'isolated_blockers/20261109_a/code/AP444_BOOLEAN_EXACT.patch',
            'isolated_blockers/20261109_a/code/test_sim_json.cpp'],
    'mavros':['linux_gpu/MAVROS.Dockerfile','mavros_time_patch/src/sys_time.cpp',
              'mavros_time_patch/CMakeLists.txt','mavros_time_patch/package.xml',
              'mavros_time_patch/plugins.xml','mavros_time_patch/LICENSE.md'],
}

def prepare(kind,output):
    root=Path(__file__).resolve().parents[1]
    output=Path(output).resolve()
    if not output.is_relative_to(root/'linux_gpu_runs') or output.exists():
        raise ValueError('Require a NEW context under integration/linux_gpu_runs')
    files={name:root/name for name in INPUTS[kind]}
    if any(not path.is_file() or path.is_symlink() for path in files.values()):raise ValueError('Missing/nonregular source')
    output.mkdir(parents=True,exist_ok=False)
    hashes={}
    for name,path in files.items():
        dest=output/('Dockerfile' if path.name.endswith('.Dockerfile') else name)
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(path,dest)
        hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest()
    (output/'SOURCE_HASHES.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(output)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind',choices=sorted(INPUTS),required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();prepare(args.kind,args.output)
