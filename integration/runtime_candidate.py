"""Verify exported native build artifacts for PREPARATION ONLY, never admission.

Matching source hashes, ELF architecture and tests describe a candidate. They do
not prove clock behavior and cannot add a binary to AppliedClockMapping's pin.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct

ROOT=Path(__file__).resolve().parent
AP_COMMIT='9279df6191a772c098b2e4797c95ac954f258d88'
MACHINES={'amd64':62,'arm64':183}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def elf_arch(path):
    with path.open('rb') as source:
        header=source.read(20)
    if len(header)!=20 or header[:6]!=b'\x7fELF\x02\x01':
        raise ValueError('Require ELF64 little-endian: '+str(path))
    machine=struct.unpack_from('<H',header,18)[0]
    for arch,expected in MACHINES.items():
        if machine==expected:return arch
    raise ValueError('Unsupported ELF machine: '+str(machine))


def image_id(value):
    if not re.fullmatch(r'sha256:[0-9a-f]{64}',value):
        raise ValueError('Require immutable local image ID, not tag: '+value)
    return value


def context_matches(directory,kind):
    from linux_gpu.prepare_context import INPUTS
    recorded=json.loads((directory/'source-hashes.json').read_text())
    expected={name:sha(ROOT/name) for name in INPUTS[kind]}
    if recorded!=expected:raise ValueError('Build context/source mismatch: '+kind)


def verify_builds(firmware,record,overlay,arch):
    if arch not in MACHINES:raise ValueError('Unsupported architecture')
    binary=firmware/'arducopter'
    if elf_arch(binary)!=arch:raise ValueError('Firmware architecture mismatch')
    if (firmware/'ardupilot-commit.txt').read_text().strip()!=AP_COMMIT:
        raise ValueError('Unpinned ArduPilot source')
    expected_machine={'amd64':'x86_64','arm64':'aarch64'}[arch]
    for directory in (firmware,record):
        if (directory/'architecture.txt').read_text().strip()!=expected_machine:
            raise ValueError('Build architecture record mismatch')
        for name in ('packages.txt','compiler.txt'):
            if not (directory/name).read_text().strip():raise ValueError('Missing build record '+name)
    for directory,repository in ((firmware,'ubuntu'),(record,'ros')):
        if not re.fullmatch(repository+r'@sha256:[0-9a-f]{64}',(directory/'base-image.txt').read_text().strip()):
            raise ValueError('Native candidate requires recorded immutable public base')
    # Check original actual-parser test output, not a claimed numeric count.
    parser=(firmware/'parser-tests.txt').read_text()
    tests=('ExactLiteralsAndWhitespace','LegacyDecimalIntegers','RejectMalformedTokens',
           'MissingOptionalFieldResetsToDefault','RejectMissingRequiredField')
    if '[  PASSED  ] 5 tests.' not in parser or '[  FAILED  ]' in parser or any(
        not re.search(r'\[\s*OK\s*\] JSONParser\.'+name+r'\b',parser) for name in tests):
        raise ValueError('Actual JSONParser5/5 evidence missing')
    if (firmware/'firmware.sha256').read_text().split()[0]!=sha(binary):
        raise ValueError('Firmware digest record mismatch')
    if not (firmware/'submodules.txt').read_text().strip() or not (firmware/'applied-source.patch').read_text().strip():
        raise ValueError('Missing upstream/submodule/source-diff evidence')
    context_matches(firmware,'sitl');context_matches(record,'mavros')
    patches={line.split()[1]:line.split()[0] for line in (firmware/'patches.sha256').read_text().splitlines()}
    required={'/tmp/instrumentation.patch':'isolated_blockers/20261109_a/evidence/instrumentation.patch',
              '/tmp/boolean.patch':'isolated_blockers/20261109_a/code/AP444_BOOLEAN_EXACT.patch',
              'libraries/SITL/tests/test_sim_json.cpp':'isolated_blockers/20261109_a/code/test_sim_json.cpp'}
    if any(patches.get(name)!=sha(ROOT/path) for name,path in required.items()):
        raise ValueError('Applied parser/instrumentation source mismatch')
    versions=json.loads((record/'ros-versions.json').read_text())
    if versions!={name:'2.16.0' for name in ('mavros','mavros_msgs','libmavconn')}:
        raise ValueError('Require actual MAVROS/libmavconn/messages2.16.0')
    if (record/'overlay-source.sha256').read_text().split()[0]!=sha(ROOT/'mavros_time_patch/src/sys_time.cpp'):
        raise ValueError('TIMESYNC overlay source mismatch')
    library=overlay/'lib/libgnss_mavros_time_patch.so'
    if elf_arch(library)!=arch:raise ValueError('Overlay ELF architecture mismatch')
    if not (overlay/'setup.bash').is_file():raise ValueError('Overlay setup missing')
    return binary,library


def load_candidate(path):
    path=Path(path).resolve()
    data=json.loads(path.read_text())
    if data.get('schema')!=1 or data.get('runtime_admitted') is not False or data.get('decision')!='HOLD':
        raise ValueError('Candidate cannot assert runtime approval')
    if data.get('execution_allowed') is not False or data.get('forwarding_enabled') is not False:
        raise ValueError('Safety flags must be false')
    for value in data['images'].values():image_id(value)
    if set(data['images'])!={'physics','fcu','transport'}:raise ValueError('Require three image identities')
    locations={name:(path.parent/data[name]).resolve() for name in ('firmware_dir','mavros_record_dir','overlay_dir')}
    if not all(v.is_relative_to(ROOT/'linux_gpu_runs') for v in locations.values()):
        raise ValueError('Candidate artifacts must remain under integration/linux_gpu_runs')
    binary,library=verify_builds(locations['firmware_dir'],locations['mavros_record_dir'],locations['overlay_dir'],data['architecture'])
    if data['firmware_sha256']!=sha(binary) or data['overlay_sha256']!=sha(library):
        raise ValueError('Exported candidate changed')
    return dict(data,firmware=binary,overlay=locations['overlay_dir'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--firmware-dir',type=Path,required=True)
    p.add_argument('--mavros-record-dir',type=Path,required=True)
    p.add_argument('--overlay-dir',type=Path,required=True)
    p.add_argument('--architecture',choices=sorted(MACHINES),required=True)
    for name in ('physics','fcu','transport'):p.add_argument('--'+name+'-image',type=image_id,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    output=a.output.resolve()
    if not output.is_relative_to(ROOT/'linux_gpu_runs') or output.exists():
        raise ValueError('Require NEW output under integration/linux_gpu_runs')
    firmware,record,overlay=[v.resolve() for v in (a.firmware_dir,a.mavros_record_dir,a.overlay_dir)]
    if not all(v.is_relative_to(ROOT/'linux_gpu_runs') for v in (firmware,record,overlay)):
        raise ValueError('Require owned exported artifacts')
    binary,library=verify_builds(firmware,record,overlay,a.architecture)
    import os
    data=dict(schema=1,architecture=a.architecture,images={name:getattr(a,name+'_image') for name in ('physics','fcu','transport')},
        firmware_dir=os.path.relpath(firmware,output.parent),mavros_record_dir=os.path.relpath(record,output.parent),
        overlay_dir=os.path.relpath(overlay,output.parent),firmware_sha256=sha(binary),overlay_sha256=sha(library),
        runtime_admitted=False,decision='HOLD',execution_allowed=False,forwarding_enabled=False,
        missing=['native applied-clock contract/cross-check admission','amd64 isolated TIMESYNC/reset results',
                 'source continuity','continuous capture-only transport'])
    with output.open('x') as result:json.dump(data,result,indent=2);result.write('\n')
    print(output)


if __name__=='__main__':main()
