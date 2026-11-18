"""Offline ZIP of an explicit evidence allowlist; never runs/approves a test.

Provided PASS/FAIL/BLOCKED claims remain unverified. No directory recursion,
symlinks, source modification, overwrite, network access or runtime actions.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import zipfile

ROOT=Path(__file__).resolve().parent
STATUSES={'PASS','FAIL','BLOCKED','NOT TESTED'}


def evidence_path(root,name):
    p=PurePosixPath(name)
    if (not name or p.is_absolute() or str(p)!=name or '\\' in name or ':' in name
            or any(part in ('..','.git','.ssh','.env','id_rsa','id_ed25519','_bundle') for part in p.parts)):
        raise ValueError('Unsafe/reserved evidence path: '+name)
    target=root
    for part in p.parts:
        target=target/part
        if target.is_symlink():raise ValueError('Symlink evidence forbidden: '+name)
    if not target.is_file() or not target.resolve().is_relative_to(root):
        raise ValueError('Missing/nonregular evidence: '+name)
    return target


def normalized_summary(value,selected):
    if value.get('decision','HOLD')!='HOLD' or any(value.get(k,False) is not False
            for k in ('execution_allowed','forwarding_enabled','runtime_admitted')):
        raise ValueError('Evidence bundle cannot enable/approve execution')
    if not re.fullmatch(r'[0-9a-f]{40}',value.get('source_commit','')):
        raise ValueError('Require full source commit SHA')
    if type(value.get('dirty')) is not bool:raise ValueError('Require explicit dirty boolean')
    if value.get('commands') not in selected:raise ValueError('Command log must be selected')
    if value['dirty'] and value.get('changes') not in selected:
        raise ValueError('Dirty source requires selected CHANGES patch')
    stages=value.get('stages',[])
    if not stages:raise ValueError('Require stage summaries')
    names=set();normalized=[]
    for row in stages:
        name=row.get('name');status=row.get('status','').replace('_',' ')
        evidence=row.get('evidence',[])
        if not isinstance(name,str) or not name or name in names or status not in STATUSES:
            raise ValueError('Invalid/duplicate stage name or status')
        if not isinstance(evidence,list) or any(p not in selected for p in evidence):
            raise ValueError('Stage evidence must be in allowlist: '+name)
        if status=='PASS' and not evidence:raise ValueError('PASS needs cited evidence: '+name)
        if status!='PASS' and not row.get('note'):raise ValueError('Non-PASS needs a reason: '+name)
        names.add(name);normalized.append(dict(row,status=status))
    return dict(value,stages=sorted(normalized,key=lambda r:r['name']),decision='HOLD',
        execution_allowed=False,forwarding_enabled=False,runtime_admitted=False,
        claims_verified=False,scope='Provided evidence/claims; not independently verified runtime results')


def package(root,names,summary,output,max_bytes=256*1024*1024):
    root=Path(root).resolve();output=Path(output)
    if output.exists():raise FileExistsError(output)
    if max_bytes<=0 or not names or len(names)!=len(set(names)):
        raise ValueError('Require unique nonempty allowlist and positive size bound')
    selected=set(names)
    report=normalized_summary(summary,selected)
    payloads={};manifest=[];snapshots={};total=0
    for name in sorted(names):
        path=evidence_path(root,name);before=path.stat();total+=before.st_size
        if total>max_bytes:raise ValueError('Evidence byte budget exceeded; review selection')
        # Bound the read too, even if a collector appends after the initial stat.
        with path.open('rb') as source:data=source.read(before.st_size+1)
        after=path.stat()
        snapshot=(before.st_size,before.st_mtime_ns)
        if snapshot!=(after.st_size,after.st_mtime_ns) or len(data)!=before.st_size:
            raise ValueError('Evidence changed while reading: '+name)
        snapshots[name]=snapshot;payloads[name]=data
        manifest.append(dict(path=name,bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
    for name,snapshot in snapshots.items():
        current=evidence_path(root,name).stat()
        if (current.st_size,current.st_mtime_ns)!=snapshot:
            raise ValueError('Evidence changed during collection: '+name)
    manifest=dict(files=manifest,total_bytes=total,claims_verified=False,
        decision='HOLD',execution_allowed=False,forwarding_enabled=False,
        note='Hashes identify packaged bytes, not authenticity/completeness or runtime success')
    payloads['_bundle/summary.json']=(json.dumps(report,indent=2,sort_keys=True)+'\n').encode()
    payloads['_bundle/manifest.json']=(json.dumps(manifest,indent=2,sort_keys=True)+'\n').encode()
    with zipfile.ZipFile(output,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for name,data in sorted(payloads.items()):
            info=zipfile.ZipInfo(name,date_time=(1980,1,1,0,0,0))
            info.create_system=3;info.external_attr=0o100644<<16
            info.compress_type=zipfile.ZIP_DEFLATED
            archive.writestr(info,data,compress_type=zipfile.ZIP_DEFLATED,compresslevel=6)
    return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--files',type=Path,required=True,help='Explicit root-relative file names, one per line')
    p.add_argument('--summary',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not any(a.output.resolve().is_relative_to(ROOT/name) for name in ('linux_gpu_runs','source_scope_audit')):
        raise ValueError('ZIP output must be in an ignored integration runtime directory')
    result=package(a.root,[s for s in a.files.read_text().splitlines() if s],
        json.loads(a.summary.read_text()),a.output)
    print(json.dumps(dict(output=str(a.output),files=len(result['files']),
        bytes=result['total_bytes'],decision='HOLD',claims_verified=False)))


if __name__=='__main__':main()
