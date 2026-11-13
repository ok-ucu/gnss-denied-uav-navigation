"""Read-only source allowlist audit. No staging, imports with side effects or Docker.

The allowlist is the proposed commit, not all untracked integration files.
Generated evidence is never rewritten. External runtime artifacts are reported
separately rather than silently counted as source-complete reproduction.
"""
import ast
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import importlib.util
import sysconfig

ROOT=Path(__file__).resolve().parents[1]
INTEGRATION=ROOT/'integration'
EXTERNAL={'numpy','cv2','matplotlib','rclpy','geometry_msgs','mavros_msgs','nav_msgs',
          'rosgraph_msgs','sensor_msgs','std_msgs','rosidl_runtime_py','gz',
          'vio_core','vio_frontend','vio_ros_out','vio_ros_source','fastcrc'}
RUNTIME_ONLY={
    'integration/isolated_blockers/20261109_a/evidence/arducopter_patched':'generated patched ARM64 firmware; never source',
    'integration/timesync_patch_runs/20261108/patch_install/setup.bash':'generated colcon overlay; never source',
}
EXPLICIT_LOCAL={'mount_math':'integration/isolated_blockers/20261109_a/code/mount_math.py'}


def standard_library(name):
    if name in getattr(sys,'stdlib_module_names',()) or name in sys.builtin_module_names:return True
    spec=importlib.util.find_spec(name)
    if spec is None:return False
    if spec.origin in ('built-in','frozen'):return True
    if not spec.origin:return False
    origin=Path(spec.origin).resolve()
    return origin.is_relative_to(Path(sysconfig.get_path('stdlib'))) and not any(
        origin.is_relative_to(Path(sysconfig.get_path(key))) for key in ('purelib','platlib'))


def main():
    paths=(INTEGRATION/'SOURCE_FILES.txt').read_text().splitlines()
    errors=[];external=set();absolute=[];secrets=[];syntax=0;xml=0
    if paths!=sorted(set(paths)):errors.append('Allowlist must be sorted and unique')
    selected=set(paths)
    modules={p.stem:p for p in INTEGRATION.glob('*.py')}
    modules.update({p.stem:p for p in (INTEGRATION/'tests').glob('*.py')})
    modules.update({p.stem:p for p in (INTEGRATION/'linux_gpu').glob('*.py')})
    secret_rules={
        'private-key':r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
        'github-token':r'\b(?:ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{50,})\b',
        'aws-access-id':r'\bAKIA[A-Z0-9]{16}\b',
    }
    sizes={}
    for name in paths:
        p=ROOT/name
        if (name!='.gitignore' and not name.startswith('integration/')) or '..' in Path(name).parts:
            errors.append('Out-of-scope path: '+name);continue
        if not p.is_file() or p.is_symlink():errors.append('Missing/nonregular allowlist file: '+name);continue
        sizes[name]=p.stat().st_size
        ignored=subprocess.run(['git','check-ignore','--no-index','--quiet',name],cwd=ROOT).returncode==0
        if ignored:errors.append('Allowlisted file is ignored: '+name)
        text=p.read_text()
        if re.search(r'/(?:Users|home)/[A-Za-z0-9_.-]+/',text):
            # /home/docker is an intentional, fixed path INSIDE the pinned image.
            if re.search(r'/Users/|/home/(?!docker/)[A-Za-z0-9_.-]+/',text):absolute.append(name)
        for label,rule in secret_rules.items():
            if re.search(rule,text):secrets.append(dict(path=name,rule=label))
        if p.suffix=='.py':
            tree=ast.parse(text,filename=name);syntax+=1
            for node in ast.walk(tree):
                imports=([a.name for a in node.names] if isinstance(node,ast.Import) else
                         [node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
                for module in imports:
                    first=module.split('.')[0]
                    package_target='integration/'+module.replace('.','/')+'.py'
                    if (ROOT/package_target).is_file():
                        if package_target not in selected:errors.append(name+' imports excluded '+package_target)
                    elif first in ('linux_gpu',):
                        # Namespace package; imported child modules are checked by
                        # their own selected source and the clean-tree test run.
                        for alias in getattr(node,'names',[]):
                            target='integration/'+module.replace('.','/')+'/'+alias.name+'.py'
                            if (ROOT/target).is_file() and target not in selected:errors.append(name+' imports excluded '+target)
                    elif first in modules:
                        target=str(modules[first].relative_to(ROOT))
                        if target not in selected:errors.append(name+' imports excluded '+target)
                    elif (p.parent/(first+'.py')).is_file():
                        target=str((p.parent/(first+'.py')).relative_to(ROOT))
                        if target not in selected:errors.append(name+' imports excluded '+target)
                    elif first=='matched_physics':
                        target='integration/'+module.replace('.','/')+'.py'
                        if target not in selected:errors.append(name+' imports excluded '+target)
                    elif first in EXPLICIT_LOCAL:
                        if EXPLICIT_LOCAL[first] not in selected:
                            errors.append(name+' imports excluded '+EXPLICIT_LOCAL[first])
                    elif first in EXTERNAL or not standard_library(first):
                        external.add(first)
        if p.suffix in ('.sdf','.xml') or p.name=='model.config':ET.fromstring(text);xml+=1
        if p.suffix=='.md':
            for target in re.findall(r'\]\(([^)]+)\)',text):
                if target.startswith(('https:','http:','#')):continue
                resolved=(p.parent/target.split('#')[0]).resolve()
                if not resolved.is_relative_to(ROOT) or str(resolved.relative_to(ROOT)) not in selected:
                    errors.append('Non-package Markdown link: '+name+' -> '+target)
    unknown=external-EXTERNAL
    if unknown:errors.append('Undeclared external imports: '+','.join(sorted(unknown)))
    if absolute:errors.append('Machine-specific host paths: '+','.join(absolute))
    if secrets:errors.append('Potential secret patterns; inspect paths without printing values')
    # Explicit exceptions: small immutable regression inputs, source patches,
    # model template and the generated protocol SOURCE must stay visible.
    required=['integration/matched_physics/results/SIM_JSON_review_20261109.cpp',
              'integration/matched_physics/results/SIM_JSON_review_20261109.h',
              'integration/matched_physics/results/camera_mount_review_20261109.json',
              'integration/isolated_blockers/20261109_a/code/AP444_BOOLEAN_EXACT.patch',
              'integration/isolated_blockers/20261109_a/evidence/instrumentation.patch',
              'integration/mavros_live_runs/20261108/mavlink_v20.py']
    for name in required:
        if name not in selected:errors.append('Missing explicit source/fixture: '+name)
    result=dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,
                source_scope_status='PASS' if not errors else 'FAIL',errors=errors,
                files=len(paths),total_worktree_bytes=sum(sizes.values()),python_ast_files=syntax,xml_files=xml,
                external_imports=sorted(external),potential_secret_patterns=secrets,
                largest_files=sorted((dict(path=k,bytes=v) for k,v in sizes.items()),key=lambda r:r['bytes'],reverse=True)[:5],
                runtime_only=[dict(path=k,present_locally=(ROOT/k).is_file(),commit=False,reason=v) for k,v in RUNTIME_ONLY.items()],
                runtime_bootstrap_status='Camera-only Linux recipes included; Linux build/GPU NOT TESTED here. Full Phase1 still needs reviewed firmware pin, images and MAVROS overlay; see LINUX_GPU_HANDOFF.md',
                staged_files=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT,text=True).splitlines())
    print(json.dumps(result,indent=2))
    return 0 if not errors else 1


if __name__=='__main__':raise SystemExit(main())
