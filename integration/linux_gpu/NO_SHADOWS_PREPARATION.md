# Phase 1 no-shadows Linux handoff

**HOLD / NO GO.** Keep `execution_allowed=false` and `forwarding_enabled=false`.
All procedures here are for DISARMED simulation preparation or evidence.
No ARM or flight operation is authorized.

This integration is based on `348cdd8a6350daf68e1c22203b2735b8d1276727`
from `origin/phase1-integration-merge`. Commit `72a18ef` already supplies the 16 m
noise fixture with 0.02 m detail. The integrated changes retain the existing
render instrumentation, simulator parameters, VIO, watchdogs and safety gates.

The image build changes the directional sun in the pinned plugin's
`worlds/iris_runway.sdf`. Both launchers derive a new `models/world.sdf` from
the manifest-verified historical template. Both Compose physics commands
require that file to contain a single directional sun with shadows disabled
before starting Gazebo. Historical source and evidence are preserved.

## 1. Transfer and verify the source

On Mac, from the reviewed integration checkout, create a bundle. Record the
printed SHA and transfer the bundle through the team's approved channel.

```bash
set -euo pipefail
BASE_SHA=348cdd8a6350daf68e1c22203b2735b8d1276727
: "${BUNDLE:?Set a new absolute path for the bundle}"
test ! -e "$BUNDLE"
test "$(git rev-parse HEAD^)" = "$BASE_SHA"
test -z "$(git status --porcelain --untracked-files=no)"
git bundle create "$BUNDLE" phase1-no-shadows "^$BASE_SHA"
git bundle list-heads "$BUNDLE" refs/heads/phase1-no-shadows
```

On the Linux NVIDIA host, set `ORIGIN_URL` to the team's repository URL,
`BUNDLE` to the transferred file, `REVIEWED_SHA` to the exact SHA received
with the bundle and `LINUX_CHECKOUT` to a new absolute checkout path. The local
branch name is the launcher's existing integration branch name in this fresh
checkout. No existing Linux checkout or team branch is changed.

```bash
set -euo pipefail
BASE_SHA=348cdd8a6350daf68e1c22203b2735b8d1276727
: "${ORIGIN_URL:?Set the team Git repository URL}"
: "${BUNDLE:?Set the transferred bundle path}"
: "${REVIEWED_SHA:?Set the reviewed Mac commit SHA}"
: "${LINUX_CHECKOUT:?Set a new absolute directory}"
test ! -e "$LINUX_CHECKOUT"
git init "$LINUX_CHECKOUT"
cd "$LINUX_CHECKOUT"
git fetch "$ORIGIN_URL" refs/heads/phase1-integration-merge
git merge-base --is-ancestor "$BASE_SHA" FETCH_HEAD
git bundle verify "$BUNDLE"
git fetch "$BUNDLE" refs/heads/phase1-no-shadows
test "$(git rev-parse FETCH_HEAD)" = "$REVIEWED_SHA"
test "$(git rev-parse FETCH_HEAD^)" = "$BASE_SHA"
git switch -c phase1-integration-merge FETCH_HEAD
git diff --check "$BASE_SHA" HEAD
git cat-file -e d33f66008fd7b6de332c96dd324b8ddbd320c194^{commit}
```

## 2. Build the image, verify both worlds and create the candidate

Run the remaining blocks from the new checkout in the same Bash session. Set `OLD_CANDIDATE` to the existing reviewed
native amd64 candidate JSON. The example copies its three referenced export
directories into the new checkout, where candidate validation requires them
to reside. Preserve the original artifacts and their hashes.

```bash
set -euo pipefail
: "${OLD_CANDIDATE:?Set the existing reviewed native candidate JSON path}"
test -z "$(git status --porcelain --untracked-files=no)"
test "$(git rev-parse HEAD)" = "$REVIEWED_SHA"
id="$(date -u +%Y%m%d)-noshadows$(date -u +%H%M%S)"
export out="$PWD/integration/linux_gpu_runs/$id"
mkdir -p "$PWD/integration/linux_gpu_runs"
mkdir "$out"
git rev-parse HEAD > "$out/source-commit.txt"
docker image ls --no-trunc --filter reference=gnss-phase1-camera:linux-gpu > "$out/old-camera-image.txt"
python3 -B integration/linux_gpu/prepare_context.py --kind camera --output "$out/ctx-camera"
DOCKER_BUILDKIT=1 docker build --platform linux/amd64 --progress=plain \
  -t gnss-phase1-camera:linux-gpu "$out/ctx-camera" 2>&1 | tee "$out/camera-build.log"
PHYSICS_IMAGE=$(docker image inspect --format '{{.Id}}' gnss-phase1-camera:linux-gpu)
export PHYSICS_IMAGE
printf '%s\n' "$PHYSICS_IMAGE" | tee "$out/physics-image-id.txt"
docker image inspect "$PHYSICS_IMAGE" > "$out/new-camera-image.json"

docker run --rm -i --network none --entrypoint python3 "$PHYSICS_IMAGE" -B - <<'PY_IMAGE' | tee "$out/image-world-check.txt"
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

commit = 'f116b03552d05c8d0b0bed5ed67033a1183121d1'
repo = '/opt/ardupilot_gazebo'
assert Path('/opt/phase1/plugin-commit.txt').read_text().strip() == commit
original = subprocess.check_output(['git', '-C', repo, 'show', commit+':worlds/iris_runway.sdf'])
actual = Path(repo, 'worlds/iris_runway.sdf').read_bytes()
suns = ET.fromstring(actual).findall("./world[@name='iris_runway']/light[@name='sun']")
assert len(suns) == 1 and suns[0].get('type') == 'directional'
assert len(suns[0].findall('cast_shadows')) == 1
assert suns[0].findtext('cast_shadows').strip() == 'false'
assert actual == original.replace(b'<cast_shadows>true</cast_shadows>',
                                  b'<cast_shadows>false</cast_shadows>', 1)
print('Image world verified against pinned upstream bytes')
PY_IMAGE

PYTHONPATH=integration python3 -B - <<'PY_WORLD' | tee "$out/runtime-world.sha256"
import os
from pathlib import Path
from launch_phase1 import install_runtime_world, verified_file, verify_runtime_world

world = Path(os.environ['out'], 'world.sdf')
source = verified_file('models/patched_c_world.sdf')
digest = install_runtime_world(source, world)
assert verify_runtime_world(world) == digest
assert digest == 'bf3dce911f0d02f5bd4a6f04710b62473c6784892917f1b6fb20257b14274df5'
verified_file('models/patched_c_world.sdf')
print(digest)
PY_WORLD

PYTHONPATH=integration python3 -B - "$OLD_CANDIDATE" <<'PY_CANDIDATE' | tee "$out/candidate-check.txt"
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from runtime_candidate import load_candidate

old_path = Path(sys.argv[1]).resolve(strict=True)
old = json.loads(old_path.read_text())
assert old['schema'] == 1 and old['architecture'] == 'amd64' and old['decision'] == 'HOLD'
assert all(old[key] is False for key in ('runtime_admitted','execution_allowed','forwarding_enabled'))
out = Path(os.environ['out'])
candidate = out / 'candidate.json'
args = [sys.executable, '-B', 'integration/runtime_candidate.py', '--architecture', 'amd64']
for key, flag, name in [('firmware_dir','--firmware-dir','firmware'),
                        ('mavros_record_dir','--mavros-record-dir','mavros'),
                        ('overlay_dir','--overlay-dir','patch_install')]:
    source = (old_path.parent / old[key]).resolve(strict=True)
    dest = out / name
    shutil.copytree(source, dest, symlinks=True)
    args.extend([flag, str(dest)])
expected = dict(old['images'], physics=os.environ['PHYSICS_IMAGE'])
for kind, image in expected.items():
    identity = subprocess.check_output(
        ['docker','image','inspect','--format','{{.Id}} {{.Os}}/{{.Architecture}}',image], text=True)
    assert identity.strip() == f'{image} linux/amd64'
    args.extend(['--'+kind+'-image', image])
subprocess.run(args + ['--output', str(candidate)], check=True)
new = load_candidate(candidate)
assert new['images'] == expected
assert new['firmware_sha256'] == old['firmware_sha256']
assert new['overlay_sha256'] == old['overlay_sha256']
assert new['decision'] == 'HOLD'
assert all(new[key] is False for key in ('runtime_admitted','execution_allowed','forwarding_enabled'))
print('Candidate validated and bound to immutable local image IDs')
PY_CANDIDATE
```

The later ground run compares the actual mounted runtime SDF to this generated
world and queries the running Gazebo scene. Keep the image ID and both SDF
checks with the candidate. The build uses the existing base and cache policy.
Retain the old image ID and existing containers.

## 3. Collect a fresh DISARMED fusion proof after approval

Use the existing bounded simulation-only route after explicit operator approval
and review of the native runtime and clock prerequisites. This run has its own
owned SITL FCU and the existing DISARMED checks. The candidate keeps
`execution_allowed=false` and `forwarding_enabled=false`.

The vision permit binds the candidate hash after checking the live container
image IDs. The proof hashes the permit and redecodes DataFlash. Flight proof
verification does not enforce cross-run physics-image identity. The launch and
Compose source hashes are bound. Collect a new proof for this image and world
and preserve the separate provenance record below.

```bash
set -euo pipefail
: "${DISARMED_FUSION_APPROVAL:?Separate explicit approval and preflight required}"
VIO_DIR="$out/vio"
python3 -B integration/linux_gpu/prepare_vio.py --output "$VIO_DIR"
python3 -B -c 'from pymavlink import DFReader'
: "${SIM_ORIGIN:?Reviewed lat,lon,alt_m matching the isolated SITL home/reference}"
export CANDIDATE="$out/candidate.json"
export FUSION_ID="$(date -u +%Y%m%d)-noshadowfusion$(date -u +%H%M%S)"
export FUSION_RUN="$PWD/integration/phase1_runs/$FUSION_ID"
PYTHONPATH=integration python3 -B - <<'PY'
import subprocess
from launch_phase1 import validate_branch
validate_branch(subprocess.check_output(['git','branch','--show-current'],text=True).strip())
PY
python3 -B integration/launch_phase1.py --run-id "$FUSION_ID" \
  --runtime-candidate "$CANDIDATE" --vio-dir "$VIO_DIR" --gpu-mode nvidia \
  --isolated-imu --simulation-disarmed-vision --simulation-origin="$SIM_ORIGIN" --execute
# STOP on nonzero exit, gaps, invalid clock mapping, stale data, cleanup failure,
# missing actual FCU aiding/innovations, or any other failed existing gate.
# No retries with weakened limits and no substitution of synthetic/FallbackFlowFrontend.
python3 -B - <<'PY'
import json,os,subprocess,sys
from pathlib import Path
run=Path(os.environ['FUSION_RUN'])
bins=sorted((run/'state/fcu/logs').glob('*.BIN'))
if len(bins)!=1:raise SystemExit('HOLD: identify one exact closed-run DataFlash BIN; do not guess')
subprocess.run([sys.executable,'-B','integration/simulation_flight.py',
                '--run',str(run),'--bin',str(bins[0])],check=True)
sys.path.insert(0,'integration')
from simulation_flight import verify_proof
from runtime_candidate import load_candidate,sha
from simulation_vision import BOUND_SOURCES
from launch_phase1 import verify_runtime_world
baseline,permit=verify_proof(run/'evidence/flight_fusion_proof.json') # redecodes actual BIN
candidate=load_candidate(os.environ['CANDIDATE'])
assert permit['candidate_sha256']==sha(Path(os.environ['CANDIDATE']))
assert permit['source_hashes']=={n:sha(Path('integration')/n) for n in BOUND_SOURCES}
assert permit['firmware_sha256']==candidate['firmware_sha256']
world_sha=verify_runtime_world(run/'models/world.sdf')
assert world_sha == Path(os.environ['out'], 'runtime-world.sha256').read_text().strip()
owned=json.loads((run/'evidence/simulation-owned-initial.json').read_text())
expected = {
    f"gnss-p1-{kind}-{os.environ['FUSION_ID']}": candidate['images'][image]
    for kind, image in [('physics','physics'),('vio','physics'),('fcu','fcu'),('transport','transport')]
}
assert len(owned) == len(expected)
assert {row['Name'].lstrip('/'): row['Image'] for row in owned} == expected
with (run/'evidence/no-shadows-provenance.json').open('x') as out:
    json.dump(dict(physics_image=candidate['images']['physics'],world_sha256=world_sha,
        candidate_sha256=permit['candidate_sha256'],source_hashes=permit['source_hashes'],
        proof_sha256=sha(run/'evidence/flight_fusion_proof.json'),
        execution_allowed=False,forwarding_enabled=False),out,indent=2)
print('NEW CLOSED DISARMED PROOF VERIFIED',baseline)
print('physics image',candidate['images']['physics'],'active world sha256',world_sha)
print('Decision: HOLD / NO GO')
PY
```

The existing fusion validator uses its default ground fixture. The separate
noise capture below supplies evidence for the noise scene.

## 4. Capture and preserve raw ground evidence

This Linux procedure starts a new isolated Gazebo and JSON SITL stack for
camera capture, using the existing passive DISARMED checker and exact-ID
cleanup. The existing native mount probe briefly pauses and resumes the new
world for the camera orientation check.

Use the verified image and candidate from step 2. Ground capture can run
independently of step 3 while fusion approval is pending.

```bash
set -euo pipefail
python3 -B integration/vio_ground_fixture.py --help | grep -F -- '--pattern'
export GROUND_ID="$(date -u +%Y%m%d)-shadowground$(date -u +%H%M%S)"
export GROUND_RUN="$PWD/integration/linux_gpu_runs/$GROUND_ID"
FCU_IMAGE=$(python3 -B - "$out/candidate.json" <<'PY'
import sys
sys.path.insert(0,'integration')
from runtime_candidate import load_candidate
print(load_candidate(sys.argv[1])['images']['fcu'])
PY
)
python3 -B integration/linux_gpu/run_source_runtime.py \
  --run-id "$GROUND_ID" --camera-image "$PHYSICS_IMAGE" --fcu-image "$FCU_IMAGE" \
  --egl-vendor-file /usr/share/glvnd/egl_vendor.d/10_nvidia.json --ros-domain 123
# Preparation/config only above: intentionally no --execute or --probe.

python3 -B - <<'PY'
import json, os, shlex, signal, subprocess, sys
from pathlib import Path
sys.path.insert(0,'integration')
from linux_gpu import run_source_runtime as runtime
from launch_phase1 import verify_runtime_world

run=Path(os.environ['GROUND_RUN']).resolve()
plan=json.loads((run/'SOURCE_RUNTIME.json').read_text())
assert plan['run_id']==os.environ['GROUND_ID']
assert plan['execution_allowed'] is False and plan['forwarding_enabled'] is False
world_sha = verify_runtime_world(run/'models/world.sdf')
assert world_sha == Path(os.environ['out'], 'runtime-world.sha256').read_text().strip()
(run/'evidence/ground-world.sha256').write_text(world_sha + '\n')
compose=['docker','compose','--env-file',str(run/'compose.env'),
         '-f','integration/linux_gpu/camera.compose.yaml','-f',str(run/'device.override.yaml')]
evidence=run/'evidence';initial=None;fixture=False;errors=[]
audit=(evidence/'ground-commands.jsonl').open('x')
result=dict(decision='HOLD',capture='NOT TESTED',cleanup='NOT TESTED',
            execution_allowed=False,forwarding_enabled=False)
def call(cmd,label,timeout=30):
    runtime.recorded_command(evidence,audit,cmd,label,timeout)
def shell(code,label,timeout=30):
    call(['docker','exec',plan['names'][0],'bash','-lc',
          'set -e; source /opt/ros/jazzy/setup.bash; '+code],label,timeout)
def interrupted(*_):raise KeyboardInterrupt('Ground capture interrupted')
signal.signal(signal.SIGTERM,interrupted)
heartbeat=('python3 -B /opt/integration/linux_gpu/verified_disarmed.py '
           '--expected-system 1 --expected-component 1 --directory ')
fixture_cmd='python3 -B /opt/integration/vio_ground_fixture.py '
try:
    # Require affirmative absence, not an arbitrary Docker/API error.
    for kind,name in [('container',n) for n in plan['names']]+[('network',plan['partition'])]:
        r=subprocess.run(['docker',kind,'inspect',name],capture_output=True,text=True,timeout=10)
        if r.returncode==0 or 'no such' not in r.stderr.lower():
            raise RuntimeError('HOLD: resource exists or Docker absence not verified: '+name)
    # Create and record full IDs BEFORE starting either service.
    call(compose+['config','--quiet'],'ground-compose-config')
    call(compose+['create','--no-build','--pull','never','physics','fcu'],'ground-create',60)
    initial=json.loads(subprocess.check_output(['docker','inspect',*plan['names']],text=True,timeout=10))
    (evidence/'ground-owned-initial.json').write_text(json.dumps(initial,indent=2)+'\n')
    runtime.owned_ids(initial,plan['partition'],plan['names'])
    prep=json.loads((run/'PREPARATION.json').read_text())
    for row,key in zip(initial,('camera_image','fcu_image')):
        assert row['Name'].lstrip('/')==plan['names'][('camera_image','fcu_image').index(key)]
        assert row['Image']==prep[key]  # immutable IDs supplied above, never mutable tags
    call(compose+['start','physics','fcu'],'ground-start',30)
    current=json.loads(subprocess.check_output(['docker','inspect',*plan['names']],text=True,timeout=10))
    assert [r['Id'] for r in current]==[r['Id'] for r in initial]
    runtime.validate_runtime(current,plan,run)
    assert all(r['State']['Running'] for r in current)
    net=json.loads(subprocess.check_output(['docker','network','inspect',plan['partition']],text=True,timeout=10))[0]
    assert net['Internal'] is True
    runtime.renderer_preflight(plan['names'][0],plan['egl_vendor'],call)
    code=("import sys;sys.path.insert(0,'/opt/integration');from pathlib import Path;"
          "from vio_source_probe import hardware_renderer;"
          "s=Path('/tmp/render_gl_identity.txt').read_text();r=hardware_renderer(s);"
          "assert 'vendor=NVIDIA Corporation' in s;"
          "assert r in Path('/root/.gz/rendering/ogre2.log').read_text();print(s)")
    shell('python3 -B -c '+shlex.quote(code),'ground-actual-renderer')
    shell('cat /root/.gz/rendering/ogre2.log', 'ground-ogre2')
    call(['docker','exec',plan['names'][1],'bash','-lc',
          'set -e; test "$(cat /opt/phase1/ardupilot-commit.txt)" = 9279df6191a772c098b2e4797c95ac954f258d88; '
          'sha256sum -c /opt/phase1/firmware.sha256; '
          "grep -F '[  PASSED  ] 5 tests.' /opt/phase1/parser-tests.txt"],'ground-firmware')
    # Check the actual bind-mounted file and the world argument used by PID 1.
    code=("import sys;sys.path.insert(0,'/opt/integration');from pathlib import Path;"
          "from launch_phase1 import verify_runtime_world;"
          "digest=verify_runtime_world('/run/integration/models/world.sdf');"
          "assert digest==sys.argv[1];print(digest);"
          "c=Path('/proc/1/cmdline').read_bytes();print(repr(c));"
          "assert b'/run/integration/models/world.sdf' in c.split(bytes([0]))")
    shell('python3 -B -c '+shlex.quote(code)+' '+shlex.quote(world_sha),'ground-active-world')
    # Check Gazebo's scene through its typed protobuf reply.
    code="""import json,subprocess
from google.protobuf import text_format
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.light_pb2 import Light
from pathlib import Path
r=subprocess.run(['gz','service','-s','/world/iris_runway/scene/info',
 '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','5000','--req',''],
 capture_output=True,text=True,timeout=8)
out=Path('/run/integration/evidence')
(out/'ground-scene.pbtxt').write_text(r.stdout)
(out/'ground-scene.stderr.txt').write_text(r.stderr)
if r.returncode:raise SystemExit('HOLD: scene service failed')
scene=text_format.Parse(r.stdout,Scene())
sun=[x for x in scene.light if x.name=='sun']
if len(sun)!=1 or sun[0].type!=Light.DIRECTIONAL or sun[0].cast_shadows:
 raise SystemExit('HOLD: actual scene sun missing, ambiguous, wrong type or shadow-enabled')
print(json.dumps(dict(sun=sun[0].name,id=sun[0].id,cast_shadows=sun[0].cast_shadows)))
"""
    shell('python3 -B -c '+shlex.quote(code),'ground-scene-check')
    shell(heartbeat+'/run/integration/evidence/ground-disarmed-before','ground-disarmed-before',12)
    fixture=True  # Remove even if create fails after a partial operation.
    shell(fixture_cmd+'create --directory /run/integration/artifacts/ground-noise '
          '--size-m 16 --tile-size-m .02 --pattern noise --seed 0','ground-noise-create',30)
    shell('python3 -B /opt/integration/isolated_blockers/20261109_a/code/probe_experiment.py '
          '--run /run/integration --variant integrated --duration 20','ground-mount-camera',40)
    mount=json.loads((evidence/'integrated/summary.json').read_text())
    assert mount['mount_status']=='PASS' and mount['settled_snapshots']>0
    assert max(mount['max_angles_deg'])<=1 and not mount['unexpected_commands']
    frame=evidence/'integrated/frame_corrected.ppm'
    image = frame.read_bytes()
    header = b'P6\n640 480\n255\n'
    assert image.startswith(header) and len(image)==len(header)+640*480*3
    import hashlib
    (evidence/'ground-frame.sha256').write_text(hashlib.sha256(image).hexdigest()+'\n')
    shell(heartbeat+'/run/integration/evidence/ground-disarmed-after','ground-disarmed-after',12)
    result['capture']='PASS: file/scene/mount/frame checks; human image inspection still required'
finally:
    if fixture:
        try:shell(fixture_cmd+'remove --directory /run/integration/artifacts/ground-noise','ground-noise-remove',15)
        except Exception as exc:errors.append('Fixture cleanup: '+str(exc))
    if initial:
        try:errors+=runtime.stop_owned(plan['names'],plan['partition'],evidence,call,initial)
        except Exception as exc:errors.append('INCOMPLETE cleanup: '+str(exc))
    else:errors.append('INCOMPLETE: initial IDs unavailable; no name-based stop authorized')
    result['cleanup']=runtime.cleanup_status(errors);result['cleanup_errors']=errors
    (evidence/'GROUND_CAPTURE_RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    audit.close()
    if errors:raise RuntimeError('HOLD: cleanup not fully verified: '+str(errors))
PY
```

Inspect `$GROUND_RUN/evidence/integrated/frame_corrected.ppm` visually: noise
surface, downward view, no drone shadow. Keep `native.jsonl`, mount `summary.json`,
frame, noise PNG/SDF, scene reply, command streams, renderer identity/Ogre log,
both heartbeat directories, initial/final full Docker IDs and result JSON.
The mount probe also saves other raw PPM frames and native timestamps.
Provide them to Person 3 for reviewing the shadow, texture and tracking with
the pinned frontend. Person 3 owns the image and tracking assessment, including
the target of approximately 150 or more inliers. The approved ground procedure
has no 1 m capture step. Keep the aircraft on the ground.

The scene check uses the Gazebo Sim 8 service and the gz-msgs10 light
definition. [Scene service](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/scene_broadcaster/SceneBroadcaster.cc)
[Light schema](https://github.com/gazebosim/gz-msgs/blob/gz-msgs10/proto/gz/msgs/light.proto)

## 5. Package evidence for Person 3

When both runs are complete, package the available native proof and raw ground
evidence, plus their provenance, from the Linux checkout. Review before sending.

```bash
set -euo pipefail
: "${EVIDENCE_ARCHIVE:?Set a new absolute .tar.gz path}"
test ! -e "$EVIDENCE_ARCHIVE"
test -s "$GROUND_RUN/evidence/integrated/frame_corrected.ppm"
test -s "$FUSION_RUN/evidence/no-shadows-provenance.json"
tar -czf "$EVIDENCE_ARCHIVE" -C "$PWD/integration" \
  "linux_gpu_runs/$GROUND_ID" "phase1_runs/$FUSION_ID" \
  "linux_gpu_runs/$id/source-commit.txt" "linux_gpu_runs/$id/physics-image-id.txt" \
  "linux_gpu_runs/$id/new-camera-image.json" "linux_gpu_runs/$id/camera-build.log" \
  "linux_gpu_runs/$id/image-world-check.txt" "linux_gpu_runs/$id/world.sdf" \
  "linux_gpu_runs/$id/runtime-world.sha256" "linux_gpu_runs/$id/candidate.json" \
  "linux_gpu_runs/$id/candidate-check.txt"
sha256sum "$EVIDENCE_ARCHIVE"
```

Keep the native export directories on the validation host for candidate
revalidation. For a ground-only interim review, share the ground run directory
with the image ID, candidate JSON and world hash, and mark fusion as pending.
Person 3 should review the actual raw frames and tracking before marking the
image validated. The overall decision stays HOLD / NO GO.
