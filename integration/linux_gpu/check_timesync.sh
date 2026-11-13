#!/usr/bin/env bash
# Run only inside a NEW --network none transport container; never near FCU.
set -euo pipefail
[[ $# == 1 ]] || { echo 'Require a NEW output directory'; exit 2; }
for interface in /sys/class/net/*; do
  [[ "${interface##*/}" == lo ]] || { echo 'Require --network none (loopback only)'; exit 2; }
done
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$1"
mkdir "$out"
source /opt/ros/jazzy/setup.bash
source /tmp/patch_install/setup.bash
export PYTHONPATH="$root${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -B - "$root" <<'PY'
import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/'tests/fixtures/mavros_timesync_replay.jsonl'
metadata=json.loads(p.with_suffix('.metadata.json').read_text())
assert hashlib.sha256(p.read_bytes()).hexdigest()==metadata['fixture_sha256'], 'Replay fixture changed'
PY
common=(python3 -B "$root/probe_patched_timesync.py"
  --input "$root/tests/fixtures/mavros_timesync_replay.jsonl"
  --dialect "$root/mavros_live_runs/20261108/mavlink_v20.py"
  --config "$root/mavros_isolated.yaml" --mode MAVLINK --patched)
"${common[@]}" --output "$out/minus" --peer-offset-ns -1000000000 > "$out/minus.log" 2>&1
"${common[@]}" --output "$out/zero" --peer-offset-ns 0 > "$out/zero.log" 2>&1
"${common[@]}" --output "$out/plus" --peer-offset-ns 1000000000 > "$out/plus.log" 2>&1
"${common[@]}" --output "$out/jitter" --jitter-ms 6 > "$out/jitter.log" 2>&1
"${common[@]}" --output "$out/drift" --drift-ppm 1000 > "$out/drift.log" 2>&1
"${common[@]}" --output "$out/domain" --domain-offset-ns 1700000000000000000 > "$out/domain.log" 2>&1
"${common[@]}" --output "$out/reset" --reset-test > "$out/reset.log" 2>&1
# Raw MAVROS does not reject incomparable headers: negative fixture must FAIL.
status=0
"${common[@]}" --output "$out/bad_domain" --header-only-offset-ns 1700000000000000000 > "$out/bad_domain.log" 2>&1 || status=$?
[[ "$status" == 2 ]] || { echo 'Negative domain fixture did not fail as expected'; exit 1; }
python3 -B - "$out" <<'PY'
import hashlib,json,sys
from pathlib import Path
from vio_adapter import expand_upper_triangle,validate_covariance
out=Path(sys.argv[1])
results={name:json.loads((out/name/'result.json').read_text()) for name in ('minus','zero','plus','jitter','drift','domain','reset')}
for name,r in results.items():
    assert r['patched_overlay'] and r['offset_check']=='PASS' and r['timestamps_unchanged'],name
    assert all(r['sent_counts'][k]>0 and r['sent_counts'][k]==r['wire_counts'][k] for k in ('pose','speed')),name
    assert r['reset_counters']==[0],name
    wire=[json.loads(line)['message'] for line in (out/name/'wire.jsonl').read_text().splitlines()]
    for kind,key,size in (('VISION_POSITION_ESTIMATE','pose',21),('VISION_SPEED_ESTIMATE','speed',9)):
        messages=[m for m in wire if m['mavpackettype']==kind]
        stamps=[m['usec'] for m in messages]
        assert len(stamps)==len(set(stamps))==r['sent_counts'][key],(name,kind,'loss/duplicate')
        for message in messages:
            assert len(message['covariance'])==size,(name,kind)
            validate_covariance(expand_upper_triangle(message['covariance']) if size==21 else message['covariance'],6 if size==21 else 3)
reset=results['reset']
assert any(not row.get('allowed',True) for row in reset['reset_events']), 'No blocked reset pairs'
with (out/'suite.json').open('x') as f:
    json.dump(dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,
        overlay_sha256=hashlib.sha256(Path('/tmp/patch_install/lib/libgnss_mavros_time_patch.so').read_bytes()).hexdigest(),
        scope='Recorded protocol replay; artificial TIMESYNC peer; no FCU route',
        passed=list(results),negative_domain='EXPECTED FAIL',runtime_admitted=False),f,indent=2)
PY
