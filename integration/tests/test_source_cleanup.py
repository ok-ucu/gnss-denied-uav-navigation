"""Real local POSIX process groups; synthetic witness inputs, no simulators.

Only PIDs/groups created by this fixture may be signalled. Fixture/FCU analysis
commands are stubbed; kill/wait/sleep and witness finalization are real.
"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import vio_source_probe as probe


WORKER = r'''
import json, os, signal, socket, subprocess, sys, time, types
from pathlib import Path
from unittest.mock import patch
import vio_source_probe as probe
from linux_gpu.verified_disarmed import load_codec

os.setsid()
directory=Path(sys.argv[1]);mode=sys.argv[2]
sleep=time.sleep
child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])
(directory/'family.json').write_text(json.dumps(dict(pid=os.getpid(),pgid=os.getpgrp(),child=child.pid)))
if mode=='hung':
    signal.signal(signal.SIGTERM,lambda *_:None)
    (directory/'witness.ready.json').write_text('{}')
    (directory/'native.jsonl').write_text('{}\n')
    while True:
        child.poll()  # Reap the TERM-cooperative child before leader needs KILL.
        sleep(.01)

ns=types.SimpleNamespace
class Clock:
    now=0.
clock=Clock()
class Native:
    def subscribe(self,kind,topic,callback):
        stamp=ns(sec=5,nsec=0)
        if topic==probe.CAMERA:
            callback(ns(header=ns(stamp=stamp),width=640,height=480,data=b'fixture',pixel_format_type=3))
        elif topic==probe.IMU:callback(ns(header=ns(stamp=stamp)))
        else:callback(ns(sim=stamp))
        return True
    def unsubscribe(self,topic):pass
wire=load_codec()
packet=wire.MAVLink_heartbeat_message(2,3,0,0,3,3).pack(wire.MAVLink(None,srcSystem=1,srcComponent=1))
class Stream:
    def __enter__(self):return self
    def __exit__(self,*_):pass
    def settimeout(self,*_):pass
    def recv(self,*_):
        sleep(.01);clock.now+=.25
        if clock.now.is_integer() and (mode!='heartbeat' or clock.now<=3):return packet
        raise socket.timeout()
    def send(self,*_):raise AssertionError('Unexpected TX')
modules={name:types.ModuleType(name) for name in ('gz','gz.transport13','gz.msgs10',
    'gz.msgs10.image_pb2','gz.msgs10.clock_pb2','gz.msgs10.imu_pb2')}
modules['gz.transport13'].Node=Native
modules['gz.msgs10.image_pb2'].Image=object
modules['gz.msgs10.clock_pb2'].Clock=object
modules['gz.msgs10.imu_pb2'].IMU=object
save=probe.save
def slow_save(path,value):
    if path.name=='witness.json':sleep(.6)  # Parent cleanup overlaps actual export.
    return save(path,value)
try:
    with patch.dict(sys.modules,modules), \
         patch.dict(os.environ,{'GNSS_EXPECTED_SYSTEM':'1','GNSS_EXPECTED_COMPONENT':'1'}), \
         patch.object(probe.socket,'create_connection',return_value=Stream()), \
         patch.object(probe.time,'monotonic',lambda:clock.now), \
         patch.object(probe,'resource_sample',return_value=dict(kind='resources',host=0)), \
         patch.object(probe,'save',slow_save):
        status=probe.witness(directory)
finally:
    # Deliberately no child.terminate(): the production group cleanup must do it.
    child.wait(timeout=2)
raise SystemExit(status)
'''


BRIDGE_PARENT = r'''
import json, os, subprocess, sys, time
from pathlib import Path
from vio_sensor_capture import start_sensor_bridge, stop_sensor_bridge
os.setsid()
directory=Path(sys.argv[1]);shared=sys.argv[2]=='shared'
child_code="""
import json, os, signal, sys, time
from pathlib import Path
def stop(*_):
    Path(sys.argv[2]).write_text(json.dumps(dict(pid=os.getpid(),signal='TERM')))
    raise SystemExit(0)
signal.signal(signal.SIGTERM,stop)
Path(sys.argv[1]).write_text('ready')
while True:time.sleep(.01)
"""
child=start_sensor_bridge([sys.executable,'-c',child_code,str(directory/'bridge.ready'),
    str(directory/'bridge.stopped.json')],subprocess.DEVNULL,probe_owned=shared)
deadline=time.monotonic()+5
while not (directory/'bridge.ready').exists():
    if time.monotonic()>deadline:raise RuntimeError('Bridge fixture not ready')
    time.sleep(.01)
(directory/'family.json').write_text(json.dumps(dict(pid=os.getpid(),pgid=os.getpgrp(),
    sid=os.getsid(0),child=child.pid,child_pgid=os.getpgid(child.pid),child_sid=os.getsid(child.pid))))
if sys.argv[3]=='stop':
    stop_sensor_bridge(child,probe_owned=shared)
    (directory/'parent.after-stop').write_text(str(child.returncode))
else:
    while True:time.sleep(.01)
'''

# Test-only subreaper on Linux: reap an orphan after its real parent is killed,
# including in CI containers whose PID 1 does not reap. No simulator involvement.
REAPER = r'''
import ctypes, os, subprocess, sys, time
if sys.platform.startswith('linux'):
    if ctypes.CDLL(None,use_errno=True).prctl(36,1,0,0,0)!=0:
        raise OSError(ctypes.get_errno(),'Could not enable fixture child subreaper')
launcher=subprocess.Popen(sys.argv[1:])
while launcher.returncode is None:
    try:pid,status=os.waitpid(-1,os.WNOHANG)
    except ChildProcessError:break
    if pid==launcher.pid:launcher.returncode=os.waitstatus_to_exitcode(status)
    if pid==0:time.sleep(.01)
raise SystemExit(launcher.wait())
'''


class SourceCleanupTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)
        self.worker=self.root/'worker.py';self.worker.write_text(WORKER)
        text=(probe.ROOT/'run_vio_source_probe.sh').read_text()
        self.cleanup=re.search(r'^cleanup\(\) \{\n.*?^\}',text,re.M|re.S).group(0)

    def wait_file(self,path,process,timeout=8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if path.is_file() and path.stat().st_size:return
            if process.poll() is not None:break
            time.sleep(.01)
        self.fail('Fixture did not reach expected state: '+str(path))

    def run_case(self,mode):
        run=self.root/('gnss-vio-source-'+mode);run.mkdir()
        env=dict(os.environ,PYTHONPATH=str(probe.ROOT),PYTHONDONTWRITEBYTECODE='1')
        control=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True)
        harness=r'''set -euo pipefail
run="$1"; here="$2"; helper="$here/vio_source_probe.py"; PYTHON="$3"
worker="$4"; mode="$5"
source_pid=; observer_pid=; camera_pid=; witness_pid=; fixture_owned=1
disarmed() { printf 'fixture-only DISARMED check\n'; }
python3() {
  if [[ "$3" == evidence ]]; then "$PYTHON" "$@"
  else printf 'fixture-only external command: %s\n' "$*"; fi
}
'''+self.cleanup+r'''
trap cleanup EXIT
trap 'exit 124' TERM
"$PYTHON" -B "$worker" "$run" "$mode" &
witness_pid=$!
for unused in {1..800}; do
  [[ ! -s "$run/witness.ready.json" ]] || break
  kill -0 "$witness_pid" || exit 1
  sleep .01
done
[[ -s "$run/witness.ready.json" ]] || exit 1
if [[ "$mode" == heartbeat ]]; then
  for unused in {1..800}; do
    [[ ! -s "$run/witness.failed.json" ]] || exit 1
    sleep .01
  done
  exit 2
elif [[ "$mode" == timeout ]]; then
  echo ready > "$run/timeout.ready"
  while :; do sleep .05; done
fi
exit 0
'''
        cleanup_confirmed=False
        process=subprocess.Popen(['bash','-c',harness,'fixture',str(run),str(probe.ROOT),
            sys.executable,str(self.worker),mode],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
            text=True,start_new_session=True)
        try:
            self.wait_file(run/'family.json',process)
            family=json.loads((run/'family.json').read_text())
            self.assertEqual(family['pid'],family['pgid'])
            self.assertNotEqual(family['pgid'],os.getpgid(control.pid))
            if mode=='timeout':
                self.wait_file(run/'timeout.ready',process)
                process.send_signal(signal.SIGTERM)  # Only this test's harness PID.
            stdout,stderr=process.communicate(timeout=35)
            self.assertIsNone(control.poll(),'Production cleanup affected control process')
            for pid in (family['pid'],family['child']):
                with self.assertRaises(ProcessLookupError):os.kill(pid,0)
            cleanup_confirmed=True
            report=json.loads((run/'witness.evidence.json').read_text())
            if mode=='hung':
                self.assertNotEqual(process.returncode,0,stdout+stderr)
                self.assertEqual(report['status'],'INCOMPLETE')
                self.assertIn('forced shutdown',(run/'cleanup.log').read_text())
            else:
                self.assertEqual(process.returncode,{'normal':0,'heartbeat':1,'timeout':124}[mode],stdout+stderr)
                self.assertEqual(report['status'],'COMPLETE',report)
                self.assertEqual(report['run_directory'],str(run.resolve()))
                names=['native.jsonl','witness.json']+(['witness.failed.json'] if mode=='heartbeat' else [])
                for name in names:self.assertGreater((run/name).stat().st_size,0,name)
                complete=json.loads((run/'witness.complete.json').read_text())
                self.assertEqual(complete['run_directory'],str(run.resolve()))
                self.assertEqual(complete['files']['native.jsonl'],(run/'native.jsonl').stat().st_size)
                meta=json.loads((run/'witness.json').read_text())
                if mode=='heartbeat':
                    marker=json.loads((run/'witness.failed.json').read_text())
                    self.assertIn('stale >3s',marker['reason']);self.assertTrue(meta['failure'])
                    # Fixture clock freezes once the real guard raises. Export
                    # completes later in host time without fabricated clock ticks.
                    self.assertGreaterEqual(meta['end'],marker['host'])
                self.assertNotIn('forced shutdown',(run/'cleanup.log').read_text())
            return run
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:process.communicate(timeout=25)
                except subprocess.TimeoutExpired:
                    process.kill();process.communicate(timeout=3)
            # Exact fixture-owned PGID only, also when an assertion/fixture fails.
            if not cleanup_confirmed and (run/'family.json').exists():
                family=json.loads((run/'family.json').read_text())
                try:os.killpg(family['pgid'],signal.SIGKILL)
                except ProcessLookupError:pass
            control.terminate();control.wait(timeout=3)

    def test_real_owned_parent_child_and_unrelated_control_across_cleanup_paths(self):
        for mode in ('normal','heartbeat','timeout'):
            with self.subTest(mode=mode):self.run_case(mode)

    def test_hung_export_is_bounded_and_marked_incomplete_not_pass(self):
        started=time.monotonic();self.run_case('hung')
        self.assertLess(time.monotonic()-started,30)

    def test_missing_truncated_or_wrong_run_export_is_incomplete(self):
        run=self.run_case('heartbeat')
        complete=run/'witness.complete.json';original=complete.read_text()
        for change in ('missing','truncated','wrong_run'):
            with self.subTest(change=change):
                if change=='missing':complete.unlink()
                elif change=='truncated':complete.write_text('{')
                else:
                    value=json.loads(original);value['run_directory']='wrong-run'
                    complete.write_text(json.dumps(value))
                self.assertEqual(probe.witness_evidence(run)['status'],'INCOMPLETE')
                complete.write_text(original)
        (run/'native.jsonl').write_text('')
        self.assertEqual(probe.witness_evidence(run)['status'],'INCOMPLETE')

    def test_crashed_capture_retains_bridge_in_owned_group_and_cleanup_spares_control(self):
        run=self.root/'bridge-crash';run.mkdir()
        worker=self.root/'bridge_parent.py';worker.write_text(BRIDGE_PARENT)
        env=dict(os.environ,PYTHONPATH=str(probe.ROOT),PYTHONDONTWRITEBYTECODE='1')
        control=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
        harness=r'''set -euo pipefail
run="$1"; here="$2"; PYTHON="$3"; worker="$4"; helper="$here/vio_source_probe.py"
source_pid=; observer_pid=; camera_pid=; witness_pid=; fixture_owned=0
disarmed() { printf 'fixture-only DISARMED check\n'; }
python3() { printf 'fixture-only post-run analysis\n'; }
'''+self.cleanup+r'''
trap cleanup EXIT
"$PYTHON" -B "$worker" "$run" shared crash &
source_pid=$!
for unused in {1..800}; do
  [[ ! -s "$run/release" ]] || break
  sleep .01
done
[[ -s "$run/release" ]] || exit 2
kill -KILL "$source_pid"
child_status=0
wait "$source_pid" || child_status=$?
[[ "$child_status" == 137 ]] || exit 2
# Deliberately retain the dead leader's PGID, as the real launcher now does.
exit 1
'''
        process=subprocess.Popen([sys.executable,'-c',REAPER,'bash','-c',harness,'fixture',
            str(run),str(probe.ROOT),sys.executable,str(worker)],env=env,stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,text=True,start_new_session=True)
        cleanup_confirmed=False
        try:
            self.wait_file(run/'family.json',process)
            family=json.loads((run/'family.json').read_text())
            self.assertEqual(family['pid'],family['pgid'])
            self.assertEqual(family['pgid'],family['child_pgid'])
            self.assertEqual(family['sid'],family['child_sid'])
            self.assertNotEqual(family['pgid'],os.getpgid(control.pid))
            (run/'release').write_text('kill own capture parent')
            stdout,stderr=process.communicate(timeout=30)
            self.assertEqual(process.returncode,1,stdout+stderr)
            stopped=json.loads((run/'bridge.stopped.json').read_text())
            self.assertEqual(stopped['pid'],family['child'])
            for pid in (family['pid'],family['child']):
                with self.assertRaises(ProcessLookupError):os.kill(pid,0)
            cleanup_confirmed=True
            self.assertIsNone(control.poll(),'Cleanup affected unrelated control')
            self.assertNotIn('forced shutdown',(run/'cleanup.log').read_text())
            self.assertEqual((run/'launcher.exit').read_text().strip(),'1')
        finally:
            if not cleanup_confirmed and (run/'family.json').exists():
                family=json.loads((run/'family.json').read_text())
                try:os.killpg(family['pgid'],signal.SIGKILL)
                except ProcessLookupError:pass
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.communicate(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.communicate(timeout=3)
            control.terminate();control.wait(timeout=3)

    def test_bridge_default_session_unchanged_and_shared_stop_does_not_kill_parent(self):
        worker=self.root/'bridge_parent.py';worker.write_text(BRIDGE_PARENT)
        env=dict(os.environ,PYTHONPATH=str(probe.ROOT),PYTHONDONTWRITEBYTECODE='1')
        for mode in ('default','shared'):
            with self.subTest(mode=mode):
                run=self.root/mode;run.mkdir()
                result=subprocess.run([sys.executable,'-B',str(worker),str(run),mode,'stop'],
                    env=env,capture_output=True,text=True,timeout=15)
                self.assertEqual(result.returncode,0,result.stderr)
                family=json.loads((run/'family.json').read_text())
                self.assertEqual(family['child_pgid'],family['pgid'] if mode=='shared' else family['child'])
                self.assertEqual(family['child_sid'],family['sid'] if mode=='shared' else family['child'])
                self.assertEqual((run/'parent.after-stop').read_text(),'0')
                with self.assertRaises(ProcessLookupError):os.kill(family['child'],0)

    def test_shared_bridge_without_dedicated_source_session_is_rejected_before_spawn(self):
        code='''from vio_sensor_capture import start_sensor_bridge
import subprocess,sys
start_sensor_bridge([sys.executable,'-c',"raise SystemExit('must not start')"],subprocess.DEVNULL,probe_owned=True)
'''
        result=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,timeout=5,
            env=dict(os.environ,PYTHONPATH=str(probe.ROOT),PYTHONDONTWRITEBYTECODE='1'))
        self.assertNotEqual(result.returncode,0)
        self.assertIn('dedicated source session/process group leader',result.stderr)


if __name__=='__main__':unittest.main()
