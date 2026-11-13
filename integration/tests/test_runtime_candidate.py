"""Offline build-record/ELF fixtures, never amd64 execution or clock admission."""
import json
import io
from contextlib import redirect_stdout
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import runtime_candidate as runtime
from linux_gpu.prepare_context import INPUTS
from physics_clock_mapping import AppliedClockMapping
from phase1_transport import prepared_firmware_digest
import launch_phase1


def elf(machine):
    data=bytearray(64);data[:6]=b'\x7fELF\x02\x01';struct.pack_into('<H',data,18,machine)
    return data


class RuntimeCandidateTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)
        self.fw=self.root/'firmware';self.fw.mkdir()
        self.record=self.root/'record';self.record.mkdir()
        self.overlay=self.root/'patch_install';(self.overlay/'lib').mkdir(parents=True)
        (self.fw/'arducopter').write_bytes(elf(62))
        (self.overlay/'lib/libgnss_mavros_time_patch.so').write_bytes(elf(62))
        (self.overlay/'setup.bash').write_text('# fixture only\n')
        for directory,kind in ((self.fw,'sitl'),(self.record,'mavros')):
            (directory/'architecture.txt').write_text('x86_64\n')
            for name in ('compiler.txt','packages.txt'):(directory/name).write_text('offline fixture\n')
            (directory/'source-hashes.json').write_text(json.dumps({n:runtime.sha(runtime.ROOT/n) for n in INPUTS[kind]}))
            (directory/'base-image.txt').write_text(('ubuntu' if kind=='sitl' else 'ros')+'@sha256:'+'0'*64+'\n')
        (self.fw/'ardupilot-commit.txt').write_text(runtime.AP_COMMIT+'\n')
        names=('ExactLiteralsAndWhitespace','LegacyDecimalIntegers','RejectMalformedTokens',
            'MissingOptionalFieldResetsToDefault','RejectMissingRequiredField')
        (self.fw/'parser-tests.txt').write_text(''.join('[       OK ] JSONParser.'+n+' (0 ms)\n' for n in names)+'[  PASSED  ] 5 tests.\n')
        (self.fw/'firmware.sha256').write_text(runtime.sha(self.fw/'arducopter')+'  /opt/phase1/arducopter\n')
        (self.fw/'submodules.txt').write_text('fixture submodule IDs\n')
        (self.fw/'applied-source.patch').write_text('fixture diff\n')
        paths={'/tmp/instrumentation.patch':'isolated_blockers/20261109_a/evidence/instrumentation.patch',
               '/tmp/boolean.patch':'isolated_blockers/20261109_a/code/AP444_BOOLEAN_EXACT.patch',
               'libraries/SITL/tests/test_sim_json.cpp':'isolated_blockers/20261109_a/code/test_sim_json.cpp'}
        (self.fw/'patches.sha256').write_text(''.join(runtime.sha(runtime.ROOT/n)+'  '+p+'\n' for p,n in paths.items()))
        (self.record/'ros-versions.json').write_text(json.dumps({n:'2.16.0' for n in ('mavros','mavros_msgs','libmavconn')}))
        (self.record/'overlay-source.sha256').write_text(runtime.sha(runtime.ROOT/'mavros_time_patch/src/sys_time.cpp')+'  source\n')

    def test_matching_records_remain_unadmitted(self):
        binary,_=runtime.verify_builds(self.fw,self.record,self.overlay,'amd64')
        self.assertEqual(runtime.elf_arch(binary),'amd64')
        with self.assertRaisesRegex(ValueError,'Unreviewed'):
            AppliedClockMapping('offline',runtime.sha(binary))

    def test_wrong_architecture_and_binary_changes_fail(self):
        with self.assertRaises(ValueError):runtime.verify_builds(self.fw,self.record,self.overlay,'arm64')
        (self.fw/'arducopter').write_bytes(elf(183))
        with self.assertRaisesRegex(ValueError,'architecture'):runtime.verify_builds(self.fw,self.record,self.overlay,'amd64')

    def test_source_commit_parser_and_context_cannot_be_claimed(self):
        for file,replacement in (('ardupilot-commit.txt','other'),('parser-tests.txt','5/5 PASS'),('source-hashes.json','{}')):
            with self.subTest(file=file):
                path=self.fw/file;original=path.read_bytes();path.write_text(replacement)
                with self.assertRaises(ValueError):runtime.verify_builds(self.fw,self.record,self.overlay,'amd64')
                path.write_bytes(original)

    def test_wrong_mavros_or_overlay_fails(self):
        path=self.record/'ros-versions.json';original=path.read_bytes()
        path.write_text(json.dumps({n:'2.17.0' for n in ('mavros','mavros_msgs','libmavconn')}))
        with self.assertRaises(ValueError):runtime.verify_builds(self.fw,self.record,self.overlay,'amd64')
        path.write_bytes(original)
        (self.overlay/'lib/libgnss_mavros_time_patch.so').write_bytes(elf(183))
        with self.assertRaisesRegex(ValueError,'Overlay'):runtime.verify_builds(self.fw,self.record,self.overlay,'amd64')

    def test_candidate_execute_rejected_before_docker(self):
        with patch.object(sys,'argv',['launch_phase1.py','--run-id','20261113-unit1','--runtime-candidate','candidate.json','--execute']), \
             patch('runtime_candidate.load_candidate',return_value={}), \
             patch.object(launch_phase1.subprocess,'run',side_effect=AssertionError('Docker must not run')):
            with self.assertRaisesRegex(ValueError,'--execute forbidden'):launch_phase1.main()

    def test_candidate_preparation_binds_native_artifacts_but_starts_nothing(self):
        sources={n:launch_phase1.verified_file(n) for n in (
            'models/patched_c/iris_with_gimbal/model.sdf','models/patched_c/iris_with_gimbal/model.config',
            'models/patched_c_world.sdf','evidence/patched_c_gimbal_original.sdf',
            'code/probe_experiment.py','code/mount_math.py')}
        (self.root/'ekf3_dry_run.parm').write_text((runtime.ROOT/'ekf3_dry_run.parm').read_text())
        candidate=dict(architecture='amd64',images={n:'sha256:'+str(i)*64 for i,n in enumerate(('physics','fcu','transport'),1)},
                       firmware=self.fw/'arducopter',overlay=self.overlay)
        candidate['firmware'].chmod(0o755)
        def output(command,**kwargs):
            if command[0]=='git':return 'gazebo-sitl-integration\n'
            if command[1:3]==['image','inspect']:return 'linux/amd64\n'
            if command[1]=='compose':return 'services: {}\n'
            raise AssertionError(command)
        console=io.StringIO()
        with patch.object(sys,'argv',['launch_phase1.py','--run-id','20261113-unit2','--runtime-candidate','candidate.json','--gpu-mode','wsl']), \
             patch('runtime_candidate.load_candidate',return_value=candidate), \
             patch.object(launch_phase1,'INTEGRATION',self.root), \
             patch.object(launch_phase1,'verified_file',side_effect=lambda name:sources[name]), \
             patch.object(launch_phase1.subprocess,'check_output',side_effect=output), \
             patch.object(launch_phase1.subprocess,'run',return_value=SimpleNamespace(returncode=1)) as calls, \
             redirect_stdout(console):
            launch_phase1.main()
        self.assertIn('runtime NOT ADMITTED',console.getvalue())
        self.assertNotIn('add --execute',console.getvalue())
        run=self.root/'phase1_runs/20261113-unit2'
        identity=json.loads((run/'artifacts/run_identity.json').read_text())
        self.assertFalse(identity['runtime_admitted'])
        self.assertFalse(identity['execution_allowed'])
        self.assertFalse(identity['forwarding_enabled'])
        self.assertEqual((run/'artifacts/arducopter').read_bytes(),candidate['firmware'].read_bytes())
        self.assertTrue((run/'artifacts/arducopter').stat().st_mode & 0o111)
        self.assertIn('P1_OVERLAY_ROOT='+str(self.overlay),(run/'compose.env').read_text())
        for call in calls.call_args_list:
            self.assertIn('inspect',call.args[0])  # no create/up/start/stop

    def test_image_tags_and_self_approval_fail(self):
        with self.assertRaises(ValueError):runtime.image_id('image:latest')
        path=self.root/'candidate.json';path.write_text(json.dumps(dict(schema=1,runtime_admitted=True,decision='GO')))
        with self.assertRaises(ValueError):runtime.load_candidate(path)

    def test_transport_cannot_open_unadmitted_or_changed_preparation(self):
        with self.assertRaisesRegex(ValueError,'not admitted'):
            prepared_firmware_digest(self.root,dict(runtime_admitted=False))
        artifacts=self.root/'artifacts';artifacts.mkdir()
        binary=artifacts/'arducopter';binary.write_bytes(elf(62))
        digest=runtime.sha(binary)
        self.assertEqual(prepared_firmware_digest(self.root,dict(runtime_admitted=True,firmware_sha256=digest)),digest)
        binary.write_bytes(elf(183))
        with self.assertRaisesRegex(ValueError,'changed'):
            prepared_firmware_digest(self.root,dict(runtime_admitted=True,firmware_sha256=digest))

    def test_replay_fixture_is_complete_and_hash_bound_without_runtime_logs(self):
        path=runtime.ROOT/'tests/fixtures/mavros_timesync_replay.jsonl'
        meta=json.loads(path.with_suffix('.metadata.json').read_text())
        self.assertEqual(runtime.sha(path),meta['fixture_sha256'])
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(len(rows),meta['records'])
        by_kind={k:[r['stamp_ns'] for r in rows if r['kind']==k] for k in ('pose','speed','clock')}
        self.assertEqual(len(by_kind['pose']),20)
        self.assertEqual(by_kind['pose'],by_kind['speed'])
        for values in by_kind.values():
            self.assertTrue(all(type(v) is int for v in values))
            self.assertTrue(all(b>a for a,b in zip(values,values[1:])))


if __name__=='__main__':unittest.main()
