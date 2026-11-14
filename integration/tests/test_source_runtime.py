"""Offline startup/ownership contracts. Never starts Docker/Gazebo/SITL."""
import copy
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from linux_gpu import run_camera, run_source_runtime as runtime


class SourceRuntimeTests(unittest.TestCase):
    def test_prepare_reuses_verified_scene_and_requires_explicit_egl(self):
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(run_camera,'INTEGRATION',Path(temp)), \
             patch.object(runtime.prepare_vio,'export') as export:
            run,plan=runtime.prepare('20261114-unit1','camera:local','fcu:local',
                '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',123)
            export.assert_called_once_with(run/'artifacts/vio')
            self.assertEqual((run/'models/world.sdf').read_bytes(),run_camera.verified_file('models/patched_c_world.sdf').read_bytes())
            self.assertEqual((run/'artifacts/camera.parm').read_text(),
                             'SERIAL0_PROTOCOL 2\nLOG_DISARMED 1\nSYSID_THISMAV 1\n')
            env=json.loads((run/'device.override.yaml').read_text())['services']['physics']['environment']
            self.assertEqual(env['__EGL_VENDOR_LIBRARY_FILENAMES'],plan['egl_vendor'])
            self.assertEqual(plan['partition'],'gnss-gpu-20261114-unit1')
            self.assertEqual(env['GNSS_EXPECTED_SYSTEM'],'1')
            self.assertFalse(plan['execution_allowed']);self.assertFalse(plan['forwarding_enabled'])
            with self.assertRaises(ValueError):runtime.prepare('20261114-unit1','c','f',plan['egl_vendor'],123)

    def test_invalid_vendor_domain_rejected_before_preparation(self):
        for vendor,domain in (('',123),('10_nvidia.json',123),('/tmp/a.json;bad',123),
                              ('/tmp/../a.json',123),('/tmp/a.json',0),('/tmp/a.json',233)):
            with self.subTest(vendor=vendor,domain=domain),self.assertRaises(ValueError):
                runtime.prepare('20261114-unit2','c','f',vendor,domain)

    def fixture(self):
        names=['gnss-gpu-physics-unit','gnss-gpu-fcu-unit'];project='gnss-gpu-unit'
        plan=dict(names=names,partition=project,ros_domain=123,egl_vendor='/nvidia.json')
        records=[dict(Id=name+'-id',Name='/'+name,State=dict(Running=True),
            Config=dict(Labels={'com.docker.compose.project':project},Env=[]),
            HostConfig=dict(RestartPolicy=dict(Name='no'),PortBindings={},PublishAllPorts=False),
            Mounts=[dict(Destination='/opt/integration',Source=str(runtime.ROOT),RW=False),
                    dict(Destination='/run/integration',Source='/new-run',RW=True)]) for name in names]
        records[0]['HostConfig']['NanoCpus']=3_000_000_000
        records[0]['NetworkSettings']=dict(Networks={project:{}})
        records[1]['HostConfig']['NetworkMode']='container:'+records[0]['Id']
        records[0]['Config']['Env']=['GZ_PARTITION='+project,'ROS_DOMAIN_ID=123',
            '__EGL_VENDOR_LIBRARY_FILENAMES=/nvidia.json','GNSS_EXPECTED_SYSTEM=1','GNSS_EXPECTED_COMPONENT=1']
        return records,plan

    def test_renaming_alone_cannot_satisfy_runtime_contract(self):
        original,plan=self.fixture()
        runtime.validate_runtime(original,plan,Path('/new-run'))
        for change in ('owner','restart','cpu','namespace','mount','write','domain','egl','ports'):
            with self.subTest(change=change):
                r=copy.deepcopy(original)
                if change=='owner':r[0]['Config']['Labels']['com.docker.compose.project']='other'
                elif change=='restart':r[0]['HostConfig']['RestartPolicy']['Name']='always'
                elif change=='cpu':r[0]['HostConfig']['NanoCpus']=1_000_000_000
                elif change=='namespace':r[1]['HostConfig']['NetworkMode']='host'
                elif change=='mount':r[0]['Mounts'][0]['Source']='/other-checkout'
                elif change=='write':r[0]['Mounts'][0]['RW']=True
                elif change=='ports':r[0]['HostConfig']['PortBindings']={'5760/tcp':[{}]}
                else:r[0]['Config']['Env']=[s for s in r[0]['Config']['Env'] if not s.startswith('ROS_DOMAIN_ID=' if change=='domain' else '__EGL_VENDOR')]
                with self.assertRaises(RuntimeError):runtime.validate_runtime(r,plan,Path('/new-run'))

    def test_collection_error_does_not_skip_stop_and_foreign_owner_never_stopped(self):
        records,plan=self.fixture();calls=[]
        def inspect(cmd,**kwargs):
            selected=[r for r in records if r['Name'].lstrip('/')==cmd[-1]]
            return subprocess.CompletedProcess(cmd,0,json.dumps(selected))
        def call(cmd,*_):
            calls.append(cmd)
            if cmd[1]=='logs':raise RuntimeError('collection failed')
        stopped=[dict(r,State=dict(Running=False)) for r in records]
        def final(cmd,**kwargs):return json.dumps([r for r in stopped if r['Id']==cmd[-1]])
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(runtime.subprocess,'run',side_effect=inspect), \
             patch.object(runtime.subprocess,'check_output',side_effect=final):
            errors=runtime.stop_owned(plan['names'],plan['partition'],Path(temp),call)
            self.assertEqual(len(errors),2)
            self.assertEqual([c[-1] for c in calls if c[1]=='stop'],[r['Id'] for r in reversed(records)])
            final_rows=json.loads((Path(temp)/'source-owned-final.json').read_text())
            self.assertTrue(all(not r['State']['Running'] for r in final_rows))
            calls.clear();records[0]['Config']['Labels']['com.docker.compose.project']='foreign'
            with self.assertRaises(RuntimeError):runtime.stop_owned(plan['names'],plan['partition'],Path(temp),call)
            self.assertEqual(calls,[])

    def test_existing_container_is_rejected_before_any_start_or_stop(self):
        _,plan=self.fixture()
        with patch.object(runtime.subprocess,'run',return_value=subprocess.CompletedProcess([],0)) as run:
            with self.assertRaisesRegex(RuntimeError,'pre-existing'):
                runtime.execute(Path('/unused'),plan,['docker','compose'],True)
        self.assertEqual(run.call_count,1)
        self.assertEqual(run.call_args[0][0],['docker','container','inspect',plan['names'][0]])

    def test_wrong_image_architecture_blocks_before_compose_up(self):
        _,plan=self.fixture()
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'evidence').mkdir()
            (root/'PREPARATION.json').write_text(json.dumps(dict(camera_image='c',fcu_image='f')))
            with patch.object(runtime.subprocess,'run',return_value=subprocess.CompletedProcess([],1)) as run, \
                 patch.object(runtime.subprocess,'check_output',return_value=json.dumps([dict(Os='linux',Architecture='arm64')])):
                with self.assertRaisesRegex(RuntimeError,'amd64'):runtime.execute(root,plan,['docker','compose'],True)
            self.assertTrue(all(call.args[0][2]=='inspect' for call in run.call_args_list))
            result=json.loads((root/'evidence/SOURCE_RUNTIME_RESULT.json').read_text())
            self.assertEqual(result['startup'],'BLOCKED');self.assertEqual(result['vio_source'],'NOT TESTED')
            self.assertFalse(result['forwarding_enabled'])


class RendererPreflightTests(unittest.TestCase):
    """Run the actual subcheck programs on temporary files, never in Docker."""
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)
        self.paths={name:self.root/name for name in ('setup.bash','vendor.json','plugin.txt',
                    'actual.sdf','expected.sdf','identity.txt','ogre.log')}
        self.paths['setup.bash'].write_text('export ROS_DISTRO=jazzy\n')
        self.paths['vendor.json'].write_text('{}\n')
        self.paths['plugin.txt'].write_text('f116b03552d05c8d0b0bed5ed67033a1183121d1\n')
        for name in ('actual.sdf','expected.sdf'):self.paths[name].write_text('<sdf/>\n')
        self.paths['identity.txt'].write_text('vendor=NVIDIA Corporation\nrenderer=Quadro fixture\nversion=fixture\n')
        self.paths['ogre.log'].write_text('GL_RENDERER = Quadro fixture\n')
        self.mapping={
            '/opt/ros/jazzy/setup.bash':self.paths['setup.bash'],
            '/opt/phase1/plugin-commit.txt':self.paths['plugin.txt'],
            '/opt/ardupilot_gazebo/models/gimbal_small_3d/model.sdf':self.paths['actual.sdf'],
            '/run/integration/evidence/integrated_gimbal_original.sdf':self.paths['expected.sdf'],
            '/tmp/render_gl_identity.txt':self.paths['identity.txt'],
            '/root/.gz/rendering/ogre2.log':self.paths['ogre.log'],
        }

    def run_checks(self,name):
        self.evidence=self.root/name;self.evidence.mkdir()
        self.audit=io.StringIO();self.labels=[]
        def call(cmd,label,timeout):
            self.assertEqual(cmd[:3],['docker','exec','fixture-only'])
            self.assertGreater(timeout,0);self.assertLessEqual(timeout,20)
            command=list(cmd[3:])
            for i,arg in enumerate(command):
                for original,fixture in self.mapping.items():arg=arg.replace(original,str(fixture))
                command[i]=arg
            # Do not wait10s for a deliberately missing fixture. Production
            # command and overall-budget values are checked separately below.
            if label=='source-renderer-identity':command[-1]='0'
            self.labels.append(label)
            runtime.recorded_command(self.evidence,self.audit,command,label,timeout)
        runtime.renderer_preflight('fixture-only',str(self.paths['vendor.json']),call)

    def records(self):return [json.loads(line) for line in self.audit.getvalue().splitlines()]

    def test_success_has_six_separate_streams_statuses_and_monotonic_spans(self):
        self.run_checks('success')
        self.assertEqual(len(self.records()),6)
        self.assertEqual(len(set(self.labels)),6)
        for r in self.records():
            self.assertEqual(r['exit'],0)
            self.assertGreaterEqual(r['host_end'],r['host_start'])
            self.assertTrue((self.evidence/r['stdout']).is_file())
            self.assertTrue((self.evidence/r['stderr']).is_file())
        checks=dict(runtime.renderer_checks('/vendor.json'))
        self.assertEqual(checks['source-renderer-identity'][-1],'10')
        self.assertEqual(checks['source-renderer-ogre-log'][-1],'0')

    def test_missing_vendor_and_dangling_symlink_fail_with_target_evidence(self):
        vendor=self.paths['vendor.json'];vendor.unlink()
        for kind in ('missing','dangling'):
            with self.subTest(kind=kind):
                if kind=='dangling':vendor.symlink_to('absent-nvidia.json')
                with self.assertRaisesRegex(RuntimeError,'source-renderer-egl-vendor failed'):
                    self.run_checks(kind)
                r=self.records()[-1];self.assertEqual(r['exit'],1)
                info=json.loads((self.evidence/r['stdout']).read_text())
                self.assertFalse(info['is_file']);self.assertEqual(info['symlink'],kind=='dangling')
                if kind=='dangling':self.assertEqual(info['symlink_target'],'absent-nvidia.json')
                self.assertIn('vendor JSON missing',(self.evidence/r['stderr']).read_text())
                self.assertEqual(len(self.labels),2)  # No provenance/render/admission after failure.

    def test_valid_vendor_symlink_reports_resolved_target(self):
        vendor=self.paths['vendor.json'];target=self.root/'real-vendor.json'
        vendor.rename(target);vendor.symlink_to(target.name)
        self.run_checks('symlink')
        r=self.records()[1];info=json.loads((self.evidence/r['stdout']).read_text())
        self.assertTrue(info['is_file']);self.assertTrue(info['symlink'])
        self.assertEqual(info['resolved'],str(target.resolve()))

    def test_mismatched_plugin_commit_stops_before_sdf_and_reports_actual(self):
        self.paths['plugin.txt'].write_text('unexpected-commit\n')
        with self.assertRaisesRegex(RuntimeError,'source-renderer-plugin-provenance failed'):
            self.run_checks('plugin')
        r=self.records()[-1]
        self.assertEqual(r['exit'],1);self.assertEqual(len(self.labels),3)
        self.assertIn('unexpected-commit',(self.evidence/r['stdout']).read_text())
        self.assertIn('plugin commit mismatch',(self.evidence/r['stderr']).read_text())

    def test_plugin_check_does_not_normalize_spaces_or_crlf_into_valid_pin(self):
        pin=b'f116b03552d05c8d0b0bed5ed67033a1183121d1'
        for i,data in enumerate((b' '+pin+b'\n',pin+b' \n',pin+b'\r\n')):
            with self.subTest(data=data):
                self.paths['plugin.txt'].write_bytes(data)
                with self.assertRaisesRegex(RuntimeError,'source-renderer-plugin-provenance failed'):
                    self.run_checks('plugin-whitespace-'+str(i))

    def test_sdf_mismatch_preserves_cmp_diagnostic_and_exit_status(self):
        self.paths['actual.sdf'].write_text('<changed/>\n')
        with self.assertRaisesRegex(RuntimeError,'source-renderer-gimbal-sdf failed'):
            self.run_checks('sdf')
        r=self.records()[-1]
        self.assertEqual(r['exit'],1);self.assertEqual(len(self.labels),4)
        self.assertTrue((self.evidence/r['stdout']).read_text() or (self.evidence/r['stderr']).read_text())

    def test_missing_or_empty_identity_and_ogre_log_fail_closed(self):
        for filename,label in (('identity.txt','source-renderer-identity'),('ogre.log','source-renderer-ogre-log')):
            original=self.paths[filename].read_text()
            for mode in ('missing','empty'):
                with self.subTest(file=filename,mode=mode):
                    if mode=='missing':self.paths[filename].unlink()
                    else:self.paths[filename].write_text('')
                    with self.assertRaisesRegex(RuntimeError,label+' failed'):
                        self.run_checks(filename+'-'+mode)
                    r=self.records()[-1];self.assertEqual(r['exit'],1)
                    self.assertIn('evidence missing or empty',(self.evidence/r['stderr']).read_text())
                    self.assertEqual(self.labels[-1],label)
            self.paths[filename].write_text(original)

    def test_ros_setup_failure_has_own_reason_and_keeps_stderr(self):
        self.paths['setup.bash'].write_text('echo setup-started\necho setup-broken >&2\nreturn 7\n')
        with self.assertRaisesRegex(RuntimeError,'source-renderer-ros-environment failed .exit 7'):
            self.run_checks('ros')
        r=self.records()[-1];self.assertEqual(r['exit'],7);self.assertEqual(len(self.labels),1)
        self.assertIn('setup-started',(self.evidence/r['stdout']).read_text())
        self.assertIn('setup-broken',(self.evidence/r['stderr']).read_text())

    def test_timeout_preserves_partial_streams_without_fabricated_exit_status(self):
        out=self.root/'timeout';out.mkdir();audit=io.StringIO()
        def timeout(cmd,**kwargs):
            kwargs['stdout'].write('partial stdout\n');kwargs['stderr'].write('partial stderr\n')
            raise subprocess.TimeoutExpired(cmd,20)
        with patch.object(runtime.subprocess,'run',side_effect=timeout):
            with self.assertRaisesRegex(RuntimeError,'source-renderer-identity failed: TimeoutExpired'):
                runtime.recorded_command(out,audit,['fixture-only'],'source-renderer-identity',20)
        row=json.loads(audit.getvalue())
        self.assertIsNone(row['exit']);self.assertTrue(row['timed_out'])
        self.assertGreaterEqual(row['host_end'],row['host_start'])
        self.assertEqual((out/row['stdout']).read_text(),'partial stdout\n')
        self.assertEqual((out/row['stderr']).read_text(),'partial stderr\n')

    def test_total_renderer_budget_remains_twenty_seconds(self):
        call=lambda *_:None
        with patch.object(runtime.time,'monotonic',side_effect=[100,120.001]):
            with self.assertRaisesRegex(RuntimeError,'source-renderer-ros-environment blocked.*20s'):
                runtime.renderer_preflight('fixture-only','/vendor.json',call)


if __name__=='__main__':unittest.main()
