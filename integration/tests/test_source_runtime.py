"""Offline startup/ownership contracts. Never starts Docker/Gazebo/SITL."""
import copy
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


if __name__=='__main__':unittest.main()
