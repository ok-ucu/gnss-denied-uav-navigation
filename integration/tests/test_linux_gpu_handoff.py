"""Offline source-handoff regressions, never live hardware/GPU verification."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from linux_gpu import run_camera
from linux_gpu.analyze_camera import union

class CameraHandoffTests(unittest.TestCase):
    def test_unknown_or_software_context_cannot_be_hardware_candidate(self):
        for identity in ('','renderer=llvmpipe (LLVM)\nversion=4.5',
                         'renderer=softpipe\nversion=4.5', 'renderer=NVIDIA GPU'):
            self.assertTrue(run_camera.hardware_status(identity,'GL_RENDERER = NVIDIA GPU').startswith('HOLD'))
        self.assertTrue(run_camera.hardware_status('renderer=NVIDIA GPU\nversion=4.5','missing GL renderer').startswith('HOLD'))

    def test_enumerated_software_device_does_not_override_actual_hardware_context(self):
        verdict=run_camera.hardware_status('renderer=NVIDIA GPU\nversion=4.5',
            'Enumerated EGL_MESA_device_software\nGL_RENDERER = NVIDIA GPU')
        self.assertTrue(verdict.startswith('HARDWARE CANDIDATE'))
        self.assertNotIn('PASS',verdict)

    def test_prepare_preserves_inputs_and_remains_closed_without_runtime(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(run_camera,'INTEGRATION',Path(directory)):
            run=run_camera.prepare('20261111-unit1','dri','camera:test','sitl:test',30)
            model=run/'models/world.sdf'
            source=run_camera.verified_file('models/patched_c_world.sdf')
            self.assertEqual(model.read_bytes(),source.read_bytes().replace(
                b'<cast_shadows>true</cast_shadows>',b'<cast_shadows>false</cast_shadows>',1))
            from launch_phase1 import verify_runtime_world
            verify_runtime_world(model)
            decision=json.loads((run/'PREPARATION.json').read_text())
            self.assertFalse(decision['execution_allowed'])
            self.assertFalse(decision['forwarding_enabled'])
            self.assertEqual((run/'artifacts/camera.parm').read_text(),'SERIAL0_PROTOCOL 2\nLOG_DISARMED 1\n')
            with self.assertRaises(ValueError):run_camera.prepare('20261111-unit1','dri','camera:test','sitl:test',30)

    def test_invalid_duration_mode_and_identity_are_rejected(self):
        for duration in (float('nan'),float('inf'),0,61):
            with self.assertRaises(ValueError):run_camera.prepare('20261111-unit2','dri','camera:test','sitl:test',duration)
        with self.assertRaises(ValueError):run_camera.prepare('../../other','dri','camera:test','sitl:test',30)
        with self.assertRaises(ValueError):run_camera.prepare('20261111-unit2','unknown','camera:test','sitl:test',30)

    def test_nested_scene_spans_are_union_not_double_counted(self):
        self.assertEqual(union([(0,100),(10,40),(45,60),(110,120)]),110)
        self.assertEqual(union([]),0)

    def test_passive_heartbeat_uses_real_codec_and_never_sends(self):
        import importlib.util
        from linux_gpu import disarmed
        file=Path(__file__).resolve().parents[1]/'mavros_live_runs/20261108/mavlink_v20.py'
        spec=importlib.util.spec_from_file_location('handoff_test_wire',file)
        wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(wire)
        class ReceiveOnly:
            def __init__(self,data):self.data=data
            def __enter__(self):return self
            def __exit__(self,*_):pass
            def settimeout(self,*_):pass
            def recv(self,*_):return self.data
            def send(self,*_):raise AssertionError('Passive check transmitted data')
            def sendall(self,*_):raise AssertionError('Passive check transmitted data')
        for mode in (0,128):
            payload=wire.MAVLink_heartbeat_message(2,3,mode,0,3,3).pack(wire.MAVLink(None))
            with patch.object(disarmed.socket,'create_connection',return_value=ReceiveOnly(payload)):
                if mode:
                    with self.assertRaisesRegex(RuntimeError,'ARMED'):disarmed.check()
                else:disarmed.check()



class CameraNoShadowsTests(unittest.TestCase):
    """Execute the actual Docker RUN Python offline against temporary SDFs."""
    WORLD = '/opt/ardupilot_gazebo/worlds/iris_runway.sdf'

    @classmethod
    def script(cls):
        text=(Path(__file__).resolve().parents[1]/'linux_gpu/Camera.Dockerfile').read_text()
        marker="RUN python3 - <<'NO_SHADOWS'\n"
        if text.count(marker)!=1:
            raise AssertionError('Missing/duplicate actual camera no-shadows build step')
        return text.split(marker,1)[1].split('\nNO_SHADOWS\n',1)[0]

    def world(self, sun='<light type="directional" name="sun"><cast_shadows>true</cast_shadows></light>'):
        return ('<sdf version="1.9"><world name="iris_runway">'
            '<physics><real_time_factor>0.69</real_time_factor><max_step_size>0.001</max_step_size></physics>'+
            sun+'<light name="lamp" type="point"><cast_shadows>true</cast_shadows></light>'
            '<model name="test"><link name="test"><visual name="test"><cast_shadows>true</cast_shadows></visual>'
            '<sensor name="camera" type="camera"><update_rate>10</update_rate><camera><horizontal_fov>2</horizontal_fov>'
            '<image><width>640</width><height>480</height></image></camera></sensor></link></model></world></sdf>')

    def execute_step(self, source):
        import subprocess,sys
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'iris_runway.sdf'
            if source is not None:path.write_text(source)
            script=self.script()
            self.assertEqual(script.count(repr(self.WORLD)),1)
            script=script.replace(repr(self.WORLD),repr(str(path)))
            result=subprocess.run([sys.executable,'-B','-c',script],capture_output=True,text=True,timeout=5)
            output=path.read_text() if path.exists() else None
            return result,output

    def test_correct_camera_recipe_and_build_order(self):
        text=(Path(__file__).resolve().parents[1]/'linux_gpu/Camera.Dockerfile').read_text()
        self.assertLess(text.index('checkout --detach ${PLUGIN_COMMIT}'),text.index("RUN python3 - <<'NO_SHADOWS'"))
        self.assertLess(text.index('test "$(git -C /opt/ardupilot_gazebo rev-parse HEAD)"'),text.index("RUN python3 - <<'NO_SHADOWS'"))
        self.assertLess(text.index('\nNO_SHADOWS\n'),text.index('cmake -S /opt/ardupilot_gazebo'))
        self.assertNotIn('real_time_factor',self.script())
        self.assertNotIn('sensor',self.script())

    def test_only_sun_changes_exactly_once(self):
        source=self.world();result,output=self.execute_step(source)
        self.assertEqual(result.returncode,0,result.stderr)
        expected=source.replace('<light type="directional" name="sun"><cast_shadows>true',
                                '<light type="directional" name="sun"><cast_shadows>false')
        self.assertEqual(output,expected)
        self.assertIn('replacements=1',result.stdout)

    def test_whitespace_and_reordered_attributes(self):
        source=self.world("<light name='sun' type='directional'>\n<cast_shadows> true \n</cast_shadows></light>")
        result,output=self.execute_step(source)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(output,source.replace('> true \n<','> false \n<'))

    def test_missing_file_fails(self):
        result,output=self.execute_step(None)
        self.assertNotEqual(result.returncode,0)
        self.assertIsNone(output)

    def test_missing_or_duplicate_sun_fails_without_write(self):
        sun='<light name="sun" type="directional"><cast_shadows>true</cast_shadows></light>'
        for xml in (self.world(''),self.world(sun+sun),self.world(sun).replace('iris_runway','other')):
            with self.subTest(xml=xml):
                result,output=self.execute_step(xml)
                self.assertNotEqual(result.returncode,0)
                self.assertEqual(output,xml)

    def test_missing_duplicate_or_unexpected_flag_fails_without_write(self):
        for flag in ('','<cast_shadows>false</cast_shadows>','<cast_shadows>1</cast_shadows>',
                     '<cast_shadows>true</cast_shadows><cast_shadows>true</cast_shadows>'):
            source=self.world('<light type="directional" name="sun">'+flag+'</light>')
            with self.subTest(flag=flag):
                result,output=self.execute_step(source)
                self.assertNotEqual(result.returncode,0)
                self.assertEqual(output,source)

    def test_wrong_light_type_or_malformed_xml_fails_without_write(self):
        for source in (self.world().replace('directional','point'),self.world()[:-2]):
            result,output=self.execute_step(source)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(output,source)

    def test_comment_cannot_supply_a_fake_shadow_replacement(self):
        source=self.world('<light type="directional" name="sun"><!-- <cast_shadows>true</cast_shadows> -->'
                          '<cast_shadows>true</cast_shadows></light>')
        result,output=self.execute_step(source)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(output,source)

    def test_optional_original_pinned_upstream_world(self):
        import os
        name=os.environ.get('NO_SHADOWS_UPSTREAM_SDF')
        if not name:self.skipTest('Original pinned upstream SDF not supplied')
        source=Path(name).read_text();result,output=self.execute_step(source)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(output,source.replace('<cast_shadows>true</cast_shadows>',
                                             '<cast_shadows>false</cast_shadows>',1))

class ActiveWorldNoShadowsTests(unittest.TestCase):
    """Exercise the derived active file and actual pre-exec rendering prerequisite."""
    def install(self, source):
        from launch_phase1 import install_runtime_world, verify_runtime_world
        with tempfile.TemporaryDirectory() as directory:
            src=Path(directory)/'historical.sdf';dest=Path(directory)/'world.sdf'
            src.write_bytes(source)
            digest=install_runtime_world(src,dest)
            self.assertEqual(src.read_bytes(),source)
            self.assertEqual(verify_runtime_world(dest),digest)
            return dest.read_bytes()

    def test_historical_template_and_manifest_are_preserved(self):
        from launch_phase1 import VERIFIED, verified_file
        manifest=(VERIFIED/'SHA256SUMS.json').read_bytes()
        src=verified_file('models/patched_c_world.sdf');before=src.read_bytes()
        self.assertEqual(self.install(before),before.replace(
            b'<cast_shadows>true</cast_shadows>',b'<cast_shadows>false</cast_shadows>',1))
        self.assertEqual(src.read_bytes(),before)
        self.assertEqual((VERIFIED/'SHA256SUMS.json').read_bytes(),manifest)

    def test_only_sun_changes_not_physics_camera_or_other_lights(self):
        source=CameraNoShadowsTests().world().encode()
        expected=source.replace(b'<light type="directional" name="sun"><cast_shadows>true',
                                b'<light type="directional" name="sun"><cast_shadows>false')
        self.assertEqual(self.install(source),expected)

    def test_runtime_whitespace_and_attribute_order(self):
        source=CameraNoShadowsTests().world(
            "<light name='sun' type='directional'>\n<cast_shadows> true \n</cast_shadows></light>").encode()
        self.assertEqual(self.install(source),source.replace(b'> true \n<',b'> false \n<'))

    def test_invalid_source_never_creates_output(self):
        from launch_phase1 import install_runtime_world
        import xml.etree.ElementTree as ET
        sun='<light name="sun" type="directional"><cast_shadows>true</cast_shadows></light>'
        valid=CameraNoShadowsTests().world(sun)
        invalid=[valid[:-2],valid.replace('iris_runway','other'),
            CameraNoShadowsTests().world(''),CameraNoShadowsTests().world(sun+sun),
            valid.replace('directional','point'),valid.replace('>true<','>false<'),
            valid.replace('>true<','>1<'),valid.replace('<cast_shadows>true</cast_shadows>','',1),
            valid.replace('<cast_shadows>true</cast_shadows>',
                          '<cast_shadows>true</cast_shadows><cast_shadows>true</cast_shadows>',1),
            valid.replace('<cast_shadows>true</cast_shadows>',
                          '<!-- <cast_shadows>true</cast_shadows> --><cast_shadows>true</cast_shadows>',1)]
        with tempfile.TemporaryDirectory() as directory:
            src=Path(directory)/'historical.sdf';out=Path(directory)/'world.sdf'
            for xml in invalid:
                with self.subTest(xml=xml):
                    src.write_text(xml)
                    with self.assertRaises((ValueError,ET.ParseError)):install_runtime_world(src,out)
                    self.assertEqual(src.read_text(),xml)
                    self.assertFalse(out.exists())

    def test_existing_world_or_symlink_is_not_overwritten(self):
        from launch_phase1 import install_runtime_world
        with tempfile.TemporaryDirectory() as directory:
            src=Path(directory)/'source';out=Path(directory)/'world'
            source=CameraNoShadowsTests().world().encode();src.write_bytes(source)
            out.write_bytes(b'old evidence')
            with self.assertRaises(FileExistsError):install_runtime_world(src,out)
            self.assertEqual(out.read_bytes(),b'old evidence')
            out.unlink();out.symlink_to(src)
            with self.assertRaises(FileExistsError):install_runtime_world(src,out)
            self.assertEqual(src.read_bytes(),source)
            with self.assertRaises(FileExistsError):install_runtime_world(src,src)

    def test_pre_exec_verifier_fails_closed_on_missing_true_or_invalid_world(self):
        from launch_phase1 import verify_runtime_world
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'world'
            with self.assertRaises(ValueError):verify_runtime_world(path)
            for xml in (CameraNoShadowsTests().world(),'<sdf>',
                        '<sdf><world name="iris_runway"/></sdf>'):
                path.write_text(xml)
                with self.assertRaises((ValueError,ET.ParseError)):verify_runtime_world(path)
            path.write_bytes(self.install(CameraNoShadowsTests().world().encode()))
            link=path.with_name('link');link.symlink_to(path)
            with self.assertRaises(ValueError):verify_runtime_world(link)

    def test_both_launchers_install_the_derived_world(self):
        import launch_phase1
        for module in (launch_phase1,run_camera):
            tree=ast.parse(Path(module.__file__).read_text())
            calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)
                   and isinstance(n.func,ast.Name) and n.func.id=='install_runtime_world']
            self.assertEqual(len(calls),1,module.__name__)

    def test_both_compose_commands_block_gazebo_until_actual_file_passes(self):
        import os,shlex,subprocess,sys
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            world=Path(directory)/'world.sdf';sentinel=Path(directory)/'would-start-gazebo'
            for name in ('phase1.compose.yaml','linux_gpu/camera.compose.yaml'):
                lines=(root/name).read_text().splitlines()
                # Decode the actual YAML single-quoted shell scalar, not a parallel fixture.
                line=next(line for line in lines if 'exec gz sim' in line)
                shell=line.split("'source ",1)[1].rsplit("'",1)[0].replace("''", "'")
                check=shell.split('PYTHONPATH=/opt/integration ',1)[1].split(' && exec gz sim',1)[0]
                args=shlex.split(check)
                self.assertEqual(args[:3],['python3','-B','-c'])
                self.assertIn("verify_runtime_world('/run/integration/models/world.sdf')",args[3])
                code=args[3].replace('/run/integration/models/world.sdf',str(world))
                command=shlex.join([sys.executable,'-B','-c',code])+' && touch '+shlex.quote(str(sentinel))
                for valid in (False,True):
                    world.write_bytes(self.install(CameraNoShadowsTests().world().encode()) if valid
                                      else CameraNoShadowsTests().world().encode())
                    result=subprocess.run(['bash','-c',command],capture_output=True,text=True,timeout=5,
                                          env=dict(os.environ,PYTHONPATH=str(root)))
                    self.assertEqual(result.returncode==0,valid,result.stderr)
                    self.assertEqual(sentinel.exists(),valid)
                    if sentinel.exists():sentinel.unlink()


if __name__=='__main__':unittest.main()
