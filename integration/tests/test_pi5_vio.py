"""Synthetic physical-adapter contracts and REAL CPU frontend/EKF; no ROS/devices."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import numpy as np
import pi5_vio as h
import run_pi5_vio as runner
import vio_core
import vio_frontend


class Pi5Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.base=Path(self.tmp.name)
        evidence=b'SYNTHETIC TEST ONLY: not physical calibration or clock proof\n'
        (self.base/'fixture.txt').write_bytes(evidence)
        ref=dict(path='fixture.txt',sha256=hashlib.sha256(evidence).hexdigest())
        self.c=dict(schema=1,mode='physical-cpu-only',execution_allowed=False,forwarding_enabled=False,
            calibration_verified=True,calibration_evidence=ref,clock_evidence=ref,
            clock_domain='ros-system-unix-ns',stamp_semantics='acquisition',clock_uncertainty_ns=100000,
            topics=dict(camera='/sensors/image',imu='/sensors/imu',range='/sensors/range'),
            frames=dict(camera='physical_camera_optical',imu='physical_imu',range='physical_range'),
            acceleration='specific-force-m/s2',angular_velocity='rad/s',attitude='imu-to-ENU',
            R_body_imu=np.diag([1,-1,-1]).tolist(),R_body_camera=h.CAMERA_FRD.tolist(),
            camera_lever_frd_m=[0,0,.02],range_lever_frd_m=[0,0,.01],range_axis_frd=[0,0,1],
            surface='locally-static-horizontal-plane',range_sigma_m=.02,max_range_age_s=.1,max_tilt_deg=30.,
            camera=dict(K=[[200.,0,160],[0,200.,120],[0,0,1]],D=[0.]*5,width=320,height=240,distortion_model='plumb_bob'))
        self.raw=dict(force=[0,0,9.80665],gyro=[0,0,0],q=[0,0,math.sqrt(.5),math.sqrt(.5)],
            force_cov=np.eye(3)*.01,gyro_cov=np.eye(3)*.001,att_cov=np.eye(3)*.0001)
        self.epoch=1_800_000_000_000_000_001
        self.image=np.random.default_rng(42).integers(0,256,(240,320),dtype=np.uint8)

    def engine(self):return h.HardwareVio(self.c,vio_core,vio_frontend)

    def sequence(self,engine,frames=7,shift_px=0):
        rows=[];i=0
        for frame in range(frames):
            ns=self.epoch+frame*100_000_000;mono=1_000_000_000+frame*100_000_000
            while self.epoch+i*5_000_000<=ns+5_000_000:
                stamp=self.epoch+i*5_000_000
                engine.add_imu(stamp,1_000_000_000+i*5_000_000,**self.raw);i+=1
            engine.add_range(ns,mono,.99,.05,4.)
            gray=self.image
            if shift_px:
                import cv2
                gray=cv2.warpAffine(gray,np.float32([[1,0,frame*shift_px],[0,1,0]]),(320,240),borderMode=cv2.BORDER_REFLECT)
            rows.append(engine.process(gray,ns,mono,ns+10_000_000,mono+10_000_000))
        return rows

    def test_arm64_and_jazzy_python_admission(self):
        h.host_admission('aarch64','Linux',(3,12),'jazzy')
        for args in [('x86_64','Linux',(3,12),'jazzy'),('arm64','Darwin',(3,12),'jazzy'),
                     ('aarch64','Linux',(3,13),'jazzy'),('aarch64','Linux',(3,12),'humble')]:
            with self.subTest(args=args),self.assertRaises(ValueError):h.host_admission(*args)

    def test_config_passes_only_with_explicit_contract_and_hashed_evidence(self):
        self.assertEqual(h.validate_config(self.c,self.base),self.c)
        (self.base/'fixture.txt').write_text('changed')
        with self.assertRaisesRegex(ValueError,'evidence'):h.validate_config(self.c,self.base)

    def test_example_intentionally_refuses_without_physical_measurements(self):
        with self.assertRaisesRegex(ValueError,'verified acquisition'):h.load_config(Path(h.__file__).with_name('pi5_vio.example.json'))

    def test_unsafe_config_and_clock_claims_are_refused(self):
        for key,value in [('execution_allowed',True),('forwarding_enabled',True),('clock_domain','unknown'),
                          ('stamp_semantics','host-receipt'),('calibration_verified',False),
                          ('clock_uncertainty_ns',5_000_000),('acceleration','gravity-removed'),
                          ('attitude','unavailable'),('range_sigma_m',float('nan')),('max_tilt_deg',61),
                          ('max_range_age_s',1.)]:
            c=copy.deepcopy(self.c);c[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):h.validate_config(c,self.base)

    def test_gazebo_or_mavros_are_not_sensor_adapters(self):
        for name in ['/world/iris/camera','/mavros/local_position/pose','/ground_truth/height','/clock']:
            c=copy.deepcopy(self.c);c['topics']['camera']=name
            with self.subTest(name=name),self.assertRaises(ValueError):h.validate_config(c,self.base)

    def test_bad_mount_and_reflection_refused(self):
        c=copy.deepcopy(self.c);c['R_body_camera']=np.eye(3).tolist()
        with self.assertRaisesRegex(ValueError,'Camera'):h.validate_config(c,self.base)
        with self.assertRaisesRegex(ValueError,'SO'):h.rotation(np.diag([1,1,-1]))

    def test_raw_integer_timestamp_preserved(self):
        ns=h.stamp_ns(S(sec=1_800_000_000,nanosec=1));self.assertEqual(ns,self.epoch)
        for stamp in [S(sec=1.,nanosec=1),S(sec=0,nanosec=0),S(sec=1,nanosec=10**9)]:
            with self.assertRaises(ValueError):h.stamp_ns(stamp)

    def test_enu_flu_specific_force_and_gravity_compensation_once(self):
        m=h.imu_measurement(self.c,**self.raw)
        np.testing.assert_allclose(m['r'],np.eye(3),atol=1e-14)
        np.testing.assert_allclose(m['f'],[0,0,-9.80665])
        ekf=vio_core.VioEKF();ekf.predict(m['f'],m['r'],.01)
        np.testing.assert_allclose(ekf.x[:6],np.zeros(6),atol=1e-12)

    def test_real_range_scale_includes_measured_levers(self):
        root,camera=h.range_heights(self.c,.99,.05,4.,np.eye(3))
        self.assertAlmostEqual(root,1.);self.assertAlmostEqual(camera,.98)
        r=vio_core.rot_body_to_ned(0,math.radians(20),0)
        root,camera=h.range_heights(self.c,1.,.05,4.,r)
        self.assertAlmostEqual(root,1.01*math.cos(math.radians(20)))
        self.assertAlmostEqual(camera,.99*math.cos(math.radians(20)))

    def test_missing_out_of_range_and_high_tilt_height_refused(self):
        for slant in [float('nan'),float('inf'),0,.05,4.]:
            with self.assertRaises(ValueError):h.range_heights(self.c,slant,.05,4.,np.eye(3))
        with self.assertRaisesRegex(ValueError,'Tilt'):
            h.range_heights(self.c,1.,.05,4.,vio_core.rot_body_to_ned(0,math.radians(31),0))

    def test_real_cpu_frontend_ekf_static_sequence_and_exact_stamp(self):
        engine=self.engine();rows=self.sequence(engine)
        self.assertTrue(rows[-1]['ready']);self.assertGreater(engine.counts['estimates'],0)
        self.assertEqual(rows[-1]['stamp_ns'],self.epoch+600_000_000)
        np.testing.assert_allclose(rows[-1]['p'],0,atol=1e-10)
        self.assertEqual(rows[-1]['max_imu_gap_ns'],5_000_000)
        h.check_publication(rows[-1],rows[-1]['stamp_ns']+20_000_000)

    def test_actual_cpu_flow_sign_and_metric_scale_on_translated_fixture(self):
        row=self.sequence(self.engine(),shift_px=2)[-1]
        self.assertTrue(row['ready']);self.assertLess(row['p'][1],0)
        self.assertAlmostEqual(row['v'][1],-.98/200*2/.1,delta=.01)
        self.assertAlmostEqual(row['v'][0],0,delta=.01)

    def test_height_update_runs_at_its_stamp_and_late_sample_is_rejected(self):
        e=self.engine();self.sequence(e,1)
        for i in range(2,22):e.add_imu(self.epoch+i*5_000_000,1_000_000_000+i*5_000_000,**self.raw)
        e.add_range(self.epoch+50_000_000,1_050_000_000,.99,.05,4.)
        times=[];original=e.ekf.update_height
        def measured(*args,**kwargs):
            times.append(e.filter_ns);return original(*args,**kwargs)
        with patch.object(e.ekf,'update_height',side_effect=measured):
            e.process(self.image,self.epoch+100_000_000,1_100_000_000,self.epoch+110_000_000,1_110_000_000)
        self.assertEqual(times,[self.epoch+50_000_000])
        e.add_range(self.epoch+90_000_000,1_120_000_000,.99,.05,4.)
        for i in range(22,42):e.add_imu(self.epoch+i*5_000_000,1_000_000_000+i*5_000_000,**self.raw)
        with self.assertRaisesRegex(ValueError,'no rewind'):
            e.process(self.image,self.epoch+200_000_000,1_200_000_000,self.epoch+210_000_000,1_210_000_000)
        self.assertFalse(e.gate.ready)

    def test_no_imu_watermark_or_range_never_produces_estimate(self):
        for missing in ('imu','range'):
            engine=self.engine()
            if missing!='imu':
                engine.add_imu(self.epoch-1_000_000,1_000_000_000,**self.raw)
                engine.add_imu(self.epoch+1_000_000,1_001_000_000,**self.raw)
            if missing!='range':engine.add_range(self.epoch,1_000_000_000,1.,.05,4.)
            with self.subTest(missing=missing),self.assertRaises(ValueError):
                engine.process(self.image,self.epoch,1_000_000_000,self.epoch+10_000_000,1_010_000_000)
            self.assertEqual(engine.counts['ready'],0)

    def test_imu_duplicates_reset_gap_nonfinite_latch(self):
        for delta in (0,-1,20_000_001):
            e=self.engine();e.add_imu(self.epoch,1_000_000_000,**self.raw)
            with self.subTest(delta=delta),self.assertRaises(ValueError):e.add_imu(self.epoch+delta,1_001_000_000,**self.raw)
            with self.assertRaises(ValueError):e.add_imu(self.epoch+5_000_000,1_002_000_000,**self.raw)
        e=self.engine();bad=dict(self.raw,force=[float('nan'),0,0])
        with self.assertRaises(ValueError):e.add_imu(self.epoch,1_000_000_000,**bad)

    def test_imu_and_camera_receipt_dropout_fails_closed(self):
        e=self.engine();self.sequence(e)
        with self.assertRaisesRegex(ValueError,'receipt'):e.check_alive(2_200_000_000)
        self.assertFalse(e.gate.ready)

    def test_duplicate_camera_and_stale_acquisition(self):
        for kind in ('duplicate','stale','future'):
            e=self.engine();self.sequence(e,1)
            ns=self.epoch if kind=='duplicate' else self.epoch+100_000_000
            now=ns+180_000_001 if kind=='stale' else ns-1
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                e.process(self.image,ns,1_100_000_000,now,1_110_000_000)

    def test_stale_range_and_wrong_image_shape(self):
        e=self.engine();self.sequence(e,1)
        for i in range(2,42):e.add_imu(self.epoch+i*5_000_000,1_000_000_000+i*5_000_000,**self.raw)
        with self.assertRaisesRegex(ValueError,'range'):e.process(self.image,self.epoch+200_000_000,1_200_000_000,self.epoch+210_000_000,1_210_000_000)
        with self.assertRaisesRegex(ValueError,'dimensions'):self.engine().rectified(np.zeros((10,10),np.uint8))

    def test_low_contrast_no_ready_and_no_constant_fallback(self):
        self.image[:]=0;e=self.engine();rows=self.sequence(e)
        self.assertFalse(any(r['ready'] for r in rows));self.assertEqual(e.counts['estimates'],0)

    def test_invalid_covariance_quaternion_and_unknown_attitude(self):
        for key,value in [('force_cov',np.diag([-1,1,1])),('gyro_cov',np.zeros((3,3))),
                          ('att_cov',np.diag([float('inf'),1,1])),('q',[0,0,0,0])]:
            with self.subTest(key=key),self.assertRaises(ValueError):h.imu_measurement(self.c,**dict(self.raw,**{key:value}))

    def test_bad_posterior_covariance_blocks_readiness(self):
        e=self.engine();e.ekf.P[0,0]=-1;rows=self.sequence(e)
        self.assertFalse(any(r['ready'] for r in rows))

    def test_final_publication_age_and_covariance_rechecked(self):
        row=self.sequence(self.engine())[-1]
        with self.assertRaisesRegex(ValueError,'stale'):h.check_publication(row,row['stamp_ns']+180_000_001)
        row['P_vel'][0,0]=-1
        with self.assertRaises(ValueError):h.check_publication(row,row['stamp_ns']+10_000_000)

    def test_range_uses_its_acquisition_attitude_and_is_not_refused_repeatedly(self):
        e=self.engine();self.sequence(e,1)
        with patch.object(e.ekf,'update_height',wraps=e.ekf.update_height) as update:
            for i in range(2,12):e.add_imu(self.epoch+i*5_000_000,1_000_000_000+i*5_000_000,**self.raw)
            e.process(self.image,self.epoch+50_000_000,1_050_000_000,self.epoch+60_000_000,1_060_000_000)
            update.assert_not_called()  # same physical range sample, not new information
            e.add_range(self.epoch+50_000_000,1_055_000_000,.99,.05,4.)
            for i in range(12,22):e.add_imu(self.epoch+i*5_000_000,1_000_000_000+i*5_000_000,**self.raw)
            e.process(self.image,self.epoch+100_000_000,1_100_000_000,self.epoch+110_000_000,1_110_000_000)
            update.assert_called_once()

    def test_force_noise_not_reduced_and_future_range_not_used(self):
        e=self.engine();high=dict(self.raw,force_cov=np.eye(3)*4.)
        for i in range(2):e.add_imu(self.epoch+i*5_000_000,1_000_000_000+i*5_000_000,**high)
        e.add_range(self.epoch,1_000_000_000,.99,.05,4.)
        e.process(self.image,self.epoch,1_000_000_000,self.epoch+10_000_000,1_010_000_000)
        for i in range(2,22):e.add_imu(self.epoch+i*5_000_000,1_000_000_000+i*5_000_000,**high)
        e.add_range(self.epoch+100_000_000,1_100_000_000,.99,.05,4.)
        e.process(self.image,self.epoch+100_000_000,1_100_000_000,self.epoch+110_000_000,1_110_000_000)
        self.assertGreaterEqual(e.ekf.sigma_acc,2.)
        e=self.engine();e.add_imu(self.epoch,1_000_000_000,**self.raw)
        e.add_imu(self.epoch+1_000_000,1_001_000_000,**self.raw)
        e.add_range(self.epoch+1,1_000_000_000,1.,.05,4.)
        with self.assertRaisesRegex(ValueError,'range'):
            e.process(self.image,self.epoch,1_000_000_000,self.epoch+10_000_000,1_010_000_000)

    def test_missing_range_does_not_enable_candidate_publish(self):
        e=self.engine();rows=self.sequence(e)
        self.assertTrue(rows[-1]['ready'])
        e.receipts.update(imu=2_200_000_000,camera=2_200_000_000)
        with self.assertRaisesRegex(ValueError,'range'):e.check_alive(2_200_000_000)
        self.assertFalse(e.gate.ready)

    def test_missing_config_prevents_ros_initialization(self):
        config=self.base/'bad.json';config.write_text('{}')
        with patch('sys.argv',['run_pi5_vio','--config',str(config),'--vio-dir',str(self.base),'--receive']),\
             patch.object(runner,'load_vio') as deps:
            with self.assertRaises(ValueError):runner.main()
            deps.assert_not_called()

    def test_changed_vio_dependency_refused_before_import(self):
        for name in h.FILES:(self.base/name).write_text('modified')
        with self.assertRaisesRegex(ValueError,'Unpinned'):h.load_vio(self.base)


if __name__=='__main__':unittest.main()
