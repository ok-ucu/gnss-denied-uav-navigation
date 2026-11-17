"""Synthetic component contracts only, NOT live motion accuracy evidence."""
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,os.environ.get('VIO_SOURCE_DIR',str(Path(__file__).resolve().parents[2]/'vio')))
try:
    import numpy as np
    import cv2
    from vio_frontend import FlowFrontend
    from vio_core import VioEKF,G
except ImportError:
    np=None


@unittest.skipIf(np is None,'Requires existing NumPy/OpenCV VIO runtime')
class GeometryTests(unittest.TestCase):
    def texture(self):
        return np.random.default_rng(18).integers(0,256,(480,640),dtype=np.uint8)
    def test_image_axes_to_ned_velocity_sign_and_scale(self):
        image=self.texture()
        for dx,dy,expected in [(2,0,[0,-.1]),(0,2,[.1,0])]:
            f=FlowFrontend(200.)
            self.assertIsNone(f.process(image,1,1,np.zeros(3),np.eye(3)))
            shifted=cv2.warpAffine(image,np.float32([[1,0,dx],[0,1,dy]]),(640,480))
            result=f.process(shifted,1.1,1,np.zeros(3),np.eye(3))
            self.assertIsNotNone(result)
            np.testing.assert_allclose(result.v_xy,expected,atol=.002)
    def test_gyro_compensation_sign_contract(self):
        image=self.texture();f=FlowFrontend(200.)
        f.process(image,1,1,np.zeros(3),np.eye(3))
        result=f.process(image,1.1,1,np.array([.1,0,0]),np.eye(3))
        np.testing.assert_allclose(result.v_xy,[0,.1],atol=1e-6)
    def test_lever_arm_velocity_sign(self):
        from vio_geometry import camera_to_body_xy
        np.testing.assert_allclose(camera_to_body_xy([0,-.1],np.eye(3),[1,0,0],[0,0,.1]),[0,0],atol=1e-9)
        with self.assertRaises(ValueError):camera_to_body_xy([0,0],np.eye(3)*2,[0,0,0],[0,0,0])
    def test_height_scale(self):
        image=self.texture();shift=cv2.warpAffine(image,np.float32([[1,0,2],[0,1,0]]),(640,480))
        estimates=[]
        for h in [1.,2.]:
            f=FlowFrontend(200.);f.process(image,1,h,np.zeros(3),np.eye(3))
            estimates.append(f.process(shift,1.1,h,np.zeros(3),np.eye(3)).v_xy)
        np.testing.assert_allclose(estimates[1],2*estimates[0],atol=1e-6)
    def test_world_and_body_basis(self):
        n=np.array([[0,1,0],[1,0,0],[0,0,-1]])
        b=np.diag([1,-1,-1]);rz=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        np.testing.assert_allclose(n@rz@b,np.eye(3))
    def test_gravity_and_bias_convention(self):
        e=VioEKF();bias=np.array([.01,-.02,.03]);e.x[6:9]=bias
        for _ in range(100):e.predict(np.array([0.,0.,-G])+bias,np.eye(3),.001)
        np.testing.assert_allclose(e.x[:6],np.zeros(6),atol=1e-10)
        self.assertTrue(np.linalg.eigvalsh(e.P).min()>=0)
    def test_covariance_bias_cross_propagation(self):
        e=VioEKF();p=e.P.copy();dt=.01
        f=np.eye(9);f[:3,3:6]=np.eye(3)*dt;f[:3,6:9]=-np.eye(3)*dt*dt/2;f[3:6,6:9]=-np.eye(3)*dt
        q=np.diag([1e-6*dt]*3+[e.sigma_acc**2*dt]*3+[e.sigma_bias**2*dt]*3)
        e.predict(np.array([0.,0.,-G]),np.eye(3),dt)
        np.testing.assert_allclose(e.P,f@p@f.T+q)


if __name__=='__main__':unittest.main()
