"""Offline fixtures validate the analyzer; real wire evidence is separate."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.append(str(Path(__file__).resolve().parents[1]))
from analyze_mavros_wire import expected
from mavros_wire_relay import validate


class WireContractTests(unittest.TestCase):
    def row(self,kind):
        a=np.arange(36).reshape(6,6)*.001+np.eye(6)
        return dict(kind=kind,stamp_ns=123456789000,frame='odom',
                    values=[1.,2.,3.,0.,0.,0.,1.] if kind=='pose' else [1.,2.,3.,4.,5.,6.],
                    covariance=(a@a.T).ravel().tolist())
    def test_pose_ned_frd_and_full_upper_triangle(self):
        row=self.row('pose');out=expected(validate(row));self.assertEqual(out['position'],[2,1,-3])
        np.testing.assert_allclose(out['rpy'],[0,0,np.pi/2],atol=1e-12)
        self.assertEqual(len(out['covariance']),21)
        c=np.array([[0,1,0],[1,0,0],[0,0,-1]]);j=np.kron(np.eye(2),c)
        p=j@np.reshape(row['covariance'],(6,6))@j.T
        np.testing.assert_allclose(out['covariance'],[p[i,k] for i in range(6) for k in range(i,6)])
    def test_speed_world_ned_covariance(self):
        out=expected(validate(self.row('speed')));self.assertEqual(out['speed'],[2,1,-3])
        self.assertEqual(len(out['covariance']),9)
    def test_bad_relay_data(self):
        for change in [dict(stamp_ns=0),dict(frame='odom_ned'),dict(values=[float('nan')]*7),dict(covariance=[0.]*36)]:
            with self.assertRaises(ValueError):validate(dict(self.row('pose'),**change))
        self.assertEqual(validate(dict(kind='clock',stamp_ns=123))['stamp_ns'],123)


if __name__=='__main__':unittest.main()
