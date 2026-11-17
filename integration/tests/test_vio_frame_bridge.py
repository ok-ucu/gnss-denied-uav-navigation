"""Analytic basis/cross-covariance regression; synthetic, not live evidence."""
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path.append(str(Path(__file__).resolve().parents[1]))
import numpy as np
from vio_frame_bridge import convert, StreamGuard, quat_matrix, matrix_quat, N, B


def message():
    def v(x=0, y=0, z=0): return S(x=x, y=y, z=z)
    rng = np.random.default_rng(99)
    a = rng.normal(size=(6, 6))*.1
    c = (a@a.T).ravel().tolist()
    return S(header=S(frame_id='odom_ned', stamp=S(sec=100, nanosec=123456789)),
             child_frame_id='base_link_frd',
             pose=S(pose=S(position=v(1, 2, 3), orientation=S(x=0., y=0., z=0., w=1.)), covariance=c),
             twist=S(twist=S(linear=v(4, 5, 6), angular=v(7, 8, 9)), covariance=c.copy()))


class FrameBridgeTests(unittest.TestCase):
    def test_vectors_frames_stamp_and_source_unchanged(self):
        m = message(); before = copy.deepcopy(m); o = convert(m)
        self.assertEqual(vars(o.pose.pose.position), dict(x=2., y=1., z=-3.))
        self.assertEqual(vars(o.twist.twist.linear), dict(x=4., y=-5., z=-6.))
        self.assertEqual(vars(o.twist.twist.angular), dict(x=7., y=-8., z=-9.))
        self.assertEqual((o.header.frame_id, o.child_frame_id), ('odom', 'base_link'))
        self.assertEqual(vars(o.header.stamp), vars(m.header.stamp))
        self.assertEqual(m, before)
    def test_full_covariance_with_cross_members(self):
        m = message(); o = convert(m)
        for actual, rot in [(o.pose.covariance, N), (o.twist.covariance, B)]:
            j = np.zeros((6, 6)); j[:3, :3] = rot; j[3:, 3:] = rot
            expected = j@np.asarray(m.pose.covariance).reshape(6, 6)@j.T
            np.testing.assert_allclose(np.reshape(actual, (6, 6)), expected, atol=1e-14)
            self.assertTrue(np.linalg.eigvalsh(np.reshape(actual, (6, 6))).min() >= -1e-12)
        self.assertNotEqual(o.twist.covariance[3], 0)  # linear/angular cross term retained
    def test_orientation_geometric_invariant_arbitrary_rotations(self):
        for q in [[0, 0, 0, 1], [1, 0, 0, 0], [.2, -.4, .1, .7]]:
            q = np.asarray(q, float); q /= np.linalg.norm(q)
            m = message(); m.pose.pose.orientation = S(**dict(zip(('x','y','z','w'), q)))
            o = convert(m); out_q = [getattr(o.pose.pose.orientation, k) for k in ('x','y','z','w')]
            r, out_r = np.asarray(quat_matrix(q)), np.asarray(quat_matrix(out_q))
            np.testing.assert_allclose(out_r, np.asarray(N)@r@np.asarray(B), atol=1e-14)
            v = np.array([.3, -.7, 2.])
            np.testing.assert_allclose(out_r@(np.asarray(B)@v), np.asarray(N)@r@v, atol=1e-14)
    def test_identity_ned_frd_becomes_enu_yaw90(self):
        q = convert(message()).pose.pose.orientation
        np.testing.assert_allclose(quat_matrix([q.x,q.y,q.z,q.w]), [[0,-1,0],[1,0,0],[0,0,1]], atol=1e-14)
    def test_bad_payload_rejected(self):
        for change in [lambda m:setattr(m.header, 'frame_id','map'),
                       lambda m:setattr(m.pose.pose.position,'x',float('nan')),
                       lambda m:setattr(m.twist.twist.angular,'y',float('inf')),
                       lambda m:setattr(m.pose.pose.orientation,'w',0),
                       lambda m:m.pose.covariance.__setitem__(0,-1),
                       lambda m:m.twist.covariance.__setitem__(0,-1),
                       lambda m:m.pose.covariance.__setitem__(1,.99),
                       lambda m:setattr(m.twist,'covariance',[0.]*35)]:
            m = message(); change(m)
            with self.assertRaises(ValueError):convert(m)
    def test_unknown_angular_legacy_is_explicit_approximation(self):
        m = message()
        for i in (3,4,5):m.twist.covariance[i*6+i] = -1
        o = convert(m)
        self.assertEqual(vars(o.twist.twist.angular), dict(x=0., y=0., z=0.))
        self.assertEqual([o.twist.covariance[i*6+i] for i in (3,4,5)], [100.]*3)
        m.twist.covariance[3*6+3] = 1
        with self.assertRaises(ValueError):convert(m)
    def test_guard_duplicates_reset_stale_clock_and_restart(self):
        g = StreamGuard(); g.check(100,100.1,1,1)
        for args in [(100,100.1,1,1),(99,99.1,1,1),(101,102,2,2),
                     (102,101,2,2),(102,102.1,3,2),(102,102.1,3,None),
                     (float('nan'),102,3,3)]:
            with self.assertRaises(ValueError):g.check(*args)
        g.check(103,103.1,4,4)  # no timer/cached message exists during silence
        StreamGuard().check(1,1.1,5,5)  # explicit new bridge epoch


if __name__ == '__main__': unittest.main()
