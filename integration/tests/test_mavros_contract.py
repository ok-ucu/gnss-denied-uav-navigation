"""Offline contract tests: synthetic ROS duck types, not MAVROS/FCU proof."""
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
import numpy as np
sys.path.append(str(Path(__file__).resolve().parents[1]))
from vio_adapter import convert_pair,convert,validate_covariance
from mavros_epoch import EpochJoin


def odom():
    def v(x,y,z):return S(x=x,y=y,z=z)
    a=np.arange(36).reshape(6,6)*.001+np.eye(6)*.1;c=(a@a.T).ravel().tolist()
    return S(header=S(frame_id='odom',stamp=S(sec=100,nanosec=17)),child_frame_id='base_link',
        pose=S(pose=S(position=v(1,2,3),orientation=S(x=0.,y=0.,z=2**-.5,w=2**-.5)),covariance=c),
        twist=S(twist=S(linear=v(1,2,3),angular=v(4,5,6)),covariance=c.copy()))


class MavrosContractTests(unittest.TestCase):
    def test_world_speed_covariance_and_same_stamp(self):
        m=odom();p,s,ns=convert_pair(m,S,S,100_100_000_000,max_age=.2)
        np.testing.assert_allclose([s.twist.twist.linear.x,s.twist.twist.linear.y,s.twist.twist.linear.z],[-2,1,3],atol=1e-14)
        r=np.array([[0,-1,0],[1,0,0],[0,0,1]]);j=np.zeros((6,6));j[:3,:3]=r;j[3:,3:]=r
        np.testing.assert_allclose(np.reshape(s.twist.covariance,(6,6)),j@np.reshape(m.twist.covariance,(6,6))@j.T,atol=1e-14)
        self.assertEqual(p.pose.covariance,m.pose.covariance)
        self.assertEqual(vars(p.header.stamp),vars(s.header.stamp))
    def test_bad_age_frames_covariance_and_duplicates(self):
        for kwargs in [dict(max_age=float('nan')),dict(future_tolerance=float('inf')),dict(last_ns=100_000_000_017)]:
            with self.assertRaises(ValueError):convert(odom(),S,100_100_000_000,**kwargs)
        with self.assertRaises(ValueError):convert_pair(odom(),S,S,101_000_000_000,max_age=.2)
        m=odom();m.twist.covariance[0]=-1
        with self.assertRaises(ValueError):convert_pair(m,S,S,100_100_000_000)
        m=odom();m.header.frame_id='odom_ned'
        with self.assertRaises(ValueError):convert_pair(m,S,S,100_100_000_000)
    def test_epoch_exact_join_initial_approval_and_reset_latch(self):
        g=EpochJoin();tag=dict(session='sessionA',epoch=0,stamp_ns=100,epoch_start_ns=99,ready=True)
        g.tag(tag)
        with self.assertRaises(ValueError):g.consume(100)
        g.approve(('sessionA',0));g.consume(100)
        with self.assertRaises(ValueError):g.consume(100)
        g.tag(dict(tag,epoch=1,stamp_ns=101,epoch_start_ns=101))
        with self.assertRaises(ValueError):g.consume(101)
        with self.assertRaises(ValueError):g.approve(('sessionA',1))
        with self.assertRaises(ValueError):g.tag(tag)
    def test_new_session_and_unready_tokens_fail_closed(self):
        g=EpochJoin();tag=dict(session='A',epoch=0,stamp_ns=100,epoch_start_ns=99,ready=True)
        g.tag(tag);g.approve(('A',0))
        g.tag(dict(tag,session='B',stamp_ns=101))
        self.assertTrue(g.latched)
        with self.assertRaises(ValueError):g.approve(('B',0))
        with self.assertRaises(ValueError):EpochJoin().tag(dict(tag,ready=False))
    def test_bounded_missing_tokens_and_singular_covariance(self):
        g=EpochJoin(capacity=2)
        for ns in [1,2,3]:g.tag(dict(session='A',epoch=0,stamp_ns=ns,epoch_start_ns=1,ready=True))
        g.approve(('A',0))
        with self.assertRaises(ValueError):g.consume(1)
        validate_covariance(np.diag([1.,0,0,0,0,0]).ravel().tolist())


if __name__=='__main__':unittest.main()
