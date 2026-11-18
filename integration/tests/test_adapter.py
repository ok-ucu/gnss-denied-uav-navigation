"""Offline covariance/transport scenarios. No autopilot or ROS process involved."""
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vio_adapter import convert_pair, validate_covariance
from test_offline import odometry


def sample(second=10, scale=1):
    msg = odometry()
    msg.header.stamp.sec = second
    msg.header.stamp.nanosec = 0
    cov = [0.]*36
    for i, value in enumerate((1., 4., 9., .01, .02, .03)):
        cov[i*6+i] = value*scale
    cov[1] = cov[6] = .5*scale
    msg.pose.covariance = [v*scale for v in msg.pose.covariance]
    msg.twist = NS(covariance=cov, twist=NS(linear=NS(x=2., y=1., z=-3.),
                                          angular=NS(x=.1, y=.2, z=.3)))
    return msg


class CovarianceTests(unittest.TestCase):
    def test_singular_psd_and_scale_invariance(self):
        for scale in (1e-24, 1., 1e24):
            # rank-one covariance with a zero leading row/column
            vector = [0., 1., -2., 3., 0., 1e-8]
            cov = [scale*a*b for a in vector for b in vector]
            validate_covariance(cov)
            # Positive definite, very small leading pivot and nonzero correlation.
            cov = [0.]*36
            for i in range(6): cov[7*i] = scale
            cov[0] = scale*1e-20
            cov[1] = cov[6] = scale*5e-11
            validate_covariance(cov)

    def test_invalid_matrices(self):
        identity = [float(i == j) for i in range(6) for j in range(6)]
        cases = [[], [0.]*36, [math.nan]*36, [math.inf]*36]
        asymmetric = identity.copy(); asymmetric[1] = .01
        indefinite = identity.copy(); indefinite[1] = indefinite[6] = 2.
        negative = identity.copy(); negative[0] = -1e-30
        zero_pivot = identity.copy(); zero_pivot[0] = 0
        zero_pivot[1] = zero_pivot[6] = .1
        cases.extend([asymmetric, indefinite, negative, zero_pivot])
        for cov in cases:
            with self.subTest(cov=cov):
                with self.assertRaises(ValueError): validate_covariance(cov)


class SpeedTests(unittest.TestCase):
    def test_body_to_enu_rotation_covariance_and_timestamp(self):
        msg = sample()
        msg.pose.pose.orientation.z = math.sqrt(.5)
        msg.pose.pose.orientation.w = math.sqrt(.5)
        msg.twist.covariance[3] = msg.twist.covariance[18] = .02
        original = copy.deepcopy(msg)
        pose, speed, stamp = convert_pair(msg, NS, NS, 10_000_000_000)
        self.assertEqual(stamp, 10_000_000_000)
        self.assertEqual(speed.header, pose.header)
        self.assertEqual(speed.header.frame_id, 'odom')
        velocity = speed.twist.twist.linear
        for actual, expected in zip((velocity.x, velocity.y, velocity.z), (-1,2,-3)):
            self.assertAlmostEqual(actual, expected)
        cov = speed.twist.covariance
        for index, expected in ((0,4), (7,1), (14,9), (1,-.5), (6,-.5)):
            self.assertAlmostEqual(cov[index], expected)
        self.assertAlmostEqual(cov[10], .02)  # rotated linear/angular cross term
        self.assertAlmostEqual(cov[25], .02)
        self.assertEqual(msg, original)

    def test_no_invented_velocity_or_covariance(self):
        for kind in ('missing', 'zero_cov', 'missing_vertical', 'nan'):
            msg = sample()
            if kind == 'missing': del msg.twist
            if kind == 'zero_cov': msg.twist.covariance = [0.]*36
            if kind == 'missing_vertical': msg.twist.covariance[14] = 0
            if kind == 'nan': msg.twist.twist.linear.x = math.nan
            with self.subTest(kind=kind):
                with self.assertRaises(ValueError):
                    convert_pair(msg, NS, NS, 10_000_000_000)

    def test_normal_inflated_dropout_recovery_scenario(self):
        # Fake publishers observe only adapter conversion, never EKF behaviour.
        pose_pub, speed_pub = Mock(), Mock()
        last = None
        def deliver(msg, now):
            nonlocal last
            pose, speed, ns = convert_pair(msg, NS, NS, now, last)
            pose_pub.publish(pose)
            speed_pub.publish(speed)
            last = ns
        deliver(sample(10), 10_000_000_000)             # normal
        deliver(sample(11, scale=10), 11_000_000_000)  # inflated uncertainty
        normal = speed_pub.publish.call_args_list[0].args[0]
        inflated = speed_pub.publish.call_args_list[1].args[0]
        self.assertEqual(normal.twist.twist, inflated.twist.twist)
        normal_pose = pose_pub.publish.call_args_list[0].args[0]
        inflated_pose = pose_pub.publish.call_args_list[1].args[0]
        self.assertEqual(inflated_pose.pose.covariance,
                         [10*v for v in normal_pose.pose.covariance])
        self.assertEqual(inflated.twist.covariance,
                         [10*v for v in normal.twist.covariance])
        # During silence no new message exists. Delayed old data must fail.
        before = speed_pub.publish.call_count
        with self.assertRaises(ValueError): deliver(sample(12), 15_000_000_000)
        self.assertEqual(speed_pub.publish.call_count, before)
        self.assertEqual(pose_pub.publish.call_count, before)
        deliver(sample(16), 16_000_000_000)             # fresh recovery
        self.assertEqual(speed_pub.publish.call_count, 3)
        recovered = speed_pub.publish.call_args.args[0]
        self.assertEqual(recovered.twist.covariance, normal.twist.covariance)
        self.assertEqual(last, 16_000_000_000)
        with self.assertRaises(ValueError): deliver(sample(16), 16_000_000_000)
        self.assertEqual(speed_pub.publish.call_count, 3)


class LaunchStaticTests(unittest.TestCase):
    def test_node_launch_arguments_and_speed_not_denied(self):
        root = Path(__file__).resolve().parents[1]
        launch = ET.parse(root/'mavros.launch.xml').getroot()
        include = launch.find('include')
        self.assertTrue(include.attrib['file'].endswith('/launch/node.launch'))
        names = {arg.attrib['name'] for arg in include.findall('arg')}
        self.assertTrue({'gcs_url','fcu_url','tgt_system','tgt_component',
                         'pluginlists_yaml','config_yaml'} <= names)
        denied = [line.strip()[2:] for line in (root/'mavros_plugins.yaml').read_text().splitlines()
                  if line.strip().startswith('- ')]
        self.assertNotIn('vision_speed_estimate', denied)
        self.assertNotIn('vision_pose_estimate', denied)


if __name__ == '__main__':
    unittest.main()
