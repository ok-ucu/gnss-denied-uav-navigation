"""Analytic conversions at Pi IMU origin, synthetic Odometry, no ROS/FCU."""
import copy
import unittest
import numpy as np

from physical_vio_contract import PiCandidateGuard, mavlink_preview
from navigation_clock import ClockMapping
from linux_gpu.verified_disarmed import load_codec
from test_vio_frame_bridge import message


def candidate():
    m = message()
    m.header.frame_id, m.child_frame_id = 'pi5_odom_ned', 'pi5_imu_frd'
    return m


class PhysicalVioTests(unittest.TestCase):
    def guard(self):
        return PiCandidateGuard('A', 'reference-A', clock_domain='ros-system-unix-ns', clock_uncertainty_ns=1000)

    def admit(self, guard, m=None, **kwargs):
        c = dict(session='A', reference='reference-A', epoch=0, ready=True,
                 now_system_ns=100_130_000_000, receipt_host_ns=1_000_000_000)
        c.update(kwargs)
        return guard.admit(m or candidate(), **c)

    def test_real_basis_rotation_cross_covariance_and_world_velocity(self):
        m = candidate(); before = copy.deepcopy(m)
        out = self.admit(self.guard(), m)
        enu = out['odometry']
        self.assertEqual((enu.header.frame_id, enu.child_frame_id), ('pi5_odom_enu', 'pi5_imu_flu'))
        np.testing.assert_allclose(list(vars(enu.pose.pose.position).values()), [2, 1, -3])
        # identity NED/FRD orientation: world NED velocity equals body FRD [4,5,6]
        np.testing.assert_allclose(list(vars(out['speed'].twist.twist.linear).values()), [5, 4, -6], atol=1e-12)
        n = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.]])
        j = np.zeros((6, 6)); j[:3, :3] = n; j[3:, 3:] = n
        np.testing.assert_allclose(np.reshape(out['pose'].pose.covariance, (6, 6)),
                                   j@np.reshape(m.pose.covariance, (6, 6))@j.T)
        np.testing.assert_allclose(np.reshape(out['speed'].twist.covariance, (6, 6)),
                                   j@np.reshape(m.twist.covariance, (6, 6))@j.T, atol=1e-14)
        self.assertEqual(out['acquisition_ns'], 100_123_456_789)
        self.assertFalse(out['forwarding_authorized']); self.assertEqual(m, before)

    def test_unknown_angular_covariance_is_rejected_not_approximated(self):
        for value in (-1., 0., float('nan')):
            m = candidate(); m.twist.covariance[21] = value
            with self.assertRaises(ValueError): self.admit(self.guard(), m)

    def test_bad_frames_covariance_quaternion_and_clock_configuration(self):
        for change in (lambda m: setattr(m.header, 'frame_id', 'odom_ned'),
                       lambda m: setattr(m.pose.pose.orientation, 'w', 2.),
                       lambda m: m.pose.covariance.__setitem__(1, 999),
                       lambda m: setattr(m.header.stamp, 'nanosec', 1_000_000_000)):
            m = candidate(); change(m)
            with self.assertRaises(ValueError): self.admit(self.guard(), m)
        for domain, uncertainty in (('unknown', 0), ('ros-system-unix-ns', 5_000_000), ('ros-system-unix-ns', True)):
            with self.assertRaises(ValueError):
                PiCandidateGuard('A', 'ref', clock_domain=domain, clock_uncertainty_ns=uncertainty)

    def test_stale_future_unready_reset_and_new_session_latch(self):
        for change in (dict(now_system_ns=101_000_000_000), dict(now_system_ns=100_000_000_000),
                       dict(ready=False), dict(epoch=1), dict(epoch=True), dict(session='B'), dict(reference='new')):
            g = self.guard()
            with self.subTest(change=change), self.assertRaises(ValueError): self.admit(g, **change)
            with self.assertRaises(ValueError): self.admit(g)  # cannot auto-recover

    def test_duplicate_timestamp_receipt_dropout_and_clock_reset(self):
        g = self.guard(); self.admit(g)
        with self.assertRaises(ValueError): self.admit(g)
        g = self.guard(); self.admit(g)
        with self.assertRaisesRegex(ValueError, 'loss'):
            g.check_availability(1_500_000_001)
        with self.assertRaises(ValueError): self.admit(g)
        g = self.guard(); self.admit(g)
        m = candidate(); m.header.stamp.nanosec += 1
        with self.assertRaises(ValueError): self.admit(g, m, now_system_ns=100_129_000_000)

    def model(self, verified=True):
        anchors = [dict(source_ns=100_000_000_000+i*100_000_000,
                        target_ns=1_000_000_000+i*100_000_000, host_ns=1_000_000_000+i*100_000_000) for i in range(3)]
        return ClockMapping(anchors, source_epoch='A', verified=verified, uncertainty_ns=1000)

    def preview(self, c, g, model=None, **kwargs):
        params = dict(guard=g, now_system_ns=100_200_000_000, source_system=42, source_component=197)
        params.update(kwargs)
        return mavlink_preview(c, model or self.model(), 1_200_000_000, **params)

    def test_offline_mavlink_preview_packet_types_covariance_units_and_mapped_acquisition(self):
        g = self.guard(); c = self.admit(g)
        out = self.preview(c, g)
        parser = load_codec().MAVLink(None)
        messages = parser.parse_buffer(out['position']+out['velocity'])
        self.assertEqual([m.get_type() for m in messages], ['VISION_POSITION_ESTIMATE', 'VISION_SPEED_ESTIMATE'])
        self.assertEqual([len(m.covariance) for m in messages], [21, 9])
        self.assertEqual(messages[0].usec, 1_123_456)
        self.assertEqual(messages[0].usec, messages[1].usec)
        self.assertEqual([messages[0].x, messages[0].y, messages[0].z], [1,2,3])
        np.testing.assert_allclose([messages[1].x, messages[1].y, messages[1].z], [4,5,6])
        self.assertEqual([m.reset_counter for m in messages], [0,0])
        self.assertFalse(out['forwarding_authorized'])
        self.assertTrue(out['non_transmittable'])
        self.assertEqual([m.get_seq() for m in messages], [0, 1])
        self.assertIn('NOT a continuous stream', out['sequence_scope'])
        repeat = self.preview(c, g)
        again = load_codec().MAVLink(None).parse_buffer(repeat['position']+repeat['velocity'])
        self.assertEqual([m.get_seq() for m in again], [0, 1])  # explicitly isolated fixtures

    def test_preview_rejects_unverified_expired_and_wrong_clock_epoch(self):
        for model, now in ((self.model(False), 1_200_000_000), (self.model(), 1_450_000_001)):
            g = self.guard(); c = self.admit(g)
            with self.assertRaises(ValueError):
                mavlink_preview(c, model, now, guard=g, now_system_ns=100_200_000_000,
                                source_system=42, source_component=197)
        g = self.guard(); c = self.admit(g)
        c['session'] = 'B'
        with self.assertRaises(ValueError):
            self.preview(c, g)

    def test_final_boundary_rejects_each_mutated_field_and_latches(self):
        changes = [lambda c:setattr(c['pose'].pose.pose.position, 'x', float('nan')),
                   lambda c:setattr(c['speed'].twist.twist.linear, 'y', float('inf')),
                   lambda c:c['pose'].pose.covariance.__setitem__(0, -1),
                   lambda c:c['speed'].twist.covariance.__setitem__(1, 100),
                   lambda c:setattr(c['odometry'].pose.pose.orientation, 'w', 2.),
                   lambda c:setattr(c['pose'].header.stamp, 'nanosec', 2),
                   lambda c:c.update(acquisition_ns=c['acquisition_ns']+1),
                   lambda c:c.update(epoch=False), lambda c:c.update(ready=False),
                   lambda c:c.update(reference='wrong'), lambda c:c.update(forwarding_authorized=True),
                   lambda c:c.update(_admission=object()),
                   lambda c:c.update(receipt_host_ns=0), lambda c:c.update(clock_domain='unknown')]
        for change in changes:
            g = self.guard(); c = self.admit(g)
            change(c)
            with self.subTest(change=change), self.assertRaises(ValueError): self.preview(c, g)
            self.assertIsNotNone(g.failure)

    def test_final_boundary_rejects_stale_data_reset_loss_and_replaced_candidate(self):
        g = self.guard(); c = self.admit(g)
        with self.assertRaisesRegex(ValueError, 'Stale'):
            self.preview(c, g, now_system_ns=c['acquisition_ns']+180_000_000)
        g = self.guard(); c = self.admit(g)
        with self.assertRaises(ValueError): g.check_availability(1_500_000_001)
        with self.assertRaises(ValueError): self.preview(c, g)
        g = self.guard(); old = self.admit(g)
        m = candidate(); m.header.stamp.nanosec += 1
        self.admit(g, m, receipt_host_ns=1_100_000_000)
        with self.assertRaisesRegex(ValueError, 'admission'): self.preview(old, g)
        g = self.guard(); c = self.admit(g)
        with self.assertRaisesRegex(ValueError, 'clock reset'):
            self.preview(c, g, now_system_ns=100_129_000_000)

    def test_unadmitted_or_foreign_guard_candidate_is_rejected(self):
        g = self.guard(); c = self.admit(g)
        with self.assertRaises(ValueError): self.preview(c, self.guard())
        with self.assertRaises(ValueError): self.preview(c, None)

    def test_queued_receipt_is_not_confused_with_later_consumer_poll(self):
        g = self.guard(); c = self.admit(g)
        self.preview(c, g)
        m = candidate(); m.header.stamp.nanosec += 1
        # Arrived before the previous poll, but is newer than the last receipt.
        self.admit(g, m, receipt_host_ns=1_100_000_000, now_system_ns=100_200_000_000)
        self.assertIsNone(g.failure)


if __name__ == '__main__': unittest.main()
