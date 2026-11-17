"""Offline reconstruction of recorded monotonic boundaries, not live latency."""
import unittest
from permission_deadlines import timeline


class PermissionDeadlineTests(unittest.TestCase):
    def test_recorded_expiration_does_not_identify_socket_delay(self):
        start=16771348400000;end=16771348584559
        mapping=dict(acquisition_ns=13100000000,fcu_ns=13098180000,
            approved_host_ns=16771345705059,allowed_host_deadline_ns=16771365704059,
            publish_host_ns=end+100000)
        stages=[dict(stage='mavros_enqueue',acquisition_ns=mapping['acquisition_ns'],start_host_ns=start,end_host_ns=end)]
        wire=[dict(message=dict(usec=mapping['fcu_ns']//1000,mavpackettype='VISION_SPEED_ESTIMATE'),
            arrival_host_ns=16771369377475,checked_host_ns=16771370600184,valid_capture=False)]
        result=timeline(stages,[mapping],wire)[0]
        self.assertAlmostEqual(result['remaining_at_enqueue_end_ms'],17.119500)
        self.assertAlmostEqual(result['enqueue_end_to_userspace_receive_ms'],20.792916)
        self.assertAlmostEqual(result['userspace_receive_to_final_check_ms'],1.222709)
        self.assertAlmostEqual(result['remaining_at_final_check_ms'],-4.896125)
        self.assertAlmostEqual(result['mapping_log_after_enqueue_ms'],.1)
        self.assertIsNone(result['kernel_receive_host_ns'])
        self.assertIsNone(result['socket_queue_wait_ms'])
        self.assertIsNone(result['sender_processing_ms'])

    def test_conversion_expiration_has_no_downstream_measurement(self):
        stage=dict(stage='permission_created',source_ns=13300000000,
            host_ns=13972731762328,deadline_host_ns=13972727810744)
        row=timeline([stage],[],[])[0]
        self.assertEqual(row['status'],'NOT ENQUEUED')
        # Raw marker host timestamp is1.334us later than its precomputed
        # remaining_ns field. Use the actual marker boundary, not that field.
        self.assertAlmostEqual(row['remaining_at_created_marker_ms'],-3.951584)
        self.assertIn('NOT TESTED',row['downstream_latency'])

    def test_missing_enqueue_is_not_replaced_by_mapping_log(self):
        mapping=dict(acquisition_ns=1,fcu_ns=1000,publish_host_ns=2)
        row=timeline([],[mapping],[dict(message=dict(usec=1))])[0]
        self.assertEqual(row['status'],'NOT TESTED')

    def test_unaligned_host_clocks_fail_analysis(self):
        stages=[dict(stage='mavros_enqueue',acquisition_ns=1,start_host_ns=10,end_host_ns=20)]
        mapping=dict(acquisition_ns=1,fcu_ns=1000,approved_host_ns=5,allowed_host_deadline_ns=100)
        wire=[dict(message=dict(usec=1),arrival_host_ns=19,checked_host_ns=21,valid_capture=True)]
        with self.assertRaises(ValueError):timeline(stages,[mapping],wire)


if __name__=='__main__':unittest.main()
