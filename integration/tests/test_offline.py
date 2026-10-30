"""Offline unit tests; fake ROS messages/MAVLink transports, NOT integration evidence."""
import csv
import io
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch, mock_open
import warnings

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vio_adapter import convert, expand_upper_triangle, validate_covariance
from analyze_trajectories import compare, gps_to_enu, timestamp, load, main as analyze_main
from check_integration import monitor, main as monitor_main


def odometry():
    covariance = [float(i == j) for i in range(6) for j in range(6)]
    covariance[1] = covariance[6] = 0.25
    return NS(header=NS(frame_id='odom', stamp=NS(sec=10, nanosec=123)),
              child_frame_id='base_link', pose=NS(covariance=covariance,
              pose=NS(position=NS(x=1., y=2., z=3.),
                      orientation=NS(x=0., y=0., z=0., w=1.))))


class AdapterTests(unittest.TestCase):
    def test_preserves_stamp_pose_and_all_covariance(self):
        msg = odometry()
        out, stamp = convert(msg, NS, 10000000123)
        self.assertEqual(stamp, 10000000123)
        self.assertEqual(out.pose, msg.pose)
        self.assertEqual(out.header, msg.header)
        self.assertIsNot(out.pose, msg.pose)

    def test_rejections(self):
        for mutation in ('frame', 'child', 'quaternion', 'nan', 'zero', 'asymmetric', 'negative'):
            with self.subTest(mutation=mutation):
                msg = odometry()
                if mutation == 'frame': msg.header.frame_id = 'ned'
                if mutation == 'child': msg.child_frame_id = 'camera'
                if mutation == 'quaternion': msg.pose.pose.orientation.w = 2.
                if mutation == 'nan': msg.pose.pose.position.x = math.nan
                if mutation == 'zero': msg.pose.covariance = [0.]*36
                if mutation == 'asymmetric': msg.pose.covariance[1] = 10.
                if mutation == 'negative': msg.pose.covariance[0] = -1.
                with self.assertRaises(ValueError): convert(msg, NS, 10000000123)

    def test_time_rejections(self):
        for now, last in ((20000000000, None), (9000000000, None),
                          (10000000123, 10000000123)):
            with self.assertRaises(ValueError): convert(odometry(), NS, now, last)

    def test_triangle_and_psd(self):
        cov = expand_upper_triangle(list(range(21)))
        self.assertEqual(cov[1], cov[6])
        self.assertEqual(cov[35], 20)
        msg = odometry()
        msg.pose.covariance[1] = msg.pose.covariance[6] = 2
        with self.assertRaises(ValueError): validate_covariance(msg.pose.covariance)


class AnalysisTests(unittest.TestCase):
    def test_time_units(self):
        for unit, value in [('s','1'), ('ms','1000'), ('us','1000000'), ('ns','1000000000'),
                            ('iso','1970-01-01T00:00:01Z')]:
            self.assertEqual(timestamp(value, unit), 1)
        with self.assertRaises(ValueError): timestamp('2026-01-01T00:00:00', 'iso')

    def test_csv_formats_and_bad_time_order(self):
        data = 't,x,y,z\n1000,2,3,-4\n2000,5,6,-7\n'
        with patch('builtins.open', mock_open(read_data=data)):
            rows = load('unused', 'ned', 't', 'ms', offset=10)
        self.assertEqual(rows, [(11, (3,2,4)), (12, (6,5,7))])
        with patch('builtins.open', mock_open(read_data='timestamp,east,north,up\n1,0,0,0\n1,0,0,0\n')):
            with self.assertRaises(ValueError): load('unused')

    def test_enu_column_sets_and_coordinate_order(self):
        for header in ('timestamp,east,north,up', 'timestamp,east_m,north_m,up_m'):
            with self.subTest(header=header):
                with patch('builtins.open', mock_open(read_data=header+'\n1,2,3,4\n2,5,6,7\n')):
                    self.assertEqual(load('unused'), [(1, (2,3,4)), (2, (5,6,7))])

    def test_sitl_enu_csv_and_explicit_truth_time_column_cli(self):
        truth = ('t_s,lat,lon,alt_m,north_m,east_m,up_m\n'
                 '1,50,30,100,20,10,3\n2,50,30,100,24,12,5\n')
        vio = 'timestamp,east,north,up\n1,10,20,3\n2,12,24,5\n'
        with patch('builtins.open', mock_open(read_data=truth)):
            self.assertEqual(load('truth.csv', time_column='t_s'),
                             [(1, (10,20,3)), (2, (12,24,5))])
        def open_csv(path, **kwargs):
            return io.StringIO(truth if path == 'truth.csv' else vio)
        with patch('builtins.open', side_effect=open_csv), \
             patch.object(sys, 'argv', ['analyze', 'truth.csv', 'vio.csv',
                                       '--truth-format', 'enu',
                                       '--truth-time-column', 't_s', '--synchronized']), \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(analyze_main(), 0)
        result = json.loads(output.getvalue())
        self.assertEqual(result['samples'], 2)
        self.assertEqual(result['horizontal_rmse_m'], 0)
        self.assertEqual(result['spatial_rmse_m'], 0)

    def test_enu_rejects_incomplete_column_sets(self):
        with patch('builtins.open', mock_open(read_data='timestamp,east,north_m,up_m\n1,1,2,3\n2,1,2,3\n')):
            with self.assertRaises(ValueError): load('unused')

    def test_gps_origin_and_east(self):
        origin = (0, 0, 0)
        self.assertEqual(gps_to_enu(*origin, origin), (0., 0., 0.))
        self.assertAlmostEqual(gps_to_enu(0, .001, 0, origin)[0], 111.31949, places=4)

    def test_metrics_interpolation_and_drift(self):
        truth = [(0., (0,0,0)), (1., (1,0,0)), (2., (2,0,0))]
        vio = [(.5, (.5,3,4)), (1.5, (1.5,3,4))]
        result = compare(truth, vio, True, 1)
        self.assertEqual(result['horizontal_rmse_m'], 3)
        self.assertEqual(result['spatial_rmse_m'], 5)
        self.assertEqual(result['spatial_drift_m'], 0)
        result = compare(truth, [(0., (0,0,0)), (2., (4,0,0))], True)
        self.assertEqual(result['spatial_drift_m'], 2)
        self.assertEqual(result['spatial_drift_percent'], 100)

    def test_unsynchronized_gaps_no_overlap(self):
        truth = [(0., (0,0,0)), (10., (1,0,0))]
        with warnings.catch_warnings(record=True) as caught:
            with self.assertRaises(ValueError): compare(truth, truth)
            self.assertTrue(caught)
        for vio in ([(1., (0,0,0)), (2., (0,0,0))],
                    [(20., (0,0,0)), (21., (0,0,0))]):
            with self.assertRaises(ValueError): compare(truth, vio, True)


class MonitorTests(unittest.TestCase):
    def test_stream_loss_recovery_source_filter_and_csv(self):
        now = [0.]
        def message(kind, system=1, **kw):
            return NS(get_type=lambda:kind, get_srcSystem=lambda:system,
                      get_srcComponent=lambda:1, **kw)
        messages = [None, message('HEARTBEAT', 2), None,
                    message('LOCAL_POSITION_NED', x=1, y=2, z=-3, time_boot_ms=50),
                    message('EKF_STATUS_REPORT', flags=7, pos_horiz_variance=.3,
                            velocity_variance=.1),
                    message('STATUSTEXT', severity=4, text='warning, "test"'),
                    message('HEARTBEAT')]
        def receive(**kwargs):
            now[0] += 1
            return messages.pop(0)
        master = Mock(target_system=1, target_component=1)
        master.recv_match.side_effect = receive
        stream = io.StringIO()
        monitor(master, stream, lambda m:'LOITER', 2, 7, clock=lambda:now[0], wall=lambda:100)
        rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        events = [row['event'] for row in rows]
        self.assertEqual(events.count('telemetry_lost'), 1)
        self.assertEqual(events.count('telemetry_restored'), 1)
        self.assertEqual(rows[-1]['mode'], 'LOITER')
        self.assertEqual(next(r for r in rows if r['event']=='STATUSTEXT')['text'], 'warning, "test"')
        self.assertTrue(all(r['source_system'] in ('', '1') for r in rows))

    def test_connection_timeout_closes_transport(self):
        master = Mock()
        master.recv_match.return_value = None
        mavutil = Mock()
        mavutil.mavlink_connection.return_value = master
        fake_module = NS(mavutil=mavutil)
        with patch.dict(sys.modules, {'pymavlink': fake_module}), \
             patch.object(sys, 'argv', ['monitor', '--connect-timeout', '1']), \
             patch('check_integration.time.monotonic', side_effect=[0, 0, 2]), \
             patch('sys.stderr', new_callable=io.StringIO) as error:
            self.assertEqual(monitor_main(), 1)
        self.assertIn('timeout', error.getvalue())
        master.close.assert_called_once()

    def test_transport_exception_keeps_flushed_data(self):
        master = Mock(target_system=1, target_component=1)
        master.recv_match.side_effect = OSError('transport failed')
        stream = io.StringIO()
        with self.assertRaises(OSError): monitor(master, stream, str)
        self.assertIn('connected', stream.getvalue())


if __name__ == '__main__':
    unittest.main()
