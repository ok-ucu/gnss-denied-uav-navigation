"""Offline experiment tests. Fake clocks/transports are not SITL evidence."""
import csv
import importlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, mock_open, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import test_ekf_covariance as tool


class Clock:
    def __init__(self): self.now = 0.
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


def sample(stamp):
    return tool.Measurement(stamp, (1.,2.,-3.), (.1,.2,.3), (.4,.5,-.6))


class CovarianceTests(unittest.TestCase):
    def test_layout_and_values(self):
        pose = tool.pose_covariance(.01, .1)
        self.assertEqual(len(pose), 21)
        self.assertEqual([pose[i] for i in (0,6,11)], [.01]*3)
        self.assertEqual([pose[i] for i in (15,18,20)], [.1]*3)
        self.assertEqual(sum(v != 0 for v in pose), 6)
        speed = tool.speed_covariance(4.)
        self.assertEqual(speed, [4.,0,0,0,4.,0,0,0,4.])
        self.assertEqual(tool.pose_covariance(4.)[0], 4.)

    def test_bad_variance_and_duration(self):
        for value in (0., -1., float('nan'), float('inf')):
            for function in (tool.pose_covariance, tool.speed_covariance):
                with self.assertRaises(ValueError): function(value)
            with self.assertRaises(ValueError): tool.phases(1, {'dropout':value})

    def test_wire_v2_roundtrip_when_available(self):
        try:
            dialect = importlib.import_module('pymavlink.dialects.v20.ardupilotmega')
        except ImportError:
            self.skipTest('pymavlink absent; real MAVLink 2 wire serialization not tested')
        sender, receiver = dialect.MAVLink(None), dialect.MAVLink(None)
        messages = [sender.vision_position_estimate_encode(1000000, 1,2,-3, .1,.2,.3,
                        covariance=tool.pose_covariance(.01)),
                    sender.vision_speed_estimate_encode(1000000, .4,.5,-.6,
                        covariance=tool.speed_covariance(4.))]
        for msg, count in zip(messages, (21,9)):
            packet = msg.pack(sender)
            self.assertEqual(packet[0], 0xFD)
            decoded = receiver.parse_char(packet)
            self.assertEqual(len(decoded.covariance), count)
            for actual, expected in zip(decoded.covariance, msg.covariance):
                self.assertAlmostEqual(actual, expected, places=6)


class ExperimentTests(unittest.TestCase):
    def run_experiment(self, master=None, source=None, stream=None):
        clock = Clock()
        master = master or Mock()
        source = source or NS(sample=lambda elapsed, now: sample(int((now+1)*1e6)))
        stream = stream or io.StringIO()
        counts = tool.experiment(master, source, stream, tool.phases(1), rate=2,
                                 clock=clock, wall=lambda:1000+clock(), sleep=clock.sleep)
        return master, list(csv.DictReader(io.StringIO(stream.getvalue()))), counts

    def test_four_phases_covariance_counts_and_dropout(self):
        master, rows, counts = self.run_experiment()
        self.assertEqual([r['phase'] for r in rows if r['event']=='phase_start'],
                         ['normal','inflated','dropout','recovery'])
        self.assertEqual(counts, (6,6))
        self.assertEqual(master.mav.vision_position_estimate_send.call_count, 6)
        self.assertEqual(master.mav.vision_speed_estimate_send.call_count, 6)
        sent = [r for r in rows if r['event']=='sent']
        self.assertEqual([float(r['configured_variance']) for r in sent], [.01,.01,4,4,.01,.01])
        for row in sent:
            self.assertEqual(len(json.loads(row['pose_covariance_sent'])),21)
            self.assertEqual(len(json.loads(row['speed_covariance_sent'])),9)
        dropout = [r for r in rows if r['event']=='dropout']
        self.assertEqual(len(dropout),2)
        self.assertTrue(all(r['position_sent']=='0' and r['speed_sent']=='0' for r in dropout))
        self.assertTrue(all(not r['pose_covariance_sent'] for r in dropout))
        self.assertEqual(rows[-1]['event'], 'complete')
        self.assertEqual(rows[-1]['total_speed_count'], '6')

    def test_partial_send_error_counts_and_flush(self):
        master, stream = Mock(), io.StringIO()
        master.mav.vision_speed_estimate_send.side_effect = OSError('network failed')
        with self.assertRaises(OSError): self.run_experiment(master=master, stream=stream)
        rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        partial = next(r for r in rows if r['event']=='send_error')
        self.assertEqual((partial['position_sent'],partial['speed_sent']),('1','0'))
        self.assertTrue(partial['pose_covariance_sent'])
        self.assertFalse(partial['speed_covariance_sent'])
        self.assertEqual(rows[-1]['total_position_count'], '1')
        self.assertEqual(rows[-1]['event'], 'error')

    def test_no_source_or_invalid_source_is_not_success(self):
        for measurement in (None, sample(-1)):
            stream = io.StringIO()
            with self.assertRaises((RuntimeError,ValueError)):
                self.run_experiment(source=NS(sample=lambda *args:measurement), stream=stream)
            self.assertIn('error', stream.getvalue())
            self.assertNotIn(',complete,', stream.getvalue())

    def test_no_duplicate_timestamp_replay(self):
        clock, master, stream = Clock(), Mock(), io.StringIO()
        with self.assertRaises(RuntimeError):
            tool.experiment(master, NS(sample=lambda *args:sample(100)), stream,
                            tool.phases(.1), rate=20, clock=clock,
                            sleep=clock.sleep)
        self.assertEqual(master.mav.vision_position_estimate_send.call_count, 1)
        self.assertIn('duplicate_or_old_measurement', stream.getvalue())


class SourceTests(unittest.TestCase):
    def test_csv_selection_stamp_staleness_and_bad_order(self):
        header = 'elapsed_s,'+','.join(tool.CsvSource.COLUMNS)+'\n'
        text = header+'0,1,2,-3,.1,.2,.3,.4,.5,-.6\n1,4,5,-6,0,0,0,1,2,3\n'
        with patch('builtins.open', mock_open(read_data=text)):
            source = tool.CsvSource('unused', max_age=.2, start_unix=1000)
        self.assertIsNone(source.sample(-.1,0))
        measurement = source.sample(.1,0)
        self.assertEqual(measurement.usec, 1000000000)
        self.assertEqual(measurement.position, (1,2,-3))
        self.assertIsNone(source.sample(.3,0))
        self.assertEqual(source.sample(1,0).velocity, (1,2,3))
        with patch('builtins.open', mock_open(read_data=header+'0,'+','.join(['1']*9)+'\n0,'+','.join(['2']*9)+'\n')):
            with self.assertRaises(ValueError): tool.CsvSource('unused')

    def test_telemetry_frames_source_skew_and_age(self):
        def msg(kind, boot, system=1, **kwargs):
            return NS(get_type=lambda:kind, get_srcSystem=lambda:system,
                      get_srcComponent=lambda:1, time_boot_ms=boot, **kwargs)
        position = msg('LOCAL_POSITION_NED',1000,x=1,y=2,z=-3,vx=4,vy=5,vz=6)
        attitude = msg('ATTITUDE',1000,roll=.1,pitch=.2,yaw=.3)
        foreign = msg('LOCAL_POSITION_NED',1000,system=2,x=99,y=99,z=99,vx=0,vy=0,vz=0)
        master = Mock(target_system=1,target_component=1)
        master.recv_match.side_effect = [foreign, position, attitude, None]
        source = tool.TelemetrySource(master)
        measurement = source.sample(0,0)
        self.assertEqual(measurement, tool.Measurement(1000000,(1,2,-3),(.1,.2,.3),(4,5,6)))
        master.recv_match.side_effect = None
        master.recv_match.return_value = None
        self.assertIsNone(source.sample(1,1))
        master.recv_match.side_effect = [msg('ATTITUDE',2000,roll=0,pitch=0,yaw=0),None]
        self.assertIsNone(source.sample(1,1))


class LifecycleTests(unittest.TestCase):
    def test_timeout_and_gcs_filter(self):
        master = Mock()
        master.recv_match.return_value = None
        with self.assertRaises(TimeoutError):
            tool.wait_autopilot(master,1,clock=Mock(side_effect=[0,0,2]))
        gcs = NS(autopilot=8)
        vehicle = NS(autopilot=3,get_srcSystem=lambda:7,get_srcComponent=lambda:1)
        master.recv_match.side_effect = [gcs,vehicle]
        tool.wait_autopilot(master,1,clock=lambda:0)
        self.assertEqual(master.target_system,7)

    def test_main_closes_on_timeout_or_transport_error(self):
        for failure in (TimeoutError('heartbeat'),OSError('transport')):
            master = Mock()
            with patch.dict(sys.modules, {'pymavlink':NS(mavutil=Mock())}), \
                 patch.object(tool,'connection',return_value=master), \
                 patch.object(tool,'wait_autopilot',side_effect=failure), \
                 patch.object(sys,'argv',['test_ekf_covariance']), \
                 patch('sys.stderr',new_callable=io.StringIO):
                self.assertEqual(tool.main(),1)
            master.close.assert_called_once()

    def test_connection_uses_explicit_v2_encoder(self):
        master, mavutil, dialect = Mock(), Mock(), Mock()
        mavutil.mavlink_connection.return_value = master
        with patch.object(tool.importlib,'import_module',return_value=dialect) as load:
            self.assertIs(tool.connection(mavutil,'unused'),master)
        load.assert_called_once_with('pymavlink.dialects.v20.ardupilotmega')
        self.assertEqual(master.WIRE_PROTOCOL_VERSION,'2.0')
        self.assertIs(master.mav,dialect.MAVLink.return_value)


if __name__ == '__main__':
    unittest.main()
