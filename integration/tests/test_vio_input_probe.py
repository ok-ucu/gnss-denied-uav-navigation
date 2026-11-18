"""Offline probe tests with explicit fake transport; not live flight evidence."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import probe_vio_inputs as p


def message(kind, version=0xFD, source=1, **fields):
    return NS(get_type=lambda: kind, get_srcSystem=lambda: source,
              get_srcComponent=lambda: 1, get_msgbuf=lambda: bytes([version]), **fields)


class FakeLink:
    def __init__(self, events):
        self.events = iter(events)
        self.now = 0
        self.mav = Mock()

    def recv_match(self, **kwargs):
        self.now += .001
        return next(self.events, None)


def run_case(orientation=25, current=100, version=0xFD, missing=None):
    events = [message('HEARTBEAT', autopilot=3, base_mode=0)]
    events += [message('COMMAND_ACK', command=511, result=0) for _ in range(4)]
    for _ in range(50):
        for kind in p.RATES:
            if kind != missing:
                fields = dict(orientation=orientation, current_distance=current,
                              min_distance=20, max_distance=5000) if kind == 'DISTANCE_SENSOR' else {}
                events.append(message(kind, version=version, **fields))
    link = FakeLink(events)
    return p.probe(link, 1, 1, True, clock=lambda: link.now), link


class ProbeTests(unittest.TestCase):
    def test_valid_streams_and_interval_commands_only(self):
        report, link = run_case()
        self.assertEqual(report['status'], 'PASS')
        self.assertTrue(report['mavlink2'])
        self.assertFalse(report['armed_observed'])
        self.assertEqual(report['streams']['RAW_IMU']['count'], 50)
        self.assertEqual(len(link.mav.command_long_send.call_args_list), 4)
        self.assertEqual([r['ack_result'] for r in report['interval_requests']], [0]*4)
        self.assertTrue(all(call.args[2] == 511 for call in link.mav.command_long_send.call_args_list))

    def test_missing_wrong_orientation_invalid_range_and_v1(self):
        for fields in [dict(missing='RAW_IMU'), dict(orientation=0),
                       dict(current=0), dict(current=5001), dict(version=0xFE)]:
            with self.subTest(fields=fields):
                report, _ = run_case(**fields)
                self.assertEqual(report['status'], 'FAIL')

    def test_heartbeat_timeout(self):
        link = FakeLink([])
        report = p.probe(link, 1, .01, clock=lambda: link.now)
        self.assertEqual(report['status'], 'FAIL')
        self.assertFalse(report['heartbeat'])

    def test_interval_rejected_and_timeout(self):
        link = FakeLink([message('COMMAND_ACK', command=511, result=3)])
        results = p.request_intervals(link, (1, 1), lambda: link.now)
        self.assertEqual(results[0]['ack_result'], 3)
        self.assertIsNone(results[1]['ack_result'])

    def test_ack_for_other_target_ignored(self):
        link = FakeLink([message('COMMAND_ACK', command=511, result=0, target_system=42),
                         message('COMMAND_ACK', command=511, result=2, target_system=245)])
        results = p.request_intervals(link, (1, 1), lambda: link.now)
        self.assertEqual(results[0]['ack_result'], 2)

    def test_ground_range_classified_as_out_of_range(self):
        report, _ = run_case(current=0)
        self.assertEqual(report['distance_samples_cm'][0]['range_status'], 'out_of_range_low')
        self.assertEqual(report['status'], 'FAIL')

    def test_v1_heartbeat_fails(self):
        link = FakeLink([message('HEARTBEAT', version=0xFE, autopilot=3, base_mode=0)])
        report = p.probe(link, .01, 1, clock=lambda: link.now)
        self.assertFalse(report['mavlink2'])
        self.assertIn('Heartbeat is not MAVLink 2', report['failures'])

    def test_transport_failure_during_measurement(self):
        link = FakeLink([])
        link.recv_match = Mock(side_effect=[message('HEARTBEAT', autopilot=3, base_mode=0),
                                          EOFError('closed')])
        report = p.probe(link, 1, 1, clock=lambda: link.now)
        self.assertEqual(report['status'], 'FAIL')
        self.assertIn('Transport error: closed', report['failures'])

    def test_rate_and_boundary_gaps(self):
        samples = {kind: [.5] for kind in p.RATES}
        report = p.summarize(samples, 2, True)
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['streams']['RAW_IMU']['hz'], .5)
        self.assertEqual(report['streams']['RAW_IMU']['max_gap_s'], 1.5)

    def test_validation(self):
        for duration in [0, -1, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                p.probe(None, duration=duration)

    def test_foreign_source_not_counted(self):
        link = FakeLink([message('HEARTBEAT', autopilot=3, base_mode=0),
                         message('RAW_IMU', source=2)])
        result = p.probe(link, .01, 1, clock=lambda: link.now)
        self.assertEqual(result['streams']['RAW_IMU']['count'], 0)

    def test_main_closes_on_transport_failure(self):
        link = Mock()
        link.recv_match.side_effect = OSError('broken')
        from pymavlink import mavutil
        with patch.object(mavutil, 'mavlink_connection', return_value=link), patch('builtins.print'):
            self.assertEqual(p.main([]), 1)
        link.close.assert_called_once()

    def test_opt_in_range(self):
        root = Path(__file__).resolve().parents[1]
        values = dict(line.split() for line in (root/'sitl_range.parm').read_text().splitlines()
                      if line and not line.startswith('#'))
        self.assertEqual(values['RNGFND1_ORIENT'], '25')
        self.assertEqual(values['RNGFND1_MIN_CM'], '20')
        self.assertEqual(values['RNGFND1_MAX_CM'], '5000')
        self.assertIn('${SITL_RANGE:=0}', (root/'start_sitl.sh').read_text())
        self.assertIn('${SITL_STREAMRATE:=10}', (root/'start_sitl.sh').read_text())

    def test_invalid_streamrate_rejected_before_launch(self):
        import os
        import subprocess
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(['bash', str(root/'start_sitl.sh')],
                                env=dict(os.environ, SITL_STREAMRATE='bad'),
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main()
