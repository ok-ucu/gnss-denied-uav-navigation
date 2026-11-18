"""Synthetic wire fixtures only: no live VIO, FCU or EKF evidence."""
import contextlib
import io
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_external_nav import TYPES, analyze, main, read_capture, validate_measurement
from test_ekf_covariance import pose_covariance, speed_covariance
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def messages(times, speed_times=None):
    encoder = mavlink.MAVLink(None, srcSystem=42, srcComponent=191)
    records = []
    for kind, samples in enumerate((times, times if speed_times is None else speed_times)):
        for index, arrival in enumerate(samples):
            stamp = 1000000 + index * 50000
            if kind == 0:
                msg = encoder.vision_position_estimate_encode(
                    stamp, 1, 2, -3, 0, 0, 0, covariance=pose_covariance(.1))
            else:
                msg = encoder.vision_speed_estimate_encode(
                    stamp, .1, .2, .3, covariance=speed_covariance(.1))
            wire = msg.pack(encoder)
            decoded = mavlink.MAVLink(None).parse_char(wire)
            records.append((decoded, arrival, 2))
    return sorted(records, key=lambda row: row[1])


class ExternalNavTests(unittest.TestCase):
    def setUp(self):
        self.times = [i / 20 for i in range(21)]

    def test_valid_wire_stream(self):
        report = analyze(messages(self.times))
        self.assertEqual(report['status'], 'PASS')
        for stream in report['streams'].values():
            self.assertEqual(stream['count'], 21)
            self.assertAlmostEqual(stream['observed_rate_hz'], 20)
            self.assertAlmostEqual(stream['max_interarrival_gap_s'], .05)
            self.assertEqual(stream['recovery_status'], 'NOT TESTED')
        self.assertEqual(report['capture_to_ekf_latency'], 'NOT TESTED')
        self.assertEqual(report['ekf_acceptance'], 'NOT TESTED')

    def test_missing_velocity(self):
        report = analyze(messages(self.times, []))
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['streams'][TYPES[1]]['count'], 0)

    def test_low_rate(self):
        report = analyze(messages([0, .1, .2]))
        self.assertEqual(report['status'], 'FAIL')
        self.assertAlmostEqual(report['streams'][TYPES[0]]['observed_rate_hz'], 10)

    def test_irregular_gap_despite_high_average(self):
        report = analyze(messages([0, .001, .002, .003, .004, .254]), min_rate=10)
        stream = report['streams'][TYPES[0]]
        self.assertGreater(stream['observed_rate_hz'], 10)
        self.assertEqual(stream['status'], 'FAIL')
        self.assertEqual(len(stream['dropouts']), 1)
        self.assertTrue(stream['dropouts'][0]['recovered'])

    def test_dropout_recovery_and_terminal_loss(self):
        report = analyze(messages([0, .05, .1, .5, .55], [0, .05, .1]))
        pose, speed = (report['streams'][kind] for kind in TYPES)
        self.assertEqual(pose['recovery_status'], 'PASS')
        self.assertAlmostEqual(pose['dropouts'][0]['first_return_capture_s'], .5)
        self.assertFalse(speed['dropouts'][0]['recovered'])
        self.assertEqual(speed['recovery_status'], 'NOT TESTED')
        self.assertEqual(report['status'], 'FAIL')

    def test_sender_resets_duplicates_separate_from_arrival(self):
        for stamp in (1, 1000000):
            rows = messages(self.times)
            rows[2][0].usec = stamp
            report = analyze(rows)
            pose = report['streams'][TYPES[0]]
            self.assertEqual(pose['status'], 'FAIL')
            self.assertEqual(pose['timing_status'], 'PASS')
            self.assertTrue(pose['sender_reset_indices'] or pose['sender_duplicate_indices'])

    def test_bad_covariance_on_wire(self):
        for kind in TYPES:
            rows = messages(self.times)
            row = next(row for row in rows if row[0].get_type() == kind)
            row[0].covariance[0] = math.nan
            wire = row[0].pack(mavlink.MAVLink(None))
            decoded = mavlink.MAVLink(None).parse_char(wire)
            index = rows.index(row)
            rows[index] = (decoded, row[1], 2)
            self.assertEqual(analyze(rows)['streams'][kind]['status'], 'FAIL')

    def test_covariance_shape_symmetry_psd_and_policy(self):
        for kind, size in ((TYPES[0], 21), (TYPES[1], 9)):
            for cov in ([1.] * (size-1), [0.] * size, [math.inf] * size, [-1.] * size):
                msg = SimpleNamespace(get_type=lambda: kind, x=0, y=0, z=0,
                                      roll=0, pitch=0, yaw=0, covariance=cov)
                with self.assertRaises(ValueError):
                    validate_measurement(msg)
        msg = messages([0])[1][0]  # speed
        msg.covariance[1] = 2
        with self.assertRaises(ValueError):
            validate_measurement(msg)
        msg.covariance[3] = 2
        with self.assertRaises(ValueError):
            validate_measurement(msg)
        msg.covariance = [1., 0, 0, 0, 0, 0, 0, 0, 0]  # singular PSD allowed
        validate_measurement(msg)

    def test_raw_no_receiver_timing(self):
        rows = [(msg, None, version) for msg, _, version in messages(self.times)]
        self.assertEqual(analyze(rows)['status'], 'NOT TESTED')

    def test_capture_clock_reset_and_bad_settings(self):
        rows = messages(self.times)
        rows[-1] = (rows[-1][0], -1, 2)
        with self.assertRaises(ValueError):
            analyze(rows)
        for value in (math.nan, math.inf, 0, -1):
            with self.assertRaises(ValueError):
                analyze([], min_rate=value)
            with self.assertRaises(ValueError):
                analyze([], max_gap=value)

    def test_mavlink1_and_multiple_sources(self):
        rows = messages(self.times)
        rows[0] = (rows[0][0], 0, 1)
        self.assertEqual(analyze(rows)['status'], 'FAIL')
        rows = messages(self.times)
        rows[0][0]._header.srcSystem = 43
        self.assertEqual(analyze(rows)['status'], 'FAIL')
        self.assertEqual(analyze(rows, system=42)['status'], 'PASS')

    def test_tlog_and_raw_roundtrip_truncation_crc_and_cli(self):
        encoder = mavlink.MAVLink(None)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'fixture.tlog'
            data = b''.join(struct.pack('>Q', int((10+t)*1e6)) + msg.pack(encoder)
                            for msg, t, _ in messages(self.times))
            path.write_bytes(data)
            self.assertEqual(analyze(read_capture(path))['status'], 'PASS')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([str(path)]), 0)
            raw = b''.join(msg.pack(encoder) for msg, _, _ in messages(self.times))
            path.write_bytes(raw)
            self.assertEqual(analyze(read_capture(path, raw=True))['status'], 'NOT TESTED')
            for damaged in (data[:-1], data[:4], b'abcdefghX', data[:-2] + b'\x00\x00'):
                path.write_bytes(damaged)
                with self.assertRaises(ValueError):
                    list(read_capture(path))
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main([str(path)]), 1)


if __name__ == '__main__':
    unittest.main()
