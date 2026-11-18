#!/usr/bin/env python3
"""Self-contained tests for the DataFlash exporter."""

import csv
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from pymavlink import mavutil

from dataflash_export import export


class FakeMessage:
    def __init__(self, kind, **fields):
        self.kind = kind
        self.fields = fields
        self.__dict__.update(fields)

    def get_type(self):
        return self.kind

    def to_dict(self):
        return dict(self.fields)


class FakeLog:
    def __init__(self, messages, fail_after=None):
        self.messages = iter(messages)
        self.calls = 0
        self.fail_after = fail_after

    def recv_match(self):
        self.calls += 1
        if self.fail_after is not None and self.calls > self.fail_after:
            raise RuntimeError("Simulated read failure")
        return next(self.messages, None)


def sample_messages():
    return [
        FakeMessage(
            "GPA", I=0, TimeUS=1000,
            HAcc=0.3, VAcc=0.4,
        ),
        FakeMessage(
            "GPS", I=0, TimeUS=1000,
            Status=3, Lat=50.0, Lng=30.0,
            Alt=100.0, HDop=1.2, Spd=2.0, NSats=10,
        ),
        FakeMessage(
            "IMU", I=0, TimeUS=1100,
            GyrX=0.1, GyrY=0.2, GyrZ=0.3,
            AccX=1.0, AccY=2.0, AccZ=-9.8,
        ),
        FakeMessage(
            "ATT", TimeUS=1200,
            Roll=1.0, Pitch=2.0, Yaw=3.0,
        ),
    ]


class TestDataFlashExport(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / "sample.BIN"
        self.source.write_bytes(b"synthetic test input")
        self.destination = self.root / "output"

    def run_export(self, messages, fail_after=None):
        with patch.object(
            mavutil,
            "mavlink_connection",
            return_value=FakeLog(messages, fail_after),
        ):
            return export(self.source, self.destination)

    def read_csv(self, name):
        with (self.destination / name).open() as f:
            return list(csv.DictReader(f))

    def test_export_values_and_timestamps(self):
        summary = self.run_export(sample_messages())

        self.assertEqual(summary["gps_samples"], 1)
        self.assertEqual(summary["imu_samples"], 1)
        self.assertEqual(summary["attitude_samples"], 1)

        gps = self.read_csv("gps.csv")[0]
        imu = self.read_csv("imu.csv")[0]
        att = self.read_csv("attitude.csv")[0]

        self.assertEqual(gps["timestamp_ap_ns"], "1000000")
        self.assertEqual(gps["hacc_m"], "0.3")
        self.assertEqual(gps["vacc_m"], "0.4")
        self.assertEqual(gps["valid_fix"], "1")
        self.assertEqual(gps["lat_deg"], "50.0")

        self.assertEqual(imu["timestamp_ap_ns"], "1100000")
        self.assertEqual(float(imu["gz_rad_s"]), 0.3)
        self.assertEqual(float(imu["az_m_s2"]), -9.8)

        self.assertEqual(att["timestamp_ap_ns"], "1200000")
        self.assertEqual(float(att["yaw_deg"]), 3.0)

        self.assertFalse(summary["camera_synchronized"])

    def test_no_gps(self):
        messages = [
            m for m in sample_messages()
            if m.get_type() not in ("GPS", "GPA")
        ]
        summary = self.run_export(messages)

        self.assertEqual(summary["gps_samples"], 0)
        self.assertEqual(self.read_csv("gps.csv"), [])
        self.assertEqual(summary["imu_samples"], 1)

    def test_missing_source(self):
        self.source.unlink()

        with self.assertRaises(FileNotFoundError):
            export(self.source, self.destination)

        self.assertFalse(self.destination.exists())

    def test_existing_output_protected(self):
        self.destination.mkdir()
        marker = self.destination / "important.txt"
        marker.write_text("KEEP")

        with self.assertRaises(FileExistsError):
            export(self.source, self.destination)

        self.assertEqual(marker.read_text(), "KEEP")

    def test_interrupted_export_cleanup(self):
        with self.assertRaises(RuntimeError):
            self.run_export(sample_messages(), fail_after=2)

        self.assertFalse(self.destination.exists())
        self.assertEqual(
            sorted(p.name for p in self.root.iterdir()),
            ["sample.BIN"],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
