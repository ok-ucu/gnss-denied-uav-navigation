"""Tests for GPS-to-ENU reference trajectory conversion."""

import csv
import tempfile
import unittest
from pathlib import Path

from pyproj import Geod

from gps_reference import convert


class TestGPSReference(unittest.TestCase):

    def test_enu_conversion(self):
        geod = Geod(ellps="WGS84")
        lat0, lon0, alt0 = 50.0, 30.0, 100.0

        lon_e, lat_e, _ = geod.fwd(lon0, lat0, 90, 10)
        lon_n, lat_n, _ = geod.fwd(lon0, lat0, 0, 10)

        points = [
            (1000, 0.0, 0.0, 0.0, 0),
            (2000, lat0, lon0, alt0, 1),
            (3000, lat_e, lon_e, alt0, 1),
            (4000, lat_n, lon_n, alt0, 1),
            (5000, lat0, lon0, alt0 + 5, 1),
        ]

        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "gps.csv"
            output = Path(tmp) / "reference.csv"

            fields = [
                "time_us", "timestamp_ap_ns", "instance",
                "lat_deg", "lon_deg", "alt_m", "valid_fix",
            ]

            with source.open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()

                for t, lat, lon, alt, valid in points:
                    writer.writerow({
                        "time_us": t,
                        "timestamp_ap_ns": t * 1000,
                        "instance": 0,
                        "lat_deg": lat,
                        "lon_deg": lon,
                        "alt_m": alt,
                        "valid_fix": valid,
                    })

            count = convert(source, output)

            with output.open(newline="") as f:
                rows = list(csv.DictReader(f))

            self.assertEqual(count, 4)
            self.assertEqual(len(rows), 4)

            def xyz(index):
                row = rows[index]
                return tuple(
                    float(row[k]) for k in ("x_m", "y_m", "z_m")
                )

            origin = xyz(0)
            east = xyz(1)
            north = xyz(2)
            up = xyz(3)

            for value in origin:
                self.assertAlmostEqual(value, 0, delta=0.01)

            self.assertAlmostEqual(east[0], 10, delta=0.02)
            self.assertAlmostEqual(east[1], 0, delta=0.02)
            self.assertAlmostEqual(north[0], 0, delta=0.02)
            self.assertAlmostEqual(north[1], 10, delta=0.02)
            self.assertAlmostEqual(up[2], 5, delta=0.02)

            self.assertEqual(rows[0]["timestamp_ap_ns"], "2000000")

    def test_no_valid_gps(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "gps.csv"
            output = Path(tmp) / "reference.csv"

            source.write_text(
                "time_us,timestamp_ap_ns,instance,"
                "lat_deg,lon_deg,alt_m,valid_fix\n"
                "1000,1000000,0,50,30,100,0\n"
            )

            with self.assertRaises(ValueError):
                convert(source, output)

            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
