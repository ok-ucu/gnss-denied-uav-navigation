"""Offline checks of synthetic error model; no SITL or pymavlink required."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_live_ekf import synthetic_drift
from analyze_live_ekf import rms


class LiveAnalysisTests(unittest.TestCase):
    def test_same_error_sequence_in_normal_and_inflated(self):
        for t in (0,.2,1.3,4.7,19.9):
            for a,b in zip(synthetic_drift(t),synthetic_drift(t+20)):
                self.assertAlmostEqual(a,b,places=12)

    def test_velocity_is_position_error_derivative(self):
        for t in (0,.2,1.3,4.7):
            step=1e-5
            before=synthetic_drift(t-step)
            after=synthetic_drift(t+step)
            actual=synthetic_drift(t)
            self.assertAlmostEqual((after[0]-before[0])/(2*step),actual[2],places=8)
            self.assertAlmostEqual((after[1]-before[1])/(2*step),actual[3],places=8)

    def test_rms_and_empty_samples(self):
        self.assertEqual(rms([]),None)
        self.assertAlmostEqual(rms([3,4]),(12.5)**.5)


if __name__ == '__main__':
    unittest.main()
