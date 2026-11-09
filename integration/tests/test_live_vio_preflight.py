import unittest
from probe_live_vio_preflight import timing


class PassiveTimingTests(unittest.TestCase):
    def test_missing_stream_has_no_rate_or_gap(self):
        result = timing([])
        self.assertEqual(result['count'], 0)
        self.assertIsNone(result['sim_hz'])
        self.assertIsNone(result['max_sim_gap_s'])

    def test_clock_reset_and_duplicate_are_visible(self):
        rows = [dict(t=t, wall=i) for i, t in enumerate([10., 10.1, 10.1, 1.])]
        result = timing(rows)
        self.assertEqual(result['nonincreasing_stamps'], 2)
        self.assertIsNone(result['sim_hz'])

    def test_sim_and_wall_time_are_distinguished(self):
        rows = [dict(t=t, wall=w) for t, w in [(1., 0.), (1.1, .2), (1.5, 1.)]]
        result = timing(rows)
        self.assertEqual(result['sim_hz'], 4.)
        self.assertEqual(result['wall_hz'], 2.)
        self.assertAlmostEqual(result['max_sim_gap_s'], .4)
        self.assertEqual(result['gaps_over_200ms'], 1)
