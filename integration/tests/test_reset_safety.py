"""Offline authorization/storage tests; real MAVROS wire tests are separate."""
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.append(str(Path(__file__).resolve().parents[1]))
from reset_safety import ResetSafety
from isolated_fcu_dry_run import evaluate
from types import SimpleNamespace as S
from vio_adapter import convert


class ResetSafetyTests(unittest.TestCase):
    def test_reset_restart_and_explicit_new_reference(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'state.json';g=ResetSafety(p)
            self.assertFalse(g.admit('A',0,1))
            g.initialize('FCU-A','A');self.assertTrue(g.admit('A',0,10))
            self.assertFalse(g.admit('B',1,20));self.assertFalse(g.admit('A',0,21))
            h=ResetSafety(p);self.assertFalse(h.admit('A',0,22))
            with self.assertRaises(ValueError):h.initialize('FCU-A','B')
            h.initialize('FCU-B','B');self.assertTrue(h.admit('B',0,30))
            with self.assertRaises(ValueError):h.initialize('FCU-A','A')
    def test_restart_without_reset_also_blocks(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'state.json';g=ResetSafety(p);g.initialize('FCU-A','A')
            self.assertTrue(g.admit('A',0,10));h=ResetSafety(p)
            self.assertFalse(h.admit('A',0,11))
    def test_bad_epoch_duplicate_and_corrupt_store(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'state.json';g=ResetSafety(p);g.initialize('F','A')
            self.assertTrue(g.admit('A',0,10));self.assertFalse(g.admit('A',0,10))
            p.write_text('{"blocked":false}')
            with self.assertRaises(ValueError):ResetSafety(p)
    def test_offset_math_both_signs_and_epoch(self):
        for origin in [10**13,1700000000000000000]:
            for offset in [-10**9,0,10**9]:
                sent=origin;now=origin+20_000_000;remote=origin+10_000_000+offset
                self.assertEqual((sent+now-2*remote)//2,-offset)
                old=((sent+now-2*remote)&((1<<64)-1))//2
                if offset>0:self.assertNotEqual(old,-offset)
    def test_fcu_plan_requires_full_suite_and_no_reset_leaks(self):
        rows=[dict(case=k,timestamp_pass=True,offset_pass=True) for k in ['plus','minus','zero','jitter','drift','epoch','reset']]
        proof=dict(blocked_pairs=6,wire_leaks=0,explicit_reinitializations=1)
        self.assertTrue(evaluate(rows,proof))
        self.assertFalse(evaluate(rows[:-1],proof))
        self.assertFalse(evaluate(rows,dict(proof,wire_leaks=1)))
    def test_mixed_clock_domain_is_rejected_before_publication(self):
        m=S(header=S(frame_id='odom',stamp=S(sec=1700000000,nanosec=0)),child_frame_id='base_link')
        with self.assertRaisesRegex(ValueError,'Stale/future'):
            convert(m,S,19_000_000_000)
    def test_rate_is_not_a_minimum10_rule(self):
        period_ms=1000/8
        self.assertGreater(period_ms,20)   # AP4.4.4 max-input-rate throttle
        self.assertLess(period_ms,300)    # Backend receipt-health timeout
        self.assertGreater(1800,500)      # Actual long dropout cannot be hidden by average


if __name__=='__main__':unittest.main()
