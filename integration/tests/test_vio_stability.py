"""Offline acceptance contracts: failed intervals must not disappear from metrics."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_vio_stability import full_trajectory_valid,verified_reliable


class StabilityTests(unittest.TestCase):
    def rows(self,*statuses):return [{'status':s} for s in statuses]
    def test_actual_reliability_discovery(self):
        self.assertTrue(verified_reliable([{'reliability':'1','depth':0}]))
        self.assertFalse(verified_reliable([{'reliability':'2','depth':4096}]))
        self.assertFalse(verified_reliable([]))
    def test_initial_clock_warmup_only(self):
        self.assertTrue(full_trajectory_valid(self.rows('WARMUP','SHADOW_VALID','SHADOW_VALID'),[],1))
        self.assertFalse(full_trajectory_valid(self.rows('SHADOW_VALID','WARMUP','SHADOW_VALID'),[],1))
    def test_failed_intervals_or_resets_fail_full_run(self):
        self.assertFalse(full_trajectory_valid(self.rows('SHADOW_VALID','REJECTED','SHADOW_VALID'),[],1))
        self.assertFalse(full_trajectory_valid(self.rows('SHADOW_VALID'),[{'event':'reinitialized'}],2))
    def test_no_valid_trajectory(self):
        self.assertFalse(full_trajectory_valid([],[],1))
        self.assertFalse(full_trajectory_valid(self.rows('WARMUP'),[],1))


if __name__=='__main__':unittest.main()
