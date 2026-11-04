"""Offline checks for the bounded Gazebo command schedule, not movement evidence."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from probe_camera_motion import PHASES, phase_at


class MotionScheduleTests(unittest.TestCase):
    def test_phase_boundaries(self):
        self.assertEqual(phase_at(0), ('stationary',0,0))
        self.assertEqual(phase_at(2.5), ('world_x',.2,0))
        self.assertEqual(phase_at(5), ('pause',0,0))
        self.assertEqual(phase_at(6.5), ('world_y',0,.2))
        self.assertEqual(phase_at(9), ('stop',0,0))

    def test_final_and_expired_commands_are_zero(self):
        self.assertEqual(phase_at(sum(p[1] for p in PHASES))[1:], (0,0))
        self.assertEqual(phase_at(100)[1:], (0,0))
