"""Offline checks for the bounded Gazebo command schedule, not movement evidence."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from probe_camera_motion import PHASES, phase_at, evaluate_motion


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


class MovementVerdictTests(unittest.TestCase):
    def records(self):
        frames = [dict(phase=phase) for phase in ('world_x', 'world_y') for _ in range(2)]
        poses = [dict(phase=phase,x=.3*i if phase=='world_x' else 0,
                      y=.3*i if phase=='world_y' else 0,z=2,qx=0,qy=0,qz=0,qw=1)
                 for phase in ('world_x','world_y') for i in range(2)]
        flows = [dict(phase=phase,previous_phase=phase,dt_s=.03,tracks=6,
                      du_px=1 if phase=='world_y' else 0,
                      dv_px=1 if phase=='world_x' else 0)
                 for phase in ('world_x','world_y') for _ in range(3)]
        return frames,poses,flows

    def test_measured_motion_and_signs_pass(self):
        self.assertEqual(evaluate_motion(*self.records())['status'], 'PASS')

    def test_missing_frames_or_odometry_fail(self):
        f,o,t=self.records()
        self.assertEqual(evaluate_motion([],o,t)['status'], 'FAIL')
        self.assertEqual(evaluate_motion(f,[],t)['status'], 'FAIL')
        self.assertEqual(evaluate_motion([v for v in f if v['phase']=='world_x'],o,t)['status'], 'FAIL')

    def test_stationary_insufficient_wrong_direction_and_nan_fail(self):
        for value in (0,.01,-.3,float('nan')):
            f,o,t=self.records()
            o[1]['x']=value
            self.assertEqual(evaluate_motion(f,o,t)['status'], 'FAIL')

    def test_missing_tracks_do_not_claim_full_pass(self):
        f,o,t=self.records()
        result=evaluate_motion(f,o,[])
        self.assertEqual(result['status'], 'NOT TESTED')
        self.assertEqual(result['movement_status'], 'PASS')
        self.assertEqual(result['optical_flow']['status'], 'NOT TESTED')

    def test_wrong_pixel_sign_fails(self):
        f,o,t=self.records()
        for row in t: row['du_px']=-1;row['dv_px']=-1
        result=evaluate_motion(f,o,t)
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual(result['movement_status'], 'PASS')

    def test_unverified_attitude_not_tested(self):
        f,o,t=self.records()
        o[0]['qw']=.5
        self.assertEqual(evaluate_motion(f,o,t)['status'], 'NOT TESTED')
