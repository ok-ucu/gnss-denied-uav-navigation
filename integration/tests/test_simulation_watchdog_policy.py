"""Deterministic DISARMED policy model tests; NOT actual Gazebo/FCU evidence."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation_watchdog_policy import SimulationWatchdogPolicy


class PolicyTests(unittest.TestCase):
    def test_normal_and_lost_camera_frames_at_various_rtfs(self):
        p=SimulationWatchdogPolicy()
        for rtf in (1.0,.65,.5,.25):
            with self.subTest(rtf=rtf):
                # 10Hz VIO sim clock; gaps measured in simulator time.
                for lost in (0,1,3):
                    sim_gap=(lost+1)*.1
                    host_gap=sim_gap/rtf
                    ok,reason=p.classify(last_nav_host_s=10,now_host_s=10+host_gap,
                        last_nav_sim_s=20,now_sim_s=20+sim_gap,observed_rtf=rtf)
                    self.assertTrue(ok,(rtf,lost,reason))
                # 5 missing frames create .6s simulation gap: must fail.
                ok,_=p.classify(last_nav_host_s=10,now_host_s=10+.6/rtf,
                    last_nav_sim_s=20,now_sim_s=20.6,observed_rtf=rtf)
                self.assertFalse(ok)

    def test_real_stalls_and_pauses_always_fail_closed(self):
        p=SimulationWatchdogPolicy()
        common=dict(last_nav_host_s=10,now_host_s=10.3,last_nav_sim_s=20,now_sim_s=20.15,observed_rtf=.5)
        for fault in ('camera_alive','imu_alive','gazebo_alive'):
            self.assertFalse(p.classify(**(common|{fault:False}))[0])
        for fault in ('sim_reset','sim_paused'):
            self.assertFalse(p.classify(**(common|{fault:True}))[0])
        self.assertFalse(p.classify(**(common|{'now_host_s':12.01}))[0])
        self.assertFalse(p.classify(**(common|{'observed_rtf':0.01}))[0])
        self.assertFalse(p.classify(**(common|{'now_sim_s':19}))[0])

    def test_policy_cannot_become_unbounded(self):
        for args in ({'host_hard_stop_s':10},{'sim_freshness_s':1},
                     {'min_rtf':0},{'max_rtf':2}):
            with self.assertRaises(ValueError):SimulationWatchdogPolicy(**args)


if __name__=='__main__':unittest.main()
