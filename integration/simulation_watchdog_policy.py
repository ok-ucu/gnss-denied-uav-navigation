"""Offline DISARMED-only watchdog policy prototype, NOT wired to flight transport.

This intentionally cannot grant MAVLink permissions or override lifecycle,
clock-mapping, wire deadlines, or FCU failsafes. Policy must be separately
reviewed and re-attested before integration into a live DISARMED runner.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SimulationWatchdogPolicy:
    sim_freshness_s: float = 0.5
    host_hard_stop_s: float = 2.0
    min_rtf: float = 0.25
    max_rtf: float = 1.0

    def __post_init__(self):
        values = (self.sim_freshness_s, self.host_hard_stop_s, self.min_rtf, self.max_rtf)
        if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
            raise ValueError('Finite numeric time policy required')
        if not (0 < self.sim_freshness_s <= 0.5 and
                0 < self.host_hard_stop_s <= 2.0 and
                0 < self.min_rtf <= self.max_rtf <= 1.0):
            raise ValueError('Policy exceeds bounded DISARMED-only diagnostic limits')

    def classify(self, *, last_nav_host_s, now_host_s, last_nav_sim_s,
                 now_sim_s, observed_rtf, camera_alive=True, imu_alive=True,
                 gazebo_alive=True, sim_reset=False, sim_paused=False):
        """Return (healthy, reason), never synthesize navigation updates.

        A deliberately paused simulator is *not* considered navigation-healthy;
        the caller must block permission, not keep old VIO valid.
        """
        if sim_reset: return False, 'simulation clock reset'
        if sim_paused: return False, 'simulation paused (permissions blocked)'
        if not gazebo_alive: return False, 'Gazebo process stopped'
        if not camera_alive: return False, 'camera stopped'
        if not imu_alive: return False, 'IMU stopped'
        values = (last_nav_host_s, now_host_s, last_nav_sim_s, now_sim_s, observed_rtf)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            return False, 'missing or invalid clock evidence'
        host_age = now_host_s - last_nav_host_s
        sim_age = now_sim_s - last_nav_sim_s
        if host_age < 0 or sim_age < 0: return False, 'clock reversed'
        if host_age > self.host_hard_stop_s: return False, 'host hard-stop deadline'
        if not self.min_rtf <= observed_rtf <= self.max_rtf:
            return False, 'untrusted RTF outside policy limits'
        if sim_age > self.sim_freshness_s:
            return False, 'VIO missing in simulation time'
        return True, 'healthy within DISARMED diagnostic policy'
