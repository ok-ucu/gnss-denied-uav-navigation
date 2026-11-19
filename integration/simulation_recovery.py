"""DISARMED timing prototype. No permission, transport, or simulator interface.

A caller must supply independently authenticated observations. This classifier
cannot grant egress, substitute observations, or change a production deadline.
All times and endpoint uncertainty bounds are integer nanoseconds.
"""
from collections import deque
from dataclasses import dataclass
from fractions import Fraction

from vio_latency_contract import SOURCE_PUBLICATION_MAX_AGE_S

MS = 1_000_000
SOURCE_MAX_AGE_NS = round(SOURCE_PUBLICATION_MAX_AGE_S*1e9)
CLOCKS = ('ros', 'applied')
MAX_AGE = dict(camera=280*MS, imu=100*MS, gazebo=50*MS,
               ros=200*MS, applied=200*MS)
QUALITY_REASONS = frozenset(('inliers', 'contrast', 'distributed flow disagreement/unavailable'))


@dataclass(frozen=True)
class Observation:
    host: int
    stamp: int
    host_uncertainty: int = 0
    sim_uncertainty: int = 0


def rate_bounds(first, last):
    host = last.host-first.host
    sim = last.stamp-first.stamp
    host_error = first.host_uncertainty+last.host_uncertainty
    sim_error = first.sim_uncertainty+last.sim_uncertainty
    if host <= host_error or sim <= sim_error:
        raise ValueError('rate uncertainty')
    return (Fraction(sim-sim_error, host+host_error),
            Fraction(sim+sim_error, host-host_error))


class SimulationRecovery:
    def __init__(self, *, enabled=False, run=None, epoch=None, source=None, disarmed=False):
        self.state = 'NORMAL' if enabled and disarmed is True and all((run, epoch, source)) else 'BLOCKED'
        self.identity = run, epoch, source
        self.reason = None if self.state == 'NORMAL' else 'prototype disabled or identity unverified'
        self.transitions = []
        self.latest = {}
        self.clocks = {kind: deque() for kind in CLOCKS}
        self.last_pair = None
        self.normal_since = None
        self.deadline = None
        self.rate_checked = None
        self.started = None
        self.now = -1

    def block(self, now, reason):
        if self.state != 'BLOCKED':
            self.state, self.reason = 'BLOCKED', reason
            self.transitions.append((now, self.state, reason))
        return False

    def advance(self, now):
        if self.state == 'BLOCKED':
            return False
        if type(now) is not int or now < 0 or now < self.now:
            return self.block(now, 'host clock reset')
        self.now = now
        if self.started is None:
            self.started = now
        for kind, age in MAX_AGE.items():
            last = self.latest.get(kind)
            if now-(last.host if last else self.started) > age:
                return self.block(now, f'{kind} liveness')
        if self.state == 'RECOVERING':
            if self.recovery_expired(now):
                return self.block(now, 'absolute recovery deadline')
            if now-self.rate_checked > 50*MS:
                return self.block(now, 'recovery rate check overdue')
        elif now-(self.last_pair.host if self.last_pair else self.started) > 500*MS:
            return self.block(now, 'normal availability deadline')
        return True

    def recovery_expired(self, now):
        return now > self.deadline.host or self.sim_upper() > self.deadline.stamp

    def observe(self, kind, sample, *, identity, disarmed=True):
        if not self.advance(sample.host):
            return False
        values = (sample.host, sample.stamp, sample.host_uncertainty, sample.sim_uncertainty)
        if (kind not in MAX_AGE or identity != self.identity or disarmed is not True
                or any(type(v) is not int or v < 0 for v in values)):
            return self.block(sample.host, 'unverified observation')
        last = self.latest.get(kind)
        if last and kind != 'gazebo':
            if sample.stamp < last.stamp:
                return self.block(sample.host, f'{kind} timestamp reset')
            if sample.stamp == last.stamp:
                return False  # Duplicate cannot extend the independent deadline.
            if kind == 'camera' and not 90*MS <= sample.stamp-last.stamp <= 110*MS:
                return self.block(sample.host, 'camera cadence or missing frame')
        self.latest[kind] = sample
        if kind in CLOCKS:
            history = self.clocks[kind]
            history.append(sample)
            while len(history) > 1 and sample.host-history[1].host > 6_500*MS:
                history.popleft()
        return self.advance(sample.host)

    def sim_upper(self):
        return max(self.latest[k].stamp+self.latest[k].sim_uncertainty for k in CLOCKS)

    def verified_rate(self, now, span, windows, minimum):
        lows = []
        for i in range(windows):
            end = now-i*span
            bounds = []
            for history in self.clocks.values():
                rows = [r for r in history if end-span < r.host <= end]
                if len(rows) < minimum or rows[-1].host-rows[0].host < span-200*MS:
                    raise ValueError('insufficient advancing rate observations')
                bounds.append(rate_bounds(rows[0], rows[-1]))
            (a_low, a_high), (b_low, b_high) = bounds
            low = min(a_low, b_low)
            if low < Fraction(1, 2) or max(a_high-b_low, b_high-a_low) > Fraction(1, 20):
                raise ValueError('rate below floor or inconsistent witnesses')
            lows.append(low)
        return min(lows)

    def reject(self, now, stamp, *, identity, reason, healthy):
        if not self.advance(now):
            return False
        camera = self.latest.get('camera')
        if (self.state != 'NORMAL' or not self.last_pair or identity != self.identity
                or reason not in QUALITY_REASONS or healthy is not True
                or not MAX_AGE.keys() <= self.latest.keys() or not camera or stamp != camera.stamp
                or not 0 <= now-camera.host <= 100*MS
                or not 90*MS <= stamp-self.last_pair.stamp <= 110*MS
                or self.normal_since is None or now-self.normal_since < 2_000*MS):
            return self.block(now, 'rejection ineligible or unverified')
        try:
            self.verified_rate(now, 2_000*MS, 3, 10)
        except ValueError as exc:
            return self.block(now, str(exc))
        self.deadline = Observation(self.last_pair.host+1_150*MS, self.last_pair.stamp+510*MS)
        self.rate_checked = now
        self.state = 'RECOVERING'
        self.transitions.append((now, self.state, reason))
        return self.advance(now)

    def check_recovery_rate(self, now):
        if not self.advance(now) or self.state != 'RECOVERING':
            return False
        try:
            low = self.verified_rate(now, 500*MS, 1, 3)
        except ValueError as exc:
            return self.block(now, str(exc))
        # The remaining window must fit worst-case clock progression at the
        # measured lower bound. The absolute deadline is never extended.
        remaining_sim = self.remaining_acquisition()
        if remaining_sim/low > self.deadline.host-now:
            return self.block(now, 'recovery cannot meet host deadline')
        self.rate_checked = now
        return True

    def remaining_acquisition(self):
        return max(0, self.deadline.stamp-self.sim_upper())

    def complete(self, now, stamp, *, identity, ready, fresh, cov_valid, halves):
        if not self.advance(now):
            return False
        camera = self.latest.get('camera')
        if (identity != self.identity or not MAX_AGE.keys() <= self.latest.keys()
                or not camera or type(stamp) is not int or stamp > camera.stamp
                or any(flag is not True for flag in (ready, fresh, cov_valid))
                or tuple(halves) != ('pose', 'velocity') and tuple(halves) != ('velocity', 'pose')
                or self.last_pair and stamp <= self.last_pair.stamp):
            return self.block(now, 'invalid, stale, or incomplete pair')
        if self.last_pair is None or self.state == 'RECOVERING':
            self.normal_since = now
        self.last_pair = Observation(now, stamp)
        if self.state == 'RECOVERING':
            self.state = 'NORMAL'
            self.transitions.append((now, self.state, 'complete fresh pair'))
        return True


class AcquisitionDeliveryRecovery(SimulationRecovery):
    """Corrected offline contract. SimulationRecovery retains the baseline.

    Source acceptance and wire completion are different observations. The
    caller must authenticate them and enforce the existing pipeline validators.
    """

    def __init__(self, **config):
        super().__init__(**config)
        self.acquisition = None

    def recovery_expired(self, now):
        if now > self.deadline.host:
            return True
        # A source verdict can arrive after the acquisition stamp. Close if
        # even the latest eligible stamp could no longer pass source freshness.
        return (self.acquisition is None
                and self.sim_upper() > self.deadline.stamp + SOURCE_MAX_AGE_NS)

    def remaining_acquisition(self):
        return 0 if self.acquisition is not None else super().remaining_acquisition()

    def acquire(self, now, stamp, *, identity, source_clock, ready, estimator_ok, cov_valid):
        if not self.advance(now):
            return False
        camera = self.latest.get('camera')
        if (self.state != 'RECOVERING' or self.acquisition is not None
                or identity != self.identity or type(stamp) is not int or type(source_clock) is not int
                or camera is None or stamp != camera.stamp
                or not self.last_pair.stamp < stamp <= self.deadline.stamp
                or not 0 <= source_clock-stamp <= SOURCE_MAX_AGE_NS
                or source_clock != self.latest['ros'].stamp
                or any(flag is not True for flag in (ready, estimator_ok, cov_valid))):
            return self.block(now, 'invalid or late recovery acquisition')
        self.acquisition = Observation(now, stamp)
        return True

    def complete(self, now, stamp, *, identity, ready, fresh, cov_valid, halves,
                 wire_stamps=None, wire_valid=False):
        if not self.advance(now):
            return False
        recovering = self.state == 'RECOVERING'
        if recovering and (self.acquisition is None or stamp != self.acquisition.stamp
                           or wire_valid is not True or wire_stamps != (stamp, stamp)):
            return self.block(now, 'invalid or unmatched recovery delivery')
        if not super().complete(now, stamp, identity=identity, ready=ready, fresh=fresh,
                                cov_valid=cov_valid, halves=halves):
            return False
        if recovering:
            self.acquisition = None
        return True
