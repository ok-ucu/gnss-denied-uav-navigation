"""Rate-limited 'odometry not published' diagnostics.

Every place that skips or suppresses an odometry publication reports here with
the acquisition timestamp, the reason and the relevant state. The first
occurrence of a reason is emitted immediately; repeats of the same reason are
emitted at most once per interval and carry how many were suppressed, so a
long outage (one row per frame at 10 Hz) stays readable and the steady state
emits nothing. Diagnostics only: never consulted by any decision.
"""
import time

EVENT = 'odometry_skipped'


class SkipLog:
    def __init__(self, emit, interval_s=1.0, clock=time.monotonic):
        if not interval_s > 0:
            raise ValueError('interval must be positive')
        self.emit, self.interval, self.clock = emit, interval_s, clock
        self._last = {}       # reason -> monotonic seconds of last emission
        self._pending = {}    # reason -> [suppressed count, first, last acquisition_ns]
        self.total = 0

    def __call__(self, reason, acquisition_ns, stage, **state):
        """Record one skipped sample; returns True if a row was emitted."""
        self.total += 1
        now = self.clock()
        last = self._last.get(reason)
        if last is not None and now-last < self.interval:
            pending = self._pending.setdefault(reason, [0, acquisition_ns, acquisition_ns])
            pending[0] += 1
            pending[2] = acquisition_ns
            return False
        suppressed, first, final = self._pending.pop(reason, [0, None, None])
        self._last[reason] = now
        self.emit(dict(event=EVENT, stage=stage, reason=reason, acquisition_ns=acquisition_ns,
                       suppressed_since_last=suppressed, suppressed_first_acquisition_ns=first,
                       suppressed_last_acquisition_ns=final, **state))
        return True

    def flush(self, stage):
        """Emit the suppressed counts that have not been reported yet."""
        for reason, (suppressed, first, final) in sorted(self._pending.items()):
            self.emit(dict(event=EVENT, stage=stage, reason=reason, acquisition_ns=None,
                           suppressed_since_last=suppressed, suppressed_first_acquisition_ns=first,
                           suppressed_last_acquisition_ns=final, final_flush=True))
        self._pending.clear()
