"""Freshness-aware frame admission: process the most recent frame when behind.

A frame whose age at the start of processing plus the recent processing time
would exceed the source's publication age limit cannot be published fresh: the
existing publication gate would block it as stale, which also resets the
readiness hysteresis (3 frames / 250 ms of no output). Skipping it instead
leaves that time to the IMU backlog and the next, fresher frame.

Never skips two frames in a row, so a skip costs one camera period of output
and gaps stay within 2 x period; if the next frame is also predicted stale it is
processed anyway and the unchanged publication gate decides. The limit is the
existing gate's, not a new or wider one. Every skip is logged by the caller.
"""
from collections import deque


class Admission:
    def __init__(self, publication_max_age_s, window=5):
        if not publication_max_age_s > 0:
            raise ValueError('publication age limit must be positive')
        self.limit = publication_max_age_s
        self.recent = deque(maxlen=window)   # processing durations, simulation seconds
        self.last_skipped = False

    def expected(self):
        """Pessimistic recent processing time (max of the last few frames)."""
        return max(self.recent) if self.recent else None

    def admit(self, age_at_start_s):
        expected = self.expected()
        if self.last_skipped or expected is None or age_at_start_s+expected <= self.limit:
            self.last_skipped = False
            return True
        self.last_skipped = True
        return False

    def processed(self, duration_s):
        if duration_s >= 0:
            self.recent.append(duration_s)
