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
        """Upper recent cost, excluding one isolated spike once history is mature.

        A single descheduling stall must not predict the next five frames are
        stale. Two slow frames still raise the prediction. A frame admitted
        by this estimate must still pass the absolute publication deadline.
        """
        if len(self.recent) >= 4:
            return sorted(self.recent)[-2]
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


def keep_as_reference(state, frontend, gray, t, depth):
    """A skipped frame still becomes the tracking reference.

    Skipping only the estimation keeps the next image pair one camera period
    apart. Run 20261117-flightfea1c93152a2: after a skip at 41.3 s the next
    frame was tracked across 0.2 s during the near-ground climb (predicted
    scale 0.636), lost tracking (31 inliers, audit failed) and blocked the
    gate. Storing the image costs nothing. The gyro window restarts here so
    derotation covers the same interval as the flow; the caller integrates IMU
    up to t first. Nothing is estimated or published for this frame.
    Returns False (and changes nothing) while no reference exists yet.
    """
    if state.get('previous') is None or frontend.prev is None:
        return False
    frontend.prev, frontend.prev_t = gray, t
    state.update(previous=gray, previous_t=t, previous_depth=depth, pending_gyro=[])
    return True
