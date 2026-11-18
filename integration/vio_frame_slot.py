"""Account for camera frames overwritten in the latest-only frame slot.

The camera source (vio_ros_source.RosFrameSource, pinned teammate code) keeps
only the newest frame: when the consumer is busy, older frames are replaced
silently. That policy is correct ("process the most recent frame when behind")
but every replaced frame must be reported with its acquisition stamp. The
camera callback records each accepted stamp here; when the consumer takes a
frame, everything that arrived after the previous consumed frame and before
this one is returned as superseded.
"""
from collections import deque
import threading


class SlotAccounting:
    def __init__(self, capacity=256):
        self._arrived = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self.last_consumed_ns = None
        self.superseded = 0
        self.overflowed = 0

    def arrived(self, stamp_ns):
        """Camera callback thread: a frame with this stamp entered the slot."""
        with self._lock:
            if len(self._arrived) == self._arrived.maxlen:
                self.overflowed += 1
            self._arrived.append(stamp_ns)

    def newest_arrived_ns(self):
        """Stamp of the newest frame that entered the slot, or None."""
        with self._lock:
            return self._arrived[-1] if self._arrived else self.last_consumed_ns

    def consumed(self, stamp_ns):
        """Consumer thread: this frame was taken; return the stamps it replaced."""
        with self._lock:
            replaced = [s for s in self._arrived
                        if s < stamp_ns and (self.last_consumed_ns is None or s > self.last_consumed_ns)]
            while self._arrived and self._arrived[0] <= stamp_ns:
                self._arrived.popleft()
        self.last_consumed_ns = stamp_ns
        self.superseded += len(replaced)
        return replaced
