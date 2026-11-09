"""Fail-closed synthetic reception-bench guard; never approves a reference itself."""
from reset_safety import ResetSafety


class ReceiptGuard:
    def __init__(self, path, max_age_ns=200_000_000):
        self.durable = ResetSafety(path)
        self.max_age_ns = max_age_ns
        self.reason = 'uninitialized'
        self.last_arrival_ns = None

    def block(self, reason):
        self.reason = reason
        self.durable.state['blocked'] = True
        self.durable.save()
        return False

    def admit(self, producer, epoch, stamp_ns, now_ns):
        age = now_ns - stamp_ns
        if age < -50_000_000 or age > self.max_age_ns:
            return self.block('clock_or_freshness')
        if not self.durable.admit(producer, epoch, stamp_ns):
            return self.block('epoch_or_order_or_restart')
        self.last_arrival_ns = now_ns
        self.reason = 'ready'
        return True

    def watchdog(self, now_ns):
        if self.last_arrival_ns is not None and now_ns - self.last_arrival_ns > self.max_age_ns:
            return self.block('source_timeout')
        return not self.durable.state['blocked']
