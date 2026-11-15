"""Stage 1 supervision policy ONLY; no simulator/FCU/test launch command.

ManagerRecovery is a host-manager trust boundary. No working systemd adapter is
shipped: default admission is BLOCKED. In particular, this module never treats a
finally handler, child watchdog, shell trap or Docker timeout as recovery proof.
"""
from dataclasses import dataclass
import math
import time

from heartbeat_test_owner import Blocked


@dataclass(frozen=True)
class RecoveryRegistration:
    test_id: str
    ledger_sha256: str
    deadline_ns: int
    token: str


class ManagerRecovery:
    """Required HOST backend; implementations must be independently reviewed.

    register(): persist root-controlled recovery authority and absolute deadline;
      manager timer/recovery must survive SIGKILL of launcher AND watchdog.
    verify(): query the manager, not the workload or a self-reported flag. Prove
      immutable timer/deadline, boot/time namespace, exact container/cgroup,
      granted handles/permissions and outside-cgroup watchdog/recovery placement.
    recover(): idempotently invoke CleanupController with original deadline.
      It must NOT disarm the manager timer before a verified durable receipt.
    No implementation is allowed to rebase a restored deadline or use names as
      cleanup authority. Host manager crash/boot change => INCOMPLETE/HOLD.
    """
    def register(self, ledger):
        raise Blocked('BLOCKED: Linux manager-owned independent recovery is not integrated')

    def verify(self, registration, ledger):
        raise Blocked('BLOCKED: independent manager registration/permissions unverified')

    def recover(self, registration, ledger, reason):
        raise Blocked('BLOCKED: no reviewed host recovery helper')


class StartupBarrier:
    """No workload callback is accepted here; release is a decision, not a launch."""
    def __init__(self, ledger, manager=None, clock=time.monotonic_ns):
        self.ledger = ledger
        self.manager = manager if manager is not None else ManagerRecovery()
        self.clock, self.registration, self.released = clock, None, False

    def _verify(self, registration):
        ledger = self.ledger
        if (not isinstance(registration, RecoveryRegistration) or not registration.token or
                registration.test_id != ledger.test_id or
                registration.ledger_sha256 != ledger.digest() or
                registration.deadline_ns != ledger.deadline.end_ns):
            raise Blocked('Recovery registration identity/deadline mismatch')
        if self.manager.verify(registration, ledger) is not True:
            raise Blocked('Recovery authority/containment/permissions unavailable')

    def arm(self):
        if self.registration is not None: raise Blocked('Recovery already armed')
        self._remaining()
        registration = self.manager.register(self.ledger)
        self._verify(registration)
        self._remaining()
        self.registration = registration
        return registration

    def _remaining(self):
        d = self.ledger.deadline
        if d.remaining(self.clock(), d.term_ns+d.kill_ns+d.verify_ns+d.export_ns) <= 0:
            raise Blocked('No observation budget before reserved cleanup deadline')

    def release(self):
        if self.registration is None or self.released: raise Blocked('Unarmed/repeated startup barrier')
        self._verify(self.registration)  # immediate manager-backed authority recheck
        self._remaining()
        self.released = True
        return True


class IndependentWatchdog:
    """Policy for a distinct HOST process. Not itself the recovery authority.

    health() returns a trusted liveness/identity result, never kill-0 alone.
    The manager remains armed when this loop dies. The Linux service binding
    and crash guarantees are BLOCKED pending native implementation/verification.
    """
    def __init__(self, ledger, registration, manager, health, clock=time.monotonic_ns):
        self.ledger, self.registration, self.manager = ledger, registration, manager
        self.health, self.clock, self.state = health, clock, 'ARMED'

    def tick(self):
        d = self.ledger.deadline
        barrier = StartupBarrier(self.ledger, self.manager, self.clock)
        try:
            barrier._verify(self.registration)
            if not d.remaining(self.clock(), d.term_ns+d.kill_ns+d.verify_ns+d.export_ns):
                reason = 'reserved cleanup boundary reached'
            else:
                state = self.health()
                if state == 'HEALTHY': self.state = 'MONITORING'; return None
                if state not in ('EXITED', 'HUNG', 'UNKNOWN'):
                    raise Blocked('Invalid coordinator health proof')
                reason = 'coordinator '+state
        except Exception as exc:
            reason = 'authority/liveness unverified: '+str(exc)
        self.state = 'RECOVERY_REQUESTED'
        # Manager may still act on retained verified kernel authority if a fresh
        # API check failed. Never signal from this process using guessed IDs.
        result = self.manager.recover(self.registration, self.ledger, reason)
        self.state = 'DONE' if result.get('status') == 'PASS' else 'INCOMPLETE'
        return result

    def run(self, poll_seconds, sleep=time.sleep):
        if (not isinstance(poll_seconds, (float, int)) or not math.isfinite(poll_seconds) or
                poll_seconds <= 0):
            raise Blocked('Require explicit bounded administrative poll interval')
        while True:
            result = self.tick()
            if result is not None: return result
            d = self.ledger.deadline
            remaining = d.remaining(self.clock(),d.term_ns+d.kill_ns+d.verify_ns+d.export_ns)
            sleep(min(poll_seconds, remaining))
