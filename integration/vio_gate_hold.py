"""Gate-closed hold (design option D, DEGRADED_MODE_DESIGN.md section 5). No ROS.

While the quality gate is closed for a visual reason only, the producer sends no
odometry, only a status "VIO alive, gate closed" so the transport can tell a
closed gate from a silent VIO. EKF3 coasts on its own IMU. The hold has a hard
cap counted from the first closed-gate frame, enforced on both sides.
"""
import hashlib
import json
import math
from pathlib import Path

TOPIC = '/integration/vio/gate_status'
QOS_DEPTH = 100                      # reliable, keep-last (same as the epoch token)
KEYS = {'schema': int, 'session': str, 'epoch': int, 'seq': int, 'frame_stamp_ns': int,
        'hold_start_ns': int, 'last_normal_stamp_ns': int, 'cause': str, 'frames_closed': int}
CAUSES = ('LOW_INLIERS', 'LOW_CONTRAST', 'FLOW_REJECTED', 'RECOVERING')
_CAUSE_BY_REASON = {'inliers': 'LOW_INLIERS', 'contrast': 'LOW_CONTRAST',
                    'distributed flow disagreement/unavailable': 'FLOW_REJECTED',
                    'flow inconsistent with IMU prediction / aliased': 'FLOW_REJECTED'}
SILENCE_NS = 500_000_000             # unchanged transport watchdog limit
STATUS_MAX_AGE_NS = 200_000_000      # same sim freshness as odometry
NORMAL_BETWEEN_HOLDS = 3


def load_budget(max_sim_s, max_host_s, evidence, *, simulation=None):
    """None when not configured. simulation: True/False if known (transport), None (producer)."""
    given = [v is not None for v in (max_sim_s, max_host_s, evidence)]
    if not any(given):
        return None
    if not all(given):
        raise ValueError('gate-closed hold needs max sim, max host and budget evidence together')
    path = Path(evidence)
    doc = json.loads(path.read_text())
    if doc.get('schema') != 1 or doc.get('scope') not in ('simulation', 'hardware'):
        raise ValueError('malformed hold budget evidence')
    if doc['scope'] == 'hardware':
        raise ValueError('hardware hold cap refused: no measured cap (M4) is accepted by this version')
    if simulation is False:
        raise ValueError('simulation-scope hold cap requires --simulation-vision-permit')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in (max_sim_s, max_host_s)):
        raise ValueError('hold caps must be positive and finite')
    if (max_sim_s, max_host_s) != (doc.get('max_sim_s'), doc.get('max_host_s')):
        raise ValueError('hold caps differ from the budget evidence')
    for item in doc.get('derived_from', []):
        if hashlib.sha256((path.parent/item['path']).read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('hold budget evidence source changed: '+item['path'])
    return dict(max_sim_ns=round(max_sim_s*1e9), max_host_ns=round(max_host_s*1e9), scope=doc['scope'],
                evidence=str(path), evidence_sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def frame_cause(gate_reason, flow_nis_rejected):
    """Closed cause list; None means the reason must end any hold."""
    if gate_reason is None:
        return None
    if gate_reason.startswith('readiness hysteresis'):
        return 'RECOVERING'
    if gate_reason == 'EKF not ready':
        return 'FLOW_REJECTED' if flow_nis_rejected else None
    return _CAUSE_BY_REASON.get(gate_reason)


def nonvisual_ok(t, wall, **inputs):
    """Every non-visual gate check, with the gate's own thresholds.

    QualityGate.assess stops at the first failure, so on an image rejection the
    later checks (state, covariance) were never evaluated. A throwaway gate with
    the visual inputs forced to pass reports a hysteresis reason only if every
    other check passed.
    """
    from vio_quality_gate import QualityGate
    probe = QualityGate()
    probe.assess(t, wall, inliers=15, contrast=8., audit_ok=True, consistency_ok=True, ekf_ok=True, **inputs)
    return probe.reason is not None and probe.reason.startswith('readiness hysteresis')


class ProducerHold:
    """One status per hold-eligible processed frame; host times in seconds."""
    def __init__(self, budget, session):
        self.max_sim_ns, self.max_host_ns = budget['max_sim_ns'], budget['max_host_ns']
        self.session, self.seq = session, 0
        self.last_normal_ns = None
        self.hold = None
        self.transitions = []

    def _exit(self, event, stamp_ns, host_s, **detail):
        if self.hold is not None:
            self.transitions.append(dict(event=event, stamp_ns=stamp_ns, host_s=host_s,
                                         hold_start_ns=self.hold['start_ns'], frames=self.hold['frames'],
                                         duration_host_s=host_s-self.hold['start_host'], **detail))
        self.hold = None

    def normal(self, stamp_ns, host_s):
        self._exit('exit_normal', stamp_ns, host_s)
        self.last_normal_ns = stamp_ns

    def abort(self, reason, stamp_ns, host_s, **detail):
        self._exit('exit_fail_closed', stamp_ns, host_s, reason=reason, **detail)
        self.last_normal_ns = None          # no new hold before the next normal publication

    def status(self, stamp_ns, host_s, epoch, cause, eligible, **detail):
        """Return the status message for this frame, or None (and end any hold)."""
        if cause is None or not eligible or epoch != 0 or self.last_normal_ns is None:
            self.abort('not hold-eligible' if cause else 'non-visual cause', stamp_ns, host_s, **detail)
            return None
        if self.hold is None:
            if cause == 'RECOVERING':        # a hold starts only with a visual rejection
                self.abort('recovery without hold', stamp_ns, host_s)
                return None
            self.hold = dict(start_ns=stamp_ns, start_host=host_s, frames=0)
            self.transitions.append(dict(event='enter', stamp_ns=stamp_ns, host_s=host_s, cause=cause,
                                         last_normal_ns=self.last_normal_ns, **detail))
        h = self.hold
        if stamp_ns-h['start_ns'] > self.max_sim_ns or (host_s-h['start_host'])*1e9 > self.max_host_ns:
            self.abort('hold budget exceeded', stamp_ns, host_s, **detail)
            return None
        h['frames'] += 1
        message = dict(schema=1, session=self.session, epoch=0, seq=self.seq, frame_stamp_ns=stamp_ns,
                       hold_start_ns=h['start_ns'], last_normal_stamp_ns=self.last_normal_ns,
                       cause=cause, frames_closed=h['frames'])
        self.seq += 1
        return message


def parse_status(data, producer):
    """Raise ValueError (route latch) unless this is a well-formed status from our producer."""
    row = json.loads(data)
    if (not isinstance(row, dict) or set(row) != set(KEYS)
            or any(type(row[k]) is not t for k, t in KEYS.items())):
        raise ValueError('malformed')
    if row['schema'] != 1 or row['session'] != producer or row['epoch'] != 0 or row['cause'] not in CAUSES:
        raise ValueError('foreign/unsupported status')
    return row


class TransportHold:
    """Independent of the producer. Host times in monotonic ns."""
    def __init__(self, budget, log=lambda **row: None):
        self.max_sim_ns, self.max_host_ns = budget['max_sim_ns'], budget['max_host_ns']
        self.log = log
        self.expected_seq = self.last_frame_ns = self.last_normal_ns = None
        self.normal_since_hold = None       # None: no hold yet
        self.hold = None

    def _end(self, reason, host_ns, event='gate_hold_ended'):
        if self.hold is not None:
            self.log(event=event, reason=reason, host_ns=host_ns, hold_start_ns=self.hold['start_ns'],
                     last_frame_ns=self.hold['last_frame_ns'], duration_host_ns=host_ns-self.hold['start_host'])
            self.hold = None
            self.normal_since_hold = 0

    def normal(self, acquisition_ns, host_ns):
        self._end('normal odometry resumed', host_ns, event='gate_hold_exited_normal')
        self.last_normal_ns = acquisition_ns
        if self.normal_since_hold is not None:
            self.normal_since_hold += 1

    def on_status(self, data, host_ns, ros_clock_ns, producer):
        """True if the status keeps a hold alive. Raises ValueError only for foreign/malformed input."""
        row = parse_status(data, producer)
        seq_ok = self.expected_seq is None or row['seq'] == self.expected_seq
        self.expected_seq = row['seq']+1
        stamp = row['frame_stamp_ns']
        problem = ('status sequence gap/repeat' if not seq_ok
                   else 'status stamp not increasing' if self.last_frame_ns is not None and stamp <= self.last_frame_ns
                   else 'status stale/future' if ros_clock_ns is None or not 0 <= ros_clock_ns-stamp <= STATUS_MAX_AGE_NS
                   else 'last normal sample mismatch' if row['last_normal_stamp_ns'] != self.last_normal_ns
                   else 'hold start changed' if self.hold is not None and row['hold_start_ns'] != self.hold['start_ns']
                   else None)
        self.last_frame_ns = max(stamp, self.last_frame_ns or stamp)
        if problem:
            self.log(event='gate_status_rejected', reason=problem, host_ns=host_ns, seq=row['seq'], frame_stamp_ns=stamp)
            self._end(problem, host_ns)
            return False
        if self.hold is None:
            if (self.last_normal_ns is None or row['cause'] == 'RECOVERING'
                    or (self.normal_since_hold is not None and self.normal_since_hold < NORMAL_BETWEEN_HOLDS)
                    or not self.last_normal_ns < row['hold_start_ns'] <= stamp):
                self.log(event='gate_status_ignored', host_ns=host_ns, seq=row['seq'], cause=row['cause'],
                         normal_since_hold=self.normal_since_hold)
                return False
            self.hold = dict(start_ns=row['hold_start_ns'], start_host=host_ns)
            self.log(event='gate_hold_entered', host_ns=host_ns, hold_start_ns=row['hold_start_ns'],
                     cause=row['cause'], last_normal_ns=self.last_normal_ns)
        self.hold.update(last_status_host=host_ns, last_frame_ns=stamp, cause=row['cause'])
        return True

    def cap_exceeded(self, now_ns, ros_clock_ns):
        """Host cap from the first status; sim cap on status stamps AND on the transport's own
        /clock (plus the usual 200 ms freshness allowance), so producer silence cannot stretch it."""
        h = self.hold
        return h is not None and (now_ns-h['start_host'] > self.max_host_ns
                                  or h['last_frame_ns']-h['start_ns'] > self.max_sim_ns
                                  or ros_clock_ns is None
                                  or ros_clock_ns-h['start_ns'] > self.max_sim_ns+STATUS_MAX_AGE_NS)

    def valid(self, now_ns, ros_clock_ns):
        h = self.hold
        return (h is not None and not self.cap_exceeded(now_ns, ros_clock_ns)
                and 0 <= now_ns-h['last_status_host'] <= SILENCE_NS)
