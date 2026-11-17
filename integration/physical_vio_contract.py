"""Pure Pi IMU-origin candidate conversion; no publisher, serial or TX authority.

Requires an explicit source/reference token supplied by the caller. The present
Pi ROS publisher has no epoch side-channel: wiring this to hardware is BLOCKED.
Preserves the Pi IMU origin, not a claimed FCU/base_link origin.
"""
import copy
import math
from types import SimpleNamespace

from vio_adapter import convert_pair, validate_covariance
from vio_frame_bridge import convert as frame_convert
from analyze_mavros_wire import expected
from linux_gpu.verified_disarmed import load_codec

PUBLICATION_NS = 180_000_000  # existing Pi publication limit
AVAILABILITY_NS = 500_000_000  # existing Pi receipt watchdog


def integer_ns(value):
    if type(value) is not int or value < 0:
        raise ValueError('Expected raw nonnegative integer nanoseconds')
    return value


def message_stamp(message):
    s = message.header.stamp
    integer_ns(s.sec); integer_ns(s.nanosec)
    if s.nanosec >= 1_000_000_000:
        raise ValueError('Invalid ROS nanoseconds')
    return s.sec*1_000_000_000+s.nanosec


def payload_signature(message, kind):
    """Validate and snapshot numeric content, not object identity/ROS type."""
    fields = []
    if kind == 'pose':
        p, q = message.pose.pose.position, message.pose.pose.orientation
        fields = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        covariance = message.pose.covariance
        if abs(sum(v*v for v in fields[3:])-1) > 1e-3:
            raise ValueError('Invalid preview quaternion')
    else:
        twist = message.twist.twist
        fields = [getattr(v, axis) for v in (twist.linear, twist.angular) for axis in ('x', 'y', 'z')]
        covariance = message.twist.covariance
    if any(isinstance(v, bool) or not math.isfinite(v) for v in fields):
        raise ValueError('Nonfinite/bool preview payload')
    validate_covariance(covariance)
    return (message.header.frame_id, message_stamp(message), *fields, *covariance)


def candidate_signature(candidate):
    odom = candidate['odometry']
    return (odom.child_frame_id, payload_signature(odom, 'pose'), payload_signature(odom, 'speed'),
            payload_signature(candidate['pose'], 'pose'), payload_signature(candidate['speed'], 'speed'))


class PiCandidateGuard:
    def __init__(self, session, reference, *, clock_domain, clock_uncertainty_ns):
        if not isinstance(session, str) or not session or not isinstance(reference, str) or not reference:
            raise ValueError('Explicit new session/reference required')
        if clock_domain != 'ros-system-unix-ns':
            raise ValueError('Unknown acquisition clock domain')
        integer_ns(clock_uncertainty_ns)
        if clock_uncertainty_ns >= 5_000_000:  # existing Pi calibration contract
            raise ValueError('Unverified/excessive Pi clock uncertainty')
        self.identity = (session, reference, 0)
        self.uncertainty = clock_uncertainty_ns
        self.last_stamp = self.last_receipt = self.last_now = None
        self.failure = None
        self._admission = self._signature = None
        self._last_poll = None

    def fail(self, reason):
        self.failure = reason
        raise ValueError(reason)

    def check_availability(self, host_ns):
        if self.failure:
            raise ValueError(self.failure)
        try:
            integer_ns(host_ns)
        except ValueError as exc:
            self.fail(str(exc))
        if self._last_poll is not None and host_ns < self._last_poll:
            self.fail('Host monotonic clock reset')
        self._last_poll = host_ns
        if self.last_receipt is not None and not 0 <= host_ns-self.last_receipt <= AVAILABILITY_NS:
            self.fail('VIO receipt loss/monotonic reset >500ms; explicit reinitialization required')

    def admit(self, odom, *, session, reference, epoch, ready, now_system_ns,
              receipt_host_ns, pose_type=SimpleNamespace, speed_type=SimpleNamespace):
        try:
            if self.failure:
                self.fail(self.failure)
            integer_ns(receipt_host_ns)
            # Receipt order and polling time are different boundaries. A queued
            # frame may have arrived before the most recent availability poll.
            if self.last_receipt is not None and not 0 <= receipt_host_ns-self.last_receipt <= AVAILABILITY_NS:
                self.fail('VIO receipt loss/monotonic reset >500ms')
            integer_ns(now_system_ns)
            if type(epoch) is not int or (session, reference, epoch) != self.identity or ready is not True:
                self.fail('Unready/reset/session/reference mismatch; latched')
            if self.last_now is not None and now_system_ns < self.last_now:
                self.fail('Acquisition-domain clock reset')
            if odom.header.frame_id != 'pi5_odom_ned' or odom.child_frame_id != 'pi5_imu_frd':
                self.fail('Require calibrated Pi IMU-origin NED/FRD frames')
            s = odom.header.stamp
            integer_ns(s.sec)
            integer_ns(s.nanosec)
            if s.nanosec >= 1_000_000_000:
                self.fail('Invalid ROS nanoseconds')
            ns = s.sec*1_000_000_000+s.nanosec
            if ns <= 0 or self.last_stamp is not None and ns <= self.last_stamp:
                self.fail('Acquisition timestamp duplicate/reset/order')
            age = now_system_ns-ns
            if age < 0 or age+self.uncertainty > PUBLICATION_NS:
                self.fail('Stale/future VIO acquisition timestamp')
            # Physical data may not use the legacy bridge's unknown-angular
            # covariance substitution. Validate BEFORE invoking that bridge.
            validate_covariance(odom.pose.covariance)
            validate_covariance(odom.twist.covariance)
            if any(odom.twist.covariance[i*6+i] <= 0 for i in range(6)):
                self.fail('Measured linear AND angular uncertainty required')
            native = copy.deepcopy(odom)
            # Aliases invoke existing math AFTER semantic validation; resulting
            # frames explicitly retain the Pi IMU origin, never the FCU origin.
            native.header.frame_id = 'odom_ned'
            native.child_frame_id = 'base_link_frd'
            enu = frame_convert(native)
            enu.header.frame_id = 'pi5_odom_enu'
            enu.child_frame_id = 'pi5_imu_flu'
            pose, speed, stamp = convert_pair(enu, pose_type, speed_type, now_system_ns,
                self.last_stamp, world='pi5_odom_enu', body='pi5_imu_flu',
                max_age=PUBLICATION_NS/1e9, future_tolerance=0.)
            self.last_stamp, self.last_receipt, self.last_now = stamp, receipt_host_ns, now_system_ns
            self._admission = object()  # process-local, latest admission only; no TX authority
            result = dict(odometry=enu, pose=pose, speed=speed, acquisition_ns=stamp,
                        epoch=epoch, session=session, reference=reference,
                        ready=True, clock_domain='ros-system-unix-ns', receipt_host_ns=receipt_host_ns,
                        forwarding_authorized=False, _admission=self._admission)
            self._signature = candidate_signature(result)
            return result
        except (ValueError, AttributeError, TypeError) as exc:
            self.fail(str(exc))

    def revalidate(self, candidate, now_system_ns, host_ns):
        """Final encoding boundary: no reuse after loss, mutation or a newer frame."""
        self.check_availability(host_ns)
        integer_ns(now_system_ns)
        if (candidate.get('_admission') is not self._admission or self._admission is None or
                type(candidate.get('epoch')) is not int or
                (candidate.get('session'), candidate.get('reference'), candidate.get('epoch')) != self.identity or
                candidate.get('ready') is not True or candidate.get('forwarding_authorized') is not False or
                candidate.get('clock_domain') != 'ros-system-unix-ns'):
            self.fail('Invalid/reset/unowned preview admission')
        stamp = integer_ns(candidate['acquisition_ns'])
        if (stamp != self.last_stamp or candidate.get('receipt_host_ns') != self.last_receipt or
                now_system_ns < self.last_now):
            self.fail('Preview timestamp/reference mismatch or clock reset')
        if not 0 <= now_system_ns-stamp or now_system_ns-stamp+self.uncertainty > PUBLICATION_NS:
            self.fail('Stale/future preview acquisition')
        odom = candidate['odometry']
        if (odom.header.frame_id, odom.child_frame_id) != ('pi5_odom_enu', 'pi5_imu_flu'):
            self.fail('Preview must retain Pi IMU origin')
        if any(odom.twist.covariance[i*6+i] <= 0 for i in range(6)):
            self.fail('Unknown preview twist covariance')
        pose, speed, _ = convert_pair(odom, SimpleNamespace, SimpleNamespace, now_system_ns,
            world='pi5_odom_enu', body='pi5_imu_flu', max_age=PUBLICATION_NS/1e9, future_tolerance=0.)
        if (message_stamp(odom) != stamp or
                payload_signature(pose, 'pose') != payload_signature(candidate['pose'], 'pose') or
                payload_signature(speed, 'speed') != payload_signature(candidate['speed'], 'speed') or
                candidate_signature(candidate) != self._signature):
            self.fail('Mutated/inconsistent preview payload, covariance or timestamps')
        self.last_now = now_system_ns


def mavlink_preview(candidate, mapping, host_ns, *, guard, now_system_ns, source_system, source_component):
    """Encode an OFFLINE preview only, using the existing MAVROS wire math.

    No I/O. This preserves the Pi reference/origin, not an FCU reference claim.
    No physical forwarding authority is created by this conversion.
    """
    if type(guard) is not PiCandidateGuard:
        raise ValueError('Explicit original PiCandidateGuard required')
    try:
        if any(type(v) is not int or not 1 <= v <= 255 for v in (source_system, source_component)):
            raise ValueError('Explicit diagnostic MAVLink sender IDs required')
        guard.revalidate(candidate, now_system_ns, host_ns)
        return _encode_fixture(candidate, mapping, host_ns, source_system, source_component)
    except (ValueError, AttributeError, TypeError, KeyError, OverflowError) as exc:
        guard.fail(str(exc))


def _encode_fixture(candidate, mapping, host_ns, source_system, source_component):
    stamp = candidate['acquisition_ns']
    mapped = mapping.map(stamp, host_ns, candidate['session'])
    usec = mapped['fcu_ns']//1000
    if not 0 < usec < 2**64:
        raise ValueError('Mapped FCU timestamp must fit uint64 microseconds')
    p, q = candidate['pose'].pose.pose.position, candidate['pose'].pose.pose.orientation
    v = candidate['speed'].twist.twist.linear
    pose = expected(dict(kind='pose', values=[p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                         covariance=candidate['pose'].pose.covariance))
    speed = expected(dict(kind='speed', values=[v.x, v.y, v.z],
                          covariance=candidate['speed'].twist.covariance))
    wire = load_codec(); encoder = wire.MAVLink(None, srcSystem=source_system, srcComponent=source_component)
    position = wire.MAVLink_vision_position_estimate_message(usec, *pose['position'], *pose['rpy'],
                                                            pose['covariance'], reset_counter=0)
    velocity = wire.MAVLink_vision_speed_estimate_message(usec, *speed['speed'],
                                                        speed['covariance'], reset_counter=0)
    position_bytes = position.pack(encoder)
    encoder.seq = 1  # only the second packet of THIS isolated fixture
    return dict(position=position_bytes, velocity=velocity.pack(encoder),
                acquisition_ns=stamp, mapped_fcu_ns=mapped['fcu_ns'], uncertainty_ns=mapped['uncertainty_ns'],
                forwarding_authorized=False, non_transmittable=True,
                sequence_scope='Isolated two-packet fixture, seq 0/1; resets each call, NOT a continuous stream',
                scope='Offline Pi-reference packet preview; not FCU-aligned hardware output')
