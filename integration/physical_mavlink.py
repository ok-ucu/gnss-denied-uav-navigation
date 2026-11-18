"""Physical USB/UART diagnostic RX ONLY. No transmitter, reconnect or FCU writes.

Protocol identity is CRC + expected role/IDs + advertised UID/version, not
cryptographic authentication or proof of firmware binary/board wiring.
Importing/configuring this module never opens a port.
"""
import argparse
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import select
import stat
import termios
import time

from linux_gpu.verified_disarmed import HeartbeatGuard, load_codec


def configuration(c):
    required = {'device', 'baud', 'expected_system', 'expected_component',
                'expected_uid', 'firmware_version', 'mode'}
    if set(c) != required or c.get('mode') != 'receive-only':
        raise ValueError('Exact receive-only configuration required; TX is not implemented')
    if not isinstance(c['device'], str) or not Path(c['device']).is_absolute():
        raise ValueError('Explicit absolute USB/UART device path required (no URLs)')
    if type(c['baud']) is not int or c['baud'] <= 0 or not hasattr(termios, 'B'+str(c['baud'])):
        raise ValueError('Explicit baud unsupported by this host; no fallback')
    for name in ('expected_system', 'expected_component'):
        if type(c[name]) is not int or not 1 <= c[name] <= 255:
            raise ValueError('Explicit expected MAVLink source IDs required')
    if type(c['expected_uid']) is not int or not 0 < c['expected_uid'] < 2**64:
        raise ValueError('Expected AUTOPILOT_VERSION UID must be independently recorded')
    if c['firmware_version'] != '4.6.3':
        raise ValueError('This physical contract targets ArduCopter 4.6.3 only')
    return dict(c)


class ReadOnlySerial:
    """POSIX read-only descriptor; no write API, DTR/RTS commands or auto-reopen.

    Configures host tty raw mode/baud only. USB driver side effects, wiring,
    exclusive-access behavior and hardware disconnects require native review.
    """
    def __init__(self, c):
        c = configuration(c)
        self.fd = None
        self.original = None
        expected = os.stat(c['device'])
        if not stat.S_ISCHR(expected.st_mode):
            raise ValueError('Require character tty, not file/socket/URL')
        fd = os.open(c['device'], os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            actual = os.fstat(fd)
            if (actual.st_dev, actual.st_ino, actual.st_rdev) != (expected.st_dev, expected.st_ino, expected.st_rdev):
                raise ValueError('Device changed while opening')
            if not os.isatty(fd) or not hasattr(termios, 'TIOCEXCL'):
                raise ValueError('Require tty and exclusive-access support')
            fcntl.ioctl(fd, termios.TIOCEXCL)
            self.original = termios.tcgetattr(fd)
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[4] = attrs[5] = getattr(termios, 'B'+str(c['baud']))
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            self.fd = fd
        except BaseException:
            try:
                if self.original is not None:
                    termios.tcsetattr(fd, termios.TCSANOW, self.original)
            finally:
                os.close(fd)
            raise

    def read(self, timeout):
        if not select.select([self.fd], [], [], timeout)[0]:
            return None  # timeout is distinct from EOF
        try:
            return os.read(self.fd, 65536)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return None
            raise

    def close(self):
        if self.fd is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            finally:
                os.close(self.fd)
                self.fd = None


class PhysicalGuard:
    def __init__(self, c, started_ns, record=lambda **row: None):
        self.c = configuration(c)
        self.heartbeat = HeartbeatGuard(c['expected_system'], c['expected_component'],
                                        started_ns/1e9, record)
        self.parser = load_codec().MAVLink(None)
        self.parser.robust_parsing = True
        self.record = record
        self.version_seen = False
        self.boot_ms = None
        self.boot_first_ms = self.boot_last_host = None
        self.boot_advances = 0
        self.last_host = started_ns
        self.failure = None
        self.counts = {}

    def fail(self, reason):
        self.failure = reason
        raise RuntimeError(reason)

    def tick(self, host_ns):
        if self.failure:
            raise RuntimeError(self.failure)
        if type(host_ns) is not int or host_ns < self.last_host:
            self.fail('Host monotonic clock reversed/invalid')
        self.last_host = host_ns
        try:
            self.heartbeat.assert_fresh(host_ns/1e9)
        except RuntimeError as exc:
            self.fail(str(exc))

    def feed(self, data, host_ns):
        self.tick(host_ns)
        try:
            self.heartbeat.feed(data, host_ns/1e9)
            for msg in self.parser.parse_buffer(data) or []:
                kind = msg.get_type()
                self.counts[kind] = self.counts.get(kind, 0)+1
                if kind not in ('AUTOPILOT_VERSION', 'SYSTEM_TIME'):
                    continue
                if (msg.get_srcSystem(), msg.get_srcComponent()) != (self.c['expected_system'], self.c['expected_component']):
                    self.fail('Unexpected identity/time message source')
                if msg.get_msgbuf()[0] != 253:
                    self.fail('Expected MAVLink 2 identity/time evidence')
                if kind == 'AUTOPILOT_VERSION':
                    v = msg.flight_sw_version
                    if (v >> 24, (v >> 16) & 255, (v >> 8) & 255) != (4, 6, 3) or msg.uid != self.c['expected_uid']:
                        self.fail('FCU advertised UID/firmware version mismatch')
                    self.version_seen = True
                    self.record(kind='identity', host_ns=host_ns, uid=msg.uid, flight_sw_version=v)
                else:
                    if self.boot_ms is not None and msg.time_boot_ms < self.boot_ms:
                        self.fail('FCU restart/boot clock reset; no automatic reconnect')
                    if self.boot_ms is None:
                        self.boot_first_ms = msg.time_boot_ms
                    elif msg.time_boot_ms > self.boot_ms:
                        self.boot_advances += 1
                        self.boot_last_host = host_ns
                    self.boot_ms = msg.time_boot_ms
                    self.record(kind='fcu_clock_observation', host_ns=host_ns,
                                time_boot_ms=msg.time_boot_ms, time_unix_usec=msg.time_unix_usec,
                                mapping_verified=False)
        except (RuntimeError, ValueError) as exc:
            self.fail(str(exc))

    def protocol_ready(self, host_ns):
        self.tick(host_ns)
        return self.version_seen and self.heartbeat.ready(host_ns/1e9)

    def boot_continuity(self, host_ns):
        self.tick(host_ns)
        # Reuse the existing 3s telemetry freshness bound. Repeated/frozen boot
        # values do not renew evidence. No claim about unobserved reboots.
        observed = (self.boot_advances > 0 and self.boot_last_host is not None and
                    0 <= host_ns-self.boot_last_host <= 3_000_000_000)
        return dict(status='PASS' if observed else 'NOT VERIFIED',
                    scope='Observed passive boot-clock progression only',
                    first_boot_ms=self.boot_first_ms, last_boot_ms=self.boot_ms,
                    advances=self.boot_advances, last_progress_host_ns=self.boot_last_host,
                    unobserved_reboot_detection='NOT VERIFIED: not guaranteed detectable')

    def ready(self, host_ns):
        return self.protocol_ready(host_ns) and self.boot_continuity(host_ns)['status'] == 'PASS'


def receive(c, directory, duration=10., *, backend_factory=ReadOnlySerial,
            clock=time.monotonic_ns):
    """Bounded passive diagnostic. PASS covers protocol + observed boot evidence, never TX."""
    c = configuration(c)
    if not math.isfinite(duration) or not 3 <= duration <= 30:
        raise ValueError('Bounded receive duration must be 3..30 seconds')
    directory = Path(directory)
    directory.mkdir(exist_ok=False)
    result = dict(status='FAIL', decision='HOLD', execution_allowed=False,
                  forwarding_enabled=False, transmitted_bytes=0,
                  physical_forwarding='NOT IMPLEMENTED', clock_mapping='NOT VERIFIED',
                  protocol_identity_status='NOT VERIFIED',
                  boot_continuity=dict(status='NOT VERIFIED'),
                  identity_scope='Protocol/advertised UID only; not cryptographic or binary provenance')
    backend = None
    guard = None
    try:
        with (directory/'events.jsonl').open('x') as events, (directory/'rx.mavlink').open('xb') as raw:
            def record(**row):
                events.write(json.dumps(row, allow_nan=False)+'\n')
                events.flush()
            started = clock()
            deadline = started+round(duration*1e9)
            guard = PhysicalGuard(c, started, record)
            backend = backend_factory(c)
            while clock() < deadline:
                data = backend.read(min(.1, max(0., (deadline-clock())/1e9)))
                now = clock()
                guard.tick(now)  # unconditional, even without bytes
                if data == b'':
                    guard.fail('Physical link EOF/disconnected; explicit new session required')
                if data:
                    if raw.tell()+len(data) > 8*1024*1024:
                        guard.fail('Bounded capture exceeded')
                    record(kind='rx_chunk', host_ns=now, offset=raw.tell(), bytes=len(data))
                    raw.write(data)
                    guard.feed(data, now)
            end = clock()
            if not guard.protocol_ready(end):
                guard.fail('Missing sustained DISARMED heartbeat or passive AUTOPILOT_VERSION evidence')
            result['protocol_identity_status'] = 'PASS'
            result['boot_continuity'] = guard.boot_continuity(end)
            result['status'] = result['boot_continuity']['status']
    except Exception as exc:
        result['error'] = str(exc)
    except BaseException as exc:
        result['error'] = type(exc).__name__
        raise
    finally:
        try:
            if backend is not None:
                backend.close()
        except Exception as exc:
            result.update(status='FAIL', close_error=str(exc))
        result['counts'] = guard.counts if guard else {}
        result['bad_data'] = guard.heartbeat.bad_data if guard else 0
        (directory/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--receive', action='store_true', help='Explicit passive port open; no writes')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=10.)
    a = parser.parse_args()
    c = configuration(json.loads(a.config.read_text()))
    if not a.receive:
        print(json.dumps(dict(configuration='PASS', port_opened=False,
                              execution_allowed=False, forwarding_enabled=False)))
        return 0
    if a.output is None:
        parser.error('--receive requires a new --output evidence directory')
    result = receive(c, a.output, a.duration)
    print(json.dumps(result))
    return 0 if result['status'] == 'PASS' else (2 if result['status'] == 'NOT VERIFIED' else 1)


if __name__ == '__main__':
    raise SystemExit(main())
