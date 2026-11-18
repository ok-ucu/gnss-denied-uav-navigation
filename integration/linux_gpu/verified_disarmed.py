"""Passive, sustained DISARMED check on an isolated expected FCU TCP endpoint.

CRC/framing and source/role checks are protocol validation, NOT cryptographic
authentication. BAD_DATA never counts as heartbeat. No MAVLink bytes are sent.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import socket
import time

CODEC = Path(__file__).resolve().parents[1]/'mavros_live_runs/20261108/mavlink_v20.py'


def load_codec():
    spec = importlib.util.spec_from_file_location('verified_disarmed_wire', CODEC)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class HeartbeatGuard:
    def __init__(self, system, component, started, record=lambda **row: None):
        if any(type(v) is not int or not 1 <= v <= 255 for v in (system, component)):
            raise ValueError('Require explicit expected system/component in 1..255')
        self.system, self.component, self.started = system, component, started
        self.wire = load_codec()
        self.parser = self.wire.MAVLink(None)
        self.parser.robust_parsing = True
        self.record = record
        self.first = self.last = None
        self.count = self.bad_data = 0
        self.failure = None

    def fail(self, reason):
        self.failure = reason
        raise RuntimeError(reason)

    def assert_fresh(self, now):
        if self.failure:
            raise RuntimeError(self.failure)
        if not math.isfinite(now) or now < (self.last if self.last is not None else self.started):
            self.fail('Heartbeat monotonic time reversed/nonfinite')
        if self.last is None:
            if now-self.started > 10:
                self.fail('No expected DISARMED heartbeat within 10s')
        elif now-self.last > 3:
            self.fail('Expected DISARMED heartbeat stale >3s')

    def feed(self, data, now):
        self.assert_fresh(now)  # A late packet cannot erase an observed outage.
        for message in self.parser.parse_buffer(data) or []:
            if message.get_type() == 'BAD_DATA':
                self.bad_data += 1
                self.record(kind='bad_data', host=now, reason=str(message.reason),
                            bytes=len(message.data))
                continue
            if message.get_type() != 'HEARTBEAT':
                continue
            source = (message.get_srcSystem(), message.get_srcComponent())
            self.record(kind='heartbeat', host=now, system=source[0], component=source[1],
                mavlink2=message.get_msgbuf()[0] == 253, base_mode=message.base_mode,
                autopilot=message.autopilot, vehicle_type=message.type,
                packet_hex=bytes(message.get_msgbuf()).hex())
            if source != (self.system, self.component):
                self.fail('Unexpected HEARTBEAT source IDs: '+str(source))
            if message.get_msgbuf()[0] != 253 or message.mavlink_version != 3:
                self.fail('Expected valid MAVLink 2 HEARTBEAT')
            if (message.autopilot != self.wire.MAV_AUTOPILOT_ARDUPILOTMEGA or
                    message.type != self.wire.MAV_TYPE_QUADROTOR):
                self.fail('Unexpected HEARTBEAT autopilot/vehicle role')
            if message.base_mode & self.wire.MAV_MODE_FLAG_SAFETY_ARMED:
                self.fail('FCU ARMED: abort')
            if self.first is None:
                self.first = now
            self.last = now
            self.count += 1

    def ready(self, now):
        self.assert_fresh(now)
        return self.count >= 3 and self.last-self.first >= 2


def check(system, component, directory, duration=5):
    if not math.isfinite(duration) or not 5 <= duration <= 30:
        raise ValueError('Require finite check duration 5..30s')
    directory = Path(directory)
    directory.mkdir(exist_ok=False)
    result = dict(status='FAIL', decision='HOLD', execution_allowed=False,
        forwarding_enabled=False, expected_system=system, expected_component=component,
        authentication='CRC/framing + expected source/role; no cryptographic authentication')
    with (directory/'events.jsonl').open('x') as events, (directory/'tcp.bin').open('xb') as raw:
        def record(**row):
            events.write(json.dumps(row)+'\n')
            events.flush()
        guard = HeartbeatGuard(system, component, time.monotonic(), record)
        total = 0
        try:
            with socket.create_connection(('127.0.0.1', 5760), timeout=2) as connection:
                connection.settimeout(.1)
                deadline = time.monotonic()+duration
                while time.monotonic() < deadline:
                    try:
                        data = connection.recv(65535)
                    except socket.timeout:
                        guard.assert_fresh(time.monotonic())
                        continue
                    if not data:
                        guard.fail('FCU disconnected')
                    now = time.monotonic()
                    total += len(data)
                    if total > 4*1024*1024:
                        guard.fail('Heartbeat raw capture byte bound exceeded')
                    record(kind='tcp_chunk', host=now, offset=raw.tell(), bytes=len(data))
                    raw.write(data)
                    guard.feed(data, now)
                if not guard.ready(time.monotonic()):
                    guard.fail('Insufficient sustained expected DISARMED heartbeat evidence')
            result['status'] = 'PASS'
        except Exception as exc:
            result['error'] = str(exc)
            raise
        finally:
            result.update(heartbeats=guard.count, bad_data=guard.bad_data,
                          first_host=guard.first, last_host=guard.last, bytes=total)
            (directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-system', type=int, required=True)
    parser.add_argument('--expected-component', type=int, required=True)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=5)
    args = parser.parse_args()
    check(args.expected_system, args.expected_component, args.directory, args.duration)
