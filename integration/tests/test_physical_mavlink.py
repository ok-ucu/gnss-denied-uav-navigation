"""Synthetic bytes and local PTY only; no physical FCU or hardware evidence."""
import fcntl
import json
import os
from pathlib import Path
import pty
import tempfile
import unittest
from unittest.mock import patch

from physical_mavlink import PhysicalGuard, ReadOnlySerial, configuration, receive, main
from linux_gpu.verified_disarmed import load_codec


def config():
    return dict(device='/synthetic/tty', baud=115200, expected_system=1,
                expected_component=1, expected_uid=12345, firmware_version='4.6.3', mode='receive-only')


class PhysicalTests(unittest.TestCase):
    def setUp(self):
        self.w = load_codec()
        self.encoder = self.w.MAVLink(None, srcSystem=1, srcComponent=1)

    def hb(self, armed=False, system=1):
        e = self.w.MAVLink(None, srcSystem=system, srcComponent=1)
        return self.w.MAVLink_heartbeat_message(2, 3, 128 if armed else 0, 0, 3, 3).pack(e)

    def version(self, uid=12345, version=(4 << 24) | (6 << 16) | (3 << 8) | 255):
        return self.encoder.autopilot_version_encode(0, version, 0, 0, 0, [0]*8, [0]*8, [0]*8, 0, 0, uid).pack(self.encoder)

    def ready(self):
        g = PhysicalGuard(config(), 0)
        g.feed(self.version(), 0)
        for t in (0, 1_000_000_000, 2_000_000_000):
            g.feed(self.hb()+self.boot(t//1_000_000), t)
        self.assertTrue(g.ready(2_000_000_000))
        return g

    def boot(self, ms):
        return self.encoder.system_time_encode(0, ms).pack(self.encoder)

    def test_default_deny_and_invalid_configuration(self):
        for key, val in [('mode', 'forward'), ('device', None), ('device', 'udp:host:14550'),
                         ('baud', 0), ('baud', None), ('expected_uid', 0), ('expected_system', True),
                         ('expected_component', 0), ('firmware_version', '4.4.4')]:
            c = config(); c[key] = val
            with self.subTest(key=key, value=val), self.assertRaises(ValueError):
                configuration(c)
        self.assertFalse(hasattr(ReadOnlySerial, 'write'))
        self.assertFalse(hasattr(ReadOnlySerial, 'send'))

    def test_config_only_default_never_opens_a_port(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'config.json'; path.write_text(json.dumps(config()))
            with patch('sys.argv', ['physical_mavlink.py', '--config', str(path)]), \
                 patch('physical_mavlink.receive', side_effect=AssertionError('No port open allowed')):
                self.assertEqual(main(), 0)

    def test_valid_fragmented_heartbeat_and_version(self):
        g = PhysicalGuard(config(), 0)
        data = self.version()+self.hb()
        for octet in data:
            g.feed(bytes([octet]), 0)
        for t in (1_000_000_000, 2_000_000_000):
            g.feed(self.hb(), t)
        self.assertTrue(g.protocol_ready(2_000_000_000))
        self.assertFalse(g.ready(2_000_000_000))
        self.assertEqual(g.boot_continuity(2_000_000_000)['status'], 'NOT VERIFIED')
        self.assertEqual(g.heartbeat.bad_data, 0)

    def test_wrong_identity_firmware_and_armed_latch(self):
        for data in (self.hb(True), self.hb(system=2), self.version(uid=999), self.version(version=4 << 24)):
            g = PhysicalGuard(config(), 0)
            with self.assertRaises(RuntimeError): g.feed(data, 0)
            with self.assertRaises(RuntimeError): g.feed(self.version()+self.hb(), 1)

    def test_bad_data_does_not_authenticate_identity(self):
        g = PhysicalGuard(config(), 0)
        g.feed(b'invalid-prefix', 0)
        self.assertGreater(g.heartbeat.bad_data, 0)
        self.assertFalse(g.ready(0))
        with self.assertRaises(RuntimeError): g.tick(10_000_000_001)

    def test_missing_version_is_not_ready(self):
        g = PhysicalGuard(config(), 0)
        for t in (0, 1_000_000_000, 2_000_000_000): g.feed(self.hb(), t)
        self.assertFalse(g.ready(2_000_000_000))

    def test_stale_clock_reverse_and_fcu_restart_latch(self):
        g = self.ready()
        with self.assertRaises(RuntimeError): g.tick(5_000_000_001)
        with self.assertRaises(RuntimeError): g.feed(self.hb(), 5_100_000_000)
        g = self.ready()
        with self.assertRaises(RuntimeError): g.tick(1)
        g = self.ready()
        g.feed(self.encoder.system_time_encode(0, 3000).pack(self.encoder), 2_000_000_001)
        with self.assertRaisesRegex(RuntimeError, 'restart'):
            g.feed(self.encoder.system_time_encode(0, 1).pack(self.encoder), 2_000_000_002)

    def test_disconnect_has_one_open_no_reconnect_and_persistent_failure(self):
        calls = []
        class EOFBackend:
            def __init__(self, c): calls.append('open')
            def read(self, timeout): return b''
            def close(self): calls.append('close')
        with tempfile.TemporaryDirectory() as tmp:
            result = receive(config(), Path(tmp)/'run', backend_factory=EOFBackend, clock=lambda: 0)
            self.assertEqual(result['status'], 'FAIL')
            self.assertIn('disconnected', result['error'])
            self.assertEqual(result['transmitted_bytes'], 0)
            self.assertFalse(result['forwarding_enabled'])
            self.assertEqual(json.loads((Path(tmp)/'run/result.json').read_text())['status'], 'FAIL')
        self.assertEqual(calls, ['open', 'close'])

    def test_bounded_passive_diagnostic_and_timeout_with_fake_clock(self):
        for heartbeat in (True, False):
            now = [0]; closed = []
            owner = self
            class Backend:
                def __init__(self, c): pass
                def read(self, timeout):
                    now[0] += 1_000_000_000
                    return owner.version()+owner.hb()+owner.boot(now[0]//1_000_000) if heartbeat else None
                def close(self): closed.append(True)
            with tempfile.TemporaryDirectory() as tmp:
                result = receive(config(), Path(tmp)/'run', duration=5,
                    backend_factory=Backend, clock=lambda: now[0])
            self.assertEqual(result['status'], 'PASS' if heartbeat else 'FAIL')
            self.assertEqual(result['transmitted_bytes'], 0)
            self.assertEqual(closed, [True])

    def test_protocol_identity_does_not_prove_boot_continuity(self):
        g = PhysicalGuard(config(), 0)
        g.feed(self.version(), 0)
        for t in (0, 1_000_000_000, 2_000_000_000): g.feed(self.hb(), t)
        self.assertTrue(g.protocol_ready(2_000_000_000))
        self.assertFalse(g.ready(2_000_000_000))
        g.feed(self.boot(100), 2_000_000_000)
        g.feed(self.boot(100), 2_100_000_000)
        self.assertEqual(g.boot_continuity(2_100_000_000)['status'], 'NOT VERIFIED')
        g.feed(self.boot(200), 2_200_000_000)
        self.assertTrue(g.ready(2_200_000_000))
        self.assertIn('not guaranteed', g.boot_continuity(2_200_000_000)['unobserved_reboot_detection'])
        for t in (3_000_000_000, 4_000_000_000, 5_000_000_000, 6_000_000_000): g.feed(self.hb(), t)
        self.assertTrue(g.protocol_ready(6_000_000_000))
        self.assertFalse(g.ready(6_000_000_000))  # heartbeat cannot refresh boot evidence

    def test_bounded_rx_without_passive_boot_reports_not_verified(self):
        now = [0]; payload = self.version()+self.hb()
        class Backend:
            def __init__(self, c): pass
            def read(self, timeout): now[0] += 1_000_000_000; return payload
            def close(self): pass
        with tempfile.TemporaryDirectory() as tmp:
            report = receive(config(), Path(tmp)/'run', duration=5, backend_factory=Backend, clock=lambda:now[0])
        self.assertEqual(report['protocol_identity_status'], 'PASS')
        self.assertEqual(report['boot_continuity']['status'], 'NOT VERIFIED')
        self.assertEqual(report['status'], 'NOT VERIFIED')
        self.assertFalse(report['forwarding_enabled'])

    def test_keyboard_interrupt_closes_and_preserves_failed_audit(self):
        closed = []
        class InterruptBackend:
            def __init__(self, c): pass
            def read(self, timeout): raise KeyboardInterrupt()
            def close(self): closed.append(True)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'run'
            with self.assertRaises(KeyboardInterrupt):
                receive(config(), out, backend_factory=InterruptBackend, clock=lambda: 0)
            report = json.loads((out/'result.json').read_text())
            self.assertEqual(report['status'], 'FAIL'); self.assertEqual(report['error'], 'KeyboardInterrupt')
        self.assertEqual(closed, [True])

    def test_actual_local_pty_descriptor_is_read_only_no_upstream_application_writes(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master); self.addCleanup(os.close, slave)
        c = config(); c['device'] = os.ttyname(slave)
        port = ReadOnlySerial(c); self.addCleanup(port.close)
        self.assertEqual(fcntl.fcntl(port.fd, fcntl.F_GETFL) & os.O_ACCMODE, os.O_RDONLY)
        os.write(master, self.hb())  # synthetic source only
        self.assertEqual(port.read(.1), self.hb())
        self.assertIsNone(port.read(.001))
        with self.assertRaises(OSError): os.write(port.fd, b'not-permitted')


if __name__ == '__main__': unittest.main()
