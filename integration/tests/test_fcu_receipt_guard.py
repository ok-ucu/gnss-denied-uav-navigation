import tempfile
from pathlib import Path
import unittest
from fcu_receipt_guard import ReceiptGuard


class ReceiptTests(unittest.TestCase):
    def test_faults_latch_and_restart(self):
        for fault in ('delay', 'future', 'epoch', 'restart', 'stop'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'state.json'
                guard = ReceiptGuard(path)
                guard.durable.initialize('verified-reference', 'producer')
                self.assertTrue(guard.admit('producer', 0, 1_000_000_000, 1_000_000_000))
                if fault == 'restart':
                    guard = ReceiptGuard(path)
                elif fault == 'stop':
                    self.assertFalse(guard.watchdog(1_300_000_000))
                else:
                    stamp = {'delay': 1, 'future': 3_000_000_000, 'epoch': 1_100_000_000}[fault]
                    self.assertFalse(guard.admit('producer', int(fault == 'epoch'), stamp, 1_100_000_000))
                self.assertFalse(guard.admit('producer', 0, 1_400_000_000, 1_400_000_000))
                with self.assertRaises(ValueError):
                    guard.durable.initialize('verified-reference', 'producer')
