"""Offline buffer/recovery contracts, not live sensor evidence."""
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vio_sensor_capture import BoundedRecords,recovery_ready

class BufferRecoveryTests(unittest.TestCase):
    def test_capacity_and_overflow_are_explicit(self):
        b=BoundedRecords(2)
        for x in range(3):b.append(x)
        self.assertEqual(b.snapshot(),[1,2]);self.assertEqual(b.dropped,1)
        with self.assertRaises(ValueError):BoundedRecords(0)
    def test_concurrent_capture(self):
        b=BoundedRecords(1000)
        threads=[threading.Thread(target=lambda:[b.append(x) for x in range(100)]) for _ in range(4)]
        for t in threads:t.start()
        for t in threads:t.join()
        self.assertEqual(len(b.snapshot()),400);self.assertEqual(b.dropped,0)
    def test_wait_and_recover_without_synthetic_samples(self):
        samples=[{'t':1+i*.001} for i in range(301)]
        self.assertFalse(recovery_ready(samples,1.2,1.25))
        self.assertTrue(recovery_ready(samples,1.3,1.25))
    def test_gaps_staleness_and_duplicate_rejected(self):
        for times in ([1,1.3],[1+i*.001 for i in range(200)],
                      [1+i*.001 for i in range(301)]+[1.3]):
            self.assertFalse(recovery_ready([{'t':x} for x in times],1.3,1.25))
    def test_later_continuous_window_can_recover(self):
        times=[1,1.5]+[2+i*.001 for i in range(301)]
        self.assertTrue(recovery_ready([{'t':x} for x in times],2.3,1.75))
    def test_insufficient_continuous_span_rejected(self):
        self.assertFalse(recovery_ready([{'t':1.06+i*.001} for i in range(241)],1.3,1.25))
    def test_nonfinite_threshold_rejected(self):
        with self.assertRaises(ValueError):recovery_ready([],float('nan'),0)


class ReliableCaptureConfigurationTests(unittest.TestCase):
    def test_native_and_ros_buffers_have_independent_counters(self):
        native=BoundedRecords(10);ros=BoundedRecords(10)
        for t in range(5):native.append(t)
        for t in [0,1,4]:ros.append(t)
        self.assertEqual(set(native.snapshot())-set(ros.snapshot()),{2,3})
        self.assertEqual(ros.dropped,0)  # Missing transport data is not a ring overflow.

if __name__=='__main__':unittest.main()
