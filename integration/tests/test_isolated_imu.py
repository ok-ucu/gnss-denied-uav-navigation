"""Offline synthetic IPC/spawn regressions, NOT ROS or live VIO acceptance."""
import copy
import queue
import subprocess
import sys
import time
import unittest
from pathlib import Path

from isolated_imu import IsolatedImuReceiver, put_batch, isolated_imu_args
from vio_sensor_capture import BoundedRecords
from run_vio_stream import clock_status


def ordered_fixture(output, stop, fault, produced, session, topic, profile):
    for sequence in range(10):
        now = time.monotonic()
        packet = dict(session=session, sequence=sequence, sent_monotonic_s=now,
            imu=[dict(t=1+sequence*.001, stamp_ns=10**9+sequence*10**6,
                      arrival_monotonic_s=now, f=[0,0,-9.8], gyro=[0,0,0], q=[0,0,0,1])],
            clocks=[dict(t=1+sequence*.001, arrival_monotonic_s=now)])
        if not put_batch(output,fault,produced,packet):return
    stop.wait(10)
    output.cancel_join_thread()


def init_failure_fixture(output, stop, fault, produced, session, topic, profile):
    fault.set()


def exited_fixture(output, stop, fault, produced, session, topic, profile):
    # Unexpected exit without sending an EOF/sentinel must still block.
    return


class IsolatedImuTests(unittest.TestCase):
    def receiver(self, target=ordered_fixture, profile=False):
        buffers={name:BoundedRecords(100) for name in ('imu','clocks')}
        receiver=IsolatedImuReceiver(buffers,'/offline_fixture_only',profile,target=target)
        self.addCleanup(receiver.close)
        return receiver,buffers

    def packet(self, receiver, sequence=0, t=1.):
        return dict(session=receiver.session,sequence=sequence,sent_monotonic_s=2.,
                    imu=[dict(t=t,stamp_ns=round(t*1e9),arrival_monotonic_s=1.5,
                              f=[0,0,-9.8],gyro=[0,0,0],q=[0,0,0,1])],
                    clocks=[dict(t=t,arrival_monotonic_s=1.6)])

    def wait_for(self, predicate):
        deadline=time.monotonic()+5
        while not predicate():
            if time.monotonic()>deadline:self.fail('bounded spawned-worker test timed out')
            time.sleep(.01)

    def test_actual_spawn_preserves_order_stamps_and_shuts_down(self):
        receiver,buffers=self.receiver(profile=True)
        receiver.start()
        self.wait_for(lambda:receiver.sequence==10)
        receiver.check()
        rows=buffers['imu'].snapshot()
        self.assertEqual([row['stamp_ns'] for row in rows],[10**9+i*10**6 for i in range(10)])
        self.assertEqual([row['ipc_sequence'] for row in rows],list(range(10)))
        self.assertTrue(all(row['arrival_monotonic_s']<=row['ipc_received_monotonic_s'] for row in rows))
        self.assertEqual(receiver.row_counts,dict(imu=10,clocks=10))
        receiver.close()
        self.assertFalse(receiver.process.is_alive())
        self.assertFalse(receiver.thread.is_alive())
        with self.assertRaises(ValueError):receiver.check()

    def test_original_callback_age_is_not_refreshed_by_ipc(self):
        receiver,buffers=self.receiver(profile=True)
        packet=self.packet(receiver)
        receiver.accept(packet,received=10.)
        row=buffers['imu'].latest()
        self.assertEqual(row['arrival_monotonic_s'],1.5)
        self.assertEqual(row['ipc_received_monotonic_s'],10.)
        self.assertEqual(row['stamp_ns'],10**9)
        self.assertEqual(receiver.metrics[0]['ipc_delivery_s'],8.)
        self.assertEqual(receiver.metrics[0]['oldest_callback_age_s'],8.5)

    def test_sequence_session_reset_nonfinite_and_bad_batch_latch(self):
        for change in ('sequence','session','clock_reset','imu_duplicate','nonfinite','oversize','future'):
            with self.subTest(change=change):
                receiver,buffers=self.receiver()
                receiver.accept(self.packet(receiver),received=3.)
                packet=self.packet(receiver,sequence=1,t=1.01)
                if change=='sequence':packet['sequence']=2
                elif change=='session':packet['session']='retired-session'
                elif change=='clock_reset':packet['clocks'][0]['t']=.5
                elif change=='imu_duplicate':packet['imu'][0]['t']=1.
                elif change=='nonfinite':packet['imu'][0]['t']=float('nan')
                elif change=='oversize':packet['imu']*=129
                elif change=='future':packet['clocks'][0]['arrival_monotonic_s']=4.
                with self.assertRaises(ValueError):receiver.accept(packet,received=3.)
                self.assertEqual(len(buffers['imu'].snapshot()),1)
                with self.assertRaises(ValueError):receiver.accept(self.packet(receiver,1,1.02),received=3.)

    def test_clock_duplicates_are_preserved_without_invented_stamps(self):
        receiver,buffers=self.receiver()
        receiver.accept(self.packet(receiver),received=3.)
        packet=self.packet(receiver,1,1.01);packet['clocks'][0]['t']=1.
        receiver.accept(packet,received=3.)
        self.assertEqual([row['t'] for row in buffers['clocks'].snapshot()],[1.,1.])

    def test_queue_full_is_a_fault_never_silent_drop_or_block(self):
        receiver,_=self.receiver()
        output=queue.Queue(maxsize=1)
        self.assertTrue(put_batch(output,receiver.fault,receiver.produced,{'fixture':1}))
        self.assertFalse(put_batch(output,receiver.fault,receiver.produced,{'fixture':2}))
        self.assertTrue(receiver.fault.is_set())
        self.assertEqual(receiver.produced.value,1)
        self.assertEqual(output.get_nowait(),{'fixture':1})
        with self.assertRaises(ValueError):receiver.check()

    def test_worker_death_and_startup_failure_block_snapshot_and_latest(self):
        for target in (init_failure_fixture,exited_fixture):
            with self.subTest(target=target.__name__):
                receiver,buffers=self.receiver(target)
                receiver.start()
                self.wait_for(lambda:not receiver.process.is_alive())
                for buf in buffers.values():buf.check_health=receiver.check
                with self.assertRaises(ValueError):buffers['imu'].snapshot()
                with self.assertRaises(ValueError):buffers['clocks'].latest()
                receiver.close()
                self.assertFalse(receiver.thread.is_alive())

    def test_flag_opt_in_and_all_entry_points_parse_without_ros(self):
        self.assertEqual(isolated_imu_args(False),[])
        self.assertEqual(isolated_imu_args(True),['--isolated-imu'])
        integration=Path(__file__).resolve().parents[1]
        for name in ('launch_phase1.py','run_phase1_validation.py','run_latency_matrix.py','run_vio_stream.py'):
            result=subprocess.run([sys.executable,'-B',str(integration/name),'--help'],capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('--isolated-imu',result.stdout)

    def test_clock_diagnostic_distinguishes_receipt_delay_and_clock_lag(self):
        self.assertTrue(clock_status(1.,dict(t=1.02,arrival_monotonic_s=9.99),10.)['valid'])
        lag=clock_status(1.,dict(t=.999,arrival_monotonic_s=9.99),10.)
        self.assertEqual(lag['reasons'],['simulation clock behind frame'])
        stale=clock_status(1.,dict(t=1.02,arrival_monotonic_s=9.),10.)
        self.assertEqual(stale['reasons'],['callback receipt stale/future/nonfinite'])
        late_ipc=clock_status(1.,dict(t=1.02,arrival_monotonic_s=9.,ipc_received_monotonic_s=10.),10.)
        self.assertFalse(late_ipc['valid'])
        self.assertEqual(late_ipc['ipc_delivery_s'],1.)
        self.assertFalse(clock_status(1.,dict(t=1.181,arrival_monotonic_s=9.99),10.,limit=.18)['valid'])
        self.assertTrue(clock_status(1.,dict(t=1.181,arrival_monotonic_s=9.99),10.,limit=.2)['valid'])

if __name__=='__main__':unittest.main()
