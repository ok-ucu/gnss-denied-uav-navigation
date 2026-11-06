"""Offline validation only; these records are not live ROS evidence."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from probe_camera import evaluate


def sample():
    return dict(duration_s=1, topic_types={'/camera/image_raw':['sensor_msgs/msg/Image'],
                '/clock':['rosgraph_msgs/msg/Clock']}, imu_stamps_s=[],
                clock_stamps_s=[1, 1.5, 2],
                frames=[dict(width=640, height=480, encoding='rgb8', data_valid=True,
                             nonzero=True, stamp_s=1+i/30) for i in range(30)])


class CameraProbeTests(unittest.TestCase):
    def test_valid_records_optional_imu(self):
        self.assertEqual(evaluate(sample())['status'], 'PASS')

    def test_no_streams_cannot_pass(self):
        r=sample(); r['frames']=[]; r['clock_stamps_s']=[]
        self.assertEqual(evaluate(r)['status'], 'FAIL')

    def test_wrong_type_black_frames_clock_and_slow_rate(self):
        for mode in ['type', 'black', 'clock', 'slow', 'stamp', 'dimensions', 'data']:
            r=sample()
            if mode=='type': r['topic_types']={}
            if mode=='black':
                for f in r['frames']: f['nonzero']=False
            if mode=='clock': r['clock_stamps_s']=[1,1]
            if mode=='slow': r['frames']=r['frames'][:10]
            if mode=='stamp': r['frames'][1]['stamp_s']=0
            if mode=='dimensions': r['frames'][0]['width']=320
            if mode=='data': r['frames'][0]['data_valid']=False
            with self.subTest(mode=mode):
                self.assertEqual(evaluate(r)['status'], 'FAIL')


def timed_sample():
    r=sample()
    r['shared_clock_domain']=True
    r['clock_samples']=[dict(arrival_s=i/100,stamp_s=1+i/100) for i in range(101)]
    r['clock_stamps_s']=[v['stamp_s'] for v in r['clock_samples']]
    for i,f in enumerate(r['frames']): f['arrival_s']=i/30+.001
    return r


class ClockConsistencyTests(unittest.TestCase):
    def test_shared_domain_consistent_observations(self):
        r=evaluate(timed_sample())
        self.assertEqual(r['status'], 'PASS')
        self.assertEqual(r['clock_consistency']['status'], 'PASS')

    def test_unknown_domain_and_missing_arrivals_not_tested(self):
        r=timed_sample();r['shared_clock_domain']=False
        self.assertEqual(evaluate(r)['clock_consistency']['status'], 'NOT TESTED')
        r=sample();r['shared_clock_domain']=True
        self.assertEqual(evaluate(r)['clock_consistency']['status'], 'NOT TESTED')

    def test_stale_and_future_images_fail(self):
        for offset in (-.4,1):
            r=timed_sample()
            for f in r['frames']:f['stamp_s']+=offset
            self.assertEqual(evaluate(r)['clock_consistency']['status'], 'FAIL')
            self.assertEqual(r['status'], 'FAIL')

    def test_clock_and_image_resets_fail(self):
        for stream in ('clock','image'):
            r=timed_sample()
            if stream=='clock':
                r['clock_samples'][50]['stamp_s']=0
                r['clock_stamps_s'][50]=0
            else:r['frames'][15]['stamp_s']=0
            self.assertEqual(evaluate(r)['status'], 'FAIL')

    def test_small_image_reset_within_age_tolerance_still_fails(self):
        r=timed_sample()
        r['frames'][15]['stamp_s']=r['frames'][14]['stamp_s']-.001
        self.assertEqual(evaluate(r)['clock_consistency']['status'], 'FAIL')

    def test_clock_arrival_gap_fails(self):
        r=timed_sample()
        r['clock_samples']=[v for v in r['clock_samples'] if not .2<v['arrival_s']<.65]
        self.assertEqual(evaluate(r)['arrival_gaps']['status'], 'FAIL')
        self.assertEqual(r['status'], 'FAIL')

    def test_image_gap_fails_even_above_average_rate_threshold(self):
        r=timed_sample()
        r['frames']=[dict(r['frames'][0],arrival_s=i/60+.001,stamp_s=1+i/60) for i in range(60)]
        r['frames']=[f for f in r['frames'] if not .2<f['arrival_s']<.5]
        self.assertGreater(len(r['frames']),20)
        self.assertEqual(evaluate(r)['arrival_gaps']['status'], 'FAIL')

    def test_unpaired_clock_is_not_tested(self):
        r=timed_sample();r['clock_samples']=r['clock_samples'][2:]
        self.assertEqual(evaluate(r)['clock_consistency']['status'], 'NOT TESTED')

    def test_nonfinite_stamps_fail(self):
        r=timed_sample();r['frames'][3]['stamp_s']=float('nan')
        self.assertEqual(evaluate(r)['status'], 'FAIL')
