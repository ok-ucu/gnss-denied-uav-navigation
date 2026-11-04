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
