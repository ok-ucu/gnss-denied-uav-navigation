import json
from pathlib import Path
import tempfile
import unittest
from analyze_ekf3_fusion import analyze


class BenchAnalysisTests(unittest.TestCase):
    def test_timestamp_mismatch_and_nonfinite_covariance_are_visible(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)
            (p / 'fcu').mkdir()
            (p / 'attempt02').mkdir()
            record = dict(mavpackettype='VISP', TimeUS=1000000, CTimeMS=1000, RTimeUS=1, Ign=0, Rst=0)
            (p / 'fcu/decoded.json').write_text(json.dumps(dict(records=[record], params={})))
            message = dict(mavpackettype='VISION_POSITION_ESTIMATE', usec=2, covariance=[float('nan')] * 21)
            (p / 'attempt02/wire.jsonl').write_text(json.dumps(dict(direction='to_fcu', message=message)) + '\n')
            (p / 'attempt02/decisions.json').write_text('[]')
            result = analyze(p)
            self.assertFalse(result['channels']['VISION_POSITION_ESTIMATE']['timestamps_exact'])
            self.assertFalse(result['channels']['VISION_POSITION_ESTIMATE']['covariance_finite_and_size'])
            self.assertFalse(result['heartbeat']['all_disarmed'])
            self.assertIsNone(result['channels']['VISION_SPEED_ESTIMATE']['rate_hz'])
