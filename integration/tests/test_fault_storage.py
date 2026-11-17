"""Offline fault protocol regressions, not live failure-injection evidence."""
import io
import json
from pathlib import Path
import tempfile
import unittest
import uuid
from contextlib import redirect_stdout
from unittest.mock import patch

import stage_timing
from stage_timing import FaultFile
from run_latency_matrix import fault_write_code


class FaultStorageTests(unittest.TestCase):
    def write(self,path,fault):
        row=dict(fault=fault,generation=str(uuid.uuid4()),host_utc_ns=123)
        with redirect_stdout(io.StringIO()):
            exec(fault_write_code(str(path),row),{})
        self.assertEqual(json.loads(path.read_text()),row)
        self.assertFalse(list(path.parent.glob('*.writing')))

    def test_shared_readers_activation_revocation_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fault.json'
            self.write(path,None)
            first,second=FaultFile(path),FaultFile(path)
            with patch.object(stage_timing.time,'monotonic',return_value=1):
                self.assertIsNone(first.read()['fault']);self.assertIsNone(second.read()['fault'])
            self.write(path,'camera_loss')
            # Preserve the existing 20ms cache window, not a new async cache.
            with patch.object(stage_timing.time,'monotonic',return_value=1.01):
                self.assertIsNone(first.read()['fault'])
            with patch.object(stage_timing.time,'monotonic',return_value=1.021):
                self.assertEqual(first.read()['fault'],'camera_loss')
                self.assertEqual(second.read()['fault'],'camera_loss')
                self.assertEqual(FaultFile(path).read()['fault'],'camera_loss')
            self.write(path,None)  # New writer invocation, same persistent file.
            with patch.object(stage_timing.time,'monotonic',return_value=1.05):
                self.assertIsNone(first.read()['fault']);self.assertIsNone(second.read()['fault'])

    def test_missing_or_malformed_file_never_falls_back_after_read_due(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fault.json';self.write(path,None);reader=FaultFile(path)
            with patch.object(stage_timing.time,'monotonic',return_value=1):reader.read()
            path.unlink()
            with patch.object(stage_timing.time,'monotonic',return_value=1.03):
                with self.assertRaises(FileNotFoundError):reader.read()
            path.write_text('{broken')
            with patch.object(stage_timing.time,'monotonic',return_value=1.06):
                with self.assertRaises(json.JSONDecodeError):reader.read()
            self.write(path,'not-a-fault')
            with patch.object(stage_timing.time,'monotonic',return_value=1.09):
                with self.assertRaisesRegex(ValueError,'Invalid fault'):reader.read()

    def test_existing_twenty_ms_read_limit_and_disabled_source(self):
        with tempfile.TemporaryDirectory() as directory:
            reader=FaultFile(Path(directory)/'control.json')
            with patch.object(stage_timing.time,'monotonic',return_value=1),patch.object(Path,'read_text',return_value='{"fault":null}') as reads:
                for _ in range(100):reader.read()
                self.assertEqual(reads.call_count,1)
            with patch.object(stage_timing.time,'monotonic',return_value=1.021),patch.object(Path,'read_text',return_value='{"fault":"imu_loss"}') as reads:
                self.assertEqual(reader.read()['fault'],'imu_loss');self.assertEqual(reads.call_count,1)
        self.assertEqual(FaultFile(None).read(),{})


if __name__=='__main__':unittest.main()
