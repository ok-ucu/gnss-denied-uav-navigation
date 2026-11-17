"""Synthetic expected-failure evidence. Never a Linux/native safety PASS."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import heartbeat_test_evidence as evidence
from heartbeat_stage2_fixture import StreamFixture
from heartbeat_test_evidence import finalize_bundle,verify_bundle,verify_witness,seal_witness
import vio_source_probe


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.stream=StreamFixture(self.root)
        self.addCleanup(self.stream.close)

    def prepare(self,cleanup=True):
        self.stream.stale();self.stream.finish(cleanup)

    def finalize(self):return finalize_bundle(self.root,self.stream.ledger,self.stream.clock)

    def test_complete_synthetic_bundle_and_portable_copy(self):
        self.prepare();result=self.finalize()
        self.assertEqual(result['status'],'PASS',result)
        self.assertEqual(result['evidence_scope'],'synthetic_expected_failure_only')
        self.assertFalse(result['execution_allowed']);self.assertFalse(result['forwarding_enabled'])
        self.assertEqual(vio_source_probe.witness_evidence(self.root)['status'],'COMPLETE')
        with tempfile.TemporaryDirectory() as temp:
            copied=Path(temp)/'portable-copy';shutil.copytree(self.root,copied)
            self.assertEqual(verify_bundle(copied,self.stream.ledger)['status'],'PASS')
            self.assertEqual(vio_source_probe.witness_evidence(copied)['status'],'COMPLETE')

    def test_missing_cleanup_receipt_cannot_publish_final_pass(self):
        self.prepare(cleanup=False);result=self.finalize()
        self.assertEqual(result['status'],'INCOMPLETE')
        self.assertFalse((self.root/'bundle.complete.json').exists())
        self.assertFalse((self.root/'bundle.manifest.json').exists())

    def test_partial_cleanup_receipt_cannot_pass(self):
        self.prepare();(self.root/'cleanup.receipt.json').write_text('{"schema":')
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')
        self.assertFalse((self.root/'bundle.complete.json').exists())

    def test_same_size_witness_corruption_is_rejected(self):
        self.prepare();path=self.root/'native.jsonl';raw=path.read_bytes()
        path.write_bytes(raw.replace(b'idle_open',b'idle_opem',1))
        self.assertEqual(path.stat().st_size,len(raw))
        with self.assertRaises(ValueError):verify_witness(self.root,self.stream.test_id)
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')

    def test_partial_jsonl_never_gets_fabricated_completion(self):
        self.prepare();(self.root/'witness.complete.json').unlink()  # own temporary fixture only
        path=self.root/'native.jsonl';path.write_bytes(path.read_bytes()+b'{"unfinished":')
        with self.assertRaises(ValueError):seal_witness(self.root,self.stream.test_id)
        self.assertFalse((self.root/'witness.complete.json').exists())
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')

    def test_invalid_json_and_missing_watchdog_are_incomplete(self):
        self.prepare();(self.root/'resources.json').write_text('{"broken":')
        (self.root/'watchdog.jsonl').unlink()
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')

    def test_inconsistent_counts_rejected_even_before_sealing(self):
        self.prepare();(self.root/'witness.complete.json').unlink()
        path=self.root/'witness.json';row=json.loads(path.read_text());row['count']+=1
        path.write_text(json.dumps(row))
        with self.assertRaises(ValueError):seal_witness(self.root,self.stream.test_id)
        self.assertFalse((self.root/'witness.complete.json').exists())

    def test_proxy_delivery_count_mismatch_is_incomplete(self):
        self.prepare();path=self.root/'proxy.json';row=json.loads(path.read_text());row['delivered']+=1
        path.write_text(json.dumps(row))
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')

    def test_final_manifest_missing_required_file_never_passes(self):
        self.prepare();self.assertEqual(self.finalize()['status'],'PASS')
        path=self.root/'bundle.manifest.json';row=json.loads(path.read_text());del row['files']['control.json']
        path.write_text(json.dumps(row))
        self.assertEqual(verify_bundle(self.root,self.stream.ledger)['status'],'INCOMPLETE')

    def test_missing_final_marker_and_same_size_proxy_corruption(self):
        self.prepare();self.assertEqual(self.finalize()['status'],'PASS')
        (self.root/'bundle.complete.json').unlink()
        self.assertEqual(verify_bundle(self.root,self.stream.ledger)['status'],'INCOMPLETE')
        path=self.root/'proxy.jsonl';raw=path.read_bytes();path.write_bytes(raw.replace(b'trigger',b'triggez',1))
        self.assertEqual(len(raw),path.stat().st_size)
        self.assertEqual(verify_bundle(self.root,self.stream.ledger)['status'],'INCOMPLETE')

    def test_partial_manifest_publication_returns_incomplete(self):
        self.prepare();original=evidence.durable_new
        def fail(path,value):
            if path.name=='bundle.complete.json':raise OSError('synthetic storage failure')
            return original(path,value)
        with patch.object(evidence,'durable_new',side_effect=fail):result=self.finalize()
        self.assertEqual(result['status'],'INCOMPLETE')
        self.assertTrue((self.root/'bundle.manifest.json').exists())
        self.assertFalse((self.root/'bundle.complete.json').exists())

    def test_native_claims_do_not_override_missing_linux_authority(self):
        self.prepare();path=self.root/'admission.json';row=json.loads(path.read_text());row['evidence_scope']='native'
        path.write_text(json.dumps(row))
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')
        self.assertFalse((self.root/'bundle.complete.json').exists())

    def test_upstream_write_violation_remains_fail_with_missing_cleanup(self):
        self.stream.stale()
        with self.assertRaises(RuntimeError):self.stream.rx.send(b'not sent')
        self.stream.finish(cleanup=False)
        result=self.finalize();self.assertEqual(result['status'],'FAIL')
        self.assertTrue(result['incomplete']);self.assertFalse((self.root/'bundle.complete.json').exists())

    def test_expired_global_deadline_cannot_finalize_pass(self):
        self.prepare();self.stream.clock.now=self.stream.ledger.deadline.end_ns+1
        self.assertEqual(self.finalize()['status'],'FAIL')
        self.assertFalse((self.root/'bundle.complete.json').exists())

    def test_no_overwrite_of_completion(self):
        self.prepare();path=self.root/'witness.complete.json';original=path.read_bytes()
        with self.assertRaises(FileExistsError):seal_witness(self.root,self.stream.test_id)
        self.assertEqual(path.read_bytes(),original)

    def test_read_only_verifier_cli_and_missing_manifest_exit_codes(self):
        self.prepare();self.assertEqual(self.finalize()['status'],'PASS')
        command=[sys.executable,'-B',str(Path(evidence.__file__)),'--directory',str(self.root),
                 '--ledger',str(self.root/'ownership.json')]
        answer=subprocess.run(command,capture_output=True,text=True,timeout=5)
        self.assertEqual(answer.returncode,0,answer.stderr)
        self.assertEqual(json.loads(answer.stdout)['status'],'PASS')
        (self.root/'bundle.complete.json').unlink()
        answer=subprocess.run(command,capture_output=True,text=True,timeout=5)
        self.assertEqual(answer.returncode,2,answer.stderr)
        self.assertEqual(json.loads(answer.stdout)['status'],'INCOMPLETE')

    def test_inconsistent_failure_exit_status_cannot_get_completion(self):
        self.prepare();(self.root/'witness.complete.json').unlink()
        path=self.root/'witness.json';row=json.loads(path.read_text());row['exit_code']=0
        path.write_text(json.dumps(row))
        with self.assertRaises(ValueError):seal_witness(self.root,self.stream.test_id)
        self.assertFalse((self.root/'witness.complete.json').exists())

    def test_supervision_receipt_binding_and_original_deadline_required(self):
        self.prepare();path=self.root/'watchdog.jsonl'
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        rows[-1]['cleanup_receipt_sha256']='0'*64
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')

    def test_evidence_outside_original_deadline_cannot_pass(self):
        self.prepare();path=self.root/'watchdog.jsonl'
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        rows[-1]['host_monotonic_ns']=self.stream.ledger.deadline.end_ns+1
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')

    def test_post_publication_deadline_failure_revokes_provisional_marker(self):
        self.prepare();original=evidence.durable_new
        def delayed(path,value):
            original(path,value)
            if path.name=='bundle.complete.json':self.stream.clock.now=self.stream.ledger.deadline.end_ns+1
        with patch.object(evidence,'durable_new',side_effect=delayed):result=self.finalize()
        self.assertEqual(result['status'],'FAIL')
        self.assertTrue((self.root/'bundle.finalization.failed.json').exists())
        self.assertEqual(verify_bundle(self.root,self.stream.ledger)['status'],'FAIL')

    def test_duplicate_finalization_refuses_without_altering_existing_bundle(self):
        self.prepare();self.assertEqual(self.finalize()['status'],'PASS')
        before={p.name:p.read_bytes() for p in self.root.iterdir()}
        self.assertEqual(self.finalize()['status'],'INCOMPLETE')
        self.assertEqual(before,{p.name:p.read_bytes() for p in self.root.iterdir()})
        self.assertEqual(verify_bundle(self.root,self.stream.ledger)['status'],'PASS')


if __name__=='__main__':unittest.main()
