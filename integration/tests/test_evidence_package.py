"""Offline ZIP fixtures only; no Linux evidence or stage success is fabricated."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from package_linux_evidence import package


class EvidencePackageTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)
        for name,text in (('commands.jsonl','{"command":["example"]}\n'),('heartbeat.log','fixture error\n'),('CHANGES.patch','fixture diff\n')):
            (self.root/name).write_text(text)
        self.names=['commands.jsonl','heartbeat.log','CHANGES.patch']
        self.summary=dict(source_commit='a'*40,dirty=True,changes='CHANGES.patch',commands='commands.jsonl',
            stages=[dict(name='builds',status='PASS',evidence=['commands.jsonl']),
                    dict(name='heartbeat',status='BLOCKED',evidence=['heartbeat.log'],note='Fixture only; no live runtime')])

    def test_reproducible_hashes_and_unverified_claims_without_source_changes(self):
        before={name:(self.root/name).read_bytes() for name in self.names}
        a=self.root/'a.zip';b=self.root/'b.zip'
        package(self.root,self.names,self.summary,a)
        package(self.root,list(reversed(self.names)),self.summary,b)
        self.assertEqual(a.read_bytes(),b.read_bytes())
        with zipfile.ZipFile(a) as archive:
            manifest=json.loads(archive.read('_bundle/manifest.json'))
            summary=json.loads(archive.read('_bundle/summary.json'))
            self.assertFalse(summary['claims_verified']);self.assertEqual(summary['decision'],'HOLD')
            self.assertFalse(summary['forwarding_enabled']);self.assertFalse(summary['execution_allowed'])
            self.assertEqual(summary['stages'][1]['status'],'BLOCKED')
            for row in manifest['files']:
                self.assertEqual(hashlib.sha256(archive.read(row['path'])).hexdigest(),row['sha256'])
        self.assertEqual(before,{name:(self.root/name).read_bytes() for name in self.names})

    def test_unsafe_paths_duplicate_selection_and_symlinks_rejected(self):
        for extra in ('../outside','/etc/passwd','C:\\secret','_bundle/fake.json','.git/config','./heartbeat.log','heartbeat.log'):
            with self.subTest(extra=extra):
                with self.assertRaises(ValueError):package(self.root,self.names+[extra],self.summary,self.root/'bad.zip')
        (self.root/'linked.log').symlink_to(self.root/'heartbeat.log')
        with self.assertRaisesRegex(ValueError,'Symlink'):
            package(self.root,self.names+['linked.log'],self.summary,self.root/'bad.zip')

    def test_no_false_approval_or_uncited_pass(self):
        for change in ('enabled','go','missing_evidence','uncited_pass','missing_patch'):
            with self.subTest(change=change):
                summary=copy.deepcopy(self.summary)
                if change=='enabled':summary['execution_allowed']=True
                elif change=='go':summary['decision']='GO'
                elif change=='missing_evidence':summary['stages'][0]['evidence']=['absent.log']
                elif change=='uncited_pass':summary['stages'][0]['evidence']=[]
                else:summary['changes']='absent.patch'
                with self.assertRaises(ValueError):package(self.root,self.names,summary,self.root/'bad.zip')

    def test_existing_output_and_oversize_fail(self):
        path=self.root/'existing.zip';path.write_bytes(b'preserved')
        with self.assertRaises(FileExistsError):package(self.root,self.names,self.summary,path)
        self.assertEqual(path.read_bytes(),b'preserved')
        with self.assertRaisesRegex(ValueError,'budget'):
            package(self.root,self.names,self.summary,self.root/'big.zip',max_bytes=1)
        self.assertFalse((self.root/'big.zip').exists())


if __name__=='__main__':unittest.main()
