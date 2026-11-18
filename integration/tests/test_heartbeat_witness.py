"""Blocked admission / shell dispatch only. No endpoint connection permitted."""
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from heartbeat_test_owner import Blocked
from heartbeat_witness import WitnessAuthority,witness_only
from test_heartbeat_test import FakeManager
from test_heartbeat_test_owner import fixture

ROOT=Path(__file__).resolve().parents[1]


def plan_for(ledger):
    return dict(schema=2,mode='witness-only',test_id=ledger.test_id,ledger_sha256=ledger.digest(),
        execution_allowed=False,forwarding_enabled=False,runtime_approval_reference='SYNTHETIC fixture only',
        expected_system=1,expected_component=1,endpoint=dict(transport='tcp',address='127.0.0.1',port=9,
            host_network_namespace='SYNTHETIC',fcu_output_identity='NO FCU; synthetic endpoint',current_clients=[],
            noninterference_evidence_sha256='a'*64,person1_approval_reference='SYNTHETIC fixture assertion'))


class SyntheticAuthority(FakeManager,WitnessAuthority):
    def __init__(self):super().__init__();self.endpoint=True;self.containment=True
    def verify_endpoint(self,plan,ledger):self.calls.append('endpoint');return self.endpoint
    def verify_network_containment(self,plan,ledger):self.calls.append('containment');return self.containment
    def start_witness_roles(self,plan,ledger,registration):
        self.calls.append('witness-only-dispatch');return {'dispatched':'synthetic-no-process'}


class WitnessAdmissionTests(unittest.TestCase):
    def test_default_native_authority_stays_closed_even_with_self_reported_approval(self):
        clock,ledger=fixture()
        with patch('socket.create_connection') as connect:
            with self.assertRaisesRegex(Blocked,'not independently verified'):
                witness_only(plan_for(ledger),ledger,clock=clock)
            connect.assert_not_called()

    def test_missing_authorization_endpoint_ids_or_safety_mismatch_never_dispatches(self):
        clock,ledger=fixture();base=plan_for(ledger)
        for field,value in [('endpoint',{}),('runtime_approval_reference',None),('expected_system',0),
                            ('mode','ON'),('execution_allowed',True),('forwarding_enabled',True),
                            ('test_id','wrong'),('ledger_sha256','wrong')]:
            authority=SyntheticAuthority();plan=copy.deepcopy(base);plan[field]=value
            with self.subTest(field=field),self.assertRaises(Blocked):witness_only(plan,ledger,authority,clock)
            self.assertNotIn('witness-only-dispatch',authority.calls)

    def test_endpoint_containment_registration_and_release_checked_before_dispatch(self):
        clock,ledger=fixture();plan=plan_for(ledger)
        for fail in ('endpoint','containment','valid'):
            authority=SyntheticAuthority();setattr(authority,fail,False)
            with self.subTest(fail=fail),self.assertRaises(Blocked):witness_only(plan,ledger,authority,clock)
            self.assertNotIn('witness-only-dispatch',authority.calls)
        authority=SyntheticAuthority()
        self.assertEqual(witness_only(plan,ledger,authority,clock),{'dispatched':'synthetic-no-process'})
        self.assertEqual(authority.calls,['endpoint','containment','register','verify','verify','witness-only-dispatch'])

    def test_shell_dispatch_precedes_prerequisites_and_preserves_legacy_body(self):
        text=(ROOT/'run_vio_source_probe.sh').read_text()
        original=subprocess.check_output(['git','show','HEAD:integration/run_vio_source_probe.sh'],cwd=ROOT,text=True)
        legacy=text[text.index('[[ "$(uname -s)"'):]
        self.assertEqual(legacy,original[original.index('[[ "$(uname -s)"'):])
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);record=folder/'python.args'
            python=folder/'python3';python.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$TEST_ARGS"\nexit 2\n');python.chmod(0o700)
            uname=folder/'uname';uname.write_text('#!/bin/sh\necho FORBIDDEN >&2\nexit 99\n');uname.chmod(0o700)
            env=dict(os.environ,PATH=str(folder)+os.pathsep+os.environ['PATH'],TEST_ARGS=str(record))
            result=subprocess.run(['bash',str(ROOT/'run_vio_source_probe.sh'),'--witness-only',
                '--plan','synthetic-plan.json','--ledger','synthetic-ledger.json'],env=env,
                capture_output=True,text=True,timeout=3)
            self.assertEqual(result.returncode,2);self.assertNotIn('FORBIDDEN',result.stderr)
            self.assertEqual(record.read_text().splitlines(),['-B',str(ROOT/'heartbeat_witness.py'),
                '--plan','synthetic-plan.json','--ledger','synthetic-ledger.json'])
            self.assertEqual(set(p.name for p in folder.iterdir()),{'python3','uname','python.args'})

    def test_actual_witness_only_cli_blocks_without_ros_vio_or_network(self):
        _,ledger=fixture()
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);ledger.publish(folder)
            (folder/'plan.json').write_text(json.dumps(plan_for(ledger)))
            before=set(folder.iterdir())
            result=subprocess.run(['bash',str(ROOT/'run_vio_source_probe.sh'),'--witness-only',
                '--plan',str(folder/'plan.json'),'--ledger',str(folder/'ownership.json')],
                capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,2,result.stderr)
            row=json.loads(result.stdout);self.assertEqual(row['status'],'BLOCKED')
            self.assertFalse(row['execution_allowed']);self.assertFalse(row['forwarding_enabled'])
            self.assertEqual(set(folder.iterdir()),before)


if __name__=='__main__':unittest.main()
