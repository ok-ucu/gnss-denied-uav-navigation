"""Stage 2 heartbeat-only consumers and fail-closed launcher admission.

No socket connect, ROS/Gazebo import or workload start implementation. Native
manager/network authority is unavailable; CLI is BLOCKED before any connection.
Synthetic tests supply already-connected loopback sockets/one-way read pipes.
"""
import argparse
import json
from pathlib import Path
import resource
import re
import time
import uuid

from heartbeat_fault_proxy import ReadinessAuthority
from heartbeat_test_evidence import EventJournal, parse_json, seal_witness
from heartbeat_test_owner import Blocked, Ledger, durable_new
from linux_gpu.heartbeat_test import ManagerRecovery, StartupBarrier
from linux_gpu.verified_disarmed import HeartbeatGuard


class HeartbeatConsumer:
    def __init__(self,reader,directory,test_id,system,component,role,clock=time.monotonic_ns,
                 evidence_scope='synthetic',readiness_issuer=None,max_records=100000):
        if role not in ('witness','control') or evidence_scope not in ('synthetic','native'):
            raise ValueError('Explicit heartbeat consumer role/scope required')
        if str(uuid.UUID(test_id))!=test_id:raise ValueError('Canonical run UUID required')
        self.reader=reader;self.directory=Path(directory);self.test_id=test_id
        if hasattr(reader,'setblocking'):reader.setblocking(False)
        self.role=role;self.clock=clock;self.scope=evidence_scope;self.issuer=readiness_issuer
        self.started=clock();self.failure_code=None;self.failure_ns=None;self.ready=False
        self.bytes=0;self.last_poll=None;self.hb_times=[];self.proof=None;self.closed=False
        if role=='witness' and not isinstance(readiness_issuer,ReadinessAuthority):
            raise ValueError('Private verified readiness authority required')
        self.journal=EventJournal(self.directory/('native.jsonl' if role=='witness' else 'control.jsonl'),
                                  test_id,clock,max_records)
        self.guard=HeartbeatGuard(system,component,self.started/1e9,self._record_guard)
        self.journal.emit('started',role=role)

    def _record_guard(self,**row):
        kind=row.pop('kind')
        self.journal.emit(kind,received_ns=self.current_rx_ns,**row)

    def fail(self,code,reason):
        if self.failure_code:return
        self.failure_code=code;self.failure_ns=self.clock()
        if self.role=='witness':
            durable_new(self.directory/'witness.failed.json',dict(schema=2,test_id=self.test_id,
                status='FAIL',code=code,reason=reason,host_monotonic_ns=self.failure_ns))
        self.journal.emit('failure',code=code,reason=reason)

    def _guard_failure(self,exc):
        reason=str(exc)
        if reason=='Expected DISARMED heartbeat stale >3s':
            code='PARTIAL_MAVLINK_STALE' if self.guard.parser.buf_len() else 'HEARTBEAT_STALE'
        elif reason.startswith('No expected DISARMED heartbeat'):code='NO_HEARTBEAT'
        else:code='INVALID_HEARTBEAT'
        self.fail(code,reason)

    def step(self):
        if self.closed or self.failure_code:return
        now=self.clock()
        if self.last_poll is not None and now<self.last_poll:
            self.fail('CLOCK_REVERSED','Host monotonic clock reversed');return
        self.last_poll=now
        # Always latch freshness before reading, but inspect nonblocking socket
        # state before classifying the failure. EOF/reset must never masquerade
        # as expected silence when both become observable in the same iteration.
        freshness_error=None
        try:self.guard.assert_fresh(now/1e9)
        except Exception as exc:freshness_error=exc
        try:data=self.reader.recv(65536)
        except (BlockingIOError,InterruptedError):data=None
        except OSError as exc:self.fail('DOWNSTREAM_DISCONNECT',str(exc));return
        if data==b'':self.fail('DOWNSTREAM_EOF','Connection closed, not heartbeat silence');return
        if data:
            self.current_rx_ns=self.clock()
            self.journal.emit('tcp_chunk',offset=self.bytes,bytes=len(data),hex=data.hex(),received_ns=self.current_rx_ns)
            self.bytes+=len(data)
            if freshness_error:
                self.fail('LATE_DATA_AFTER_STALE',str(freshness_error));return
            before=self.guard.count
            try:self.guard.feed(data,self.current_rx_ns/1e9)
            except Exception as exc:
                self.hb_times.extend([self.current_rx_ns]*(self.guard.count-before))
                self._guard_failure(exc);return
            self.hb_times.extend([self.current_rx_ns]*(self.guard.count-before))
            if self.guard.bad_data:self.fail('MALFORMED_MAVLINK','BAD_DATA is not valid heartbeat');return
        else:self.journal.emit('idle_open')
        if freshness_error:self._guard_failure(freshness_error);return
        try:ready=self.guard.ready(self.clock()/1e9)
        except Exception as exc:self._guard_failure(exc);return
        if ready and not self.ready:
            self.ready=True
            if self.role=='witness':
                self.proof=self.issuer.issue(self.test_id,self.guard,self.hb_times[0],self.hb_times[-1],self.clock())
                durable_new(self.directory/'witness.ready.json',dict(schema=2,test_id=self.test_id,status='READY',
                    heartbeat_count=self.guard.count,first_ns=self.proof.first_ns,last_ns=self.proof.last_ns,
                    host_monotonic_ns=self.proof.ready_ns,native_sequence=self.journal.count))
            self.journal.emit('ready',heartbeats=self.guard.count)

    def finish(self):
        if self.closed:raise ValueError('Consumer already finalized')
        self.closed=True
        try:
            self.journal.emit('closed',failure_code=self.failure_code)
            self.journal.close()
            end=self.clock()
            summary=dict(schema=2,test_id=self.test_id,evidence_scope=self.scope,
                start_ns=self.started,end_ns=end,ready=self.ready,heartbeats=self.guard.count,
                bad_data=self.guard.bad_data,bytes=self.bytes,count=self.journal.count,
                expected_system=self.guard.system,expected_component=self.guard.component,
                first_ns=self.hb_times[0] if self.hb_times else None,
                last_ns=self.hb_times[-1] if self.hb_times else None,
                max_gap_ns=max((b-a for a,b in zip(self.hb_times,self.hb_times[1:])),default=0),
                failure_code=self.failure_code,failure_ns=self.failure_ns,
                exit_code=1 if self.failure_code else 0,
                execution_allowed=False,forwarding_enabled=False)
            durable_new(self.directory/(self.role+'.json'),summary)
            if self.role=='witness':
                usage=resource.getrusage(resource.RUSAGE_SELF)
                durable_new(self.directory/'resources.json',dict(schema=2,test_id=self.test_id,
                    scope='own consumer process only',host_monotonic_ns=end,
                    user_cpu_s=usage.ru_utime,system_cpu_s=usage.ru_stime))
                seal_witness(self.directory,self.test_id)
            return summary
        finally:
            # Export exceptions propagate; completion is never fabricated.
            if not self.journal.closed:self.journal.close()
            self.reader.close()


class WitnessAuthority(ManagerRecovery):
    """Extension to Stage 1's unavailable native host-manager adapter."""
    def verify_endpoint(self,plan,ledger):
        raise Blocked('BLOCKED: exact Person 1 endpoint approval/noninterference not independently verified')
    def verify_network_containment(self,plan,ledger):
        raise Blocked('BLOCKED: receive-only namespace/firewall/IPC containment unverified')
    def start_witness_roles(self,plan,ledger,registration):
        raise Blocked('BLOCKED: native separate-process witness bootstrap not integrated')


def validate_plan(plan,ledger):
    if (plan.get('schema')!=2 or plan.get('mode')!='witness-only' or plan.get('test_id')!=ledger.test_id or
            plan.get('ledger_sha256')!=ledger.digest() or plan.get('execution_allowed') is not False or
            plan.get('forwarding_enabled') is not False):raise Blocked('Missing witness-only identity/safety authorization')
    endpoint=plan.get('endpoint',{})
    required=('transport','address','port','host_network_namespace','fcu_output_identity',
              'current_clients','noninterference_evidence_sha256','person1_approval_reference')
    if (not isinstance(endpoint,dict) or any(k not in endpoint for k in required) or
            endpoint['transport']!='tcp' or not isinstance(endpoint['address'],str) or not endpoint['address'] or
            type(endpoint['port']) is not int or not 1<=endpoint['port']<=65535 or
            not isinstance(endpoint['current_clients'],list) or
            any(not isinstance(endpoint[k],str) or not endpoint[k] for k in (
                'host_network_namespace','fcu_output_identity','person1_approval_reference')) or
            not isinstance(endpoint['noninterference_evidence_sha256'],str) or
            not re.fullmatch('[a-f0-9]{64}',endpoint['noninterference_evidence_sha256'])):
        raise Blocked('Explicit approved test-only endpoint/client inventory/evidence required')
    if not isinstance(plan.get('runtime_approval_reference'),str) or not plan['runtime_approval_reference'].strip():
        raise Blocked('Separate runtime approval absent; offline implementation approval is insufficient')
    for key in ('expected_system','expected_component'):
        if type(plan.get(key)) is not int or not 1<=plan[key]<=255:raise Blocked('Explicit expected source IDs required')
    return plan


def witness_only(plan,ledger,authority=None,clock=time.monotonic_ns):
    """No connection can precede all authority checks. No JSON-only bypass."""
    validate_plan(plan,ledger)
    authority=authority if authority is not None else WitnessAuthority()
    if authority.verify_endpoint(plan,ledger) is not True:raise Blocked('Endpoint authorization BLOCKED')
    if authority.verify_network_containment(plan,ledger) is not True:raise Blocked('Containment BLOCKED')
    barrier=StartupBarrier(ledger,authority,clock)
    registration=barrier.arm();barrier.release()
    return authority.start_witness_roles(plan,ledger,registration)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--ledger',type=Path,required=True)
    args=parser.parse_args()
    try:
        ledger=Ledger.restore(parse_json(args.ledger.read_bytes()))
        witness_only(parse_json(args.plan.read_bytes()),ledger)
    except Exception as exc:
        print(json.dumps(dict(status='BLOCKED',reason=str(exc),decision='HOLD',
            execution_allowed=False,forwarding_enabled=False)))
        return 2
    raise Blocked('No native runtime implementation admitted')


if __name__=='__main__':raise SystemExit(main())
