"""Stage 2 portable evidence integrity; never authorizes a runtime or FCU route."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import time

from heartbeat_test_owner import Ledger, canonical, durable_new, verify_receipt


class EvidenceError(ValueError):pass


def parse_json(raw):
    def pairs(items):
        answer={}
        for key,value in items:
            if key in answer:raise EvidenceError('Duplicate JSON key: '+key)
            answer[key]=value
        return answer
    def nonfinite(value):raise EvidenceError('Nonfinite JSON value: '+value)
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=nonfinite)


def local_file(root,name):
    relative=Path(name)
    if (relative.is_absolute() or not relative.parts or '..' in relative.parts or
            str(relative)!=name):raise EvidenceError('Require canonical relative evidence path')
    path=Path(root)
    for part in relative.parts:
        path=path/part
        if path.is_symlink():raise EvidenceError('Evidence symlink forbidden: '+name)
    if not stat.S_ISREG(path.stat().st_mode):raise EvidenceError('Nonregular evidence: '+name)
    return path


def inspect_file(root,name,test_id):
    path=local_file(root,name);before=path.stat();raw=path.read_bytes();after=path.stat()
    if (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(
            after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns):
        raise EvidenceError('Evidence changed while reading: '+name)
    if not raw:raise EvidenceError('Empty evidence: '+name)
    description=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
    if path.suffix=='.jsonl':
        if not raw.endswith(b'\n'):raise EvidenceError('Unterminated JSONL: '+name)
        rows=[parse_json(line) for line in raw.splitlines()]
        stamps=[]
        for index,row in enumerate(rows):
            if (not isinstance(row,dict) or row.get('test_id')!=test_id or
                    type(row.get('sequence')) is not int or row['sequence']!=index or
                    type(row.get('host_monotonic_ns')) is not int):
                raise EvidenceError('JSONL identity/sequence invalid: '+name)
            stamps.append(row['host_monotonic_ns'])
        if any(b<a for a,b in zip(stamps,stamps[1:])):raise EvidenceError('JSONL clock reversed: '+name)
        description['records']=len(rows);return description,rows
    row=parse_json(raw)
    if not isinstance(row,dict) or row.get('test_id')!=test_id:
        raise EvidenceError('JSON test identity mismatch: '+name)
    return description,row


class EventJournal:
    def __init__(self,path,test_id,clock=time.monotonic_ns,max_records=100000):
        if type(max_records) is not int or max_records<=0:raise ValueError('Explicit positive event bound')
        self.path=Path(path);self.test_id=test_id;self.clock=clock
        self.limit=max_records;self.count=0;self.last=None;self.closed=False
        self.file=self.path.open('xb')
    def emit(self,kind,**data):
        if self.closed or self.count>=self.limit:raise EvidenceError('Journal closed/capacity exceeded')
        if {'test_id','sequence','host_monotonic_ns','kind'} & set(data):
            raise EvidenceError('Reserved journal identity/time fields')
        now=self.clock()
        if type(now) is not int or self.last is not None and now<self.last:
            raise EvidenceError('Journal monotonic clock reversed')
        row=dict(test_id=self.test_id,sequence=self.count,host_monotonic_ns=now,kind=kind,**data)
        self.file.write(canonical(row));self.file.flush();os.fsync(self.file.fileno())
        self.count+=1;self.last=now
    def close(self):
        if not self.closed:
            self.file.flush();os.fsync(self.file.fileno());self.file.close();self.closed=True


def _witness_payload(root,test_id,files):
    meta=files['witness.json'];native=files['native.jsonl']
    if (meta.get('schema')!=2 or meta.get('count')!=len(native) or
            meta.get('evidence_scope') not in ('synthetic','native') or
            type(meta.get('start_ns')) is not int or type(meta.get('end_ns')) is not int or
            meta['end_ns']<meta['start_ns'] or meta.get('execution_allowed') is not False or
            meta.get('forwarding_enabled') is not False):raise EvidenceError('Witness counts/schema invalid')
    if (type(meta.get('ready')) is not bool or any(type(meta.get(k)) is not int or meta[k]<0
            for k in ('count','bytes','heartbeats','bad_data'))):raise EvidenceError('Invalid witness numeric types')
    if any(not meta['start_ns']<=row['host_monotonic_ns']<=meta['end_ns'] for row in native):
        raise EvidenceError('Witness event outside capture bounds')
    chunks=[r for r in native if r['kind']=='tcp_chunk'];offset=0
    for row in chunks:
        raw=bytes.fromhex(row['hex'])
        if (row['offset']!=offset or row['bytes']!=len(raw) or type(row.get('received_ns')) is not int or
                not meta['start_ns']<=row['received_ns']<=row['host_monotonic_ns']):
            raise EvidenceError('TCP offsets/counts/receive time invalid')
        offset+=len(raw)
    if offset!=meta['bytes']:raise EvidenceError('Witness raw byte count mismatch')
    if meta['ready']:
        ready=files['witness.ready.json']
        if (ready.get('status')!='READY' or type(ready.get('heartbeat_count')) is not int or ready['heartbeat_count']<3 or
                type(ready.get('native_sequence')) is not int or not 0<ready['native_sequence']<len(native) or
                native[ready['native_sequence']]['kind']!='ready' or
                ready['last_ns']-ready['first_ns']<2_000_000_000 or
                not ready['last_ns']<=ready['host_monotonic_ns']<=meta['end_ns']):
            raise EvidenceError('Invalid witness readiness marker')
    if meta['failure_code'] is not None:
        failed=files['witness.failed.json']
        events=[r for r in native if r['kind']=='failure']
        if (failed.get('status')!='FAIL' or failed.get('code')!=meta['failure_code'] or
                failed.get('host_monotonic_ns')!=meta['failure_ns'] or meta.get('exit_code')!=1 or
                len(events)!=1 or events[0]['code']!=meta['failure_code'] or
                events[0]['host_monotonic_ns']<meta['failure_ns']):
            raise EvidenceError('Inconsistent witness failure marker')
    if meta['failure_code']=='HEARTBEAT_STALE':
        # Actual pinned codec: re-validate the captured fragments, not counters.
        from linux_gpu.verified_disarmed import HeartbeatGuard
        guard=HeartbeatGuard(meta['expected_system'],meta['expected_component'],meta['start_ns']/1e9)
        for row in chunks:guard.feed(bytes.fromhex(row['hex']),row['received_ns']/1e9)
        if (guard.count!=meta['heartbeats'] or guard.bad_data or
                meta['bad_data']!=guard.bad_data or guard.parser.buf_len() or meta['failure_ns']/1e9-guard.last<=3):
            raise EvidenceError('Stale failure not supported by protocol/time evidence')
        accepted=[r['received_ns'] for r in native if r['kind']=='heartbeat']
        if (len(accepted)!=guard.count or meta['first_ns']!=accepted[0] or meta['last_ns']!=accepted[-1] or
                meta['max_gap_ns']!=max((b-a for a,b in zip(accepted,accepted[1:])),default=0)):
            raise EvidenceError('Witness heartbeat timing counts inconsistent')
        if meta['ready']:
            ready=files['witness.ready.json']
            before=HeartbeatGuard(meta['expected_system'],meta['expected_component'],meta['start_ns']/1e9)
            prefix=native[:ready['native_sequence']]
            for row in prefix:
                if row['kind']=='tcp_chunk':before.feed(bytes.fromhex(row['hex']),row['received_ns']/1e9)
            prefix_hb=[r['received_ns'] for r in prefix if r['kind']=='heartbeat']
            if (not before.ready(ready['host_monotonic_ns']/1e9) or before.count!=ready['heartbeat_count'] or
                    ready['first_ns']!=prefix_hb[0] or ready['last_ns']!=prefix_hb[-1]):
                raise EvidenceError('Readiness not verified on wire')
    return meta


def seal_witness(root,test_id):
    """Call ONLY after native writer is closed; marker last, never overwrite."""
    _,meta=inspect_file(root,'witness.json',test_id)
    names=['native.jsonl','resources.json','witness.json']
    if meta['ready']:names.append('witness.ready.json')
    if meta['failure_code'] is not None:names.append('witness.failed.json')
    descriptions={};payload={}
    for name in names:descriptions[name],payload[name]=inspect_file(root,name,test_id)
    _witness_payload(root,test_id,payload)
    marker=dict(schema=2,test_id=test_id,files=descriptions,scope='closed witness files only')
    durable_new(Path(root)/'witness.complete.json',marker)
    return verify_witness(root,test_id)


def verify_witness(root,test_id):
    _,marker=inspect_file(root,'witness.complete.json',test_id)
    if marker.get('schema')!=2 or not isinstance(marker.get('files'),dict):raise EvidenceError('Incomplete witness manifest')
    payload={}
    for name,expected in marker['files'].items():
        actual,payload[name]=inspect_file(root,name,test_id)
        if actual!=expected:raise EvidenceError('Witness hash/count mismatch: '+name)
    if not {'native.jsonl','resources.json','witness.json'}<=set(payload):raise EvidenceError('Missing witness file')
    meta=payload['witness.json'];required={'native.jsonl','resources.json','witness.json'}
    if meta.get('ready'):required.add('witness.ready.json')
    if meta.get('failure_code') is not None:required.add('witness.failed.json')
    if set(payload)!=required:raise EvidenceError('Witness manifest set mismatch')
    return _witness_payload(root,test_id,payload)


BUNDLE_FIXED={'ownership.json','admission.json','witness.ready.json','witness.failed.json',
    'native.jsonl','resources.json','witness.json','witness.complete.json',
    'proxy.json','proxy.jsonl','collector.jsonl','control.json','control.jsonl',
    'watchdog.jsonl','cleanup.receipt.json'}


def _bundle_inputs(root,ledger):
    """Collect independent errors; a verified violation is not hidden by missing proof."""
    failures=[];missing=[];payload={};descriptions={}
    try:
        meta=verify_witness(root,ledger.test_id)
        if meta['failure_code']!='HEARTBEAT_STALE':failures.append('Wrong witness failure cause')
        if not meta['ready']:failures.append('Witness failed before readiness')
    except Exception as exc:missing.append('witness: '+str(exc))
    try:
        receipt=verify_receipt(Path(root)/'cleanup.receipt.json',ledger)
        extra=receipt['events']['path']
        if receipt['status']=='FAIL':failures.append('Cleanup safety failure')
        elif receipt['status']!='PASS':missing.append('Cleanup not verified PASS')
    except Exception as exc:extra=None;missing.append('cleanup: '+str(exc))
    names=BUNDLE_FIXED | ({extra} if extra else set())
    for name in sorted(names):
        try:descriptions[name],payload[name]=inspect_file(root,name,ledger.test_id)
        except Exception as exc:missing.append(name+': '+str(exc))
    try:
        if canonical(payload['ownership.json'])!=canonical(ledger.record()):raise EvidenceError('Ledger snapshot mismatch')
    except Exception as exc:missing.append(str(exc))
    proxy=payload.get('proxy.json',{});control=payload.get('control.json',{});admission=payload.get('admission.json',{})
    try:
        for name,rows in payload.items():
            if name.endswith('.jsonl') and any(not ledger.deadline.start_ns<=r['host_monotonic_ns']<ledger.deadline.end_ns for r in rows):
                raise EvidenceError('Evidence outside original host deadline: '+name)
        watchdog=payload['watchdog.jsonl'];witness=payload['witness.json']
        if (watchdog[0]['kind']!='synthetic_manager_armed' or
                watchdog[0].get('deadline_ns')!=ledger.deadline.end_ns or
                watchdog[0]['host_monotonic_ns']>witness['start_ns'] or
                watchdog[-1]['kind']!='synthetic_cleanup_finished' or watchdog[-1].get('status')!='PASS' or
                watchdog[-1].get('cleanup_receipt_sha256')!=descriptions['cleanup.receipt.json']['sha256'] or
                watchdog[-1]['host_monotonic_ns']<payload['cleanup.receipt.json']['end_ns']):
            raise EvidenceError('Synthetic supervision/cleanup lifecycle not proven')
    except Exception as exc:missing.append('deadline/supervision: '+str(exc))
    try:
        required_proxy={'received','delivered','discarded','upstream_bytes','upstream_application_writes',
                        'upstream_write_attempts','collector_chunks','cutoff','failures','state'}
        if not required_proxy<=set(proxy):raise EvidenceError('Proxy summary fields missing')
        if (any(type(proxy[k]) is not int or proxy[k]<0 for k in required_proxy-
                {'cutoff','failures','state'}) or not isinstance(proxy['failures'],list)):
            raise EvidenceError('Proxy summary numeric types invalid')
        receive=[r for r in payload['collector.jsonl'] if r['kind']=='upstream_receive']
        proxy_receive=[r for r in payload['proxy.jsonl'] if r['kind']=='proxy_receive']
        sends=[r for r in payload['proxy.jsonl'] if r['kind']=='witness_send']
        for records in (receive,proxy_receive):
            offset=0
            for index,row in enumerate(records):
                if row['offset']!=offset or row['chunk_sequence']!=index:raise EvidenceError('Proxy offsets/order mismatch')
                offset+=row['bytes']
        if (sum(r['bytes'] for r in receive)!=proxy['upstream_bytes'] or len(receive)!=proxy['collector_chunks'] or
                sum(r['bytes'] for r in proxy_receive)!=proxy['received'] or
                sum(r['bytes'] for r in sends)!=proxy['delivered'] or
                proxy['received']!=proxy['delivered']+proxy['discarded'] or
                payload['witness.json']['bytes']!=proxy['delivered']):
            raise EvidenceError('Proxy/collector/witness byte counts inconsistent')
        from linux_gpu.verified_disarmed import HeartbeatGuard
        rows=payload['control.jsonl'];offset=0;times=[];current_rx=[None];control_raw=bytearray();last_rx=None
        if any(type(control.get(k)) is not int or control[k]<0 for k in ('count','bytes','heartbeats','bad_data','max_gap_ns')):
            raise EvidenceError('Control summary numeric types invalid')
        guard=HeartbeatGuard(control['expected_system'],control['expected_component'],control['start_ns']/1e9,
            lambda **r:times.append(current_rx[0]) if r['kind']=='heartbeat' else None)
        for row in rows:
            if row['kind']!='tcp_chunk':continue
            raw=bytes.fromhex(row['hex'])
            if (row['offset']!=offset or row['bytes']!=len(raw) or type(row.get('received_ns')) is not int or
                    not control['start_ns']<=row['received_ns']<=row['host_monotonic_ns']<=control['end_ns'] or
                    last_rx is not None and row['received_ns']<last_rx):
                raise EvidenceError('Control stream count/time mismatch')
            last_rx=row['received_ns']
            offset+=len(raw);control_raw.extend(raw);current_rx[0]=row['received_ns']
            guard.feed(raw,row['received_ns']/1e9)
        # Compare protocol/arrival counts. Original integer receive times remain
        # in JSONL; guard's inherited seconds policy is not a new clock mapping.
        if (control['count']!=len(rows) or control['bytes']!=offset or control['heartbeats']!=guard.count or
                control.get('ready') is not True or control['bad_data']!=guard.bad_data or guard.bad_data or
                guard.parser.buf_len() or not guard.ready(control['end_ns']/1e9)):
            raise EvidenceError('Independent control protocol/count/readiness invalid')
        heartbeat_events=[r for r in rows if r['kind']=='heartbeat']
        if (len(heartbeat_events)!=guard.count or control['max_gap_ns']!=max(
                (b-a for a,b in zip(times,times[1:])),default=0) or
                control['last_ns']!=times[-1] or control['first_ns']!=times[0]):
            raise EvidenceError('Independent control timing counts inconsistent')
        witness_raw=b''.join(bytes.fromhex(r['hex']) for r in payload['native.jsonl'] if r['kind']=='tcp_chunk')
        if offset!=proxy['upstream_bytes'] or not bytes(control_raw).startswith(witness_raw):
            raise EvidenceError('Control/witness unchanged byte-prefix delivery not proven')
    except Exception as exc:missing.append('stream consistency: '+str(exc))
    if proxy.get('upstream_write_attempts',0) or proxy.get('upstream_application_writes',0):failures.append('Upstream application write/attempt')
    if proxy.get('failures'):failures.append('Proxy/collector failure')
    if control.get('failure_code') is not None:failures.append('Independent control failure')
    if admission.get('forbidden_actions'):failures.append('Forbidden workload action')
    # Native authority is deliberately NOT inferred from a self-reported JSON flag.
    # Until the manager/network backend exists, only synthetic-bundle PASS exists.
    if admission.get('evidence_scope')!='synthetic':missing.append('Native authority/backend not integrated')
    if (payload.get('witness.json',{}).get('evidence_scope')!=admission.get('evidence_scope') or
            control.get('evidence_scope')!=admission.get('evidence_scope')):
        missing.append('Inconsistent witness/control/admission provenance')
    if (admission.get('ledger_sha256')!=ledger.digest() or
            admission.get('endpoint_noninterference_verified') is not True or
            admission.get('manager_recovery_verified') is not True or
            admission.get('network_isolation_verified') is not True or
            admission.get('forbidden_actions')!=[]):missing.append('Admission proof incomplete')
    cutoff=proxy.get('cutoff')
    cutoff_keys=('host_monotonic_ns','received_offset','delivered_offset','last_send_ns',
                 'discarded_pending','readiness_last_heartbeat_ns')
    if (proxy.get('state')!='CLOSED' or not isinstance(cutoff,dict) or
            any(type(cutoff.get(k)) is not int or cutoff[k]<0 for k in cutoff_keys)):
        missing.append('Proxy lifecycle/cutoff missing or malformed')
    else:
        trigger=cutoff.get('host_monotonic_ns');last=control.get('last_ns')
        witness=payload.get('witness.json',{})
        if (type(trigger) is not int or type(last) is not int or
                type(witness.get('failure_ns')) is not int or last<witness['failure_ns'] or
                control.get('heartbeats',0)<3 or control.get('max_gap_ns',4_000_000_000)>3_000_000_000):
            missing.append('Independent control continuity through silence not proven')
        triggers=[row for row in payload.get('proxy.jsonl',[]) if row['kind']=='trigger']
        if (len(triggers)!=1 or triggers[0].get('cutoff')!=cutoff or
                triggers[0]['host_monotonic_ns']<trigger):
            missing.append('Trigger journal mismatch')
        closed=[row for row in payload.get('proxy.jsonl',[]) if row['kind']=='closed']
        if (len(closed)!=1 or type(witness.get('failure_ns')) is not int or
                closed[0]['host_monotonic_ns']<witness['failure_ns']):
            missing.append('Proxy closed before expected failure or terminal state unproven')
        if (type(proxy.get('delivered')) is not int or type(proxy.get('received')) is not int or
                cutoff['delivered_offset']!=proxy['delivered'] or cutoff['received_offset']>proxy['received'] or
                cutoff['last_send_ns']>trigger or cutoff['readiness_last_heartbeat_ns']>trigger):
            missing.append('Cutoff byte/time boundary inconsistent')
        if triggers:
            # Journal order, not timestamp inequality: equal host stamps are valid.
            after=payload['proxy.jsonl'][triggers[0]['sequence']+1:]
            if any(r['kind']=='witness_send' for r in after):failures.append('Witness send after silence')
        if not any(row['kind']=='idle_open' and row['host_monotonic_ns']>=trigger
                   for row in payload.get('native.jsonl',[])):
            missing.append('Open witness connection during silence not evidenced')
    status='FAIL' if failures else ('INCOMPLETE' if missing else 'PASS')
    return dict(status=status,failures=failures,incomplete=missing,files=descriptions,
                evidence_scope='synthetic_expected_failure_only' if admission.get('evidence_scope')=='synthetic' else 'native_unverified',decision='HOLD',
                execution_allowed=False,forwarding_enabled=False)


def finalize_bundle(root,ledger,clock=time.monotonic_ns,publisher=None):
    publish=durable_new if publisher is None else publisher
    if (Path(root)/'bundle.complete.json').exists():
        result=_bundle_inputs(root,ledger)
        result['incomplete'].append('Existing final marker; repeated publication refused')
        result['status']='FAIL' if result['failures'] else 'INCOMPLETE'
        return result  # Preserve a previous immutable bundle; do not revoke it.
    try:
        result=_finalize_bundle(root,ledger,clock,publish)
    except Exception as exc:
        result=_bundle_inputs(root,ledger)
        result['incomplete'].append('finalization: '+str(exc))
        result['status']='FAIL' if result['failures'] or clock()>=ledger.deadline.end_ns else 'INCOMPLETE'
    if result['status']!='PASS' and (Path(root)/'bundle.complete.json').exists():
        # A known post-publication error must revoke a provisional on-disk PASS.
        # This is best-effort failure evidence, NOT an extension of D. If storage
        # is unavailable, caller/manager must retain the failed return externally;
        # a portable file copy alone cannot prove historical fsync success.
        failure=dict(schema=2,test_id=ledger.test_id,status=result['status'],
            reason='Final publication not successfully acknowledged',host_monotonic_ns=clock(),
            execution_allowed=False,forwarding_enabled=False)
        try:publish(Path(root)/'bundle.finalization.failed.json',failure)
        except Exception as exc:result['incomplete'].append('revocation persistence unavailable: '+str(exc))
    return result


def _finalize_bundle(root,ledger,clock,publish):
    """Host-side operation after ALL writers close and cleanup, under original D."""
    root=Path(root);result=_bundle_inputs(root,ledger)
    if result['incomplete']:return result  # no fabricated manifest/completion
    if clock()>=ledger.deadline.end_ns:return dict(result,status='FAIL',failures=result['failures']+['Deadline exceeded'])
    manifest=dict(schema=2,test_id=ledger.test_id,ledger_sha256=ledger.digest(),files=result['files'],
                  evidence_scope=result['evidence_scope'])
    publish(root/'bundle.manifest.json',manifest)
    # Re-read every finalized file, including hashes/counts and cleanup receipt.
    checked=_bundle_inputs(root,ledger)
    if checked['files']!=manifest['files'] or checked['incomplete']:return dict(checked,status='INCOMPLETE')
    if clock()>=ledger.deadline.end_ns:return dict(checked,status='FAIL',failures=['Deadline exceeded during export'])
    raw=(root/'bundle.manifest.json').read_bytes()
    complete=dict(schema=2,test_id=ledger.test_id,manifest_sha256=hashlib.sha256(raw).hexdigest(),
        cleanup_sha256=manifest['files']['cleanup.receipt.json']['sha256'],status=checked['status'],
        evidence_scope=checked['evidence_scope'],
        finalized_ns=clock(),decision='HOLD',execution_allowed=False,forwarding_enabled=False)
    publish(root/'bundle.complete.json',complete)
    if clock()>=ledger.deadline.end_ns:return dict(checked,status='FAIL',failures=['Deadline exceeded finalizing marker'])
    return verify_bundle(root,ledger)


def verify_bundle(root,ledger):
    result=_bundle_inputs(root,ledger)
    try:
        if (Path(root)/'bundle.finalization.failed.json').exists():
            _,failure=inspect_file(root,'bundle.finalization.failed.json',ledger.test_id)
            if failure.get('status')=='FAIL':result['failures'].append('Finalization/deadline violation')
            raise EvidenceError('Final marker revoked by failed finalization')
        _,manifest=inspect_file(root,'bundle.manifest.json',ledger.test_id)
        _,complete=inspect_file(root,'bundle.complete.json',ledger.test_id)
        if (manifest.get('schema')!=2 or manifest.get('ledger_sha256')!=ledger.digest() or
                manifest.get('evidence_scope')!=result['evidence_scope'] or
                complete.get('evidence_scope')!=result['evidence_scope'] or
                manifest.get('files')!=result['files'] or result['incomplete'] or
                complete.get('schema')!=2 or complete.get('manifest_sha256')!=hashlib.sha256(
                    (Path(root)/'bundle.manifest.json').read_bytes()).hexdigest() or
                complete.get('cleanup_sha256')!=result['files']['cleanup.receipt.json']['sha256'] or
                complete.get('status')!=result['status'] or
                type(complete.get('finalized_ns')) is not int or complete['finalized_ns']>=ledger.deadline.end_ns or
                complete.get('execution_allowed') is not False or complete.get('forwarding_enabled') is not False):
            raise EvidenceError('Incomplete/inconsistent final manifest or marker')
    except Exception as exc:
        result['incomplete'].append('bundle: '+str(exc))
        result['status']='FAIL' if result['failures'] else 'INCOMPLETE'
    return result


def main():
    parser=argparse.ArgumentParser(description='Read-only portable heartbeat evidence verification; no runtime')
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--ledger',type=Path,required=True)
    args=parser.parse_args()
    try:
        ledger=Ledger.restore(parse_json(args.ledger.read_bytes()))
        result=verify_bundle(args.directory,ledger)
    except Exception as exc:
        result=dict(status='INCOMPLETE',reason=str(exc),decision='HOLD',
                    execution_allowed=False,forwarding_enabled=False)
    print(json.dumps(result,allow_nan=False))
    return {'PASS':0,'FAIL':1,'INCOMPLETE':2}[result['status']]


if __name__=='__main__':raise SystemExit(main())
