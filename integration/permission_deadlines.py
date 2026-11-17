"""Offline permission boundaries; userspace receive is NOT kernel packet arrival.

No permission creation, deadline changes or runtime decisions. Missing sender/
kernel/scheduler evidence remains unknown; even early kernel RX could not
authorize an expired final userspace validation.
"""
import argparse
import json
from pathlib import Path


def timeline(stages,mappings,wire):
    enqueues={r['acquisition_ns']:r for r in stages if r['stage']=='mavros_enqueue'}
    by_usec={r['fcu_ns']//1000:r for r in mappings}
    result=[]
    for packet in wire:
        meta=by_usec.get(packet['message']['usec'])
        if meta is None:
            result.append(dict(kind=packet['message'].get('mavpackettype'),status='NOT TESTED',reason='no permission metadata'))
            continue
        enqueue=enqueues.get(meta['acquisition_ns'])
        if enqueue is None:
            result.append(dict(acquisition_ns=meta['acquisition_ns'],status='NOT TESTED',reason='no measured enqueue span; publish log is not enqueue completion'))
            continue
        start,end=enqueue['start_host_ns'],enqueue['end_host_ns']
        arrival,checked=packet['arrival_host_ns'],packet['checked_host_ns']
        deadline=meta['allowed_host_deadline_ns']
        if not all(type(v) is int for v in (start,end,arrival,checked,deadline,meta['approved_host_ns'])):
            raise ValueError('Integer monotonic nanoseconds required')
        if not meta['approved_host_ns']<=start<=end<=arrival<=checked:
            raise ValueError('Unaligned/out-of-order permission boundaries')
        result.append(dict(acquisition_ns=meta['acquisition_ns'],kind=packet['message'].get('mavpackettype'),
            status='MEASURED USERSPACE BOUNDARIES',valid_capture=packet['valid_capture'],
            decision_host_ns=meta['approved_host_ns'],enqueue_start_host_ns=start,enqueue_end_host_ns=end,
            receive_host_ns=arrival,final_check_host_ns=checked,deadline_host_ns=deadline,
            decision_to_enqueue_start_ms=(start-meta['approved_host_ns'])/1e6,
            enqueue_call_wall_ms=(end-start)/1e6,
            remaining_at_enqueue_end_ms=(deadline-end)/1e6,
            enqueue_end_to_userspace_receive_ms=(arrival-end)/1e6,
            userspace_receive_to_final_check_ms=(checked-arrival)/1e6,
            remaining_at_receive_ms=(deadline-arrival)/1e6,
            remaining_at_final_check_ms=(deadline-checked)/1e6,
            mapping_log_after_enqueue_ms=None if 'publish_host_ns' not in meta else (meta['publish_host_ns']-end)/1e6,
            sender_processing_ms=None,kernel_receive_host_ns=None,socket_queue_wait_ms=None,
            limitation='DDS/MAVROS send, socket queueing and receiver scheduling are not separately measured'))
    mapped={m['acquisition_ns'] for m in mappings}
    for marker in stages:
        if marker['stage']=='permission_created' and marker['source_ns'] not in mapped:
            result.append(dict(acquisition_ns=marker['source_ns'],status='NOT ENQUEUED',
                created_marker_host_ns=marker['host_ns'],deadline_host_ns=marker['deadline_host_ns'],
                remaining_at_created_marker_ms=(marker['deadline_host_ns']-marker['host_ns'])/1e6,
                downstream_latency='NOT TESTED: no enqueue/wire evidence'))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('transport',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    def read(name):
        with (a.transport/(name+'.jsonl')).open() as source:
            return [json.loads(line) for line in source if line.strip()]
    result=dict(decision='HOLD',execution_allowed=False,forwarding_enabled=False,
        boundaries=timeline(read('stages'),read('mapping'),read('wire')),
        note='No socket optimization or attribution from aggregate enqueue-to-recv time')
    if not a.output.resolve().is_relative_to(Path(__file__).resolve().parent):
        raise ValueError('integration output only')
    with a.output.open('x') as output:json.dump(result,output,indent=2);output.write('\n')


if __name__=='__main__':main()
