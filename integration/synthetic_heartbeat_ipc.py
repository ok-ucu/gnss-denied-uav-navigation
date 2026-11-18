"""Authenticated private AF_UNIX fixture control; no listener or FCU connector."""
import hashlib
import hmac
import json
import socket

from heartbeat_test_owner import canonical
from heartbeat_test_evidence import parse_json


class AuthenticationFailure(RuntimeError):pass


class TestControl:
    """Per-direction sequence numbers, per-run/peer key, immutable deadline.

    The key is passed to an owned child through its private bootstrap pipe,
    never argv/environment/evidence. No key or unauthenticated command fallback.
    """
    def __init__(self,stream,key,test_id,sender,peer,deadline_ns,max_bytes=8192):
        if stream.family!=socket.AF_UNIX or len(key)!=32 or sender==peer:
            raise AuthenticationFailure('Private Unix socketpair/key/peer required')
        self.stream=stream;stream.setblocking(False)
        self.key=key;self.test_id=test_id;self.sender=sender;self.peer=peer
        self.deadline_ns=deadline_ns;self.limit=max_bytes;self.incoming=bytearray();self.pending=bytearray()
        self.sent=0;self.received=0;self.failed=False

    def queue(self,command,payload=None,corrupt_mac=False,wrong_run=False):
        if self.failed:raise AuthenticationFailure('Control failure latched')
        body=dict(test_id='wrong-run' if wrong_run else self.test_id,sender=self.sender,
                  sequence=self.sent,deadline_ns=self.deadline_ns,command=command,payload=payload or {})
        mac=hmac.new(self.key,canonical(body),hashlib.sha256).hexdigest()
        if corrupt_mac:mac='0'*64
        raw=canonical(dict(body=body,mac=mac))
        if len(raw)>self.limit or len(self.pending)+len(raw)>self.limit:
            self.failed=True;raise AuthenticationFailure('Control output bound exceeded')
        self.pending.extend(raw);self.sent+=1

    def flush(self):
        if not self.pending:return
        try:sent=self.stream.send(self.pending)
        except (BlockingIOError,InterruptedError):return
        if sent<=0:raise AuthenticationFailure('Control disconnected')
        del self.pending[:sent]

    def receive(self):
        if self.failed:raise AuthenticationFailure('Control failure latched')
        self.flush()
        try:data=self.stream.recv(self.limit+1)
        except (BlockingIOError,InterruptedError):data=None
        if data==b'':raise EOFError('Private control peer exited')
        if data:self.incoming.extend(data)
        if len(self.incoming)>self.limit:
            self.failed=True;raise AuthenticationFailure('Control input bound exceeded')
        rows=[]
        while b'\n' in self.incoming:
            line,_,rest=self.incoming.partition(b'\n');self.incoming=bytearray(rest)
            try:
                envelope=parse_json(line);body=envelope['body']
                expected=hmac.new(self.key,canonical(body),hashlib.sha256).hexdigest()
                if (set(envelope)!={'body','mac'} or not isinstance(envelope['mac'],str) or
                        not hmac.compare_digest(envelope['mac'],expected) or
                        body.get('test_id')!=self.test_id or body.get('sender')!=self.peer or
                        type(body.get('sequence')) is not int or body['sequence']!=self.received or
                        body.get('deadline_ns')!=self.deadline_ns or
                        not isinstance(body.get('command'),str) or not isinstance(body.get('payload'),dict)):
                    raise AuthenticationFailure('Unauthorized/run/peer/sequence/deadline mismatch')
            except Exception as exc:
                self.failed=True;raise AuthenticationFailure('Control authentication rejected: '+str(exc)) from exc
            self.received+=1;rows.append(body)
        return rows

    def close(self):self.stream.close()
