"""Explicit per-sample epoch join. No ROS/GT/network dependency.

Odometry alone carries no reset counter. Require a matching producer token,
explicit initial approval, and latch closed after any approved epoch changes.
Restart/reapproval is an operator action, not automatic coordinate continuity.
"""
from collections import OrderedDict


class EpochJoin:
    def __init__(self,capacity=32):
        self.capacity=capacity;self.current=None;self.approved=None
        self.retired=set();self.latched=False;self.ever_approved=False
        self.tokens=OrderedDict()

    def tag(self,data):
        if (not isinstance(data.get('session'),str) or not data['session']
                or type(data.get('epoch')) is not int or data['epoch']<0
                or type(data.get('stamp_ns')) is not int or data['stamp_ns']<=0
                or type(data.get('epoch_start_ns')) is not int
                or not 0<data['epoch_start_ns']<=data['stamp_ns'] or data.get('ready') is not True):
            raise ValueError('Malformed/nonready epoch token')
        key=(data['session'],data['epoch'])
        if key in self.retired:raise ValueError('Retired epoch token')
        if key!=self.current:
            if self.current is not None:self.retired.add(self.current)
            if self.ever_approved:self.latched=True
            self.current=key;self.approved=None;self.tokens.clear()
        if data['stamp_ns'] in self.tokens:raise ValueError('Duplicate token')
        self.tokens[data['stamp_ns']]=key
        if len(self.tokens)>self.capacity:self.tokens.popitem(last=False)

    def approve(self,key):
        if self.latched or tuple(key)!=self.current or self.current[1]!=0:
            raise ValueError('Only explicit initial epoch0 approval; reset requires new pipeline/operator procedure')
        self.approved=self.current;self.ever_approved=True

    def consume(self,stamp):
        if self.latched or self.current is None or self.approved!=self.current:
            raise ValueError('Epoch not approved/reset latched')
        if self.tokens.pop(stamp,None)!=self.current:raise ValueError('No exact per-sample epoch token')
