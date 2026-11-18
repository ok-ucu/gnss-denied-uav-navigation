"""Explicit affine clock-domain conversion with deny-by-default provenance.

A regression of receive-time pairs is diagnostic only. `verified` requires an
external shared-time/transport-bound proof; fitting cannot supply that proof.
No ROS/FCU transport and no wall-time substitution for acquisition timestamps.
"""
import hashlib
import json
import math


def integer(value):
    return type(value) is int and value >= 0


class ClockMapping:
    def __init__(self, anchors, *, source_epoch, verified=False,
                 uncertainty_ns=None, max_uncertainty_ns=20_000_000,
                 max_age_ns=250_000_000, extrapolation_ns=50_000_000):
        if len(anchors)<3 or not source_epoch:
            raise ValueError('Require >=3 timestamp anchors and source epoch')
        if any(not all(integer(row[k]) for k in ('source_ns','target_ns','host_ns')) for row in anchors):
            raise ValueError('Clock anchors must use finite integer nanoseconds')
        if any(any(b[k]<=a[k] for k in ('source_ns','target_ns','host_ns')) for a,b in zip(anchors,anchors[1:])):
            raise ValueError('Clock reset, pause or unordered anchors')
        if any(type(v) is not int or v<0 for v in (max_uncertainty_ns,max_age_ns,extrapolation_ns)) or max_age_ns==0:
            raise ValueError('Invalid clock budgets')
        self.first,self.last=anchors[0],anchors[-1]
        x=[r['source_ns']-self.first['source_ns'] for r in anchors]
        y=[r['target_ns']-self.first['target_ns'] for r in anchors]
        mx,my=sum(x)/len(x),sum(y)/len(y)
        self.slope=sum((a-mx)*(b-my) for a,b in zip(x,y))/sum((a-mx)**2 for a in x)
        self.intercept=my-self.slope*mx
        self.residual_ns=max(abs(b-(self.intercept+self.slope*a)) for a,b in zip(x,y))
        if not math.isfinite(self.slope) or not .1<self.slope<10:
            raise ValueError('Invalid relative clock rate')
        self.source_epoch=source_epoch
        self.uncertainty_ns=uncertainty_ns
        self.verified=(verified is True and type(uncertainty_ns) is int and uncertainty_ns>=0
                       and uncertainty_ns+self.residual_ns<=max_uncertainty_ns)
        self.max_age_ns=max_age_ns;self.extrapolation_ns=extrapolation_ns
        self.max_uncertainty_ns=max_uncertainty_ns
        self.target_per_host=(self.last['target_ns']-self.first['target_ns'])/(self.last['host_ns']-self.first['host_ns'])
        self.identity=hashlib.sha256(json.dumps(anchors,sort_keys=True).encode()).hexdigest()

    def estimate(self, source_ns):
        if not integer(source_ns):
            raise ValueError('Acquisition timestamp must be integer nanoseconds')
        return self.first['target_ns']+round(self.intercept+self.slope*(source_ns-self.first['source_ns']))

    def map(self, source_ns, host_ns, source_epoch):
        if not self.verified:
            raise ValueError('Unverified shared-clock/transport uncertainty bound')
        if source_epoch!=self.source_epoch or not integer(host_ns):
            raise ValueError('Clock epoch/domain mismatch')
        if not 0<=host_ns-self.last['host_ns']<=self.max_age_ns:
            raise ValueError('Mapping expired or host clock reset')
        if not self.first['source_ns']<=source_ns<=self.last['source_ns']+self.extrapolation_ns:
            raise ValueError('Acquisition outside calibrated clock interval')
        return dict(fcu_ns=self.estimate(source_ns),acquisition_ns=source_ns,
                    uncertainty_ns=math.ceil(self.uncertainty_ns+self.residual_ns),model=self.identity)

    def check_clock(self, source_ns, host_ns):
        mapped=self.map(source_ns,host_ns,self.source_epoch)
        expected=self.last['target_ns']+(host_ns-self.last['host_ns'])*self.target_per_host
        if abs(mapped['fcu_ns']-expected)>self.max_uncertainty_ns:
            raise ValueError('Clock jump or rate change beyond verified budget')
        return mapped
