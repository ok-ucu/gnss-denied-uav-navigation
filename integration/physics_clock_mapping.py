"""Bound acquisition labels with APPLIED JSON clock anchors, not arrival fits.

Pinned JSON backend writes model_us before fill_fdm/scheduler stop_clock.
Monotonic source/target labels bound every intermediate label by its neighbours.
ClockMapping supplies only a representative estimate, clipped to this interval;
the interval, not a regression residual, supplies the error envelope.
This is NOT a camera exposure/transport latency bound or flight authorization.
"""
from bisect import bisect_left
from collections import deque
import hashlib
import json
from navigation_clock import ClockMapping, integer

PATCHED_SHA='047e4fbe21de2f88eba6c453e8aa3091ffd4b89b2b5568544f16cbf4c38d5970'


class AppliedClockMapping:
    def __init__(self,epoch,firmware_sha,*,max_uncertainty_ns=20_000_000,max_age_ns=200_000_000):
        if not epoch or firmware_sha!=PATCHED_SHA:raise ValueError('Unreviewed clock implementation')
        self.source_epoch=epoch;self.anchors=deque(maxlen=512)
        self.max_uncertainty_ns=max_uncertainty_ns;self.max_age_ns=max_age_ns
        self.verified=False;self.broken=False;self.remote_checks=0
        self.identity='unready';self.reason='need applied anchors and FCU cross-check'

    def block(self,reason):
        self.broken=True;self.verified=False;self.reason=reason
        raise ValueError(reason)

    def feed(self,source_ns,target_ns,host_ns,*,parsed_no_time_sync,use_time_sync):
        if self.broken:raise ValueError(self.reason)
        if not all(integer(v) for v in (source_ns,target_ns,host_ns)):
            return self.block('invalid applied clock')
        if (parsed_no_time_sync,use_time_sync)!=(1,0):return self.block('parser/pacing provenance mismatch')
        row=dict(source_ns=source_ns,target_ns=target_ns,host_ns=host_ns)
        if self.anchors:
            last=self.anchors[-1]
            if any(row[k]<=last[k] for k in row):return self.block('physics clock reset or unordered applied anchors')
            if source_ns-last['source_ns']>25_000_000:return self.block('unbounded applied source interval')
        self.anchors.append(row)
        self.verified=len(self.anchors)>=3 and self.remote_checks>=10
        self.identity=hashlib.sha256(json.dumps(list(self.anchors)[-3:],sort_keys=True).encode()).hexdigest()

    def crosscheck(self,fcu_ns,sent_ns,received_ns):
        """Remote AP_HAL time must lie in a sampled MODEL-time receive bracket.

        One physics step allowance covers the observed before-stop_clock logger.
        No assumption that one-way latency is half RTT. Deferred until the later
        anchor exists. False means insufficient anchors, not a successful check.
        """
        if self.broken:raise ValueError(self.reason)
        if not all(integer(v) for v in (fcu_ns,sent_ns,received_ns)) or sent_ns>received_ns:
            return self.block('invalid FCU time witness')
        before=[a for a in self.anchors if a['host_ns']<=sent_ns]
        after=[a for a in self.anchors if a['host_ns']>=received_ns]
        if not before or not after:return False
        lo=before[-1]['target_ns']-2_000_000;hi=after[0]['target_ns']+1_000
        if not lo<=fcu_ns<=hi:return self.block('FCU/model clock domain mismatch')
        self.remote_checks+=1;self.verified=len(self.anchors)>=3 and self.remote_checks>=10
        return True

    def map(self,source_ns,host_ns,source_epoch):
        if not self.verified or self.broken:raise ValueError('unverified applied clock proof')
        if source_epoch!=self.source_epoch or not integer(source_ns) or not integer(host_ns):
            raise ValueError('clock epoch/domain mismatch')
        rows=list(self.anchors)
        self.check_freshness(host_ns)
        index=bisect_left([r['source_ns'] for r in rows],source_ns)
        if index==len(rows) or source_ns<rows[0]['source_ns']:raise ValueError('no bracketing physics samples; never extrapolate')
        if rows[index]['source_ns']==source_ns:
            target=rows[index]['target_ns'];lo=hi=target
        else:
            left,right=rows[index-1:index+1]
            diagnostic=ClockMapping(rows[max(0,index-2):min(len(rows),max(3,index+2))],source_epoch=self.source_epoch)
            lo,hi=left['target_ns'],right['target_ns']
            target=min(hi,max(lo,diagnostic.estimate(source_ns)))
        uncertainty=max(target-lo,hi-target)+1000  # text/µs rounding margin
        if uncertainty>self.max_uncertainty_ns:raise ValueError('clock envelope exceeds declared budget')
        return dict(fcu_ns=target,acquisition_ns=source_ns,uncertainty_ns=uncertainty,model=self.identity,
                    lower_fcu_ns=lo,upper_fcu_ns=hi,proof='applied monotonic source/target bracket; no arrival fit')

    def check_clock(self,source_ns,host_ns):
        return self.map(source_ns,host_ns,self.source_epoch)

    def check_freshness(self,host_ns):
        """Age the latest actual production event, never its file-read time."""
        if not self.verified or self.broken:raise ValueError('unverified applied clock proof')
        if not integer(host_ns) or not 0<=host_ns-self.anchors[-1]['host_ns']<=self.max_age_ns:
            raise ValueError('applied clock expired/paused')
        return self.anchors[-1]
