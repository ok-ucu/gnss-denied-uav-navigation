"""Final vision admission only; no socket transport. Dedicated durable store.

Start this after-MAVROS interlock before explicit reference authorization.
Its own .egress store has a single writer; upstream owns the original store.
Neither process restart can reuse the old reference. No reset_counter rewrite.
"""
import json
from collections import OrderedDict
from pathlib import Path
from reset_safety import ResetSafety
from navigation_store import StoreOwner
from stage_timing import stage


class VisionEgress:
    def __init__(self,state_path):
        self.path=Path(state_path);self.pending=OrderedDict()
        self.owner=StoreOwner(str(self.path)+'.egress')
        self.authorization=ResetSafety(str(self.path)+'.egress')
        self.authorization.save()

    def block(self):
        if self.owner.closed:return False
        self.pending.clear();self.authorization.state['blocked']=True;self.authorization.save()
        return False

    def state(self):
        if self.owner.closed:return None
        try:
            with stage('egress_read_state'):
                value=json.loads(self.path.read_text())
            if value.get('blocked') is not False or value.get('epoch')!=0:
                raise ValueError('Closed or unsupported epoch')
            return value
        except (OSError,ValueError,TypeError,AttributeError):
            self.block();return None

    def authorize(self,reference,producer,preflight):
        self.block();state=self.state()
        if preflight!='PASS' or state is None or (reference,producer)!=(state['fcu'],state['producer']):
            raise ValueError('Require matching explicit new reference after preflight')
        self.authorization.initialize(reference,producer)

    def approve(self,metadata):
        state=self.state()
        if (state is None or self.authorization.state['blocked']
                or metadata.get('reference')!=state['fcu'] or metadata.get('producer')!=state['producer']
                or (state['fcu'],state['producer'])!=(self.authorization.state['fcu'],self.authorization.state['producer'])
                or metadata.get('epoch')!=0 or type(metadata.get('acquisition_ns')) is not int
                or not 0<metadata['acquisition_ns']<=state['last_ns']
                or type(metadata.get('fcu_ns')) is not int or metadata['fcu_ns']<=0
                or type(metadata.get('approved_host_ns')) is not int
                or type(metadata.get('allowed_host_deadline_ns')) is not int):
            return False
        key=metadata['fcu_ns']//1000
        if key in self.pending:return self.block()
        self.pending[key]=dict(metadata,remaining={'VISION_POSITION_ESTIMATE','VISION_SPEED_ESTIMATE'})
        if len(self.pending)>32:self.pending.popitem(last=False)
        return True

    def check(self,kind,usec,reset_counter,host_ns,*,clock=None):
        state=self.state();sample=self.pending.get(usec)
        # A real transport must check after the potentially blocking state read,
        # rather than admit using the time captured before it. Pure fixtures may
        # keep supplying a deterministic host_ns; live callers supply a clock.
        if clock is not None:host_ns=clock()
        if state is None or self.authorization.state['blocked'] or sample is None:
            return False
        if (kind not in sample['remaining'] or sample['reference']!=state['fcu']
                or sample['producer']!=state['producer'] or reset_counter!=0
                or type(usec) is not int or type(reset_counter) is not int
                or type(host_ns) is not int or not sample['approved_host_ns']<=host_ns<=sample['allowed_host_deadline_ns']):
            return self.block()
        sample['remaining'].remove(kind)
        if not sample['remaining']:del self.pending[usec]
        return True

    def close(self):
        self.block();self.owner.close()
