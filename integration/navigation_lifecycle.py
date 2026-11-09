"""Pure fail-closed composition; never sends messages or auto-approves a reset.

Order: producer quality/token -> frame bridge -> MAVROS adapter -> verified time
mapping -> durable admission. Caller may publish the returned pair only; an
initial authorization token is consumed as reference evidence, not forwarded.
"""
from reset_safety import ResetSafety
from navigation_store import StoreOwner
from mavros_epoch import EpochJoin
from vio_frame_bridge import convert as bridge
from vio_adapter import convert_pair
from stage_timing import stage,mark


class NavigationLifecycle:
    def __init__(self,path,mapping):
        self.owner=StoreOwner(path)
        self.durable=ResetSafety(path);self.join=EpochJoin();self.mapping=mapping
        self.clock=None;self.clock_host=None;self.last_output=None;self.reason='startup closed'
        self.last_transformed=None

    def block(self,reason):
        if self.owner.closed:return None
        mark('lifecycle_block',reason=reason)
        self.reason=reason;self.durable.state['blocked']=True;self.durable.save()
        return None

    def clock_tick(self,stamp_ns,host_ns,*,validation_host_ns=None):
        if self.owner.closed:return None
        if type(stamp_ns) is not int or type(host_ns) is not int or min(stamp_ns,host_ns)<0:
            return self.block('invalid clock')
        if self.clock is not None and (stamp_ns<=self.clock or host_ns<=self.clock_host):
            return self.block('clock jump/pause/reset')
        if self.mapping.verified:
            # host_ns is the observation event used by the watchdog. Mapping
            # expiry must be evaluated NOW, not at an older common-watermark
            # event that predates the mapper's newest anchor.
            try:self.mapping.check_clock(stamp_ns,host_ns if validation_host_ns is None else validation_host_ns)
            except ValueError as error:return self.block(str(error))
        self.clock,self.clock_host=stamp_ns,host_ns

    def watchdog(self,host_ns):
        if self.owner.closed:return False
        if self.clock_host is None or not 0<=host_ns-self.clock_host<=200_000_000:
            self.block('clock lost/stalled')
        return not self.durable.state['blocked']

    def authorize(self,reference,initial_token,preflight,host_ns):
        if self.owner.closed:raise ValueError('Store owner stopped')
        self.block('explicit reference initialization pending')
        if preflight!='PASS' or not self.mapping.verified:
            raise ValueError('Require explicit new reference and verified preflight')
        if (self.clock_host is None or type(host_ns) is not int
                or not 0<=host_ns-self.clock_host<=200_000_000):
            raise ValueError('Current clock required for explicit initialization')
        self.mapping.map(initial_token['stamp_ns'],host_ns,self.mapping.source_epoch)
        if not reference or reference==self.durable.state['fcu']:
            raise ValueError('Reference must be explicitly new')
        if self.join.ever_approved:
            self.join=EpochJoin()  # Only this explicit new-reference action permits it.
        self.join.tag(initial_token);self.join.approve((initial_token['session'],initial_token['epoch']))
        self.join.consume(initial_token['stamp_ns'])
        self.durable.initialize(reference,initial_token['session'],initial_token['epoch'])
        self.durable.admit(initial_token['session'],initial_token['epoch'],initial_token['stamp_ns'])
        self.last_output=None;self.reason='explicit reference approved'

    def pair(self,odom,token,host_ns,quality_ready,pose_type,speed_type):
        if self.owner.closed:return None
        try:
            if self.durable.state['blocked'] or not self.watchdog(host_ns):
                raise ValueError('durable latch or clock unavailable')
            if quality_ready is not True:raise ValueError('quality unavailable')
            ns=odom.header.stamp.sec*10**9+odom.header.stamp.nanosec
            if not -5_000_000<=self.clock-ns<=200_000_000:raise ValueError('stale/future acquisition')
            self.join.tag(token);self.join.consume(ns)
            with stage('frame_conversion'):
                transformed=bridge(odom)
            with stage('mavros_conversion'):
                pose,speed,_=convert_pair(transformed,pose_type,speed_type,self.clock)
            with stage('clock_map'):
                mapped=self.mapping.map(ns,host_ns,self.mapping.source_epoch)
            if self.last_output is not None and mapped['fcu_ns']<=self.last_output:
                raise ValueError('mapped timestamp reset')
            with stage('durable_admit'):
                if not self.durable.admit(token['session'],token['epoch'],ns):
                    raise ValueError('durable session/epoch/order rejection')
            for message in (pose,speed):
                message.header.stamp.sec,message.header.stamp.nanosec=divmod(mapped['fcu_ns'],10**9)
            mapped.update(reference=self.durable.state['fcu'],producer=token['session'],epoch=0,approved_host_ns=host_ns,
                          allowed_host_deadline_ns=host_ns+max(0,200_000_000-max(0,self.clock-ns)-mapped['uncertainty_ns']))
            self.last_output=mapped['fcu_ns'];self.reason='validated pair'
            self.last_transformed=transformed
            return pose,speed,mapped
        except (ValueError,KeyError,TypeError,AttributeError) as error:
            return self.block(str(error))

    def close(self):
        self.block('owner shutdown');self.owner.close()
