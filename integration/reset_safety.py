"""Durable deny-by-default authorization for an isolated FCU experiment.

A new FCU initialization identity is an explicit operator assertion, not proof
that an autopilot was reinitialized. No FCU transport or automatic reapproval.
"""
import json
from pathlib import Path
from stage_timing import stage


class ResetSafety:
    def __init__(self,path):
        self.path=Path(path)
        self.state={'blocked':True,'fcu':None,'producer':None,'epoch':0,'last_ns':0,'used_fcu':[]}
        if self.path.exists():
            s=json.loads(self.path.read_text())
            if (set(s)!=set(self.state) or type(s['blocked']) is not bool or type(s['epoch']) is not int
                    or type(s['last_ns']) is not int or s['last_ns']<0 or s['epoch']!=0
                    or not isinstance(s['used_fcu'],list) or any(not isinstance(x,str) or not x for x in s['used_fcu'])
                    or any(s[k] is not None and (not isinstance(s[k],str) or not s[k]) for k in ['fcu','producer'])):
                raise ValueError('Malformed persistent authorization; refuse startup')
            self.state=s
            # Restart requires an explicit new FCU/reference initialization, even if no reset was seen.
            self.state['blocked']=True;self.save()

    def save(self):
        with stage('guard_save',store=self.path.name):
            self.path.parent.mkdir(parents=True,exist_ok=True)
            temporary=self.path.with_suffix('.pending')
            with stage('guard_write',store=self.path.name):
                temporary.write_text(json.dumps(self.state,allow_nan=False)+'\n')
            with stage('guard_rename',store=self.path.name):
                temporary.replace(self.path)

    def initialize(self,fcu,producer,epoch=0):
        if not isinstance(fcu,str) or not fcu or not isinstance(producer,str) or not producer or type(epoch) is not int or epoch!=0 or fcu in self.state['used_fcu']:
            raise ValueError('Require explicit NEW FCU/reference identity and producer epoch0')
        self.state=dict(blocked=False,fcu=fcu,producer=producer,epoch=0,last_ns=0,used_fcu=self.state['used_fcu']+[fcu]);self.save()

    def admit(self,producer,epoch,stamp_ns):
        s=self.state
        valid=(not s['blocked'] and producer==s['producer'] and type(epoch) is int and epoch==0
               and type(stamp_ns) is int and stamp_ns>s['last_ns'])
        if not valid:s['blocked']=True;self.save();return False
        s['last_ns']=stamp_ns;self.save();return True
