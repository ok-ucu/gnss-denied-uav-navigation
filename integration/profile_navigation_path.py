"""Offline microprofile using recorded REAL odometry and actual ROS types.

No publishers, network connection or FCU admission. Sequence numbers below are
benchmark-only durable-store writes; they are never navigation measurements.
"""
import argparse
import json
from pathlib import Path
import time

from stage_timing import activate, sample, stage
from navigation_lifecycle import NavigationLifecycle
from navigation_egress import VisionEgress
from vio_frame_bridge import convert
from vio_adapter import convert_pair


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--iterations',type=int,default=40)
    args=parser.parse_args()
    args.output.mkdir(exist_ok=False)
    recorder=activate(args.output/'stages.jsonl')
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped,TwistWithCovarianceStamped
    row=json.loads(args.recording.read_text().splitlines()[10])
    msg=Odometry()
    msg.header.frame_id=row['frame'];msg.child_frame_id=row['child_frame']
    msg.header.stamp.sec=row['sec'];msg.header.stamp.nanosec=row['nanosec']
    for vector,values in ((msg.pose.pose.position,row['position']),
                          (msg.twist.twist.linear,row['linear']),
                          (msg.twist.twist.angular,row['angular'])):
        vector.x,vector.y,vector.z=values
    q=msg.pose.pose.orientation;q.x,q.y,q.z,q.w=row['q']
    msg.pose.covariance=row['pose_covariance'];msg.twist.covariance=row['twist_covariance']
    ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    class BenchmarkMapping:
        verified=True
        source_epoch='offline-benchmark-only'
        def map(self,stamp,host,epoch):
            return dict(fcu_ns=stamp,acquisition_ns=stamp,uncertainty_ns=1_000,model='offline-no-wire')
    gate=NavigationLifecycle(args.output/'reference.json',BenchmarkMapping())
    egress=VisionEgress(args.output/'reference.json')
    gate.clock=ns+100_000_000;gate.clock_host=time.monotonic_ns()
    token=dict(session='offline-recorded-fixture',epoch=0,stamp_ns=ns,epoch_start_ns=ns,ready=True)
    gate.authorize('OFFLINE_BENCHMARK_ONLY',token,'PASS',time.monotonic_ns())
    egress.authorize('OFFLINE_BENCHMARK_ONLY',token['session'],'PASS')
    try:
        for index in range(args.iterations):
            current=ns+(index+1)*100_000_000
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(current,10**9)
            token['stamp_ns']=current
            gate.clock=current+100_000_000;gate.clock_host=time.monotonic_ns()
            with sample(current):
                with stage('callback_validation'):
                    convert(msg)
                with stage('full_pair'):
                    pair=gate.pair(msg,token,time.monotonic_ns(),True,PoseWithCovarianceStamped,TwistWithCovarianceStamped)
                if pair is None:
                    raise RuntimeError(gate.reason)
                with stage('egress_approve'):
                    if not egress.approve(pair[2]):raise RuntimeError('Offline permission rejected')
                with stage('egress_check'):
                    # Profile the real two reads, not a fabricated wire arrival.
                    # Live deadline admission is measured separately in transport.
                    egress.state();egress.state()
    finally:
        gate.close();egress.close();recorder.close()


if __name__=='__main__':main()
