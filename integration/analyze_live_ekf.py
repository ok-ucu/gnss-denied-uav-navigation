"""Summarize live EKF3 experiment using DataFlash, not send-call success.

VISP/VISV Ign is backend ignore, NOT an EKF innovation-rejection counter.
XKF4 gate samples are not per-measurement acceptance counts.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import statistics
from collections import Counter



def rms(values):
    return math.sqrt(statistics.mean(v*v for v in values)) if values else None


def summarize(folder, binary):
    from pymavlink import mavutil
    with open(folder/'generator.csv',newline='') as stream:
        generator = list(csv.DictReader(stream))
    starts = [r for r in generator if r['event']=='phase_start']
    ends = {r['phase']:float(r['unix_time_s']) for r in generator if r['event']=='phase_end'}
    sends = {int(r['measurement_usec']):r['phase'] for r in generator if r['event']=='sent'}
    counts = Counter(sends.values())
    records, messages = [], []
    log = mavutil.mavlink_connection(str(binary))
    wanted = {'VISP','VISV','XKF1','XKF3','XKF4','XKV1','XKFS','MSG','EV','ORGN','ARM'}
    types = Counter()
    while True:
        message = log.recv_match()
        if message is None:
            break
        kind = message.get_type()
        types[kind] += 1
        if kind in wanted:
            records.append(message.to_dict())
        if kind == 'MSG':
            messages.append(message.to_dict())
    log.close()
    anchors = [(r['RTimeUS']-r['TimeUS'])/1e6 for r in records
               if r['mavpackettype']=='VISP' and r['RTimeUS'] in sends]
    if not anchors:
        raise ValueError('No matched received VISP records: cannot align BIN with generator')
    offset = statistics.median(anchors)
    def phase_of(record):
        if record['mavpackettype'] in ('VISP','VISV'):
            return sends.get(record['RTimeUS'])
        unix = record.get('TimeUS',0)/1e6+offset
        # Exclude boundary uncertainty from sampled EKF metrics.
        for row in starts:
            if float(row['unix_time_s'])+.25 <= unix < ends[row['phase']]-.25:
                return row['phase']
        return None
    rows = []
    for start in starts:
        name = start['phase']
        selected = [r for r in records if phase_of(r)==name]
        def group(kind,core=None):
            return [r for r in selected if r['mavpackettype']==kind and
                    (core is None or r.get('C')==core)]
        poses, velocities = group('VISP'),group('VISV')
        innovations,status,covariances = group('XKF3',0),group('XKF4',0),group('XKV1',0)
        states = group('XKF1',0)
        has_measurements = bool(poses and velocities)
        result = dict(phase=name,sent_pairs=counts[name],visp_records=len(poses),
                      visv_records=len(velocities),
                      backend_pose_ignored=sum(r['Ign'] != 0 for r in poses),
                      backend_speed_ignored=sum(r['Ign'] != 0 for r in velocities),
                      ekf_accepted_measurements=None,ekf_rejected_measurements=None,
                      pose_error_values=sorted(set(round(r['PErr'],6) for r in poses)),
                      velocity_error_values=sorted(set(round(r['VErr'],6) for r in velocities)),
                      core=0,xkf3_samples=len(innovations),xkf4_samples=len(status),
                      position_innovation_xy_rms_m=rms([math.hypot(r['IPN'],r['IPE']) for r in innovations]) if has_measurements else None,
                      velocity_innovation_xy_rms_m_s=rms([math.hypot(r['IVN'],r['IVE']) for r in innovations]) if has_measurements else None,
                      sp_median=statistics.median(r['SP'] for r in status) if status else None,
                      sv_median=statistics.median(r['SV'] for r in status) if status else None,
                      sampled_position_gate_above_one=sum(r['SP']>1 for r in status) if has_measurements else None,
                      sampled_velocity_gate_above_one=sum(r['SV']>1 for r in status) if has_measurements else None,
                      position_timeout_samples=sum(bool(r['TS'] & 1) for r in status),
                      velocity_timeout_samples=sum(bool(r['TS'] & 2) for r in status),
                      primary_values=sorted(set(r['PI'] for r in status)),
                      initialized_samples=sum(bool(r['SS'] & (1<<16)) for r in status),
                      absolute_position_valid_samples=sum(bool(r['SS'] & (1<<4)) for r in status),
                      fault_values=sorted(set(r['FS'] for r in status)),
                      timeout_values=sorted(set(r['TS'] for r in status)),
                      solution_status_values=sorted(set(r['SS'] for r in status)),
                      pnn_median_m2=statistics.median(r['V07'] for r in covariances) if covariances else None,
                      pee_median_m2=statistics.median(r['V08'] for r in covariances) if covariances else None,
                      estimated_position_xy_rms_m=rms([math.hypot(r['PN'],r['PE']) for r in states]))
        rows.append(result)
    # CSV has typed records as JSON to preserve varying DataFlash field layouts.
    with open(folder/'dataflash_selected.csv','w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['type','TimeUS','aligned_unix_s','phase','core','data_json'])
        for record in records:
            writer.writerow([record['mavpackettype'],record.get('TimeUS'),
                             record.get('TimeUS',0)/1e6+offset,phase_of(record),
                             record.get('C',''),json.dumps(record,sort_keys=True)])
    result = dict(binary=str(binary),dataflash_type_counts=dict(types),
                  bin_to_unix_offset_s=offset,
                  matched_anchor_count=len(anchors),
                  anchor_offset_min_s=min(anchors),anchor_offset_max_s=max(anchors),
                  guard_s=.25,phases=rows,messages=messages,
                  limitation='EKF accepted/rejected per-measurement counters not exposed; Ign is backend only')
    (folder/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('binary',type=Path)
    args=parser.parse_args()
    summarize(args.folder,args.binary)
