#!/usr/bin/env python3
"""DISARMED loss-of-VIO demo against ArduCopter SITL. Never arms, changes mode or takes off.

A (--mode A): loop SITL's own LOCAL_POSITION_NED back as VISION_* (like send_vision.py), then
  stop it for good and report when EKF3 drops absolute horizontal position (EKF_POS_HORIZ_ABS).
B (--mode B): 10 Hz frames through the real QualityGate and the option-D hold logic
  (integration/vio_gate_hold.py) with LABELLED bad-frame injections: 2 bad frames, then 7 bad
  frames. A transport-style 500 ms watchdog decides; a latch stops vision for good.
  Hold settings are required unless --no-hold (control run). Budget scope must be simulation.

This is NOT phase1_transport.py/run_vio_stream.py: no MAVROS, no camera VIO, no Gazebo.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/'integration'))
from vio_quality_gate import QualityGate                                   # noqa: E402
from vio_gate_hold import ProducerHold, TransportHold, frame_cause, load_budget  # noqa: E402

POS_ABS = 16           # MAVLink EKF_POS_HORIZ_ABS
ORIGIN = (-35.363261, 149.165230, 584.0)
GOOD = dict(imu_age=0., frame_age=.01, clock_age=.004, max_imu_gap=.001, frame_gap=.1,
            covariance=np.eye(9)*.01, state_finite=True)


class Link:
    def __init__(self, url, log):
        from pymavlink import mavutil
        self.m, self.log = mavutil.mavlink, log
        deadline = time.monotonic()+180
        while True:
            try:
                self.c = mavutil.mavlink_connection(url, source_system=255)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise SystemExit('SITL not reachable at '+url)
                time.sleep(1)
        self.c.wait_heartbeat(timeout=60)
        self.pos = [0.]*6
        self.att = [0.]*3
        self.flags = None
        self.t0 = time.monotonic()
        ts, tc = self.c.target_system, self.c.target_component
        self.c.mav.param_set_send(ts, tc, b'LOG_DISARMED', 1, self.m.MAV_PARAM_TYPE_INT8)
        self.c.mav.param_request_read_send(ts, tc, b'VISO_QUAL_MIN', -1)
        self.c.mav.set_gps_global_origin_send(ts, int(ORIGIN[0]*1e7), int(ORIGIN[1]*1e7), int(ORIGIN[2]*1000),
                                              int(time.time()*1e6))
        for msg_id, hz in ((32, 30), (30, 30), (193, 10)):   # LOCAL_POSITION_NED, ATTITUDE, EKF_STATUS_REPORT
            self.c.mav.command_long_send(ts, tc, 511, 0, msg_id, int(1e6/hz), 0, 0, 0, 0, 0)

    def poll(self):
        while (msg := self.c.recv_match(blocking=False)) is not None:
            kind = msg.get_type()
            if kind == 'HEARTBEAT' and msg.get_srcSystem() == self.c.target_system and msg.base_mode & 128:
                raise SystemExit('Vehicle ARMED: this demo is DISARMED only; stop SITL')
            if kind == 'LOCAL_POSITION_NED':
                self.pos = [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz]
            elif kind == 'ATTITUDE':
                self.att = [msg.roll, msg.pitch, msg.yaw]
            elif kind == 'EKF_STATUS_REPORT':
                if self.flags is None or (msg.flags & POS_ABS) != (self.flags & POS_ABS):
                    self.log(event='ekf_pos_horiz_abs', value=bool(msg.flags & POS_ABS), flags=msg.flags,
                             pos_horiz_variance=msg.pos_horiz_variance)
                self.flags = msg.flags
            elif kind == 'STATUSTEXT':
                self.log(event='statustext', text=msg.text)
            elif kind == 'PARAM_VALUE' and msg.param_id.rstrip('\0') == 'VISO_QUAL_MIN':
                self.log(event='param', name='VISO_QUAL_MIN', value=msg.param_value)
                if msg.param_value > 0:
                    raise SystemExit('VISO_QUAL_MIN > 0: FCU would drop all VISION_* messages')

    def pos_ok(self):
        return self.flags is not None and bool(self.flags & POS_ABS)

    def send_vision(self):
        usec = int((time.monotonic()-self.t0)*1e6)
        x, y, z, vx, vy, vz = self.pos
        self.c.mav.vision_position_estimate_send(usec, x, y, z, *self.att)
        self.c.mav.vision_speed_estimate_send(usec, vx, vy, vz)


class DryLink:
    """--dry-run: no SITL; checks the B decision timeline offline."""
    def __init__(self, url, log):
        self.flags = POS_ABS
        self.sent = 0

    def poll(self):
        pass

    def pos_ok(self):
        return True

    def send_vision(self):
        self.sent += 1


def pause(link, seconds, tick=None):
    end = time.monotonic()+seconds
    while (now := time.monotonic()) < end:
        link.poll()
        if tick:
            tick(now)
        time.sleep(.002)


def warmup(link, log, rate_hz, frame=None):
    """Send vision until EKF3 reports absolute horizontal position for 5 s (max 150 s)."""
    start, ok_since = time.monotonic(), None
    while True:
        link.poll()
        now = time.monotonic()
        if frame is None:
            link.send_vision()
        else:
            frame(now, bad=False)
        ok_since = (ok_since or now) if link.pos_ok() else None
        if ok_since and now-ok_since >= 5:
            log(event='warmup_done', seconds=now-start)
            return
        if now-start > 150:
            raise SystemExit('FAIL: EKF3 never reported absolute horizontal position from vision (150 s)')
        pause(link, 1/rate_hz)


def coast(link, log, since, observe_s):
    """After the last vision sample: time until EKF_POS_HORIZ_ABS clears."""
    end, cleared = time.monotonic()+observe_s, None
    while time.monotonic() < end:
        link.poll()
        if cleared is None and not link.pos_ok():
            cleared = time.monotonic()-since
            log(event='ekf_lost_horizontal_position', seconds_after_last_vision=cleared)
        time.sleep(.01)
    return cleared


def run_a(link, log, observe_s):
    warmup(link, log, 30)
    last = time.monotonic()
    log(event='vision_stopped_for_good')
    cleared = coast(link, log, last, observe_s)
    ok = cleared is not None and 6.0 <= cleared <= 11.0
    return dict(mode='A', ekf_coast_s=cleared, expected='7-10 s (posRetryTimeNoVel 7 s / UseVel 10 s)',
                result='PASS' if ok else 'FAIL')


def run_b(link, log, budget, observe_s):
    gate = QualityGate()
    producer = transport = None
    if budget is not None:
        producer, transport = ProducerHold(budget, 's'), TransportHold(budget, log=lambda **r: log(**r))
    st = dict(k=0, last_vision=None, latched=None, outages=[], clock=(1_000_000_000, time.monotonic()),
              first_bad=None)

    def sim_ns(now):
        # Stand-in for /clock: the latest frame stamp advanced by host time (SITL runs at speedup 1).
        stamp, host = st['clock']
        return stamp+round((now-host)*1e9)

    def watchdog(now):
        if st['latched'] or st['last_vision'] is None or now-st['last_vision'] <= .5:
            return
        if transport is not None and transport.valid(round(now*1e9), sim_ns(now)):
            return
        cap = transport is not None and transport.cap_exceeded(round(now*1e9), sim_ns(now))
        st['latched'] = dict(reason='VIO gate-closed hold exceeded cap' if cap else 'VIO availability watchdog',
                             host=now)
        log(event='latched_closed', reason=st['latched']['reason'], gap_s=now-st['last_vision'])

    def frame(now, bad):
        k = st['k']
        st['k'] += 1
        if st['latched']:
            return
        t = round(1.0+k*.1, 9)
        stamp = round(t*1e9)
        st['clock'] = (stamp, now)
        if bad and st['first_bad'] is None:
            st['first_bad'] = now
        ready = gate.assess(t, now, inliers=0 if bad else 120, contrast=30., audit_ok=not bad,
                            ekf_ok=not bad, consistency_ok=True, **GOOD)
        if ready:
            link.send_vision()
            st['last_vision'] = now
            if producer:
                producer.normal(stamp, now)
                transport.normal(stamp, round(now*1e9))
        elif producer:
            message = producer.status(stamp, now, 0, frame_cause(gate.reason, False), True)
            if message:
                transport.on_status(json.dumps(message), round(now*1e9), stamp+4_000_000, 's')
        if bad:
            log(event='injected_bad_frame', frame=k, t=t, label='LABELLED LOCAL INJECTION (inliers=0)')

    def frames(n, bad=()):
        for i in range(n):                                 # one continuous 10 Hz schedule
            pause(link, max(0., st['next']-time.monotonic()), watchdog)
            frame(time.monotonic(), bad=i in bad)
            st['next'] += .1

    warmup(link, log, 10, frame)
    st['next'] = time.monotonic()+.1                       # warmup just sent its last frame
    for name, count in (('outage_2_frames', 2), ('outage_7_frames', 7)):
        frames(20)                                         # >= 3 normal samples between holds
        if st['latched']:
            break
        st['first_bad'] = None
        frames(12, bad=range(count))
        latched = st['latched']
        st['outages'].append(dict(name=name, bridged=latched is None,
                                  latch_reason=None if latched is None else latched['reason'],
                                  latch_after_first_bad_s=None if latched is None else latched['host']-st['first_bad']))
        if latched:
            break
    cleared = coast(link, log, st['last_vision'], observe_s) if st['latched'] else None
    o = {x['name']: x for x in st['outages']}
    if budget is not None:
        two, seven = o.get('outage_2_frames', {}), o.get('outage_7_frames', {})
        ok = (two.get('bridged') is True and seven.get('latch_reason') == 'VIO gate-closed hold exceeded cap'
              and .65 <= seven.get('latch_after_first_bad_s', 0) <= .85)
    else:
        ok = o.get('outage_2_frames', {}).get('latch_reason') == 'VIO availability watchdog'
    return dict(mode='B' if budget else 'B-nohold', outages=st['outages'], ekf_coast_after_latch_s=cleared,
                result='PASS' if ok else 'FAIL')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--mode', choices=('A', 'B'), required=True)
    p.add_argument('--connect', default='tcp:127.0.0.1:5760')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--observe-s', type=float, default=25.)
    p.add_argument('--no-hold', action='store_true', help='B control run: hold OFF')
    p.add_argument('--gate-closed-hold-max-sim-s', type=float)
    p.add_argument('--gate-closed-hold-max-host-s', type=float)
    p.add_argument('--gate-closed-hold-budget-evidence', type=Path)
    p.add_argument('--dry-run', action='store_true', help='No SITL: check the B decision timeline only')
    a = p.parse_args()
    budget = None
    if a.mode == 'B' and not a.no_hold:
        budget = load_budget(a.gate_closed_hold_max_sim_s, a.gate_closed_hold_max_host_s,
                             a.gate_closed_hold_budget_evidence, simulation=True)
        if budget is None:
            p.error('B needs the three --gate-closed-hold-* settings (or --no-hold)')
    a.out.mkdir(parents=True, exist_ok=True)
    events = (a.out/'events.jsonl').open('a')

    def log(**row):
        row = dict(host_s=time.monotonic(), **row)
        events.write(json.dumps(row, default=str)+'\n')
        events.flush()
        print(json.dumps(row, default=str), flush=True)

    link = (DryLink if a.dry_run else Link)(a.connect, log)
    if budget:
        log(event='gate_closed_hold_configured', **budget)
    summary = run_a(link, log, a.observe_s) if a.mode == 'A' else run_b(link, log, budget, 0 if a.dry_run else a.observe_s)
    summary.update(dry_run=a.dry_run, scope='SITL DISARMED demo; not phase1_transport, not flight evidence')
    (a.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print('SUMMARY '+json.dumps(summary), flush=True)
    raise SystemExit(0 if summary['result'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
