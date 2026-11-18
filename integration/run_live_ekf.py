"""Disarmed synthetic ExternalNav covariance experiment; no flight commands.

Uses SIMSTATE truth horizontally and attitude. Z=0 and Vz=0 are an explicit
stationary-ground model, not measured camera VIO. Bounded deterministic drift
is applied with its analytic derivative. Existing experiment/monitor are reused.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time

from test_ekf_covariance import Measurement, connection, experiment, wait_autopilot


def synthetic_drift(elapsed):
    """Position error and its derivative; repeats identically every 5 seconds."""
    omega = 2*math.pi/5
    return (.4*math.sin(omega*elapsed), .2*math.cos(omega*elapsed),
            .4*omega*math.cos(omega*elapsed), -.2*omega*math.sin(omega*elapsed))


class SyntheticGroundSource:
    def __init__(self, master, folder):
        self.master = master
        self.sim = None
        self.received = None
        self.stage = 'warmup'
        self.noise = False
        self.raw = open(folder/'received.jsonl','x')
        self.samples = open(folder/'synthetic_measurements.csv','x',newline='')
        self.writer = csv.writer(self.samples)
        self.writer.writerow(['unix_s','stage','elapsed_s','true_lat','true_lon',
                              'true_north_m','true_east_m','north_bias_m','east_bias_m',
                              'north_m','east_m','vnorth_m_s','veast_m_s','roll','pitch','yaw'])
        self.heartbeat_at = None

    def drain(self):
        for _ in range(500):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            if (msg.get_srcSystem(),msg.get_srcComponent()) != (
                    self.master.target_system,self.master.target_component):
                continue
            self.raw.write(json.dumps({'received_unix_s':time.time(),'message':msg.to_dict()})+'\n')
            if msg.get_type() == 'HEARTBEAT':
                self.heartbeat_at = time.monotonic()
                if msg.base_mode & 128:
                    raise RuntimeError('Ground scenario requires disarmed SITL')
            if msg.get_type() == 'SIMSTATE':
                self.sim, self.received = msg, time.monotonic()
        self.raw.flush()

    def sample(self, elapsed, now):
        self.drain()
        if self.heartbeat_at is None or now-self.heartbeat_at > 3:
            return None
        if self.sim is None or now-self.received > .3:
            return None
        msg = self.sim
        lat, lon = msg.lat*1e-7, msg.lng*1e-7
        north = (lat+35.363261)*111319.49
        east = (lon-149.165230)*111319.49*math.cos(math.radians(-35.363261))
        if abs(north) > .2 or abs(east) > .2 or abs(msg.roll) > .1 or abs(msg.pitch) > .1:
            raise RuntimeError('SITL is not stationary at configured home')
        dn,de,vn,ve = synthetic_drift(elapsed) if self.noise else (0.,0.,0.,0.)
        timestamp = time.time()
        self.writer.writerow([timestamp,self.stage,elapsed,lat,lon,north,east,dn,de,
                              north+dn,east+de,vn,ve,msg.roll,msg.pitch,msg.yaw])
        self.samples.flush()
        return Measurement(int(timestamp*1e6),(north+dn,east+de,0.),
                           (msg.roll,msg.pitch,msg.yaw),(vn,ve,0.))

    def close(self):
        self.raw.close()
        self.samples.close()


def parameter(master, name, value=None):
    deadline = time.monotonic()+5
    next_request = 0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_request:
            if value is None:
                master.mav.param_request_read_send(master.target_system,master.target_component,
                                                  name.encode(),-1)
            else:
                master.mav.param_set_send(master.target_system,master.target_component,
                                         name.encode(),float(value),9)
            next_request = time.monotonic()+.5
        msg = master.recv_match(type='PARAM_VALUE',blocking=True,timeout=.2)
        if msg is not None:
            key = msg.param_id.decode() if isinstance(msg.param_id,bytes) else msg.param_id
            if key.rstrip('\x00') == name and (value is None or abs(msg.param_value-value)<1e-4):
                return msg.param_value
    raise TimeoutError(f'Parameter {name} read/set not acknowledged')


def main():
    from pymavlink import mavutil
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--conn',default='udpin:0.0.0.0:14556')
    args = parser.parse_args()
    folder = Path(args.output)
    folder.mkdir(parents=True,exist_ok=True)
    if any((folder/name).exists() for name in ('parameters.json','received.jsonl',
                'synthetic_measurements.csv','warmup.csv','generator.csv')):
        raise FileExistsError('Use a new experiment directory; existing results are protected')
    master = connection(mavutil,args.conn)
    source = None
    try:
        wait_autopilot(master,15)
        before, after = {}, {}
        for name in ('GPS_TYPE','AHRS_EKF_TYPE','EK3_ENABLE','EK3_SRC1_POSXY',
                     'EK3_SRC1_VELXY','EK3_SRC1_POSZ','EK3_SRC1_VELZ','EK3_SRC1_YAW',
                     'VISO_TYPE','VISO_POS_M_NSE','VISO_VEL_M_NSE','VISO_YAW_M_NSE',
                     'LOG_DISARMED','LOG_BITMASK','LOG_BACKEND_TYPE','EK3_LOG_LEVEL'):
            before[name] = parameter(master,name)
        if before['GPS_TYPE'] != 0 or before['EK3_SRC1_POSXY'] != 6 or before['EK3_SRC1_VELXY'] != 6:
            raise RuntimeError('Unexpected GPS/ExternalNav configuration')
        after.update(before)
        for name,value in {'LOG_DISARMED':1,'LOG_BITMASK':65535}.items():
            after[name] = parameter(master,name,value)
        (folder/'parameters.json').write_text(json.dumps({'before':before,'during':after},indent=2))
        master.mav.set_gps_global_origin_send(master.target_system,
                     int(-35.363261*1e7),int(149.165230*1e7),584000)
        for msg_id,hz in ((164,30),(30,10),(32,10),(193,10),(0,1),(24,1),(33,5)):
            master.mav.command_long_send(master.target_system,master.target_component,
                                         511,0,msg_id,1e6/hz,0,0,0,0,0)
        source = SyntheticGroundSource(master,folder)
        with open(folder/'warmup.csv','x',newline='') as stream:
            experiment(master,source,stream,[('normal',15)],rate=20,
                       normal=.01,source_name='synthetic_SIMSTATE_ground_warmup')
        source.noise = True
        source.stage = 'experiment'
        with open(folder/'generator.csv','x',newline='') as stream:
            experiment(master,source,stream,[('normal',20),('inflated',20),
                       ('dropout',12),('recovery',20)],rate=20,normal=.01,inflated=4,
                       source_name='synthetic_SIMSTATE_ground_plus_drift')
        # Receive post-recovery status without producing more measurements.
        until = time.monotonic()+2
        while time.monotonic() < until:
            source.drain()
            time.sleep(.01)
    finally:
        if source is not None:
            source.close()
        master.close()


if __name__ == '__main__':
    main()
