"""Controlled MAVLink 2 ExternalNav experiment, not proof of EKF/VIO accuracy.

Telemetry source echoes EKF outputs (transport diagnostic). CSV source must be
aligned with SITL world, trajectory and experiment start by the operator.
"""
import argparse
import bisect
import csv
from dataclasses import dataclass
import importlib
import json
import math
from pathlib import Path
import sys
import time


def positive(value):
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Expected a finite positive number')
    return value


def pose_covariance(variance, attitude_variance=0.1):
    """21 upper-triangle values; m² position, rad² Euler attitude."""
    diagonal = [positive(variance)]*3 + [positive(attitude_variance)]*3
    return [diagonal[i] if i == j else 0.0
            for i in range(6) for j in range(i, 6)]


def speed_covariance(variance):
    """9 row-major values for world NED linear velocity, (m/s)²."""
    positive(variance)
    return [variance if i == j else 0.0 for i in range(3) for j in range(3)]


@dataclass(frozen=True)
class Measurement:
    usec: int
    position: tuple  # north, east, down [m]
    attitude: tuple  # roll, pitch, yaw [rad], body FRD -> NED
    velocity: tuple  # north, east, down [m/s], NOT body velocity

    def validate(self):
        if self.usec <= 0:
            raise ValueError('Measurement timestamp must be positive')
        for vector in (self.position, self.attitude, self.velocity):
            if len(vector) != 3 or not all(math.isfinite(x) for x in vector):
                raise ValueError('Measurement needs three finite values per vector')


class TelemetrySource:
    """Fresh LOCAL_POSITION_NED + ATTITUDE from the selected autopilot.

    Feedback of EKF output is NOT independent navigation ground truth.
    """
    def __init__(self, master, max_age=0.5, max_skew=0.05):
        self.master, self.max_age, self.max_skew = master, max_age, max_skew
        self.cache = {}

    def sample(self, elapsed, now):
        # Bound work even when the incoming transport has a large backlog.
        for _ in range(200):
            msg = self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=False)
            if msg is None:
                break
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (
                    self.master.target_system, self.master.target_component):
                continue
            kind = msg.get_type()
            previous = self.cache.get(kind)
            if previous is not None and msg.time_boot_ms < previous[0].time_boot_ms:
                raise ValueError('Autopilot clock reset/wrap: restart experiment')
            self.cache[kind] = (msg, now)
        if not all(kind in self.cache for kind in ('LOCAL_POSITION_NED', 'ATTITUDE')):
            return None
        position, received_p = self.cache['LOCAL_POSITION_NED']
        attitude, received_a = self.cache['ATTITUDE']
        if now-min(received_p, received_a) > self.max_age:
            return None
        if abs(position.time_boot_ms-attitude.time_boot_ms)/1000 > self.max_skew:
            return None
        return Measurement(position.time_boot_ms*1000,
                           (position.x, position.y, position.z),
                           (attitude.roll, attitude.pitch, attitude.yaw),
                           (position.vx, position.vy, position.vz))


class CsvSource:
    """Explicit timed replay, no interpolation or extrapolation across old data."""
    COLUMNS = ('north_m', 'east_m', 'down_m', 'roll_rad', 'pitch_rad', 'yaw_rad',
               'vnorth_m_s', 'veast_m_s', 'vdown_m_s')

    def __init__(self, path, max_age=0.5, start_unix=0):
        self.rows, self.times = [], []
        self.max_age, self.start_unix = max_age, start_unix
        with open(path, newline='') as stream:
            for row in csv.DictReader(stream):
                elapsed = float(row['elapsed_s'])
                values = tuple(float(row[k]) for k in self.COLUMNS)
                if not math.isfinite(elapsed) or elapsed < 0:
                    raise ValueError('CSV elapsed_s must be finite and nonnegative')
                if self.times and elapsed <= self.times[-1]:
                    raise ValueError('CSV times must strictly increase')
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('CSV measurement must be finite')
                self.times.append(elapsed)
                self.rows.append(values)
        if not self.rows:
            raise ValueError('CSV has no measurements')

    def sample(self, elapsed, now):
        index = bisect.bisect_right(self.times, elapsed)-1
        if index < 0 or elapsed-self.times[index] > self.max_age:
            return None
        values = self.rows[index]
        return Measurement(int((self.start_unix+self.times[index])*1e6),
                           values[:3], values[3:6], values[6:])


def phases(duration, overrides=None):
    overrides = overrides or {}
    return [(name, positive(duration if overrides.get(name) is None else overrides[name]))
            for name in ('normal', 'inflated', 'dropout', 'recovery')]


def wait_autopilot(master, timeout, clock=time.monotonic):
    deadline = clock()+positive(timeout)
    while True:
        remaining = deadline-clock()
        if remaining <= 0:
            raise TimeoutError('No autopilot heartbeat before timeout')
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=remaining)
        # MAV_AUTOPILOT_INVALID=8: do not select a GCS heartbeat.
        if heartbeat is not None and heartbeat.autopilot != 8:
            master.target_system = heartbeat.get_srcSystem()
            master.target_component = heartbeat.get_srcComponent()
            return


FIELDS = ['unix_time_s', 'elapsed_s', 'phase_elapsed_s', 'phase', 'event',
          'source', 'measurement_usec', 'position_ned', 'attitude_rpy', 'velocity_ned',
          'configured_variance', 'pose_covariance_sent', 'speed_covariance_sent',
          'position_sent', 'speed_sent', 'phase_position_count', 'phase_speed_count',
          'total_position_count', 'total_speed_count', 'detail']


def experiment(master, source, stream, schedule, rate=30, normal=0.01, inflated=4.,
               attitude_variance=0.1, source_name='telemetry',
               clock=time.monotonic, wall=time.time, sleep=time.sleep):
    """Counts reflect completed send calls, NOT acknowledged FCU acceptance."""
    positive(rate)
    for value in (normal, inflated, attitude_variance):
        positive(value)
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    stream.flush()
    start = clock()
    last_usec = None
    totals = [0, 0]
    phase_counts = [0, 0]
    phase, phase_start = '', start
    variance = None

    def emit(event, **values):
        row = dict(unix_time_s=wall(), elapsed_s=clock()-start,
                   phase_elapsed_s=clock()-phase_start, phase=phase, event=event,
                   source=source_name, configured_variance=variance,
                   phase_position_count=phase_counts[0], phase_speed_count=phase_counts[1],
                   total_position_count=totals[0], total_speed_count=totals[1])
        row.update(values)
        writer.writerow(row)
        stream.flush()

    try:
        for phase, duration in schedule:
            positive(duration)
            variance = None if phase == 'dropout' else inflated if phase == 'inflated' else normal
            phase_counts = [0, 0]
            phase_start = clock()
            emit('phase_start')
            next_send = phase_start
            while clock()-phase_start < duration:
                now = clock()
                if now < next_send:
                    sleep(min(next_send-now, 0.01))
                    continue
                # Poll during dropout too, so recovery never replays cached old input.
                measurement = source.sample(now-start, now)
                row = dict(position_sent=0, speed_sent=0)
                if variance is None:
                    emit('dropout', **row)
                elif measurement is None:
                    emit('no_fresh_measurement', **row)
                else:
                    measurement.validate()
                    row.update(measurement_usec=measurement.usec,
                               position_ned=json.dumps(measurement.position),
                               attitude_rpy=json.dumps(measurement.attitude),
                               velocity_ned=json.dumps(measurement.velocity))
                    if last_usec is not None and measurement.usec <= last_usec:
                        emit('duplicate_or_old_measurement', **row)
                    else:
                        pose_cov = pose_covariance(variance, attitude_variance)
                        speed_cov = speed_covariance(variance)
                        try:
                            master.mav.vision_position_estimate_send(
                                measurement.usec, *measurement.position, *measurement.attitude,
                                covariance=pose_cov, reset_counter=0)
                            totals[0] += 1
                            phase_counts[0] += 1
                            row.update(position_sent=1, pose_covariance_sent=json.dumps(pose_cov))
                            master.mav.vision_speed_estimate_send(
                                measurement.usec, *measurement.velocity,
                                covariance=speed_cov, reset_counter=0)
                            totals[1] += 1
                            phase_counts[1] += 1
                            row.update(speed_sent=1, speed_covariance_sent=json.dumps(speed_cov))
                        except Exception as exc:
                            emit('send_error', detail=str(exc), **row)
                            raise
                        last_usec = measurement.usec
                        emit('sent', **row)
                # Skip missed slots instead of sending a catch-up burst.
                next_send += (math.floor(max(0, clock()-next_send)*rate)+1)/rate
            emit('phase_end')
            if phase != 'dropout' and not min(phase_counts):
                raise RuntimeError(f'No measurement pair sent in {phase} phase')
        emit('complete')
    except KeyboardInterrupt:
        emit('interrupted')
        raise
    except Exception as exc:
        emit('error', detail=str(exc))
        raise
    return tuple(totals)


def connection(mavutil, url):
    # Explicit v2 dialect even if another module already imported mavutil/v1.
    dialect = importlib.import_module('pymavlink.dialects.v20.ardupilotmega')
    master = mavutil.mavlink_connection(url, source_system=245, source_component=191)
    try:
        master.mav = dialect.MAVLink(master, srcSystem=245, srcComponent=191)
        master.mav.robust_parsing = True
        master.WIRE_PROTOCOL_VERSION = '2.0'
        return master
    except Exception:
        master.close()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conn', default='udpin:127.0.0.1:14556')
    parser.add_argument('--duration', type=float, default=10., help='Default seconds per phase')
    for name in ('normal', 'inflated', 'dropout', 'recovery'):
        parser.add_argument(f'--{name}-duration', type=float)
    parser.add_argument('--rate', type=float, default=30.)
    parser.add_argument('--normal-variance', type=float, default=.01)
    parser.add_argument('--inflated-variance', type=float, default=4.)
    parser.add_argument('--attitude-variance', type=float, default=.1)
    parser.add_argument('--heartbeat-timeout', type=float, default=15.)
    parser.add_argument('--max-age', type=float, default=.5)
    parser.add_argument('--max-skew', type=float, default=.05)
    parser.add_argument('--source', choices=['telemetry', 'csv'], default='telemetry')
    parser.add_argument('--measurements', help='CSV trajectory aligned to experiment start')
    parser.add_argument('--output', default='integration/covariance_test.csv')
    args = parser.parse_args()
    try:
        for key, value in vars(args).items():
            if isinstance(value, float):
                positive(value)
        schedule = phases(args.duration, {name:getattr(args, name+'_duration')
                           for name in ('normal', 'inflated', 'dropout', 'recovery')})
        if args.inflated_variance <= args.normal_variance:
            raise ValueError('Inflated variance must exceed normal variance')
        if (args.source == 'csv') != bool(args.measurements):
            raise ValueError('--source csv requires --measurements; telemetry must not use it')
    except ValueError as exc:
        parser.error(str(exc))
    master = None
    try:
        # Parse external data before opening network transport.
        source = CsvSource(args.measurements, args.max_age) if args.source == 'csv' else None
        from pymavlink import mavutil
        master = connection(mavutil, args.conn)
        wait_autopilot(master, args.heartbeat_timeout)
        if source is None:
            source = TelemetrySource(master, args.max_age, args.max_skew)
            for message_id in (30, 32):
                master.mav.command_long_send(master.target_system, master.target_component,
                    511, 0, message_id, 1e6/args.rate, 0, 0, 0, 0, 0)
        print('Synthetic/loopback experiment; send success does not prove EKF fusion.')
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, 'w', newline='') as stream:
            if isinstance(source, CsvSource):
                source.start_unix = time.time()
            counts = experiment(master, source, stream, schedule, args.rate,
                args.normal_variance, args.inflated_variance, args.attitude_variance, args.source)
        print(f'Completed send calls: position={counts[0]}, speed={counts[1]}; log: {args.output}')
        return 0
    except KeyboardInterrupt:
        print('Experiment interrupted', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'Experiment failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if master is not None:
            try:
                master.close()
            except Exception as exc:
                print(f'Connection close failed: {exc}', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())
