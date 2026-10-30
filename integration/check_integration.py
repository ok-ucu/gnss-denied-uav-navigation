"""Streaming MAVLink diagnostics. No ROS dependency."""
import argparse
import csv
from pathlib import Path
import sys
import time

FIELDS = ['received_unix_s', 'elapsed_s', 'event', 'source_system',
          'source_component', 'mode', 'ekf_flags', 'ekf_pos_horiz_variance',
          'ekf_velocity_variance', 'north_m', 'east_m', 'down_m',
          'time_boot_ms', 'severity', 'text']
TYPES = ['HEARTBEAT', 'EKF_STATUS_REPORT', 'LOCAL_POSITION_NED', 'STATUSTEXT']


def monitor(master, stream, mode_string, telemetry_timeout=5.0, duration=None,
            clock=time.monotonic, wall=time.time):
    """master must already have an autopilot heartbeat; timeout uses monotonic time."""
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    stream.flush()
    start = last = clock()
    lost = False

    def emit(event, msg=None, **values):
        row = dict(received_unix_s=wall(), elapsed_s=clock()-start, event=event)
        if msg is not None:
            row.update(source_system=msg.get_srcSystem(),
                       source_component=msg.get_srcComponent())
        row.update(values)
        writer.writerow(row)
        stream.flush()

    emit('connected')
    while duration is None or clock()-start < duration:
        msg = master.recv_match(type=TYPES, blocking=True, timeout=0.2)
        now = clock()
        if msg is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == (
                master.target_system, master.target_component):
            if lost:
                emit('telemetry_restored')
            last, lost = now, False
            kind = msg.get_type()
            values = {}
            if kind == 'HEARTBEAT':
                values['mode'] = mode_string(msg)
            elif kind == 'EKF_STATUS_REPORT':
                values.update(ekf_flags=msg.flags,
                              ekf_pos_horiz_variance=msg.pos_horiz_variance,
                              ekf_velocity_variance=msg.velocity_variance)
            elif kind == 'LOCAL_POSITION_NED':
                values.update(north_m=msg.x, east_m=msg.y, down_m=msg.z,
                              time_boot_ms=msg.time_boot_ms)
            elif kind == 'STATUSTEXT':
                values.update(severity=msg.severity, text=msg.text)
            emit(kind, msg, **values)
        if now-last >= telemetry_timeout and not lost:
            emit('telemetry_lost', text='No selected autopilot telemetry')
            print('Telemetry lost', file=sys.stderr)
            lost = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conn', default='udpin:127.0.0.1:14552')
    parser.add_argument('--output', default=str(Path(__file__).with_name('integration_log.csv')))
    parser.add_argument('--connect-timeout', type=float, default=15)
    parser.add_argument('--telemetry-timeout', type=float, default=5)
    parser.add_argument('--duration', type=float)
    args = parser.parse_args()
    if min(args.connect_timeout, args.telemetry_timeout, args.duration or 1) <= 0:
        parser.error('timeouts and duration must be positive')
    master = None
    try:
        from pymavlink import mavutil
        master = mavutil.mavlink_connection(args.conn)
        deadline = time.monotonic() + args.connect_timeout
        while True:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError('No autopilot heartbeat before connection timeout')
            heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=remaining)
            if heartbeat is not None and heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                master.target_system = heartbeat.get_srcSystem()
                master.target_component = heartbeat.get_srcComponent()
                break
        with open(args.output, 'w', newline='') as stream:
            for message_id in (193, 32):
                master.mav.command_long_send(master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    message_id, 200000, 0, 0, 0, 0, 0)
            monitor(master, stream, mavutil.mode_string_v10,
                    args.telemetry_timeout, args.duration)
        return 0
    except KeyboardInterrupt:
        return 0
    except (ImportError, OSError, TimeoutError, ValueError) as exc:
        print(f'Monitor failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if master is not None:
            master.close()


if __name__ == '__main__':
    sys.exit(main())
