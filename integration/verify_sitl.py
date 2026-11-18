"""Read-only live check of both macOS UDP endpoints and SITL parameters.

Requests telemetry intervals/parameter values only; no arm or flight commands.
Requires pymavlink. A passed check proves connectivity, not navigation accuracy.
"""
import argparse
import json
from pathlib import Path
import sys
import time

from test_ekf_covariance import connection

EXPECTED = {'EK3_SRC1_POSXY':6, 'EK3_SRC1_VELXY':6, 'EK3_SRC1_POSZ':1,
            'EK3_SRC1_VELZ':6, 'EK3_SRC1_YAW':1, 'VISO_TYPE':1,
            'GPS_TYPE':0, 'SERIAL0_PROTOCOL':2, 'AHRS_EKF_TYPE':3, 'EK3_ENABLE':1}


def verify(urls, timeout=20):
    from pymavlink import mavutil
    clients = []
    results = [dict(connection=url, heartbeats=0, mavlink2_heartbeats=0,
                    messages={}, parameters={}, armed=False) for url in urls]
    deadline = time.monotonic()+timeout
    targets = [None]*len(urls)
    last_request = [0.]*len(urls)
    try:
        for index, url in enumerate(urls):
            client = connection(mavutil, url)
            # Different GCS IDs make simultaneous links visible independently.
            client.mav.srcSystem = 240+index
            clients.append(client)
        while time.monotonic() < deadline:
            for index, client in enumerate(clients):
                result = results[index]
                # Bounded draining, so no link can starve the other.
                for _ in range(100):
                    msg = client.recv_match(blocking=False)
                    if msg is None:
                        break
                    kind = msg.get_type()
                    if kind == 'HEARTBEAT' and msg.autopilot == 3:
                        target = (msg.get_srcSystem(), msg.get_srcComponent())
                        if targets[index] is None:
                            targets[index] = target
                            client.target_system, client.target_component = target
                        if target == targets[index]:
                            result['heartbeats'] += 1
                            result['mavlink2_heartbeats'] += int(msg.get_msgbuf()[0] == 0xFD)
                            result['armed'] |= bool(msg.base_mode & 128)
                    if targets[index] != (msg.get_srcSystem(), msg.get_srcComponent()):
                        continue
                    result['messages'][kind] = result['messages'].get(kind,0)+1
                    if kind == 'PARAM_VALUE':
                        name = msg.param_id
                        if isinstance(name, bytes):
                            name = name.decode('ascii')
                        result['parameters'][name.rstrip('\x00')] = msg.param_value
                if targets[index] and time.monotonic()-last_request[index] >= 1:
                    for msg_id in (30,32,193):
                        client.mav.command_long_send(*targets[index],511,0,
                                                    msg_id,200000,0,0,0,0,0)
                    for name in EXPECTED:
                        if name not in result['parameters']:
                            client.mav.param_request_read_send(*targets[index],name.encode('ascii'),-1)
                    last_request[index] = time.monotonic()
            if all(r['mavlink2_heartbeats'] >= 2 and
                   all(r['parameters'].get(k) == v for k,v in EXPECTED.items()) and
                   all(r['messages'].get(k,0) > 0 for k in
                       ('ATTITUDE','EKF_STATUS_REPORT')) for r in results):
                break
            time.sleep(.01)
        for result in results:
            result['local_position_available'] = result['messages'].get('LOCAL_POSITION_NED',0) > 0
            result['passed'] = (result['mavlink2_heartbeats'] >= 2 and not result['armed'] and
                all(result['parameters'].get(k) == v for k,v in EXPECTED.items()) and
                all(result['messages'].get(k,0) > 0 for k in
                    ('ATTITUDE','EKF_STATUS_REPORT')))
        return results
    finally:
        for client in clients:
            client.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--covariance-conn',default='udpin:0.0.0.0:14556')
    parser.add_argument('--monitor-conn',default='udpin:0.0.0.0:14552')
    parser.add_argument('--timeout',type=float,default=20)
    parser.add_argument('--output',default='integration/sitl_verify.json')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('timeout must be positive')
    try:
        results = verify([args.covariance_conn,args.monitor_conn],args.timeout)
        report = dict(kind='live SITL connectivity and parameter check',results=results)
        Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
        return 0 if all(r['passed'] for r in results) else 1
    except (ImportError,OSError,ValueError) as exc:
        print(f'Verification failed: {exc}',file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
