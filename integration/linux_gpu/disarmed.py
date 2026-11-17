"""Passive heartbeat check: receive bytes only; no MAVLink transmissions."""
import importlib.util
from pathlib import Path
import socket
import time

def check():
    path=Path(__file__).resolve().parents[1]/'mavros_live_runs/20261108/mavlink_v20.py'
    spec=importlib.util.spec_from_file_location('camera_only_wire',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parser=module.MAVLink(None)
    deadline=time.monotonic()+10
    with socket.create_connection(('127.0.0.1',5760),timeout=2) as connection:
        connection.settimeout(1)
        while time.monotonic()<deadline:
            try:data=connection.recv(65535)
            except socket.timeout:continue
            if not data:raise RuntimeError('FCU disconnected')
            for message in parser.parse_buffer(data) or []:
                if message.get_type()=='HEARTBEAT':
                    if message.base_mode & 128:raise RuntimeError('FCU ARMED: abort')
                    print('DISARMED heartbeat received',flush=True)
                    return
    raise RuntimeError('No DISARMED heartbeat')

if __name__=='__main__':check()
