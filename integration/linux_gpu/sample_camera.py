"""Passive bounded native camera/resource witness. No control or ROS publishers."""
import json
import sys
import time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

duration = float(sys.argv[1])
prefix = Path(sys.argv[2])
rows = []
resources = []
node = Node()
topic = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'

def image(msg):
    rows.append(dict(host_ns=time.monotonic_ns(), sim_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nsec,
                     width=msg.width, height=msg.height, pixel_format=int(msg.pixel_format_type),
                     bytes=len(msg.data)))
    if len(rows) == 5:
        prefix.with_suffix('.ppm').write_bytes(b'P6\n640 480\n255\n'+msg.data)

def read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None

assert node.subscribe(Image, topic, image)
started = time.monotonic_ns()
while (time.monotonic_ns()-started)/1e9 < duration:
    tasks = []
    for task in Path('/proc/1/task').iterdir():
        tasks.append(dict(tid=int(task.name), name=read(task/'comm'), sched=read(task/'schedstat'),
                          wchan=read(task/'wchan')))
    resources.append(dict(host_ns=time.monotonic_ns(), cpu=read(Path('/sys/fs/cgroup/cpu.stat')),
                          pressure=read(Path('/sys/fs/cgroup/cpu.pressure')), tasks=tasks))
    time.sleep(.05)
ended = time.monotonic_ns()
node.unsubscribe(topic)
prefix.with_suffix('.json').write_text(json.dumps(dict(start_ns=started, end_ns=ended, images=rows, resources=resources)))
print(json.dumps(dict(start_ns=started, end_ns=ended, frames=len(rows))), flush=True)
