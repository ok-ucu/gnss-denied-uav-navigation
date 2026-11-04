#!/usr/bin/env python3
"""Bounded real ROS image/clock probe. No ROS installation means FAIL, never mock PASS."""
import argparse
import json
import math
from pathlib import Path
import time


def evaluate(report):
    errors = []
    expected = {'/camera/image_raw': 'sensor_msgs/msg/Image', '/clock': 'rosgraph_msgs/msg/Clock'}
    for topic, kind in expected.items():
        if kind not in report['topic_types'].get(topic, []):
            errors.append(f'Missing/wrong topic type: {topic}')
    frames = report['frames']
    if not frames:
        errors.append('No real images received')
    elif any((v['width'], v['height']) != (640, 480) or
             v['encoding'] not in ('mono8', 'rgb8', 'bgr8') or not v['data_valid'] for v in frames):
        errors.append('Invalid image dimensions/encoding/data length')
    if len(frames) / report['duration_s'] < 20:
        errors.append('Image arrival rate below 20 Hz')
    stamps = [v['stamp_s'] for v in frames]
    if len(stamps) < 2 or stamps[-1] <= stamps[0] or any(b <= a for a, b in zip(stamps, stamps[1:])):
        errors.append('Image timestamps do not advance strictly')
    clocks = report['clock_stamps_s']
    if len(clocks) < 2 or clocks[-1] <= clocks[0] or any(b < a for a, b in zip(clocks, clocks[1:])):
        errors.append('Simulation clock does not advance monotonically')
    if not frames or not any(v['nonzero'] for v in frames):
        errors.append('No nonzero image content')
    report['failures'] = errors
    report['status'] = 'FAIL' if errors else 'PASS'
    report['image_hz_wall'] = len(frames) / report['duration_s']
    report['image_hz_stamp'] = ((len(stamps)-1)/(stamps[-1]-stamps[0])
                              if len(stamps) > 1 and stamps[-1] > stamps[0] else None)
    report['imu_hz_wall'] = len(report['imu_stamps_s']) / report['duration_s']
    report['scope'] = 'Static camera transport only; not VIO, physical SITL correspondence or capture latency'
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0 or not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error('Duration must be positive, warmup nonnegative; both finite')
    report = {'duration_s': args.duration, 'requested_duration_s': args.duration,
              'frames': [], 'clock_stamps_s': [],
              'imu_stamps_s': [], 'topic_types': {}}
    node = None
    initialized = False
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, Imu
        from rosgraph_msgs.msg import Clock
        rclpy.init()
        initialized = True
        node = rclpy.create_node('integration_camera_probe')
        active = False
        start = 0
        saved = False
        def stamp(value):
            return value.sec + value.nanosec * 1e-9
        def image(msg):
            nonlocal saved
            if not active:
                return
            channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}.get(msg.encoding, 0)
            valid = bool(channels and msg.step >= msg.width*channels and len(msg.data) == msg.height*msg.step)
            report['frames'].append(dict(arrival_s=time.monotonic()-start,
                width=msg.width, height=msg.height, encoding=msg.encoding, step=msg.step,
                stamp_s=stamp(msg.header.stamp), frame_id=msg.header.frame_id,
                data_valid=valid, nonzero=any(msg.data)))
            if not saved and valid:
                # Preserve actual bytes without image processing dependencies, including stride.
                args.output.with_name('first_frame.raw').write_bytes(bytes(msg.data))
                report['first_frame_byte_min'] = min(msg.data)
                report['first_frame_byte_max'] = max(msg.data)
                saved = True
        def clock(msg):
            if active:
                report['clock_stamps_s'].append(stamp(msg.clock))
        def imu(msg):
            if active:
                report['imu_stamps_s'].append(stamp(msg.header.stamp))
        node.create_subscription(Image, '/camera/image_raw', image, qos_profile_sensor_data)
        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu', imu, qos_profile_sensor_data)
        until = time.monotonic() + args.warmup
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        start = time.monotonic()
        active = True
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=.02)
        active = False
        report['duration_s'] = time.monotonic() - start
        report['topic_types'] = dict(node.get_topic_names_and_types())
        evaluate(report)
    except (ImportError, RuntimeError, OSError) as exc:
        report.update(status='FAIL', failures=[str(exc)])
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('frames', 'clock_stamps_s', 'imu_stamps_s')}, indent=2))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
