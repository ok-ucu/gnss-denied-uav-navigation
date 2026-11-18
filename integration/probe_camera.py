#!/usr/bin/env python3
"""Bounded real ROS image/clock probe. No ROS installation means FAIL, never mock PASS."""
import argparse
import bisect
import json
import math
from pathlib import Path
import time


def timing_checks(report):
    """Compare latest received /clock with image stamp only in a declared domain.

    This bounds observed stamp lag/skew, not capture latency or synchronization
    accuracy. Arrival gaps use monotonic wall time, independent of sim speed.
    """
    frames = report['frames']
    clocks = report.get('clock_samples', [])
    gap_limit = report.get('max_arrival_gap_s', .2)
    max_age = report.get('max_stamp_age_s', .2)
    future = report.get('future_tolerance_s', .05)
    if not all(math.isfinite(v) and v > 0 for v in (gap_limit, max_age, future)):
        raise ValueError('Timing thresholds must be finite and positive')
    gap = {'status': 'NOT TESTED', 'reason': 'No arrival records for both streams'}
    available = all(rows and all('arrival_s' in r for r in rows) for rows in (frames, clocks))
    if available:
        maximum = {}
        valid = True
        for name, rows in [('image', frames), ('clock', clocks)]:
            times = [r['arrival_s'] for r in rows]
            if (not all(math.isfinite(t) and 0 <= t <= report['duration_s'] for t in times)
                    or any(b < a for a,b in zip(times,times[1:]))):
                valid = False
            maximum[name] = max([times[0]] + [b-a for a,b in zip(times,times[1:])]
                                + [report['duration_s']-times[-1]])
        gap = {'status': 'PASS' if valid and max(maximum.values()) <= gap_limit else 'FAIL',
               'max_gap_s': maximum, 'limit_s': gap_limit}
    consistency = {'status': 'NOT TESTED', 'reason': 'Shared clock domain not explicitly confirmed'}
    if report.get('shared_clock_domain'):
        consistency['reason'] = 'Missing comparable image/clock arrival records'
        if available:
            arrivals = [r['arrival_s'] for r in clocks]
            stamps = [r['stamp_s'] for r in clocks]
            image_stamps = [f['stamp_s'] for f in frames]
            valid = (all(math.isfinite(t) for t in arrivals+stamps)
                     and all(b >= a for a,b in zip(arrivals,arrivals[1:]))
                     and all(b >= a for a,b in zip(stamps,stamps[1:]))
                     and all(math.isfinite(t) for t in image_stamps)
                     and all(b > a for a,b in zip(image_stamps,image_stamps[1:])))
            violations = 0
            ages = []
            unpaired = 0
            for f in frames:
                if not math.isfinite(f['arrival_s']) or not math.isfinite(f['stamp_s']):
                    violations += 1
                    continue
                i = bisect.bisect_right(arrivals, f['arrival_s'])-1 if valid else -1
                if i < 0:
                    unpaired += 1
                    continue
                age = clocks[i]['stamp_s']-f['stamp_s']
                ages.append(age)
                if (not math.isfinite(age) or age > max_age or age < -future
                        or f['arrival_s']-clocks[i]['arrival_s'] > gap_limit):
                    violations += 1
            consistency = dict(status='FAIL' if not valid or violations else 'NOT TESTED' if unpaired or not ages else 'PASS',
                compared_frames=len(ages), unpaired_frames=unpaired, violations=violations,
                max_stamp_age_s=max_age, future_tolerance_s=future,
                scope='Observed stamp consistency, not capture latency or proven synchronization')
            if consistency['status'] == 'NOT TESTED':
                consistency['reason'] = 'No preceding /clock sample for every image'
    return gap, consistency


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
    if not all(math.isfinite(t) for t in stamps) or len(stamps) < 2 or stamps[-1] <= stamps[0] or any(b <= a for a, b in zip(stamps, stamps[1:])):
        errors.append('Image timestamps do not advance strictly')
    clocks = report['clock_stamps_s']
    if not all(math.isfinite(t) for t in clocks) or len(clocks) < 2 or clocks[-1] <= clocks[0] or any(b < a for a, b in zip(clocks, clocks[1:])):
        errors.append('Simulation clock does not advance monotonically')
    if not frames or not any(v['nonzero'] for v in frames):
        errors.append('No nonzero image content')
    gaps, consistency = timing_checks(report)
    report['arrival_gaps'] = gaps
    report['clock_consistency'] = consistency
    if gaps['status'] == 'FAIL':
        errors.append('Arrival gap exceeds limit or arrival timestamps are invalid')
    if consistency['status'] == 'FAIL':
        errors.append('Camera stamps inconsistent with declared shared /clock')
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
    parser.add_argument('--shared-clock', action='store_true',
                        help='Operator confirms image stamps and /clock share the same simulation domain')
    parser.add_argument('--max-stamp-age', type=float, default=.2)
    parser.add_argument('--future-tolerance', type=float, default=.05)
    parser.add_argument('--max-arrival-gap', type=float, default=.2)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0 or not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error('Duration must be positive, warmup nonnegative; both finite')
    if not all(math.isfinite(v) and v > 0 for v in (args.max_stamp_age, args.future_tolerance, args.max_arrival_gap)):
        parser.error('Timing thresholds must be finite and positive')
    report = {'shared_clock_domain': args.shared_clock,
              'max_stamp_age_s': args.max_stamp_age, 'future_tolerance_s': args.future_tolerance,
              'max_arrival_gap_s': args.max_arrival_gap, 'clock_samples': [],
              'duration_s': args.duration, 'requested_duration_s': args.duration,
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
                report['clock_samples'].append(dict(arrival_s=time.monotonic()-start,
                                                    stamp_s=stamp(msg.clock)))
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
    print(json.dumps({k: v for k, v in report.items() if k not in ('frames', 'clock_stamps_s', 'imu_stamps_s', 'clock_samples')}, indent=2))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
