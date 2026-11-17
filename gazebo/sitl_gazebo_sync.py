#!/usr/bin/env python3
"""Forward ArduPilot LOCAL_POSITION_NED velocities to a ROS 2 Twist topic.

This is velocity replay, not an exact position/attitude synchronization.
Requires an independently running ros_gz_bridge for ROS -> Gazebo Twist.
"""
import argparse
import math
import sys
import time

TOPIC = '/model/camera_drone/cmd_vel'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conn', default='udpin:0.0.0.0:14551')
    parser.add_argument('--seconds', type=float, default=0.0)
    parser.add_argument('--mock', action='store_true', help='explicit synthetic commands, not SITL')
    parser.add_argument('--heartbeat-timeout', type=float, default=15.0)
    parser.add_argument('--data-timeout', type=float, default=3.0)
    args = parser.parse_args()
    if args.seconds < 0 or args.data_timeout <= 0 or args.heartbeat_timeout <= 0:
        parser.error('timeouts must be positive and --seconds nonnegative')

    try:
        import rclpy
        from geometry_msgs.msg import Twist
    except ImportError as exc:
        print(f'ERROR: ROS 2 Python dependencies unavailable: {exc}', file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node('sitl_gazebo_sync')
    pub = node.create_publisher(Twist, TOPIC, 10)

    def publish(vx=0.0, vy=0.0, vz=0.0):
        # LOCAL_POSITION_NED is world-frame NED. Gazebo command here is world-frame ENU.
        msg = Twist()
        msg.linear.x = float(vy)
        msg.linear.y = float(vx)
        msg.linear.z = -float(vz)
        pub.publish(msg)

    try:
        if args.mock:
            print('MOCK MODE: synthetic Twist commands; this does NOT validate SITL connectivity.', flush=True)
            until = time.monotonic() + (args.seconds or 5.0)
            while time.monotonic() < until and rclpy.ok():
                publish(vx=1.0)
                rclpy.spin_once(node, timeout_sec=0.05)
            print('MOCK commands published (Gazebo motion not independently verified).')
            return 0

        try:
            from pymavlink import mavutil
        except ImportError as exc:
            print(f'ERROR: pymavlink unavailable: {exc}; no MOCK fallback', file=sys.stderr)
            return 1

        print(f'Waiting for real MAVLink heartbeat on {args.conn}...', flush=True)
        mav = mavutil.mavlink_connection(args.conn)
        try:
            hb = mav.wait_heartbeat(timeout=args.heartbeat_timeout)
            if hb is None:
                raise TimeoutError('heartbeat timeout')
        except Exception as exc:
            print(f'ERROR: no SITL heartbeat: {exc}', file=sys.stderr)
            return 1
        print('MAVLink heartbeat received; waiting for LOCAL_POSITION_NED...', flush=True)
        # Request telemetry explicitly: SITL may not stream it by default.
        for message_id, interval_us in (
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50000),
        ):
            mav.mav.command_long_send(
                mav.target_system,
                mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0, 0, 0, 0, 0,
            )

        print('Requested LOCAL_POSITION_NED at 10 Hz and ATTITUDE at 20 Hz.', flush=True)
        start = time.monotonic()
        last_data = start
        got_data = False
        count = 0
        while rclpy.ok() and (args.seconds == 0 or time.monotonic() - start < args.seconds):
            rclpy.spin_once(node, timeout_sec=0.01)
            packet = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
            now = time.monotonic()
            if packet is not None:
                values = (packet.vx, packet.vy, packet.vz)
                if not all(math.isfinite(v) for v in values):
                    print('ERROR: invalid velocity from SITL', file=sys.stderr)
                    return 1
                publish(*values)
                last_data = now
                got_data = True
                count += 1
                if count == 1 or count % 100 == 0:
                    print(f'LIVE LOCAL_POSITION_NED #{count}: NED velocity {values} m/s', flush=True)
            elif now - last_data > args.data_timeout:
                print('ERROR: LOCAL_POSITION_NED stream timed out; stopping model', file=sys.stderr)
                return 1
        if not got_data:
            print('ERROR: no LOCAL_POSITION_NED received', file=sys.stderr)
            return 1
        print(f'LIVE stream completed; {count} position messages forwarded. Gazebo movement still requires verification.')
        return 0
    except KeyboardInterrupt:
        print('Interrupted; stopping model')
        return 130
    finally:
        for _ in range(3):
            publish()
            rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
