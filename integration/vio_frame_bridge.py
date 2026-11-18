"""ROS 2 bridge: VIO NED/FRD Odometry -> ENU/FLU Odometry.

Known angular velocity is transformed FRD -> FLU. Legacy unknown angular
velocity is replaced by zero with a configurable, large
uncertainty to satisfy a downstream adapter requiring a PSD 6x6 covariance.
This is an explicit compatibility approximation, NOT a measured angular rate.
"""
import copy
import math
import time
import json

N = ((0., 1., 0.), (1., 0., 0.), (0., 0., -1.))  # NED -> ENU
B = ((1., 0., 0.), (0., -1., 0.), (0., 0., -1.))  # FRD -> FLU


def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a):
    return [list(row) for row in zip(*a)]


def quat_matrix(q):
    x, y, z, w = q
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def matrix_quat(r):
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        w = s / 4
        x, y, z = (r[2][1]-r[1][2])/s, (r[0][2]-r[2][0])/s, (r[1][0]-r[0][1])/s
    else:
        i = max(range(3), key=lambda k: r[k][k])
        j, k = (i+1)%3, (i+2)%3
        s = 2 * math.sqrt(max(0., 1+r[i][i]-r[j][j]-r[k][k]))
        if s == 0:
            raise ValueError('Invalid rotation')
        xyz = [0., 0., 0.]
        xyz[i] = s / 4
        w = (r[k][j]-r[j][k]) / s
        xyz[j] = (r[j][i]+r[i][j]) / s
        xyz[k] = (r[k][i]+r[i][k]) / s
        x, y, z = xyz
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    return (x/norm, y/norm, z/norm, w/norm)


def vec(a, v):
    return [sum(a[i][j]*v[j] for j in range(3)) for i in range(3)]


def cov_transform(cov, rot):
    if len(cov) != 36:
        raise ValueError('Expected 36 covariance entries')
    j = [[0.]*6 for _ in range(6)]
    for i in range(3):
        for k in range(3):
            j[i][k] = j[i+3][k+3] = rot[i][k]
    c = [list(cov[i*6:(i+1)*6]) for i in range(6)]
    out = [[sum(j[i][a]*c[a][b]*j[k][b] for a in range(6) for b in range(6))
            for k in range(6)] for i in range(6)]
    return [v for row in out for v in row]


def convert(msg, angular_variance=100.0):
    """Return converted copy of nav_msgs/Odometry (also works with duck types)."""
    if (msg.header.frame_id, msg.child_frame_id) != ('odom_ned', 'base_link_frd'):
        raise ValueError('Expected odom_ned / base_link_frd')
    if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
        raise ValueError('Malformed acquisition timestamp')
    if not math.isfinite(angular_variance) or angular_variance <= 0:
        raise ValueError('angular_variance must be positive and finite')
    from vio_quality_gate import covariance_ok
    fields = [getattr(vector, axis) for vector in
              (msg.pose.pose.position, msg.twist.twist.linear, msg.twist.twist.angular)
              for axis in ('x', 'y', 'z')]
    if not all(math.isfinite(x) for x in fields):
        raise ValueError('Nonfinite position/twist')
    if not covariance_ok([list(msg.pose.covariance[i*6:(i+1)*6]) for i in range(6)], 6, math.inf):
        raise ValueError('Invalid pose covariance')
    out = copy.deepcopy(msg)
    p = msg.pose.pose.position
    xyz = vec(N, [p.x, p.y, p.z])
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = xyz
    q = msg.pose.pose.orientation
    qv = [q.x, q.y, q.z, q.w]
    if not all(math.isfinite(x) for x in qv) or abs(sum(x*x for x in qv)-1) > 1e-3:
        raise ValueError('Invalid input quaternion')
    norm = math.sqrt(sum(x*x for x in qv))
    qv = [x/norm for x in qv]
    r = mm(mm(N, quat_matrix(qv)), B)
    out_q = matrix_quat(r)
    (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
     out.pose.pose.orientation.z, out.pose.pose.orientation.w) = out_q
    out.pose.covariance = cov_transform(msg.pose.covariance, N)
    v = msg.twist.twist.linear
    (out.twist.twist.linear.x, out.twist.twist.linear.y,
     out.twist.twist.linear.z) = vec(B, [v.x, v.y, v.z])
    tc = list(msg.twist.covariance)
    if len(tc) != 36 or not all(math.isfinite(x) for x in tc):
        raise ValueError('Invalid twist covariance size/values')
    av = msg.twist.twist.angular
    (out.twist.twist.angular.x, out.twist.twist.angular.y,
     out.twist.twist.angular.z) = vec(B, [av.x, av.y, av.z])
    # Source angular velocity is explicitly unknown (-1 variance).
    # We cannot reconstruct it; use zero rate and a high variance for compatibility.
    if any(tc[i*6+i] < 0 for i in (3, 4, 5)):
        if not all(tc[i*6+i] == -1 for i in (3, 4, 5)):
            raise ValueError('Unexpected partial unknown angular covariance')
        for i in range(6):
            for k in range(6):
                if i >= 3 or k >= 3:
                    tc[i*6+k] = angular_variance if i == k and i >= 3 else 0.
        out.twist.twist.angular.x = 0.
        out.twist.twist.angular.y = 0.
        out.twist.twist.angular.z = 0.
    if not covariance_ok([tc[i*6:(i+1)*6] for i in range(6)], 6, math.inf):
        raise ValueError('Invalid twist covariance')
    out.twist.covariance = cov_transform(tc, B)
    out.header.frame_id = 'odom'
    out.child_frame_id = 'base_link'
    return out


class StreamGuard:
    """No timer publisher. Duplicate/reset/stale inputs fail closed.

    A simulation clock reset requires an explicit bridge restart/new epoch.
    Arrival freshness is wall monotonic; message age is simulation seconds.
    """
    def __init__(self, max_age=.2):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Invalid max_age')
        self.max_age = max_age
        self.last_stamp = None

    def check(self, stamp, now, wall, clock_wall):
        if clock_wall is None or not all(math.isfinite(x) for x in (stamp, now, wall, clock_wall)):
            raise ValueError('No comparable clock')
        if stamp <= 0 or now <= 0 or not 0 <= wall-clock_wall <= .5:
            raise ValueError('Unavailable/stalled clock')
        if not -1e-9 <= now-stamp <= self.max_age+1e-9:
            raise ValueError('Stale/future acquisition timestamp')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('Duplicate/out-of-order/reset timestamp')
        self.last_stamp = stamp


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    class Bridge(Node):
        def __init__(self):
            super().__init__('vio_frame_bridge')
            self.declare_parameter('angular_variance', 100.0)
            self.declare_parameter('input_topic', '/vio/odom')
            self.declare_parameter('output_topic', '/vio/odometry')
            self.declare_parameter('max_age_s', .2)
            self.declare_parameter('audit_log', '')
            self.guard = StreamGuard(self.get_parameter('max_age_s').value)
            self.clock_wall = None
            self.clock_sub = self.create_subscription(Clock, '/clock', self.on_clock, 4096)
            path = self.get_parameter('audit_log').value
            self.audit = open(path, 'x') if path else None
            self.pub = self.create_publisher(Odometry, self.get_parameter('output_topic').value, 100)
            self.sub = self.create_subscription(Odometry, self.get_parameter('input_topic').value, self.callback, 100)
            self.get_logger().info('VIO NED/FRD -> ENU/FLU bridge started')

        def on_clock(self, msg):
            self.clock_wall = time.monotonic()

        def callback(self, msg):
            row = dict(t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                       received_monotonic_s=time.monotonic(), published=False)
            try:
                self.guard.check(row['t'], self.get_clock().now().nanoseconds*1e-9,
                                 row['received_monotonic_s'], self.clock_wall)
                out = convert(msg, self.get_parameter('angular_variance').value)
                # Recheck age after conversion, retaining the original stamp.
                now = self.get_clock().now().nanoseconds*1e-9
                if not -1e-9 <= now-row['t'] <= self.guard.max_age+1e-9:
                    raise ValueError('Stale after conversion')
                row['publish_monotonic_s'] = time.monotonic()
                self.pub.publish(out)
                row['published'] = True
            except ValueError as e:
                row['reason'] = str(e)
                self.get_logger().warning(f'Dropped VIO odometry: {e}')
            if self.audit:
                self.audit.write(json.dumps(row, allow_nan=False)+'\n')
                self.audit.flush()

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.audit:
            node.audit.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
