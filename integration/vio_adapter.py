"""Odometry (ENU/FLU) -> MAVROS vision pose and speed with covariance.

Pure conversion is importable without ROS; ROS imports occur only in main.
"""
import copy
import math


def expand_upper_triangle(values):
    """MAVLink 21-element upper triangle -> ROS 36-element row-major matrix.

    Layout conversion only: this does NOT convert NED into ENU.
    """
    if len(values) != 21:
        raise ValueError('Expected 21 covariance values')
    result = [0.0] * 36
    k = 0
    for i in range(6):
        for j in range(i, 6):
            result[i*6+j] = result[j*6+i] = values[k]
            k += 1
    return result


def validate_covariance(cov, size=6):
    """Scale-normalized pivoted PSD check; numerical tolerance 1e-12.

    Singular PSD is valid. All-zero/unknown covariance is rejected by policy.
    """
    if len(cov) != size*size or not all(math.isfinite(x) for x in cov):
        raise ValueError(f'Covariance must contain {size*size} finite values')
    scale = max(abs(x) for x in cov)
    if scale == 0:
        raise ValueError('All-zero covariance is not a usable uncertainty estimate')
    matrix = [[cov[i*size+j]/scale for j in range(size)] for i in range(size)]
    tolerance = 1e-12
    for i in range(size):
        if cov[i*size+i] < 0:
            raise ValueError('Negative covariance diagonal')
        for j in range(i):
            if abs(matrix[i][j]-matrix[j][i]) > tolerance:
                raise ValueError('Covariance must be symmetric')
            matrix[i][j] = matrix[j][i] = (matrix[i][j]+matrix[j][i])/2
    # Pivot on the largest remaining diagonal, avoiding tiny leading pivots.
    for k in range(size):
        pivot = max(range(k, size), key=lambda i: matrix[i][i])
        matrix[k], matrix[pivot] = matrix[pivot], matrix[k]
        for row in matrix:
            row[k], row[pivot] = row[pivot], row[k]
        diagonal = matrix[k][k]
        if diagonal < -tolerance:
            raise ValueError('Covariance must be positive semidefinite')
        if diagonal <= tolerance:
            if any(abs(matrix[i][j]) > tolerance
                   for i in range(k, size) for j in range(k, size)):
                raise ValueError('Covariance must be positive semidefinite')
            return
        for i in range(k+1, size):
            for j in range(i, size):
                value = matrix[i][j] - matrix[i][k]*matrix[j][k]/diagonal
                matrix[i][j] = matrix[j][i] = value


def convert(odom, output_type, now_ns, last_ns=None, world='odom', body='base_link',
            max_age=0.5, future_tolerance=0.05):
    if odom.header.frame_id != world or odom.child_frame_id != body:
        raise ValueError('Expected configured ENU world and FLU body frames; no implicit TF')
    stamp = odom.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('Invalid ROS timestamp')
    ns = stamp.sec*1000000000 + stamp.nanosec
    if ns <= 0 or (last_ns is not None and ns <= last_ns):
        raise ValueError('Zero, repeated or out-of-order timestamp')
    age = (now_ns-ns)/1e9
    if age > max_age or age < -future_tolerance:
        raise ValueError('Stale/future pose: check shared ROS clock and use_sim_time')
    pose = odom.pose.pose
    q = pose.orientation
    values = [pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite pose')
    if abs(sum(v*v for v in values[3:])-1) > 1e-3:
        raise ValueError('Orientation quaternion must be normalized')
    validate_covariance(odom.pose.covariance)
    out = output_type()
    out.header = copy.deepcopy(odom.header)
    out.pose = copy.deepcopy(odom.pose)  # all 36 entries including cross terms
    return out, ns


def convert_speed(odom, output_type):
    """Validated Odometry body FLU twist -> world ENU twist.

    Call after convert() validates pose/frames/time. Rotation treats orientation
    as known; pose/twist cross-covariance is not available in Odometry.
    """
    if not hasattr(odom, 'twist'):
        raise ValueError('Missing VIO twist; velocity cannot be invented')
    cov = odom.twist.covariance
    validate_covariance(cov)
    linear_cov = [cov[i*6+j] for i in range(3) for j in range(3)]
    validate_covariance(linear_cov, 3)
    if any(linear_cov[i*3+i] <= 0 for i in range(3)):
        raise ValueError('Need measured XYZ velocity uncertainty, including vertical')
    linear, angular = odom.twist.twist.linear, odom.twist.twist.angular
    values = [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite twist')
    q = odom.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    jacobian = [[rotation[i % 3][j % 3] if i//3 == j//3 else 0.
                 for j in range(6)] for i in range(6)]
    rotated = [sum(jacobian[i][j]*values[j] for j in range(6)) for i in range(6)]
    rotated_cov = [sum(jacobian[i][a]*cov[a*6+b]*jacobian[j][b]
                       for a in range(6) for b in range(6))
                   for i in range(6) for j in range(6)]
    out = output_type()
    out.header = copy.deepcopy(odom.header)
    out.twist = copy.deepcopy(odom.twist)
    for vector, components in ((out.twist.twist.linear, rotated[:3]),
                               (out.twist.twist.angular, rotated[3:])):
        vector.x, vector.y, vector.z = components
    out.twist.covariance = rotated_cov
    return out


def convert_pair(odom, pose_type, speed_type, now_ns, last_ns=None, **kwargs):
    """Validate both messages before publishing either; no stale-data replay."""
    pose, ns = convert(odom, pose_type, now_ns, last_ns, **kwargs)
    speed = convert_speed(odom, speed_type)
    return pose, speed, ns


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

    class Adapter(Node):
        def __init__(self):
            super().__init__('vio_mavros_adapter')
            defaults = dict(input_topic='/vio/odometry',
                            output_topic='/mavros/vision_pose/pose_cov',
                            speed_topic='/mavros/vision_speed/speed_twist_cov',
                            world_frame='odom', body_frame='base_link', max_age=0.5,
                            future_tolerance=0.05)
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.last_ns = None
            self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                            self.value('output_topic'), 10)
            self.speed_pub = self.create_publisher(TwistWithCovarianceStamped,
                                                   self.value('speed_topic'), 10)
            self.sub = self.create_subscription(Odometry, self.value('input_topic'),
                                                 self.callback, qos_profile_sensor_data)

        def value(self, name):
            return self.get_parameter(name).value

        def callback(self, msg):
            try:
                out, speed, ns = convert_pair(msg, PoseWithCovarianceStamped,
                    TwistWithCovarianceStamped, self.get_clock().now().nanoseconds,
                    self.last_ns, world=self.value('world_frame'),
                    body=self.value('body_frame'), max_age=self.value('max_age'),
                    future_tolerance=self.value('future_tolerance'))
                self.pub.publish(out)
                self.speed_pub.publish(speed)
                self.last_ns = ns
            except ValueError as exc:
                self.get_logger().warning(str(exc), throttle_duration_sec=2)

    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
