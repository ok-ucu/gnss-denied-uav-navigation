"""
/vio/odom   nav_msgs/Odometry
    header.frame_id = odom_ned        світ: NED (x - північ, y - схід, z - вниз)
    child_frame_id  = base_link_frd   тіло дрона: FRD (x - вперед, y - вправо, z - вниз)
    header.stamp    = годинник вузла (при use_sim_time=true це час Gazebo)
    pose.position            - позиція в odom_ned (інтеграція швидкості з потоку + IMU)
    pose.orientation         - кватерніон body(FRD)->NED, береться з ATTITUDE автопілота
    twist.linear             - швидкість у child_frame (REP-105), тобто R^T * v_ned
    twist.angular            - НЕ оцінюється: значення 0, діагональ ковар. = -1
    pose.covariance          - 6x6 (x y z roll pitch yaw), у odom_ned, роздута за якістю
    twist.covariance         - 6x6, лінійний блок = R^T Pv R, кутовий = -1 (невідомо)
/vio/status std_msgs/String, JSON раз на секунду: джерело, якість, лічильники кадрів
"""
import json
import threading

import numpy as np


def rot_to_quat(R):
    """Матриця повороту -> кватерніон (x, y, z, w), w >= 0, одиничний."""
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def build_covariances(R, P_pos, P_vel, att_var):
    """Повертає (pose_cov36, twist_cov36) як списки по рядках.
    P_pos, P_vel - 3x3 у NED (уже роздуті), att_var - дисперсія орієнтації."""
    pose = np.zeros((6, 6))
    pose[0:3, 0:3] = P_pos
    pose[3:6, 3:6] = np.eye(3) * att_var
    twist = np.zeros((6, 6))
    twist[0:3, 0:3] = R.T @ P_vel @ R            # швидкість у осях тіла
    twist[3:6, 3:6] = np.diag([-1.0, -1.0, -1.0])  # кутова швидкість не оцінюється
    return pose.flatten().tolist(), twist.flatten().tolist()


class RosOdomPublisher:
    def __init__(self, node=None, topic="/vio/odom", status_topic="/vio/status",
                 frame_id="odom_ned", child_frame_id="base_link_frd", use_sim_time=False):
        import rclpy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        self._rclpy = rclpy
        self._own = node is None
        if self._own:
            if not rclpy.ok():
                rclpy.init()
            overrides = []
            if use_sim_time:
                from rclpy.parameter import Parameter
                overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
            node = rclpy.create_node("vio_output", parameter_overrides=overrides)
            threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        self.node = node
        self._Odometry, self._String = Odometry, String
        self.pub = node.create_publisher(Odometry, topic, 10)
        self.pub_status = node.create_publisher(String, status_topic, 10)
        self.frame_id, self.child_frame_id = frame_id, child_frame_id
        self.n_pub = 0

    def publish(self, R, p, v_ned, P_pos, P_vel, att_var):
        m = self._Odometry()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.child_frame_id = self.child_frame_id
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, p)
        qx, qy, qz, qw = rot_to_quat(R)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = float(qx), float(qy), float(qz), float(qw)
        v_body = R.T @ np.asarray(v_ned, dtype=float)
        l = m.twist.twist.linear
        l.x, l.y, l.z = map(float, v_body)
        pc, tc = build_covariances(R, P_pos, P_vel, att_var)
        m.pose.covariance = pc
        m.twist.covariance = tc
        self.pub.publish(m)
        self.n_pub += 1

    def publish_status(self, **fields):
        s = self._String()
        s.data = json.dumps(fields, default=float)
        self.pub_status.publish(s)

    def close(self):
        if self._own:
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
