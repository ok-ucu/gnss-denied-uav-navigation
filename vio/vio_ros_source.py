"""Джерело кадрів із ROS 2 (Gazebo): підписка на sensor_msgs/Image.

Працює у фоновому потоці, тримає лише НАЙНОВІШИЙ кадр (старі відкидаються,
щоб не накопичувати затримку). Без cv_bridge: перетворення в numpy робимо самі,
щоб не залежати від версії NumPy, з якою зібрано cv_bridge.

Час кадру - header.stamp (час симуляції), а не момент прибуття: так dt для
оптичного потоку правильний навіть коли мережа/ROS відстає."""
import threading
import time

import cv2
import numpy as np


def image_to_gray(msg):
    """sensor_msgs/Image -> uint8 grayscale (None, якщо кодування не підтримано)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("mono8", "8uc1"):
        return buf.reshape(msg.height, msg.step)[:, :msg.width].copy()
    chans = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(enc)
    if chans is None:
        return None
    img = buf.reshape(msg.height, msg.step)[:, :msg.width * chans]
    img = img.reshape(msg.height, msg.width, chans)
    code = {"rgb8": cv2.COLOR_RGB2GRAY, "bgr8": cv2.COLOR_BGR2GRAY,
            "rgba8": cv2.COLOR_RGBA2GRAY, "bgra8": cv2.COLOR_BGRA2GRAY}[enc]
    return cv2.cvtColor(np.ascontiguousarray(img), code)


class RosFrameSource:
    def __init__(self, topic, node_name="vio_frontend", use_sim_time=False):
        import rclpy                                   # ліниво: без ROS решта працює
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        overrides = []
        if use_sim_time:        # годинник вузла = /clock із Gazebo, тоді вік кадра має сенс
            from rclpy.parameter import Parameter
            overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        self.node = rclpy.create_node(node_name, parameter_overrides=overrides)
        self.node.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)
        self._lock = threading.Lock()
        self._latest = None
        self.n_frames = 0
        self.n_unsupported = 0
        self.n_bad_age = 0
        self._thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._thread.start()

    def _cb(self, msg):
        gray = image_to_gray(msg)
        if gray is None:
            self.n_unsupported += 1
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # скільки секунд пройшло між формуванням кадра (stamp) і прийомом вузлом;
        # має сенс лише коли годинник вузла збігається з джерелом stamp
        age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
        if not (0.0 <= age < 1.0):
            age = None
            self.n_bad_age += 1
        with self._lock:
            self._latest = (gray, stamp, time.time(), age)
            self.n_frames += 1

    def poll(self):
        """Повертає (gray, t_stamp, t_wall, age_s|None) один раз для кожного нового кадра."""
        with self._lock:
            item, self._latest = self._latest, None
        return item

    def close(self):
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
