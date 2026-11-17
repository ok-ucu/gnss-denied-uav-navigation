import struct, zlib
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw))
    png += chunk(b"IEND", b"")
    open(path, "wb").write(png)


class Saver(Node):
    def __init__(self):
        super().__init__("save_frame")
        self.create_subscription(Image, "/camera/image_raw", self.cb, 10)

    def cb(self, m):
        write_png("frame.png", m.width, m.height, bytes(m.data))
        self.get_logger().info(f"saved frame.png {m.width}x{m.height} {m.encoding}")
        raise SystemExit


rclpy.init()
try:
    rclpy.spin(Saver())
except SystemExit:
    pass