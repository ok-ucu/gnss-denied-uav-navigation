import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

class GroundTruthRecorder(Node):
    def __init__(self):
        super().__init__('gt_recorder')
        self.subscription = self.create_subscription(Odometry, '/model/camera_drone/odometry', self.listener_callback, 10)
        
        os.makedirs('/work/recordings/sample_flight/mav0/state_groundtruth_estimate0', exist_ok=True)
        self.csv_file = open('/work/recordings/sample_flight/mav0/state_groundtruth_estimate0/data.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['#timestamp', 'p_RS_R_x [m]', 'p_RS_R_y [m]', 'p_RS_R_z [m]', 'q_RS_w []', 'q_RS_x []', 'q_RS_y []', 'q_RS_z []'])

    def listener_callback(self, msg):
        ts = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.writer.writerow([ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z])

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
