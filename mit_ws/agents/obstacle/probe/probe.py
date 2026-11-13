"""측정 전용: /hk/plan/status, /hk/perception/obstacles, /hk/local/odom 을 기록 (주행 입력 아님)."""
import json, sys, time, pickle
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

class P(Node):
    def __init__(self, out, dur):
        super().__init__('hk_probe')
        self.out, self.end = out, time.monotonic() + dur
        self.status, self.clouds, self.odom = [], [], None
        self.create_subscription(String, '/hk/plan/status', self.on_status, 10)
        self.create_subscription(PointCloud2, '/hk/perception/obstacles', self.on_cloud, 1)
        self.create_subscription(Odometry, '/hk/local/odom', self.on_odom, 1)
    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9
    def on_odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.odom = (p.x, p.y, float(np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))), m.twist.twist.linear.x)
    def on_status(self, m):
        self.status.append((self.t(), self.odom, m.data))
    def on_cloud(self, m):
        r = point_cloud2.read_points(m, field_names=['x', 'y', 'cluster_id'])
        st = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        self.clouds.append((st, self.odom, np.column_stack((r['x'], r['y'])).astype(np.float32), np.asarray(r['cluster_id'])))

def main():
    rclpy.init(); n = P(sys.argv[1], float(sys.argv[2]))
    while rclpy.ok() and time.monotonic() < n.end:
        rclpy.spin_once(n, timeout_sec=0.2)
    with open(n.out, 'wb') as f:
        pickle.dump({'status': n.status, 'clouds': n.clouds}, f)

main()
