"""주차 구역 주변 LiDAR 관측점 기록 (탐색용). python3 cloud_probe.py out.csv"""
import sys, math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry

BOX = (-30.0, 12.0, 50.0, 140.0)


class P(Node):
    def __init__(self, out):
        super().__init__('pk_probe')
        self.f = open(out, 'w')
        self.f.write('t,gx,gy,gyaw,x,y,cid\n')
        self.gt = None
        self.create_subscription(Odometry, '/dev/ground_truth', self.on_gt, 10)
        self.create_subscription(PointCloud2, '/hk/perception/obstacles', self.on_cloud, 5)

    def on_gt(self, m):
        q = m.pose.pose.orientation
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y,
                   math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_cloud(self, m):
        if self.gt is None:
            return
        gx, gy, gyaw = self.gt
        if not (BOX[0] < gx < BOX[1] and BOX[2] < gy < BOX[3]):
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rows = point_cloud2.read_points(m, field_names=['x', 'y', 'cluster_id'])
        for x, y, c in zip(rows['x'], rows['y'], rows['cluster_id']):
            if BOX[0] < x < BOX[1] and BOX[2] < y < BOX[3]:
                self.f.write(f'{t:.2f},{gx:.2f},{gy:.2f},{gyaw:.3f},{x:.3f},{y:.3f},{c}\n')
        self.f.flush()


rclpy.init()
n = P(sys.argv[1])
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
