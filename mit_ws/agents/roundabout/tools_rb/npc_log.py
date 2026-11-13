"""회전교차로 NPC 관측 기록 (분석 전용). /hk/perception/obstacles 군집 중 교차로 중심 근처만 기록.
   python3 npc_log.py out.csv [seconds]"""
import csv, math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
C = (108.8353, 118.7754)

class L(Node):
    def __init__(self, out):
        super().__init__('rb_npc_log')
        self.f = open(out, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['t', 'ego_x', 'ego_y', 'ego_yaw', 'cid', 'n', 'cx', 'cy', 'r', 'ang', 'ext_x', 'ext_y', 'rmin', 'rmax'])
        self.ego = (math.nan,) * 3
        self.create_subscription(Odometry, '/dev/ground_truth', self.on_gt, 10)
        self.create_subscription(PointCloud2, '/hk/perception/obstacles', self.on_obs, 5)
        self.sf = open(out.replace('.npc.csv', '.status.txt'), 'w')
        self.create_subscription(String, '/hk/plan/status', self.on_status, 5)
    def on_status(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.sf.write(f'{t:.2f} ego=({self.ego[0]:.1f},{self.ego[1]:.1f}) {m.data}' + chr(10)); self.sf.flush()
    def on_gt(self, m):
        q = m.pose.pose.orientation
        self.ego = (m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    def on_obs(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rows = point_cloud2.read_points(m, field_names=['x', 'y', 'cluster_id'])
        if len(rows) == 0:
            self.w.writerow([f'{t:.3f}', *[f'{v:.3f}' for v in self.ego], -1, 0, '', '', '', '', '', '', '', '']); return
        xy = np.column_stack((rows['x'], rows['y'])).astype(float)
        for cid in np.unique(rows['cluster_id']):
            p = xy[rows['cluster_id'] == cid]
            d = np.hypot(p[:, 0] - C[0], p[:, 1] - C[1])
            if d.min() > 25:
                continue
            cx, cy = p.mean(axis=0)
            self.w.writerow([f'{t:.3f}', *[f'{v:.3f}' for v in self.ego], int(cid), len(p), f'{cx:.3f}', f'{cy:.3f}',
                             f'{math.hypot(cx - C[0], cy - C[1]):.3f}', f'{math.degrees(math.atan2(cy - C[1], cx - C[0])):.2f}',
                             f'{np.ptp(p[:, 0]):.2f}', f'{np.ptp(p[:, 1]):.2f}', f'{d.min():.2f}', f'{d.max():.2f}'])
        self.f.flush()

rclpy.init()
n = L(sys.argv[1]); end = time.monotonic() + float(sys.argv[2] if len(sys.argv) > 2 else 60)
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
n.f.close()
