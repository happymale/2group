"""ground truth 자세 기록 (측정 전용): t,x,y,yaw,v,gear,accel,brake,steer,ex,ey,eyaw (ex.. = /hk/local/odom)"""
import math, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from hk_interfaces.msg import CtrlCmd


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class G(Node):
    def __init__(self, out):
        super().__init__('pk_gt_log')
        self.f = open(out, 'w')
        self.f.write('t,x,y,yaw,v,gear,accel,brake,steer,ex,ey,eyaw\n')
        self.cmd = (0, 0, 0, 0)
        self.est = (math.nan, math.nan, math.nan)
        self.n = 0
        self.create_subscription(Odometry, '/dev/ground_truth', self.on_gt, 20)
        self.create_subscription(Odometry, '/hk/local/odom', self.on_est, 20)
        self.create_subscription(CtrlCmd, '/hk/control/command', self.on_cmd, 20)

    def on_cmd(self, m):
        self.cmd = (m.gear, m.accel, m.brake, m.steering)

    def on_est(self, m):
        self.est = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def on_gt(self, m):
        self.n += 1
        if self.n % 2:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) * (1 if m.twist.twist.linear.x >= 0 else -1)
        self.f.write('%.3f,%.3f,%.3f,%.4f,%.3f,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%.4f\n' % (
            (t, p.x, p.y, yaw_of(m.pose.pose.orientation), v) + self.cmd + self.est))
        self.f.flush()


rclpy.init()
try:
    rclpy.spin(G(sys.argv[1]))
except KeyboardInterrupt:
    pass
