"""후진 기어 응답 시험: 전진 2 s → 정지 → 후진 3 s → 정지. 바퀴 속력 부호, GT 이동을 기록."""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from hk_interfaces.msg import CtrlCmd


class T(Node):
    def __init__(self):
        super().__init__('pk_rev_test')
        self.pub = self.create_publisher(CtrlCmd, '/hk/control/command', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.ws = None; self.gt = None; self.t0 = None
        self.create_subscription(TwistStamped, '/hk/sensors/wheel_encoder/speed', lambda m: setattr(self, 'ws', m.twist.linear.x), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/dev/ground_truth', self.on_gt, 10)
        self.create_timer(0.02, self.tick)

    def on_gt(self, m):
        q = m.pose.pose.orientation
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2*(q.w*q.z), 1-2*q.z*q.z),
                   m.twist.twist.linear.x, m.twist.twist.linear.y)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.gt is None:
            return
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        gear, acc, brk, st = CtrlCmd.GEAR_DRIVE, 0.0, 1.0, 0.0
        if 1 < t < 2.5: acc, brk = 0.4, 0.0
        elif 2.5 <= t < 4.0: acc, brk = 0.0, 1.0
        elif 4.0 <= t < 4.5: gear, acc, brk = CtrlCmd.GEAR_REVERSE, 0.0, 1.0
        elif 4.5 <= t < 7.0: gear, acc, brk, st = CtrlCmd.GEAR_REVERSE, 0.4, 0.0, 0.5
        elif 7.0 <= t < 8.5: gear, acc, brk = CtrlCmd.GEAR_REVERSE, 0.0, 1.0
        elif 8.5 <= t < 9.5: gear, acc, brk = CtrlCmd.GEAR_REVERSE, 0.4, 0.0   # 후진 중 D로 바꾸면?
        elif 9.5 <= t < 10.5: gear, acc, brk = CtrlCmd.GEAR_DRIVE, 0.4, 0.0
        elif t > 12: raise SystemExit
        c = CtrlCmd(); c.ctrl_mode = CtrlCmd.CTRL_MODE_AUTO; c.gear = gear; c.long_cmd_type = CtrlCmd.LONG_CMD_THROTTLE
        c.accel, c.brake, c.steering = acc, brk, st
        c.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(c)
        if int(t * 50) % 10 == 0:
            g = self.gt
            print(f'{t:5.2f} gear={gear} acc={acc} ws={self.ws} gt=({g[0]:.2f},{g[1]:.2f},{math.degrees(g[2]):.1f}) vx={g[3]:.2f} vy={g[4]:.2f}', flush=True)


rclpy.init()
try:
    rclpy.spin(T())
except SystemExit:
    pass
