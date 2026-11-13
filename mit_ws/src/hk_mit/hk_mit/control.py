"""HEVEN 2026 MIT 기본 코드 3/3: 제어 (control). 참가자가 고칠 곳.

localization.py의 /hk/local/odom(위치·방향·속력)과 planner.py의 /hk/plan/local_path(따라갈 경로)를 받아
50 Hz로 /hk/control/command에 구동·제동·조향을 냅니다. 고칠 곳은 아래 `drive()`입니다.

  drive()에서 쓸 수 있는 값                       돌려줄 값
  self.x, self.y   지도 위치 (m, 뒤차축)            accel  0~1   (구동)
  self.yaw         진행 방향 (rad)                 brake  0~1   (제동)
  self.speed       속력 (m/s)                      steer  -1~1  (+ 왼쪽, - 오른쪽, 1 = 35.4°)
  self.path        따라갈 경로 [(x, y, 경로 방향), ...] (비었으면 경로 끝)
  self.nearest(x, y)  경로에서 (x, y)와 가장 가까운 점 (x, y, 경로 방향)

조향은 Stanley, 속력은 PI 제어입니다. 일부러 아주 단순하게 두었습니다: 굽은 길에서 속도를 줄이지 않고,
노면·지연·경사도 따로 생각하지 않습니다. 속도를 올리거나 옵션을 켜면서 직접 고쳐 보세요.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from hk_interfaces.msg import CtrlCmd

from .sim_events import SimEvents

# 조정할 값은 config/bringup.yaml의 파라미터입니다(공식 주행 직전에는 코드가 아니라 이 파일만 바꿀 수 있음).
# 아래는 yaml에 값이 없을 때 쓰는 기본값입니다.
TARGET_SPEED_DEFAULT = 5.0
PARAMS = {
    'target_speed': TARGET_SPEED_DEFAULT,   # 목표 속력 (m/s). 실행할 때 target_speed:=… 로도 바꿀 수 있습니다.
    'k_stanley': 1.0,      # 경로에서 옆으로 벗어난 거리에 대한 조향 세기
    'k_soft': 1.0,         # 저속에서 조향이 너무 세지지 않게 하는 값 (m/s)
    'kp': 0.5,             # 속력 P 이득
    'ki': 0.05,            # 속력 I 이득
    'brake_gain': 0.6,     # 너무 빠를 때 제동 세기
}
WHEELBASE = 3.0        # 축간 거리 (m)
MAX_STEER = math.radians(35.4)   # steer = 1 일 때 앞바퀴 각도
RATE_HZ = 50.0
SETTLE_S = 0.5         # 시작·복귀 뒤 이 시간 동안은 제동만 (시뮬레이터도 복귀 뒤 0.5초 명령을 무시함)


class Control(Node):
    def __init__(self):
        super().__init__('hk_mit_control')
        for name, default in PARAMS.items():           # self.target_speed, self.k_stanley, ... 로 씁니다
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))
        self.dt = 1.0 / RATE_HZ
        self.odom = None
        self.path = None
        self.active = False
        self.reset_time = None
        self.cmd_pub = self.create_publisher(CtrlCmd, '/hk/control/command', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, '/hk/local/odom', self.on_odom, 10)
        self.create_subscription(Path, '/hk/plan/local_path', self.on_path, 10)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(self.dt, self.tick)

    # ---- 시작·복귀 -----------------------------------------------------------------------------------
    def on_reset(self, checkpoint, reload):
        self.reset_control()
        self.odom = self.path = None
        self.reset_time = self.now()
        self.active = True

    def on_end(self):
        self.active = False
        self.send(0.0, 1.0, 0.0)

    def reset_control(self):
        """시작·복귀 때 불립니다. 제어기 상태를 처음으로 돌립니다."""
        self.speed_error_sum = 0.0

    # ---- 받은 값 -------------------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        self.odom = m
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.speed = m.twist.twist.linear.x

    def on_path(self, m):
        self.path = [(ps.pose.position.x, ps.pose.position.y,
                      2 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w)) for ps in m.poses]

    def nearest(self, x, y):
        return min(self.path, key=lambda p: math.hypot(p[0] - x, p[1] - y))

    # ---- 50 Hz ------------------------------------------------------------------------------------------
    def tick(self):
        if not self.active:
            return
        if self.odom is None or self.path is None or self.now() - self.reset_time < SETTLE_S:
            self.send(0.0, 1.0, 0.0)                 # 시작·복귀 직후, 위치·경로가 아직 없으면 정지
            return
        self.send(*self.drive())

    def drive(self):
        # 1) 앞바퀴 축 위치와, 경로에서 그와 가장 가까운 점·그 점의 경로 방향
        fx = self.x + WHEELBASE * math.cos(self.yaw)
        fy = self.y + WHEELBASE * math.sin(self.yaw)
        if not self.path:                            # 경로 끝을 지남: 곧게 서서 멈춤
            steer, target = 0.0, 0.0
        else:
            px, py, path_yaw = self.nearest(fx, fy)
            # 2) 조향: Stanley = 방향 오차 + atan(K × 옆 오차 / 속력)
            heading_error = angle_diff(path_yaw, self.yaw)
            cross_error = -math.sin(path_yaw) * (px - fx) + math.cos(path_yaw) * (py - fy)   # 경로가 왼쪽이면 +
            wheel_angle = heading_error + math.atan2(self.k_stanley * cross_error, self.k_soft + self.speed)
            steer = clamp(wheel_angle / MAX_STEER, -1.0, 1.0)
            target = self.target_speed

        # 3) 속력: PI 제어
        error = target - self.speed
        self.speed_error_sum = clamp(self.speed_error_sum + error * self.dt, -5.0, 5.0)
        u = self.kp * error + self.ki * self.speed_error_sum
        if u >= 0:
            return min(u, 1.0), 0.0, steer          # accel, brake, steer
        return 0.0, min(-u * self.brake_gain, 1.0), steer

    def send(self, accel, brake, steer):
        c = CtrlCmd()
        c.header.stamp = self.get_clock().now().to_msg()
        c.ctrl_mode = CtrlCmd.CTRL_MODE_AUTO
        c.gear = CtrlCmd.GEAR_DRIVE
        c.long_cmd_type = CtrlCmd.LONG_CMD_THROTTLE
        c.accel, c.brake, c.steering = float(accel), float(brake), float(steer)
        self.cmd_pub.publish(c)


def angle_diff(a, b):
    """a - b 를 -π~π 로."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
