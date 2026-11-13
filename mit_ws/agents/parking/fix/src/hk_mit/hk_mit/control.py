"""HEVEN 2026 MIT 기본 코드 3/3: 제어 (control). 참가자가 고칠 곳.

localization.py의 /hk/local/odom(위치·방향·속력)과 planner.py의 /hk/plan/local_path(따라갈 경로)를 받아
50 Hz로 /hk/control/command에 구동·제동·조향을 냅니다. 고칠 곳은 아래 `drive()`입니다.

  drive()에서 쓸 수 있는 값                       돌려줄 값
  self.x, self.y   지도 위치 (m, 뒤차축)            accel  0~1   (구동)
  self.yaw         진행 방향 (rad)                 brake  0~1   (제동)
  self.speed       속력 (m/s)                      steer  -1~1  (+ 왼쪽, - 오른쪽, 1 = 35.4°)
  self.path        따라갈 경로 [(x, y, 경로 방향), ...] (비었으면 경로 끝)
  self.nearest(x, y)  경로에서 (x, y)와 가장 가까운 점 (x, y, 경로 방향)

조향은 Stanley, 속력은 PI 제어입니다. planner의 /hk/plan/speed_limit을 상한으로
적용하며 빈 경로·속도 제한 0·오래된 계획에는 제동합니다. 경사는 IMU 전후 가속도 - 바퀴 속력 변화율로
추정해 피드포워드로 보상합니다(k_slope, 0 = 끔).

주차 (planner의 /hk/plan/gear, std_msgs/Int8): 0 = 일반 주행(기존과 같음), 부호 = 주차 구간 방향(+ 전진, - 후진),
크기 1 = 경로가 이어짐, 2 = 경로 끝이 정지 지점(남은 거리로 속력을 줄여 그 점에 섬, 50 Hz).
주차 전진은 Stanley + 곡률 피드포워드(경로는 뒤차축 설계 경로의 앞바퀴 축 궤적). 기어는 거의 멈췄을 때만
바꾸고(바꾸는 동안 제동하며 새 방향의 조향을 미리 맞춤), 후진은 뒤차축 되먹임(rear wheel feedback, 차 방향 + π)으로
따라갑니다.
"""
from collections import deque
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int8
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
    'latency_s': 0.1,      # 조향 계산 전에 이 시간만큼 뒤의 자세를 예측 (조향 지연 옵션이면 0.25 이상)
    'k_curvature': 0.0,    # 경로 곡률 피드포워드 비율 (0 = 끔. Stanley 방향 오차가 이미 곡률을 반영)
    'k_yaw_rate': 0.0,     # 요레이트 감쇠 (고속 진동이 생기면 0.05~0.1)
    'k_offset': 0.0,       # 곡선 바깥 목표 비율: 뒤차축이 곡선 안쪽을 파고드는 만큼(√(R²+L²)-R) 앞바퀴 축을 바깥에 둠
    'offset_span': 3.0,    # k_offset에 쓰는 곡률을 앞바퀴 축 앞뒤 이 길이(m)로 평균 (S자 전환에서 목표가 계단처럼 바뀌지 않게)
    'k_understeer': 0.0,   # 언더스티어 피드포워드 (rad per m/s²): 바퀴각 += k × 속력² × 경로 곡률(앞바퀴 축)
    'k_slope': 0.0,        # 경사 피드포워드 비율 (0 = 끔, 1 = 추정한 경사 가속도를 그대로 보상)
    'slope_tau': 0.3,      # 경사 추정 저역 통과 시간 상수 (s)
    'drive_accel': 4.0,    # accel = 1 일 때 가속도 (m/s², 저속 실측 약 4)
    'brake_decel': 8.5,    # brake = 1 일 때 감속도 (m/s², 실측 약 8.6)
    'plan_timeout': 0.5,   # 경로·계획 속도 제한이 끊기면 제동 (s)
    'k_rev_heading': 1.5,          # 후진 뒤차축 되먹임: 방향 오차 이득 (1/m)
    'k_rev_lateral': 1.0,          # 후진 뒤차축 되먹임: 옆 오차 이득 (1/m²)
    'park_stop_decel': 1.2,        # 주차 정지 지점 앞 감속도 (m/s²): 속력 ≤ √(2 a 남은 거리)
    'park_stop_tolerance': 0.1,    # 정지 지점까지 이보다 가까우면 완전히 제동 (m)
    'kp_park': 1.0,                # 주차 구간 속력 P 이득 (원호 앞 감속을 빨리 따르게)
    'gear_stop_speed': 0.05,       # 이보다 느릴 때만 기어를 바꿈 (m/s, 시뮬레이터는 움직이는 중 기어 변경을 무시)
    'gear_settle_s': 0.4,          # 기어를 바꾼 뒤 이 시간 동안 제동하며 조향을 맞춤 (s)
    'odom_timeout': 0.35,
    'clock_tolerance': 0.05,
    # 출발 보조: 출발(cp0) 직후 위치·경로·계획이 아직 준비되지 않았으면 route.csv 첫 직선(-3 → 55 m, 방향 0)을
    # 조향 0으로 곧게 달림. 바퀴 거리 launch_max_dist(m)·속력 launch_max_speed(m/s)까지, 준비되면 바로 정상 제어로
    'launch_max_dist': 8.0,    # 장애물 1개 구간(x ≥ 15 m)보다 충분히 앞에서 멈출 수 있게
    'launch_max_speed': 5.0,
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
        self.declare_parameter('launch_assist', True)
        self.launch_assist = bool(self.get_parameter('launch_assist').value)
        self.assist = False              # 지금 출발 보조를 쓸 수 있는지 (cp0 출발 뒤 정상 제어가 시작되기 전까지)
        self.assist_dist = 0.0           # 출발 보조로 달린 바퀴 거리 (m)
        self.wheel_speed, self.wheel_stamp = 0.0, None
        self.declare_parameter('require_plan_speed', True)
        self.require_plan_speed = bool(self.get_parameter('require_plan_speed').value)
        if (not all(math.isfinite(v) for v in (self.plan_timeout, self.odom_timeout, self.clock_tolerance))
                or self.plan_timeout <= 0 or self.odom_timeout <= 0 or self.clock_tolerance < 0):
            raise ValueError('Control timeouts must be positive')
        self.dt = 1.0 / RATE_HZ
        self.odom = None
        self.path = None
        self.path_stamp = None
        self.plan_speed_limit = None
        self.speed_limit_stamp = None
        self.active = False
        self.reset_time = None
        self.cmd_pub = self.create_publisher(CtrlCmd, '/hk/control/command', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, '/hk/local/odom', self.on_odom, 1)
        self.create_subscription(Path, '/hk/plan/local_path', self.on_path, 1)
        self.create_subscription(Float32, '/hk/plan/speed_limit', self.on_speed_limit, 1)
        self.plan_gear, self.gear_stamp = 0, None          # planner가 정한 주행 방식 (0, +1, -1)
        self.gear = CtrlCmd.GEAR_DRIVE                     # 지금 보내는 기어
        self.gear_since = -1e9
        self.create_subscription(Int8, '/hk/plan/gear', self.on_gear, 1)
        self.imu_ax = None
        self.create_subscription(Imu, '/hk/sensors/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/hk/sensors/wheel_encoder/speed', self.on_wheel, qos_profile_sensor_data)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(self.dt, self.tick)

    # ---- 시작·복귀 -----------------------------------------------------------------------------------
    def on_reset(self, checkpoint, reload):
        self.reset_control()
        self.gear, self.gear_since, self.plan_gear, self.gear_stamp = CtrlCmd.GEAR_DRIVE, -1e9, 0, None
        self.odom = self.path = None
        self.path_stamp = self.plan_speed_limit = self.speed_limit_stamp = None
        self.reset_time = self.now()
        self.active = True
        self.assist = self.launch_assist and reload and checkpoint == 0    # 출발(START, cp0)일 때만
        self.assist_dist = 0.0

    def on_end(self):
        self.active = False
        self.send(0.0, 1.0, 0.0)

    def reset_control(self):
        """시작·복귀 때 불립니다. 제어기 상태를 처음으로 돌립니다."""
        self.speed_error_sum = 0.0
        self.last_steer = 0.0
        # 아직 바퀴에 닿지 않은(latency_s 안에 낸) 조향 명령. 오래된 것부터
        self.steer_history = deque(maxlen=max(1, round(self.latency_s / self.dt)))
        self.ax_filtered = self.speed_filtered = None   # 경사 추정 상태
        self.grade = 0.0

    # ---- 받은 값 -------------------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        if m.header.frame_id != 'map':
            self.odom = None
            return
        self.odom = m
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.speed = m.twist.twist.linear.x
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw, self.speed)):
            self.odom = None

    def on_path(self, m):
        if m.header.frame_id != 'map':
            self.path = None
            return
        self.path = [(ps.pose.position.x, ps.pose.position.y,
                      2 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w)) for ps in m.poses]
        self.path_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not all(math.isfinite(value) for pose in self.path for value in pose):
            self.path = None

    def on_wheel(self, m):
        """바퀴 속력 (출발 보조 거리 적분용)."""
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        v = m.twist.linear.x
        if self.assist and self.wheel_stamp is not None and 0.0 < t - self.wheel_stamp < 0.5:
            self.assist_dist += abs(v) * (t - self.wheel_stamp)
        self.wheel_speed, self.wheel_stamp = v, t

    def on_imu(self, m):
        ax = m.linear_acceleration.x
        self.imu_ax = ax if math.isfinite(ax) else None

    def update_grade(self):
        """경사 가속도 (m/s², + 오르막) = IMU 전후 가속도 - 속력 변화율. 둘 다 같은 저역 통과로 거릅니다."""
        if self.imu_ax is None:
            self.grade = 0.0
            return
        if self.ax_filtered is None:
            self.ax_filtered, self.speed_filtered = self.imu_ax, self.speed
            return
        a = self.dt / (self.slope_tau + self.dt)
        self.ax_filtered += a * (self.imu_ax - self.ax_filtered)
        previous = self.speed_filtered
        self.speed_filtered += a * (self.speed - self.speed_filtered)
        self.grade = clamp(self.ax_filtered - (self.speed_filtered - previous) / self.dt, -3.0, 3.0)

    def on_speed_limit(self, m):
        self.plan_speed_limit = m.data if math.isfinite(m.data) and m.data >= 0 else 0.0
        self.speed_limit_stamp = self.now()

    def on_gear(self, m):
        self.plan_gear = int(max(-2, min(2, m.data)))
        self.gear_stamp = self.now()

    def mode(self):
        """planner의 주행 방식 (오래됐으면 0 = 일반 주행)."""
        if self.gear_stamp is None or not 0 <= self.now() - self.gear_stamp <= self.plan_timeout:
            return 0
        return self.plan_gear

    def nearest(self, x, y):
        return self.path[self.nearest_index(x, y)]

    def nearest_index(self, x, y):
        return min(range(len(self.path)), key=lambda i: math.hypot(self.path[i][0] - x, self.path[i][1] - y))

    def path_span(self, k, span):
        """k번째 점 앞뒤로 경로를 따라 span/2(m)씩 떨어진 점의 번호 (i, j)."""
        i, j = k, k
        while i > 0 and math.hypot(self.path[k][0] - self.path[i][0], self.path[k][1] - self.path[i][1]) < span / 2:
            i -= 1
        while j < len(self.path) - 1 and math.hypot(self.path[j][0] - self.path[k][0],
                                                    self.path[j][1] - self.path[k][1]) < span / 2:
            j += 1
        return i, j

    def path_heading(self, k, span=1.5):
        """k번째 점의 경로 방향을 앞뒤 span(m) 현(chord)으로 구함. route.csv 0.5 m 점 사이 방향은
        점 위치 반올림 때문에 0~4°씩 번갈아 튀므로(표준편차 0.9°, 최대 6°) 그대로 쓰면 조향이 떨립니다."""
        i, j = self.path_span(k, span)
        if i == j:
            return self.path[k][2]
        return math.atan2(self.path[j][1] - self.path[i][1], self.path[j][0] - self.path[i][0])

    def path_curvature(self, k, span=2.0):
        """k번째 점 앞뒤 span(m) 사이의 경로 방향 변화 / 거리 (+ 왼쪽으로 굽음). 양 끝 방향은 현으로 구함."""
        i, j = self.path_span(k, span)
        length = sum(math.hypot(self.path[n + 1][0] - self.path[n][0], self.path[n + 1][1] - self.path[n][1])
                     for n in range(i, j))
        return angle_diff(self.path_heading(j), self.path_heading(i)) / length if length > 1e-3 else 0.0

    # ---- 50 Hz ------------------------------------------------------------------------------------------
    def tick(self):
        if not self.active:
            return
        if self.now() - self.reset_time < SETTLE_S:
            self.send(0.0, 1.0, 0.0)                 # 시작·복귀 직후는 정지 (시뮬레이터가 명령을 무시하는 시간)
            return
        if not self.ready():
            self.reset_control()
            self.send(*self.launch_command())        # 위치·경로·계획이 아직 없으면 출발 보조 또는 정지
            return
        self.assist = False                          # 정상 제어가 한 번 시작되면 출발 보조는 끝
        self.send(*self.drive())

    def launch_command(self):
        """출발 보조 명령 (accel, brake, steer). 쓸 수 없거나 거리 한계를 넘으면 제동."""
        if not self.assist or self.assist_dist >= self.launch_max_dist:
            return 0.0, 1.0, 0.0
        if self.wheel_speed >= self.launch_max_speed:
            return 0.0, 0.0, 0.0
        return 1.0, 0.0, 0.0

    def ready(self):
        """위치(odom)·경로·계획 속도가 모두 새롭고 경로가 비지 않았는지. 아니면 제동(또는 출발 보조)."""
        if self.odom is None or self.path is None:
            return False
        now = self.now()
        odom_stamp = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
        stale_speed = self.require_plan_speed and (
            self.speed_limit_stamp is None or not 0 <= now - self.speed_limit_stamp <= self.plan_timeout)
        if (not -self.clock_tolerance - 1e-6 <= now - odom_stamp <= self.odom_timeout or self.path_stamp is None
                or not -self.clock_tolerance - 1e-6 <= now - self.path_stamp <= self.plan_timeout
                or self.path_stamp < self.reset_time or odom_stamp < self.reset_time or stale_speed):
            return False
        if not self.path or (self.require_plan_speed and (self.plan_speed_limit is None or self.plan_speed_limit <= 0)):
            return False
        self.odom_age = min(max(now - odom_stamp, 0.0), 0.25)
        return True

    def drive(self):
        mode = self.mode()
        stop_at_end = abs(mode) == 2
        mode = (mode > 0) - (mode < 0)                 # 방향: +1 전진, -1 후진, 0 일반
        want = CtrlCmd.GEAR_REVERSE if mode < 0 else CtrlCmd.GEAR_DRIVE
        if want != self.gear:
            # 기어 전환: 멈출 때까지 제동하고, 멈추면 바꿈. 그동안 새 방향의 조향을 미리 냄
            steer = self.parking_steer(mode) if self.path else 0.0
            if abs(self.speed) < self.gear_stop_speed:
                self.gear, self.gear_since = want, self.now()
            self.reset_control()
            self.last_steer = steer
            return 0.0, 1.0, steer
        if mode != 0 and self.now() - self.gear_since < self.gear_settle_s and self.path:
            steer = self.parking_steer(mode)
            self.reset_control()
            self.last_steer = steer
            return 0.0, 1.0, steer
        if not self.path or (self.require_plan_speed and (self.plan_speed_limit is None or self.plan_speed_limit <= 0)):
            self.reset_control()
            return 0.0, 1.0, 0.0
        if mode != 0:
            return self.drive_parking(mode, stop_at_end)
        # 0) 지연 보상: 지금 낸 조향이 먹힐 때(latency_s 뒤)의 자세를, 그 사이에 바퀴에 차례로 닿을
        #    지난 조향 명령들로 예측합니다. 직전 조향 하나를 latency_s 내내 쓰면 예측이 마지막 명령에
        #    너무 민감해져(이득 v·latency/L) 회피 경로처럼 휘는 곳에서 조향이 ±1로 떨립니다.
        yaw_rate = self.speed * math.tan(self.last_steer * MAX_STEER) / WHEELBASE
        x, y, yaw = self.x, self.y, self.yaw
        history = list(self.steer_history) or [self.last_steer]
        # odom이 늦게 왔으면(위치 추정 노드 부하) 그 시간만큼 먼저 지금 바퀴에 닿아 있는 조향으로 앞당김
        age = getattr(self, 'odom_age', 0.0)
        if age > 0.0:
            rate = self.speed * math.tan(history[0] * MAX_STEER) / WHEELBASE
            x += self.speed * math.cos(yaw + 0.5 * rate * age) * age
            y += self.speed * math.sin(yaw + 0.5 * rate * age) * age
            yaw += rate * age
        dt = self.latency_s / len(history)
        for past in history:
            x += self.speed * math.cos(yaw) * dt
            y += self.speed * math.sin(yaw) * dt
            yaw += self.speed * math.tan(past * MAX_STEER) / WHEELBASE * dt
        # 1) 앞바퀴 축 위치와, 경로에서 그와 가장 가까운 점·그 점의 경로 방향
        fx = x + WHEELBASE * math.cos(yaw)
        fy = y + WHEELBASE * math.sin(yaw)
        k = self.nearest_index(fx, fy)
        px, py, _ = self.path[k]
        path_yaw = self.path_heading(k)
        # 2) 조향: Stanley = 방향 오차 + atan(K × 옆 오차 / 속력) + 경로 곡률 피드포워드
        heading_error = angle_diff(path_yaw, yaw)
        cross_error = -math.sin(path_yaw) * (px - fx) + math.cos(path_yaw) * (py - fy)   # 경로가 왼쪽이면 +
        # 자전거 모형에서 경로 곡률을 따르는 것은 뒤차축이므로 곡률은 뒤차축 위치에서 읽습니다
        # (앞바퀴 축 위치에서 읽으면 축간거리만큼 일찍 꺾어 곡선 안쪽을 파고듭니다)
        kappa = self.path_curvature(self.nearest_index(x, y))
        # 앞바퀴 축이 경로 위에 있으면 뒤차축은 반지름 R 곡선에서 √(R²+L²)-R 만큼 안쪽을 지나므로
        # 앞바퀴 축 목표를 그 k_offset 배만큼 곡선 바깥으로 옮깁니다 (+ 왼쪽 곡선이면 오른쪽으로).
        # 곡률은 앞바퀴 축 위치에서 읽음: S자에서 뒤차축 곡률을 쓰면 새 곡선 입구에서 오히려 안쪽으로 밀림
        front_kappa = self.path_curvature(k, span=3.0) if (self.k_offset or self.k_understeer) else 0.0
        offset_kappa = self.path_curvature(k, span=self.offset_span) if self.k_offset else 0.0
        if abs(offset_kappa) > 1e-4:
            cross_error -= self.k_offset * (math.sqrt(1.0 + (WHEELBASE * offset_kappa)**2) - 1.0) / offset_kappa
        wheel_angle = heading_error + math.atan2(self.k_stanley * cross_error, self.k_soft + self.speed)
        wheel_angle += self.k_curvature * math.atan(WHEELBASE * kappa)
        # 언더스티어: 횡가속도 a = v²κ 가 클수록 운동학 각(L·κ)보다 더 꺾어야 같은 곡선을 돔
        # (실측: 횡가속도 4~7 m/s²에서 바퀴각이 운동학 각보다 2~4° 더 필요. 없으면 옆 오차 항이
        #  그만큼을 메우느라 곡선 뒤쪽에서 차가 0.5 m 넘게 바깥으로 밀림)
        wheel_angle += self.k_understeer * self.speed ** 2 * front_kappa
        wheel_angle -= self.k_yaw_rate * (yaw_rate - self.speed * kappa)
        steer = clamp(wheel_angle / MAX_STEER, -1.0, 1.0)
        self.last_steer = steer
        self.steer_history.append(steer)
        target = self.target_speed
        if self.require_plan_speed:
            target = min(target, self.plan_speed_limit)

        # 3) 속력: PI 제어 + 경사 피드포워드 (오르막이면 더 밟고, 내리막이면 미리 제동)
        error = target - self.speed
        self.speed_error_sum = clamp(self.speed_error_sum + error * self.dt, -5.0, 5.0)
        u = self.kp * error + self.ki * self.speed_error_sum
        self.update_grade()
        slope = self.k_slope * self.grade
        if slope >= 0:
            u += slope / self.drive_accel
        else:                                       # 제동은 brake = -u × brake_gain 이므로 그만큼 나눔
            u += slope / (self.brake_decel * max(self.brake_gain, 1e-3))
        if u >= 0:
            return min(u, 1.0), 0.0, steer          # accel, brake, steer
        return 0.0, min(-u * self.brake_gain, 1.0), steer

    # ---- 주차 (전진·후진, 경로 끝에서 정지) ----------------------------------------------------------------------
    def predicted_pose(self):
        """지연 보상한 뒤차축 자세 (drive()와 같은 방법, 후진이면 속력이 음수)."""
        x, y, yaw = self.x, self.y, self.yaw
        history = list(self.steer_history) or [self.last_steer]
        dt = self.latency_s / len(history)
        for past in history:
            x += self.speed * math.cos(yaw) * dt
            y += self.speed * math.sin(yaw) * dt
            yaw += self.speed * math.tan(past * MAX_STEER) / WHEELBASE * dt
        return x, y, yaw

    def parking_steer(self, mode):
        """주차 구간 조향. 전진은 기존 Stanley(앞바퀴 축), 후진은 뒤차축 pure pursuit (차 방향 + π)."""
        x, y, yaw = self.predicted_pose()
        if mode >= 0:
            fx, fy = x + WHEELBASE * math.cos(yaw), y + WHEELBASE * math.sin(yaw)
            k = self.nearest_index(fx, fy)
            px, py, path_yaw = self.path[k]
            heading_error = angle_diff(path_yaw, yaw)
            cross_error = -math.sin(path_yaw) * (px - fx) + math.cos(path_yaw) * (py - fy)
            wheel = heading_error + math.atan2(self.k_stanley * cross_error, self.k_soft + abs(self.speed))
            if mode > 0:
                # 곡률 피드포워드: 앞바퀴 축 궤적의 (차 방향 변화 / 거리) = κ / √(1 + L²κ²) → 뒤차축 곡률 κ
                kf = clamp(self.path_curvature(k, span=1.0), -0.99 / WHEELBASE, 0.99 / WHEELBASE)
                wheel += math.atan(WHEELBASE * kf / math.sqrt(1.0 - (WHEELBASE * kf) ** 2))
            return clamp(wheel / MAX_STEER, -1.0, 1.0)
        # 후진: 뒤로 가는 가상 차(방향 ψ' = yaw + π, 조향 δ' = -δ)에 뒤차축 되먹임(rear wheel feedback) 적용
        #   ω' = v κ cos e_ψ / (1 - κ e) - k_ψ v e_ψ - k_e v e sin(e_ψ)/e_ψ,  δ' = atan(L ω' / v)
        k = self.nearest_index(x, y)
        px, py, path_yaw = self.path[k]
        travel = path_yaw + math.pi
        e = -math.sin(travel) * (x - px) + math.cos(travel) * (y - py)        # 경로 왼쪽이면 +
        e_yaw = angle_diff(yaw + math.pi, travel)
        kappa = self.path_curvature(k, span=1.0)          # 차 방향 변화 / 거리 = 진행 방향 기준 곡률과 같음
        v = max(abs(self.speed), 0.5)
        sinc = math.sin(e_yaw) / e_yaw if abs(e_yaw) > 1e-4 else 1.0
        omega = (v * kappa * math.cos(e_yaw) / max(1.0 - kappa * e, 0.2)
                 - self.k_rev_heading * v * e_yaw - self.k_rev_lateral * v * sinc * e)
        return clamp(-math.atan(WHEELBASE * omega / v) / MAX_STEER, -1.0, 1.0)

    def remaining(self, mode):
        """경로 끝(정지 지점)까지 진행 방향으로 남은 거리 (m). 전진은 앞바퀴 축, 후진은 뒤차축 기준."""
        ex, ey, eyaw = self.path[-1]
        if mode > 0:
            rx, ry, direction = self.x + WHEELBASE * math.cos(self.yaw), self.y + WHEELBASE * math.sin(self.yaw), eyaw
        else:
            rx, ry, direction = self.x, self.y, eyaw + math.pi
        return (ex - rx) * math.cos(direction) + (ey - ry) * math.sin(direction)

    def drive_parking(self, mode, stop_at_end):
        steer = self.parking_steer(mode)
        self.last_steer = steer
        self.steer_history.append(steer)
        target = min(self.target_speed, self.plan_speed_limit if self.require_plan_speed else self.target_speed)
        if stop_at_end:
            rem = self.remaining(mode)
            if rem < self.park_stop_tolerance:
                self.reset_control()
                return 0.0, 1.0, steer
            target = min(target, math.sqrt(2 * self.park_stop_decel * max(0.0, rem - 0.5 * self.park_stop_tolerance)))
        # 속력 PI (진행 방향 속력, 후진이면 -속력). 경사 보상은 전진만
        error = target - self.speed * mode
        self.speed_error_sum = clamp(self.speed_error_sum + error * self.dt, -5.0, 5.0)
        u = self.kp_park * error + self.ki * self.speed_error_sum
        if u >= 0:
            return min(u, 1.0), 0.0, steer
        return 0.0, min(-u * self.brake_gain, 1.0), steer

    def send(self, accel, brake, steer):
        c = CtrlCmd()
        c.header.stamp = self.get_clock().now().to_msg()
        c.ctrl_mode = CtrlCmd.CTRL_MODE_AUTO
        c.gear = self.gear
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
