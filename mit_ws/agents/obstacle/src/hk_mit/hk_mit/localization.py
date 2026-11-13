"""HEVEN 2026 MIT 기본 코드 1/3: 위치 추정 (localization). 참가자가 고칠 곳.

센서를 받아 차의 위치·방향·속력을 구하고, 50 Hz로 /hk/local/odom (nav_msgs/Odometry, frame map)에 냅니다.
같은 값을 TF(map -> base_link)로도 내서 RViz에서 LiDAR 점과 경로를 지도 위에 볼 수 있습니다.
planner.py와 control.py가 이 값을 씁니다.

작은 EKF (상태 6개: x, y, yaw, 자이로 바이어스 bg, GNSS 다중경로 바이어스 mx, my)
  예측  : IMU(100 Hz)마다 바퀴 속력과 (각속도 - bg)로 추측 항법. 다중경로 바이어스는 1차 마르코프(τ)로 줄어듦.
  정지  : 바퀴 속력이 0이면 방향을 적분하지 않고 '각속도 = bg'로 바이어스를 갱신 (시작·복귀 때 잠깐 정지).
  GNSS  : 측정 = (x + mx, y + my). 마할라노비스 거리로 튀는 값을 버리고,
          크게(gnss_reset_m) 어긋난 값이 gnss_reset_count번 이어지면 GNSS로 다시 맞춤.
  시작·복귀: 자세는 코스 파일 값(정확)으로 두고 공분산을 작게 → 정지 중 GNSS로는 다중경로 바이어스를 배움.
          복귀 때 bg, mx, my는 이어 감(시작이면 처음부터). 음영 구간은 바이어스를 뺀 추측 항법으로 지남.
  내보냄: 마지막 IMU 시각 뒤 지금까지를 속력·각속도로 짧게(최대 extrapolate_max) 외삽해서 냄.
  속력  : 바퀴 속도 센서(/hk/sensors/wheel_encoder/speed) 값을 그대로 씁니다.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

from .course import declare_file_params, load_course
from .sim_events import SimEvents
from .utm import ll_to_map

RATE_HZ = 50.0
X, Y, YAW, BG, MX, MY, KS = range(7)  # EKF 상태 순서 (KS: 바퀴 속력 배율 오차, 진짜 속력 = 바퀴 × (1 - KS - 슬립))
N = 7
STILL = 1e-3                          # 이보다 느리면 정지로 봄 (m/s)

PARAMS = {                            # 기본값은 GNSS·IMU 노이즈가 모두 켜진 경우 (course.json sensors 값)
    'gnss_sigma': 0.55,               # GNSS 위치 백색 노이즈 σ (m)
    'gnss_bias_sigma': 0.3,           # GNSS 다중경로(천천히 변하는 바이어스) σ (m)
    'gnss_bias_tau': 10.0,            # 다중경로 상관 시간 (s)
    'gnss_gate': 4.0,                 # 마할라노비스 거리가 이보다 크면 그 GNSS는 버림
    'gnss_reset_m': 3.0,              # 이보다 크게 어긋난 GNSS가 gnss_reset_count번 이어지면 GNSS로 다시 맞춤
    'gnss_reset_count': 20,
    'gyro_sigma': 0.005,              # 자이로 백색 노이즈 σ (rad/s, 샘플마다)
    'gyro_bias_sigma': 0.003,         # 자이로 바이어스 초기 σ (rad/s)
    'gyro_drift_sigma': 0.001,        # 자이로 바이어스 변화 σ (rad/s), 상관 시간 gyro_drift_tau
    'gyro_drift_tau': 60.0,
    'speed_sigma': 0.01,              # 바퀴 속력 상대 백색 오차 → 진행 거리에 따른 위치 불확실성
    'wheel_scale0': 0.0045,           # 바퀴 속력 배율 오차 초기값 (노이즈 없을 때 실측: 정속에서 바퀴가 0.45% 빠름)
    'wheel_scale_sigma': 0.02,        # 배율 오차 초기 σ (GNSS로 배움)
    'wheel_scale_rate': 0.002,        # 배율 오차가 변하는 정도 (1/√s, 랜덤 워크)
    'wheel_slip_per_accel': 0.0085,   # 가속할 때 구동 슬립: 바퀴 속력이 (이 값 × 가속도 m/s²)만큼 더 큼 (실측)
    'accel_filter_tau': 0.2,          # IMU 전후 가속도 저역 통과 시간 (s)
    'lateral_slip': 0.006,            # 곡선에서 후륜 차축이 바깥으로 미끄러지는 속도 = 이 값 × v² × 각속도 (s/m... 실측 맞춤)
    'yaw_model_sigma': 0.002,         # 방향 모델 오차 (rad/√s)
    'extrapolate_max': 0.1,           # 내보낼 때 외삽하는 최대 시간 (s)
}


def stamp(m):
    """메시지의 측정 시각 (s)."""
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Localization(Node):
    def __init__(self):
        super().__init__('hk_mit_localization')
        declare_file_params(self)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
            setattr(self, k, type(v)(self.get_parameter(k).value))
        self.course = None
        self.s = np.zeros(N)             # EKF 상태
        self.P = np.zeros((N, N))        # EKF 공분산
        self.ready = False
        self.started = False             # 시작(START)을 한 번이라도 받았는지 (복귀 때 바이어스를 이어 갈지)
        self.reset(None)

        self.create_subscription(NavSatFix, '/hk/sensors/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, '/hk/sensors/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/hk/sensors/wheel_encoder/speed', self.on_wheel, qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, '/hk/local/odom', 10)
        self.tf = TransformBroadcaster(self)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(1.0 / RATE_HZ, self.publish)

    # ---- 시작·복귀 -----------------------------------------------------------------------------------
    def on_reset(self, checkpoint, reload):
        if reload or self.course is None:
            self.course = load_course(self)
        self.reset(self.course.pose(checkpoint), keep_bias=not reload)

    def on_end(self):
        self.ready = False

    def reset(self, pose, keep_bias=False):
        """(x, y, yaw) 자세에서 처음부터. pose가 None이면 자세를 모르는 상태(내보내지 않음).
        keep_bias(복귀)면 자이로·GNSS 바이어스 추정과 그 불확실성을 이어 가고, 시작이면 바이어스도 처음부터."""
        x, y, yaw = pose if pose else (0.0, 0.0, 0.0)
        bias, bias_P = self.s[BG:].copy(), self.P[BG:, BG:].copy()
        self.s = np.array([x, y, yaw] + [0.0] * (N - 3))
        self.P = np.zeros((N, N))
        self.P[X, X] = self.P[Y, Y] = 0.02 ** 2            # 출발·복귀 자세는 코스 파일 값(정확)
        self.P[YAW, YAW] = 0.002 ** 2
        if keep_bias and self.started:
            self.s[BG:], self.P[BG:, BG:] = bias, bias_P
        else:
            self.P[BG, BG] = self.gyro_bias_sigma ** 2 + self.gyro_drift_sigma ** 2
            self.P[MX, MX] = self.P[MY, MY] = self.gnss_bias_sigma ** 2
            self.s[KS], self.P[KS, KS] = self.wheel_scale0, self.wheel_scale_sigma ** 2
        self.ready = pose is not None
        self.started = self.started or self.ready
        self.speed = 0.0                 # 바퀴 속력 (m/s)
        self.v = 0.0                     # 배율·슬립을 고친 속력 (m/s)
        self.ax = 0.0                    # 저역 통과한 전후 가속도 (m/s²)
        self.w = 0.0                     # 바이어스를 뺀 마지막 각속도 (외삽용)
        self.yaw_time = None             # 마지막으로 적분한 IMU 측정 시각 (s)
        self.bad_gnss = 0                # 이어서 크게 어긋난 GNSS 수

    # ---- 센서마다 불림 ----------------------------------------------------------------------------
    def on_wheel(self, m):
        """바퀴 속도 (부호 있는 m/s, 슬립 포함)."""
        self.speed = m.twist.linear.x

    def on_imu(self, m):
        if not self.ready:
            return
        t = stamp(m)
        if self.yaw_time is not None and 0.0 < t - self.yaw_time <= 1.5:   # IMU 주기 최대 1초
            dt = t - self.yaw_time
            self.ax += min(dt / self.accel_filter_tau, 1.0) * (m.linear_acceleration.x - self.ax)
            self.predict(dt, m.angular_velocity.z)
        self.yaw_time = t

    def predict(self, dt, gz):
        """추측 항법: GNSS 사이(20 Hz)와 음영 구간을 바퀴 속력·방향으로 이어 갑니다."""
        s, vw = self.s, self.speed
        F, Q = np.eye(N), np.zeros((N, N))
        still = abs(vw) < STILL
        self.v = v = vw * (1.0 - s[KS] - self.wheel_slip_per_accel * self.ax)
        if still:
            self.w = 0.0                 # 정지: 실제 각속도는 0. 방향·위치는 그대로 두고 아래에서 바이어스 갱신
        else:
            self.w = gz - s[BG]
            mid = s[YAW] + 0.5 * self.w * dt
            c, sn = math.cos(mid), math.sin(mid)
            vy = -self.lateral_slip * v * abs(v) * self.w            # 곡선 바깥쪽 옆 미끄러짐
            s[X] += (v * c - vy * sn) * dt
            s[Y] += (v * sn + vy * c) * dt
            s[YAW] = wrap(s[YAW] + self.w * dt)
            F[X, YAW], F[Y, YAW], F[YAW, BG] = -v * sn * dt, v * c * dt, -dt
            F[X, KS], F[Y, KS] = -vw * c * dt, -vw * sn * dt
            q = (self.speed_sigma * v * dt) ** 2               # 진행 방향 거리 오차
            Q[X, X], Q[Y, Y] = q * c * c + 1e-6 * dt, q * sn * sn + 1e-6 * dt
            Q[X, Y] = Q[Y, X] = q * c * sn
            Q[YAW, YAW] = (self.gyro_sigma * dt) ** 2 + self.yaw_model_sigma ** 2 * dt
        a = math.exp(-dt / self.gnss_bias_tau)                # 다중경로 바이어스는 τ로 0을 향해 줄어듦
        s[MX] *= a
        s[MY] *= a
        F[MX, MX] = F[MY, MY] = a
        Q[MX, MX] = Q[MY, MY] = self.gnss_bias_sigma ** 2 * (1 - a * a)
        Q[BG, BG] = 2 * self.gyro_drift_sigma ** 2 / self.gyro_drift_tau * dt
        if not still:
            Q[KS, KS] = self.wheel_scale_rate ** 2 * dt     # 배율 오차는 달리면서 천천히 변함
        self.P = F @ self.P @ F.T + Q
        if still:                                             # 영각속도 갱신: 측정 각속도 = bg
            H = np.zeros((1, N))
            H[0, BG] = 1.0
            self.update(np.array([gz - s[BG]]), H, np.array([[self.gyro_sigma ** 2]]))

    def update(self, innov, H, R):
        """EKF 측정 갱신 (Joseph 형식)."""
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + R)
        self.s += K @ innov
        self.s[YAW] = wrap(self.s[YAW])
        I_KH = np.eye(N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def on_gps(self, m):
        """GNSS. 음영 구간에서는 오지 않습니다."""
        if not self.ready:
            return
        s = self.s
        gx, gy = ll_to_map(m.latitude, m.longitude, self.course.origin)
        # GNSS는 측정 시각의 위치이므로 그 뒤 추측 항법으로 이동한 만큼 앞으로 옮겨 비교
        lag = 0.0 if self.yaw_time is None else min(max(self.yaw_time - stamp(m), 0.0), 0.5)
        gx += self.v * math.cos(s[YAW]) * lag
        gy += self.v * math.sin(s[YAW]) * lag
        H = np.zeros((2, N))
        H[0, X] = H[0, MX] = H[1, Y] = H[1, MY] = 1.0
        R = np.eye(2) * self.gnss_sigma ** 2
        innov = np.array([gx - s[X] - s[MX], gy - s[Y] - s[MY]])
        d2 = float(innov @ np.linalg.solve(H @ self.P @ H.T + R, innov))
        if d2 > self.gnss_gate ** 2:                          # 튀는 값은 버림
            self.bad_gnss = self.bad_gnss + 1 if math.hypot(*innov) > self.gnss_reset_m else 0
            if self.bad_gnss >= self.gnss_reset_count:        # 계속 크게 어긋나면 GNSS로 다시 맞춤
                s[X], s[Y] = gx - s[MX], gy - s[MY]
                self.P[:2, :] = 0.0
                self.P[:, :2] = 0.0
                self.P[X, X] = self.P[Y, Y] = self.gnss_sigma ** 2 + self.gnss_bias_sigma ** 2
                self.P[YAW, YAW] += 0.05 ** 2
                self.bad_gnss = 0
            return
        self.bad_gnss = 0
        self.update(innov, H, R)

    # ---- 50 Hz로 내보냄 ----------------------------------------------------------------------------
    def publish(self):
        if not self.ready:
            return
        now = self.get_clock().now()
        x, y, yaw = self.s[X], self.s[Y], self.s[YAW]
        if self.yaw_time is not None:                # 마지막 IMU 뒤 지금까지 짧게 외삽
            dt = min(max(now.nanoseconds * 1e-9 - self.yaw_time, 0.0), self.extrapolate_max)
            mid = yaw + 0.5 * self.w * dt
            x += self.v * math.cos(mid) * dt
            y += self.v * math.sin(mid) * dt
            yaw = wrap(yaw + self.w * dt)
        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        m.twist.twist.linear.x = float(self.v if self.speed else 0.0)
        self.pub.publish(m)
        t = TransformStamped()                       # the same estimate as TF, for RViz
        t.header = m.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = m.pose.pose.position.x, m.pose.pose.position.y
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
