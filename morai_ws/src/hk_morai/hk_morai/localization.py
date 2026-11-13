"""HEVEN 2026 MORAI 기본 코드 1/3: 위치 추정 (localization). 참가자가 고칠 곳.

센서를 받아 차의 위치·방향·속력을 구하고, 50 Hz로 /hk/local/odom (nav_msgs/Odometry, frame map)에 냅니다.
같은 값을 TF(map -> base_link)로도 냅니다. planner.py와 control.py가 이 값을 씁니다.
지금은 가장 단순하게 두었습니다.

  위치  : GNSS 위경도를 지도 좌표로 바꾼 값을 그대로 씁니다(노이즈 조합에 따라 노이즈가 들어 있음).
  방향  : IMU 메시지의 자세(orientation)를 그대로 씁니다(브리지가 지도 기준으로 맞춰 줌).
  속력  : MORAI에는 바퀴 속도 센서가 없어서, 0.5초 동안 GNSS로 움직인 거리로 구합니다.

노이즈를 거르는 필터, GNSS 사이를 속력·방향으로 이어 가는 추측 항법은 넣지 않았습니다.
LiDAR·카메라는 lidar_perception.py, camera_perception.py에서 받을 수 있습니다.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

from .course import declare_file_params, load_course
from .sim_events import SimEvents
from .utm import ll_to_map

RATE_HZ = 50.0


def stamp(m):
    """메시지의 측정 시각 (s)."""
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Localization(Node):
    def __init__(self):
        super().__init__('hk_morai_localization')
        declare_file_params(self)
        self.course = None
        self.reset(None)

        self.create_subscription(NavSatFix, '/hk/sensors/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, '/hk/sensors/imu/data', self.on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, '/hk/local/odom', 10)
        self.tf = TransformBroadcaster(self)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(1.0 / RATE_HZ, self.publish)

    # ---- 시작·복귀 -----------------------------------------------------------------------------------
    def on_reset(self, checkpoint, reload):
        if reload or self.course is None:
            self.course = load_course(self)
        self.reset(self.course.pose(checkpoint))

    def on_end(self):
        self.ready = False

    def reset(self, pose):
        """(x, y, yaw) 자세에서 처음부터. pose가 None이면 자세를 모르는 상태(내보내지 않음)."""
        self.x, self.y, self.yaw = pose if pose else (0.0, 0.0, 0.0)
        self.ready = pose is not None
        self.speed = 0.0
        self.gps_history = deque()       # 최근 0.5초의 (시각, x, y): 속력 계산용

    # ---- 센서마다 불림 ----------------------------------------------------------------------------
    def on_imu(self, m):
        if not self.ready:
            return
        q = m.orientation                            # 지도 기준 자세
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_gps(self, m):
        if not self.ready:
            return
        x, y = ll_to_map(m.latitude, m.longitude, self.course.origin)
        t = stamp(m)
        self.gps_history.append((t, x, y))
        while t - self.gps_history[0][0] > 0.5:
            self.gps_history.popleft()
        (t0, x0, y0), (t1, x1, y1) = self.gps_history[0], self.gps_history[-1]
        if t1 - t0 > 0.2:                            # 0.5초 동안의 GNSS 이동 거리로 속력
            self.speed = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
        self.x, self.y = x, y                         # GNSS 위치를 그대로

    # ---- 50 Hz로 내보냄 ----------------------------------------------------------------------------
    def publish(self):
        if not self.ready:
            return
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = float(self.x), float(self.y)
        m.pose.pose.orientation.z = math.sin(self.yaw / 2)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2)
        m.twist.twist.linear.x = float(self.speed)
        self.pub.publish(m)
        t = TransformStamped()
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
