"""HEVEN 2026 MIT 기본 코드 1/3: 위치 추정 (localization). 참가자가 고칠 곳.

센서를 받아 차의 위치·방향·속력을 구하고, 50 Hz로 /hk/local/odom (nav_msgs/Odometry, frame map)에 냅니다.
같은 값을 TF(map -> base_link)로도 내서 RViz에서 LiDAR 점과 경로를 지도 위에 볼 수 있습니다.
planner.py와 control.py가 이 값을 씁니다. 지금은 가장 단순하게 두었습니다.

  위치  : GNSS 위경도를 지도 좌표로 바꾼 값을 그대로 씁니다. 노이즈도 그대로 들어가고,
          GNSS가 안 오면(음영 구간) 마지막 값에 멈춰 있습니다.
  방향  : IMU 각속도(z)를 측정 시각 간격으로 적분합니다.
  속력  : 바퀴 속도 센서(/hk/sensors/wheel_encoder/speed) 값을 그대로 씁니다.

GNSS 사이와 음영 구간을 속력·방향으로 이어 가는 추측 항법, 노이즈를 거르는 필터는 넣지 않았습니다.
여기에 직접 만들어 보세요. 시작·복귀 때는 reset()이 그 자리의 자세로 다시 시작합니다.
"""
import math

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


def stamp(m):
    """메시지의 측정 시각 (s)."""
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Localization(Node):
    def __init__(self):
        super().__init__('hk_mit_localization')
        declare_file_params(self)
        self.course = None
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
        self.reset(self.course.pose(checkpoint))

    def on_end(self):
        self.ready = False

    def reset(self, pose):
        """(x, y, yaw) 자세에서 처음부터. pose가 None이면 자세를 모르는 상태(내보내지 않음)."""
        self.x, self.y, self.yaw = pose if pose else (0.0, 0.0, 0.0)
        self.ready = pose is not None
        self.speed = 0.0
        self.yaw_time = None             # 마지막으로 적분한 IMU 측정 시각 (s)

    # ---- 센서마다 불림 ----------------------------------------------------------------------------
    def on_wheel(self, m):
        """바퀴 속도 (부호 있는 m/s, 슬립 포함)."""
        self.speed = m.twist.linear.x

    def on_imu(self, m):
        if not self.ready:
            return
        t = stamp(m)
        if self.yaw_time is not None and 0.0 < t - self.yaw_time <= 1.5:   # IMU 주기 최대 1초
            self.yaw += m.angular_velocity.z * (t - self.yaw_time)
        self.yaw_time = t

    def on_gps(self, m):
        """GNSS. 음영 구간에서는 오지 않습니다."""
        if not self.ready:
            return
        self.x, self.y = ll_to_map(m.latitude, m.longitude, self.course.origin)   # GNSS 위치를 그대로

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
