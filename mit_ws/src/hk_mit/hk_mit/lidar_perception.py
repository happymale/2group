"""HEVEN 2026 MIT LiDAR 인지 (lidar_perception). 참가자가 채울 곳.

/hk/sensors/lidar/scan (sensor_msgs/LaserScan, 10 Hz, 360°, 720빔, frame lidar_link)을 받아 self.scan에 둡니다.
지금은 받기만 하고 아무것도 내지 않습니다. 기본 주행(bringup)에서는 띄우지 않고, perception:=true일 때만 뜹니다.

쓰는 방법 예 (on_scan 안에서):
    import numpy as np
    r = np.asarray(m.ranges)                                   # 미검출 +Inf, 최소 거리 안 -Inf
    a = m.angle_min + np.arange(len(r)) * m.angle_increment    # 각 빔의 각도 (rad, lidar_link 기준)
    ok = np.isfinite(r)
    x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])          # lidar_link 좌표의 점
결과(장애물 위치 등)를 토픽으로 내서 planner.py가 받게 하면 됩니다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class LidarPerception(Node):
    def __init__(self):
        super().__init__('hk_mit_lidar_perception')
        self.scan = None
        self.create_subscription(LaserScan, '/hk/sensors/lidar/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, m):
        self.scan = m                    # 여기서 점을 처리하세요


def main(args=None):
    rclpy.init(args=args)
    node = LidarPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
