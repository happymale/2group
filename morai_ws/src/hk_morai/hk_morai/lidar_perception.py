"""HEVEN 2026 MORAI LiDAR 인지 (lidar_perception). 참가자가 채울 곳.

/hk/sensors/lidar/points (sensor_msgs/PointCloud2, VLP-16 16채널, 10 Hz, frame lidar_link)를 받아 self.cloud에
둡니다. 지금은 받기만 하고 아무것도 내지 않습니다. 기본 주행(bringup)에서는 띄우지 않고, perception:=true일 때만
뜹니다. 장착 위치는 config/sensor_mounts.yaml에 있습니다(기본: base_link에서 x 1.15, z 1.7685 m).

쓰는 방법 예 (on_cloud 안에서):
    from sensor_msgs_py import point_cloud2
    pts = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)   # (N, 3) lidar_link 좌표
결과(장애물 위치 등)를 토픽으로 내서 planner.py가 받게 하면 됩니다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2


class LidarPerception(Node):
    def __init__(self):
        super().__init__('hk_morai_lidar_perception')
        self.cloud = None
        self.create_subscription(PointCloud2, '/hk/sensors/lidar/points', self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, m):
        self.cloud = m                   # 여기서 점을 처리하세요


def main(args=None):
    rclpy.init(args=args)
    node = LidarPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
