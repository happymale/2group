"""HEVEN 2026 MIT 기본 코드 2/3: 경로 계획 (planner). 참가자가 고칠 곳.

전역 경로(0.5 m 간격)에서 차 앞쪽 30 m를 잘라 20 Hz로 /hk/plan/local_path (nav_msgs/Path, frame map)에 냅니다.
각 점의 방향(orientation)은 그 점의 경로 방향입니다. 경로 끝을 지나면 빈 경로를 내서 control.py가 멈추게 합니다.

지금은 전역 경로를 그대로 따라가기만 합니다. 장애물을 피해 옆으로 비켜 가는 경로, 주차 경로, 굽은 길에서
안쪽으로 붙는 경로 등은 여기서 만들면 됩니다(lidar_perception.py에서 LiDAR(/hk/sensors/lidar/scan)를 받아 쓰세요).
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from .course import declare_file_params, load_route, Route
from .sim_events import SimEvents

RATE_HZ = 20.0
PARAMS = {              # config/bringup.yaml에서 바꿉니다. 아래는 기본값
    'ahead_m': 30.0,    # 차 앞으로 내보낼 길이 (m)
    'behind_m': 2.0,    # 차 뒤로 남길 길이 (m)
}
SETTLE_S = 0.5          # 시작·복귀 뒤 이 시간 동안은 경로 전체에서 위치를 다시 찾음 (늦게 온 예전 위치 대비)


class Planner(Node):
    def __init__(self):
        super().__init__('hk_mit_planner')
        declare_file_params(self)
        for name, default in PARAMS.items():           # self.ahead_m, self.behind_m
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))
        self.route = Route()
        self.odom = None
        self.active = False
        self.reset_time = None
        self.create_subscription(Odometry, '/hk/local/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Path, '/hk/plan/local_path', 10)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(1.0 / RATE_HZ, self.plan)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_reset(self, checkpoint, reload):
        if reload or not self.route.points:
            self.route = load_route(self)
            self.get_logger().info(f'전역 경로 {len(self.route.points)}점')
        self.route.reset()
        self.odom = None
        self.reset_time = self.now()
        self.active = True

    def on_end(self):
        self.active = False

    def on_odom(self, m):
        self.odom = m

    def plan(self):
        if not self.active or self.odom is None or len(self.route.points) < 2:
            return
        if self.now() - self.reset_time < SETTLE_S:
            self.route.reset()
        p = self.odom.pose.pose.position
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        i = self.route.nearest(p.x, p.y)
        if not self.route.past_end(p.x, p.y):                  # 경로 끝을 지났으면 빈 경로
            first = max(0, i - int(self.behind_m / 0.5))
            last = min(len(self.route.points), i + int(self.ahead_m / 0.5) + 1)
            for k in range(first, last):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x, ps.pose.position.y = self.route.points[k]
                yaw = self.route.yaw_at(k)
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(ps)
        self.pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
