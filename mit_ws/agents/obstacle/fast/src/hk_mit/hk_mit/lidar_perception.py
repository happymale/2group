"""Jump Distance 인지와 선택적인 LiDAR CSV 기록.

입력: /hk/sensors/lidar/scan (sensor_msgs/LaserScan).
기록: ros2 run hk_mit lidar_perception --ros-args -p record_csv:=true -p csv_path:=/tmp/lidar.csv
통합 기록: ros2 launch hk_mit bringup.launch.py record_lidar_csv:=true lidar_csv_path:=/tmp/lidar.csv

record_csv 기본값은 false. true로 켜고 csv_path를 비우면 ~/hk_mit_data/lidar_<UTC 시각>.csv를
새로 만듭니다. 기존 파일은 덮어쓰지 않습니다. 각 스캔을 기록한 뒤 flush합니다.
거리의 inf/-inf/nan도 원본대로 남기며, 무효 측정은 valid=0이고 XY 칸은 비웁니다.
XY는 메시지 frame_id(보통 lidar_link) 기준이며 map 좌표로 변환하지 않습니다.
"""
import csv
from collections import deque
from datetime import datetime, timezone
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .avoidance import convex_hull, jump_distance_clusters
from .sim_events import SimEvents


CSV_FIELDS = (
    'scan_id', 'stamp_sec', 'stamp_nanosec', 'frame_id', 'beam_index',
    'beam_time_offset_s', 'angle_rad', 'range_m', 'range_min_m', 'range_max_m',
    'valid', 'x_lidar_m', 'y_lidar_m', 'intensity', 'scan_time_s',
)


class LidarPerception(Node):
    def __init__(self):
        super().__init__('hk_mit_lidar_perception')
        self.scan = None
        self.scan_count = 0
        self.csv_file = None
        self.csv_writer = None
        self.pending_scans = deque(maxlen=10)
        self.reset_stamp = None
        self.declare_parameter('record_csv', False)
        self.declare_parameter('csv_path', '')
        for name, default in {'jump_base': 0.3, 'jump_scale': 1.5, 'max_obstacle_range': 45.0,
                              'scan_timeout': 0.35, 'max_invalid_fraction': 0.1}.items():
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))
        if (not all(math.isfinite(v) for v in (self.jump_base, self.jump_scale, self.max_obstacle_range,
                                              self.scan_timeout, self.max_invalid_fraction))
                or self.jump_base <= 0 or self.jump_scale < 0 or self.max_obstacle_range <= 0
                or self.scan_timeout <= 0 or not 0 <= self.max_invalid_fraction < 1):
            raise ValueError('Invalid LiDAR perception parameters')
        if self.get_parameter('record_csv').value:
            configured_path = self.get_parameter('csv_path').value
            if configured_path:
                path = Path(configured_path).expanduser()
            else:
                timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
                path = Path.home() / 'hk_mit_data' / f'lidar_{timestamp}.csv'
            path = path.absolute()
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                self.csv_file = path.open('x', newline='', encoding='utf-8')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(CSV_FIELDS)
                self.csv_file.flush()
            except OSError:
                if self.csv_file is not None:
                    self.csv_file.close()
                self.destroy_node()
                raise
            self.get_logger().info(f'LiDAR CSV 저장: {path}')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        # A long FIFO here can make dynamic TF permanently lag behind scans.
        # Retain the latest TF message; the TF buffer keeps the time history.
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=QoSProfile(depth=1))
        self.obstacle_pub = self.create_publisher(PointCloud2, '/hk/perception/obstacles', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/hk/perception/markers', 1)
        self.create_subscription(LaserScan, '/hk/sensors/lidar/scan', self.on_scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.02, self.publish_pending_scan)
        # TF 지연 대체용: 최신 odom으로 '지금 정지 중'인지 확인 (TF 기록만 보면 지연된 과거의 정지를 믿게 됨)
        self.odom_stamp = None
        self.still_since = None
        self.create_subscription(Odometry, '/hk/local/odom', self.on_odom, 10)
        SimEvents(self, self.on_reset, self.on_end)

    def on_reset(self, checkpoint, reload):
        self.pending_scans.clear()
        self.scan = None
        self.reset_stamp = self.get_clock().now().nanoseconds
        self.clear_markers()

    def on_end(self):
        self.pending_scans.clear()
        self.clear_markers()

    def clear_markers(self):
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def on_scan(self, m):
        self.scan = m
        self.pending_scans.append(m)
        scan_id = self.scan_count
        self.scan_count += 1
        if self.csv_writer is not None:
            self.record_scan(m, scan_id)
        self.publish_pending_scan()

    def record_scan(self, m, scan_id):
        for i, distance in enumerate(m.ranges):
            angle = m.angle_min + i * m.angle_increment
            valid = math.isfinite(distance) and m.range_min <= distance <= m.range_max
            x = distance * math.cos(angle) if valid else ''
            y = distance * math.sin(angle) if valid else ''
            intensity = m.intensities[i] if i < len(m.intensities) else ''
            self.csv_writer.writerow((
                scan_id, m.header.stamp.sec, m.header.stamp.nanosec,
                m.header.frame_id, i, i * m.time_increment, angle, distance,
                m.range_min, m.range_max, int(valid), x, y, intensity, m.scan_time,
            ))
        self.csv_file.flush()

    @property
    def pending_scan(self):
        return self.pending_scans[0] if self.pending_scans else None

    def publish_pending_scan(self):
        while self.pending_scans:
            m = self.pending_scans[0]
            stamp_ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
            if age > self.scan_timeout or (self.reset_stamp is not None and stamp_ns < self.reset_stamp):
                self.pending_scans.popleft()
                continue
            # /clock and scan travel independently. If the corresponding clock
            # update is delayed, retain the scan instead of dropping it.
            if age < 0:
                return
            if self.process_scan(m):
                self.pending_scans.popleft()
            return

    def process_scan(self, m):
        """Return false only when this scan is still waiting for its exact TF."""
        ranges = np.asarray(m.ranges, dtype=float)
        metadata = [m.angle_min, m.angle_increment, m.range_min, m.range_max]
        if (not len(ranges) or not m.header.frame_id or not np.isfinite(metadata).all()
                or m.angle_increment == 0 or not 0 < m.range_min < m.range_max):
            return True
        known = np.isinf(ranges) | (np.isfinite(ranges) & (ranges >= m.range_min) & (ranges <= m.range_max))
        if 1.0 - np.mean(known) > self.max_invalid_fraction:
            self.get_logger().warning('LiDAR 무효 측정 과다: 장애물 관측 발행 중단', throttle_duration_sec=2.0)
            return True
        try:
            transform = self.tf_buffer.lookup_transform('map', m.header.frame_id, Time.from_msg(m.header.stamp))
        except TransformException as exc:
            # 부하가 크면 TF 수신이 수 초 밀려 측정 시각 TF가 끝내 오지 않고 스캔이 모두 버려집니다
            # (주차 칸 검출 대기 중 멈춤). 차가 서 있으면 최신 TF가 측정 시각 자세와 같으므로 그것을 씁니다.
            transform = self.stationary_transform(
                m.header.frame_id, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            if transform is None:
                self.get_logger().warning(f'측정 시각의 map <- {m.header.frame_id} TF 대기: {exc}', throttle_duration_sec=2.0)
                return False
        clusters = jump_distance_clusters(
            ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max,
            self.jump_base, self.jump_scale, self.max_obstacle_range,
        )
        t, q = transform.transform.translation, transform.transform.rotation
        rotation = np.array([
            [1 - 2 * (q.y*q.y + q.z*q.z), 2 * (q.x*q.y - q.z*q.w)],
            [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x*q.x + q.z*q.z)],
        ])
        clusters = [points @ rotation.T + [t.x, t.y] for points in clusters]
        header = Header(stamp=m.header.stamp, frame_id='map')
        fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                  for name, offset, datatype in [('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                                                  ('z', 8, PointField.FLOAT32), ('cluster_id', 12, PointField.UINT32)]]
        points = [(float(x), float(y), 0.0, i) for i, cluster in enumerate(clusters) for x, y in cluster]
        # An empty cloud is a fresh no-obstacle observation. No cloud means
        # unavailable/invalid data and must lead to a planner timeout.
        self.obstacle_pub.publish(point_cloud2.create_cloud(header, fields, points))
        self.publish_markers(header, clusters)
        return True

    @staticmethod
    def tf_stamp_s(t):
        return t.header.stamp.sec + t.header.stamp.nanosec * 1e-9

    def on_odom(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        if v > 0.05:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = stamp
        self.odom_stamp = stamp

    def stationary_transform(self, frame_id, scan_stamp, window_s=0.5, max_move=0.03, max_turn=0.005):
        """odom상 스캔 시각까지 정지가 이어지고, 최신 TF와 window_s 전 TF도 같으면 최신 TF, 아니면 None."""
        if (self.odom_stamp is None or self.still_since is None or abs(self.odom_stamp - scan_stamp) > 0.3
                or scan_stamp - self.still_since < window_s):
            return None
        try:
            latest = self.tf_buffer.lookup_transform('map', frame_id, Time())
            stamp = Time.from_msg(latest.header.stamp)
            earlier = self.tf_buffer.lookup_transform('map', frame_id, stamp - Duration(seconds=window_s))
        except TransformException:
            return None
        a, b = latest.transform, earlier.transform
        moved = math.hypot(a.translation.x - b.translation.x, a.translation.y - b.translation.y)
        turned = abs(a.rotation.z - b.rotation.z) + abs(a.rotation.w - b.rotation.w)
        if moved > max_move or turned > max_turn or self.tf_stamp_s(latest) < self.still_since:
            return None
        self.get_logger().info('측정 시각 TF 지연: 정지 중이라 최신 TF 사용', throttle_duration_sec=2.0)
        return latest

    def publish_markers(self, header, clusters):
        markers = [Marker(action=Marker.DELETEALL)]
        for i, cluster in enumerate(clusters):
            color = ((i * 0.37) % 0.7 + 0.3, (i * 0.61) % 0.7 + 0.3, (i * 0.23) % 0.7 + 0.3)
            marker = Marker(header=header, ns='obstacle_contours', id=2*i,
                            type=Marker.LINE_STRIP if len(cluster) > 1 else Marker.SPHERE_LIST, action=Marker.ADD)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.10
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            outline = convex_hull(cluster) if np.max(np.ptp(cluster, axis=0)) <= 8 else cluster
            if len(outline) > 2 and np.max(np.ptp(cluster, axis=0)) <= 8:
                outline = np.vstack((outline, outline[0]))
            marker.points = [Point(x=float(x), y=float(y), z=0.2) for x, y in outline]
            marker.lifetime = Duration(seconds=0.4).to_msg()
            markers.append(marker)
            label = Marker(header=header, ns='obstacle_ids', id=2*i+1, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.orientation.w = 1.0
            label.pose.position.x, label.pose.position.y = map(float, np.mean(cluster, axis=0))
            label.pose.position.z = 1.0
            label.scale.z = 0.4
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'cluster {i} ({len(cluster)} hits)'
            label.lifetime = marker.lifetime
            markers.append(label)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def destroy_node(self):
        try:
            if self.csv_file is not None:
                self.csv_file.close()
        finally:
            result = super().destroy_node()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = LidarPerception()
        # TransformListener uses its own reentrant callback group. Let it
        # receive transforms while scan processing/CSV/markers occupy the
        # node's mutually exclusive callback group.
        executor = MultiThreadedExecutor(num_threads=2)
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
