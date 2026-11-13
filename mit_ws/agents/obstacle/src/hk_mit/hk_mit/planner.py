"""Reference-route following with static-obstacle Frenet avoidance.

Avoidance runs only inside avoidance zones (by default the obstacle mission zones of course.json, widened by
avoidance_zone_before/after). Elsewhere the corridor is only 4.39 m wide: its walls are not obstacles, and
checking the vehicle body against them with safety margins in the R≈6 m S-curves rejects every candidate.
Inside a zone, fresh map-frame obstacle clouds are required; missing perception or an infeasible plan
publishes an empty Path and zero speed.
"""
from collections import Counter
from dataclasses import replace
import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .avoidance import AvoidanceConfig, Ego, ReferencePath, footprint_corners, plan_avoidance, speed_profile
from .detour import DetourMap, plan_detour, read_obstacles, scene_signature
from .course import COURSE_DIR, declare_file_params, load_route, Route
from .sim_events import SimEvents

RATE_HZ = 10.0
PARAMS = {              # config/bringup.yaml에서 바꿉니다. 아래는 기본값
    'ahead_m': 30.0,    # 차 앞으로 내보낼 길이 (m)
    'behind_m': 2.0,    # 차 뒤로 남길 길이 (m)
    'path_step': 0.2,
    'road_half_width': 3.75,
    'safety_margin': 0.35,
    'surface_padding': 0.25,
    'vehicle_length': 4.635,
    'vehicle_width': 1.892,
    'rear_overhang': 0.79,
    'wheelbase': 3.0,
    'max_steer_deg': 35.4,
    'max_lateral_accel': 1.8,
    'cruise_speed': 5.0,    # 직선 최고 속력 (m/s). 곡선은 아래 속도 프로파일이 줄입니다
    'avoidance_speed': 3.0,
    'return_time': 2.5,     # 장애물을 지난 뒤 기준 경로로 돌아오는 시간 (s, avoidance_speed 기준 길이)
    'tight_speed': 3.0,     # 여유를 줄여 찾은 경로(tight)의 속력 상한 (m/s)
    'return_gap': 1.5,      # 마지막으로 본 장애물 점 뒤에 더 기다렸다 복귀하는 거리 (m)
    'corner_lateral_accel': 4.0,   # 기준 경로 곡선에서 허용하는 횡가속도 (m/s²)
    'profile_accel': 2.0,          # 속도 프로파일 가속 한계 (m/s²)
    'profile_decel': 3.0,          # 속도 프로파일 감속 한계 (m/s²). 곡선 앞에서 미리 줄임
    'speed_preview_s': 0.6,        # 이 시간만큼 앞 구간의 제한까지 미리 반영 (구동 지연·제어 반응)
    'avoidance_zone_before': 15.0,  # 장애물 미션 구간 시작 이만큼(m) 앞부터 회피를 켬
    'avoidance_zone_after': 8.0,    # 장애물 미션 구간 끝 이만큼(m) 뒤까지 회피를 켬 (기준 경로로 돌아올 거리)
    'obstacle_timeout': 0.35,
    'odom_timeout': 0.35,
    'clock_tolerance': 0.05,
}
SETTLE_S = 0.5          # 시작·복귀 뒤 이 시간 동안은 경로 전체에서 위치를 다시 찾음 (늦게 온 예전 위치 대비)


class Planner(Node):
    def __init__(self):
        super().__init__('hk_mit_planner')
        declare_file_params(self)
        for name, default in PARAMS.items():           # self.ahead_m, self.behind_m
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))
        self.declare_parameter('avoidance_enabled', True)
        self.avoidance_enabled = bool(self.get_parameter('avoidance_enabled').value)
        self.declare_parameter('lateral_offsets', [-2.2, -1.8, -1.2, 0.0, 1.2, 1.8, 2.2])
        self.declare_parameter('transition_times', [2.0, 3.0, 4.0])
        # 구간별 속력 상한 [시작 s, 끝 s, 상한 m/s, ...] (s = route.csv 경로 거리 m). 빈 목록이면 없음
        self.declare_parameter('speed_zones', [0.0, 0.0, 100.0])
        # 회피를 켤 구간 [시작 s, 끝 s, ...]. [-1.0]이면 course.json의 장애물 미션 구간에서 자동으로 구함
        self.declare_parameter('avoidance_zones', [-1.0])
        self.declare_parameter('avoidance_mission_types', ['obstacle1', 'obstacle3'])
        # 시뮬레이터가 시도마다 쓰는 코스 파일(켠 옵션 포함). 있으면 켜진 장애물 미션 구간에서만 회피합니다
        self.declare_parameter('attempt_course_json', '/data/current/course.json')
        self.avoidance_ranges = None     # 경로를 읽기 전(시험에서 reference를 직접 넣은 경우)은 코스 전체
        self.obstacle_zones = {}         # 회피 구간 → 원래 장애물 미션 구간 (경로 거리)
        self.plan_level = 0              # 직전 계획의 여유 단계 (avoidance.relaxation_levels)
        # 전역 경로 우회(detour.py): 구간 안 LiDAR 점을 모아 장애물 좌표로 기준 경로 구간을 바꿈.
        # 실패하면 아래 Frenet 후보 회피(plan_avoidance)를 그대로 씀
        self.declare_parameter('detour_enabled', True)
        self.detour_enabled = bool(self.get_parameter('detour_enabled').value)
        # 우회 경로 속력: 지그재그 추종 지연(5 m/s에서 약 0.2 m)이 장애물 여유를 깎으므로 횡가속도를 낮게
        self.declare_parameter('detour_lateral_accel', 2.0)
        self.detour_lateral_accel = float(self.get_parameter('detour_lateral_accel').value)
        self.declare_parameter('detour_slow_speed', 4.0)     # margin을 반으로 줄인 우회 경로의 속력 상한
        self.detour_slow_speed = float(self.get_parameter('detour_slow_speed').value)
        self.detour_map = DetourMap()
        self.detour = None
        self.detour_zone = None          # 우회 경로를 만든 회피 구간
        self.detour_cap = math.inf       # 여유를 줄여 만든 우회 경로의 속력 상한
        self.current_zone = None         # 지금 있는 회피 구간 (관측점 누적 범위)
        self.last_station = 0.0
        self.wall_offset = 0.0           # 장애물 구간 벽으로 잰 추정 위치의 옆 어긋남 (m, update_wall_offset)
        self.cfg = AvoidanceConfig(
            horizon=self.ahead_m, step=self.path_step, road_half_width=self.road_half_width,
            vehicle_length=self.vehicle_length, vehicle_width=self.vehicle_width,
            rear_overhang=self.rear_overhang, wheelbase=self.wheelbase,
            max_steer_deg=self.max_steer_deg, margin=self.safety_margin,
            surface_padding=self.surface_padding, max_lateral_accel=self.max_lateral_accel,
            cruise_speed=self.cruise_speed, avoidance_speed=self.avoidance_speed,
            offsets=tuple(self.get_parameter('lateral_offsets').value),
            transition_times=tuple(self.get_parameter('transition_times').value),
            return_time=self.return_time, decel=self.profile_decel, tight_speed=self.tight_speed,
            return_gap=self.return_gap,
        )
        if (not all(math.isfinite(v) for v in (self.obstacle_timeout, self.odom_timeout, self.clock_tolerance))
                or self.obstacle_timeout <= 0 or self.odom_timeout <= 0 or self.clock_tolerance < 0):
            raise ValueError('Sensor timeouts must be positive')
        self.route = Route()
        self.reference = None
        self.previous = None
        self.obstacles = None
        self.observation_stamp = None
        self.last_status = None
        self.odom = None
        self.active = False
        self.reset_time = None
        self.create_subscription(Odometry, '/hk/local/odom', self.on_odom, 1)
        self.create_subscription(PointCloud2, '/hk/perception/obstacles', self.on_obstacles, 1)
        self.pub = self.create_publisher(Path, '/hk/plan/local_path', 10)
        self.speed_pub = self.create_publisher(Float32, '/hk/plan/speed_limit', 1)
        self.status_pub = self.create_publisher(String, '/hk/plan/status', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/hk/plan/avoidance_markers', 1)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(1.0 / RATE_HZ, self.plan)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_reset(self, checkpoint, reload):
        if reload or not self.route.points:
            self.route = load_route(self)
            self.reference = ReferencePath(self.route.points)
            self.build_profile()
        if reload or self.avoidance_ranges is None:
            self.avoidance_ranges = self.load_avoidance_ranges()
            self.get_logger().info(f'전역 경로 {len(self.route.points)}점, 속도 프로파일 '
                                   f'{self.profile_v.min():.1f}~{self.profile_v.max():.1f} m/s, 회피 구간 '
                                   + ', '.join(f'{a:.0f}~{b:.0f} m' for a, b in self.avoidance_ranges))
        self.route.reset()
        self.odom = None
        self.obstacles = self.observation_stamp = self.previous = None
        self.detour = None               # 복귀하면 지금 자세에서 다시 만듦 (모은 장애물 점은 유지)
        if reload:
            self.detour_map.clear()
        self.reset_time = self.now()
        self.active = True

    def on_end(self):
        self.active = False
        self.previous = None
        self.publish_stop('inactive')

    def on_odom(self, m):
        if m.header.frame_id == 'map':
            self.odom = m

    def on_obstacles(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if m.header.frame_id != 'map' or (self.reset_time is not None and stamp < self.reset_time):
            return
        if self.observation_stamp is not None and stamp < self.observation_stamp:
            return
        try:
            rows = point_cloud2.read_points(m, field_names=['x', 'y', 'cluster_id'])
            xy = np.column_stack((rows['x'], rows['y'])).astype(float)
            if not np.isfinite(xy).all():
                self.obstacles = None
                return
            self.obstacles = [xy[rows['cluster_id'] == cluster_id] for cluster_id in np.unique(rows['cluster_id'])]
        except (ValueError, AssertionError, KeyError):
            self.obstacles = None
            return
        self.observation_stamp = stamp
        self.accumulate(xy)

    def accumulate(self, xy):
        """지금 회피 구간 근처 관측점을 Frenet 칸으로 모읍니다 (정지 장애물·벽)."""
        if not self.detour_enabled or self.current_zone is None or self.reference is None or not len(xy):
            return
        low, high = self.current_zone[0] - 5.0, self.current_zone[1] + 5.0
        station, lateral = self.reference.project(xy, low - 5.0, high + 5.0)
        self.update_wall_offset(station, lateral)
        keep = (station > low) & (station < high) & (np.abs(lateral) < self.road_half_width + 0.75)
        if np.any(keep):
            # 벽 기준 좌표로 모음: GNSS 잡음으로 추정 위치가 옆으로 흔들려도 스캔마다 벽에 맞춰 두면 번지지 않음
            self.detour_map.add(station[keep], lateral[keep] - self.wall_offset)

    def update_wall_offset(self, station, lateral):
        """이번 스캔에서 장애물 구간 벽(코스 파일 ±3.75, LiDAR로는 약 ±3.8)이 보이는 횡위치로 추정 위치의 옆
        어긋남(wall_offset, m)을 잽니다. 장애물·우회 경로는 벽 기준으로 만들고 내보낼 때 이만큼 옮깁니다.
        GNSS 잡음(0.3~0.5 m)이 장애물 옆 여유(0.65 m)를 먹어 감점되던 것을 막습니다."""
        mission = self.obstacle_zones.get(self.current_zone, self.current_zone)
        near = ((station > max(mission[0] + 1.0, self.last_station - 5.0))
                & (station < min(mission[1] - 1.0, self.last_station + 15.0)))
        wall = self.road_half_width + 0.05
        estimates = []
        for side in (1.0, -1.0):
            points = lateral[near & (side * lateral > wall - 0.8) & (side * lateral < wall + 0.8)]
            if len(points) >= 10:
                estimates.append(float(np.median(points)) - side * wall)
        if estimates:
            measured = max(-0.8, min(0.8, float(np.mean(estimates))))
            self.wall_offset += 0.5 * (measured - self.wall_offset)

    def follow_detour(self, zone, station, ego, header):
        """우회 경로를 (필요하면 새로) 만들어 기준 경로처럼 내보냄. 못 만들면 False (Frenet 회피로)."""
        obs_s, obs_d = self.detour_map.points()
        s_ego, d_ego = self.reference.project([[ego.x, ego.y]], station - 3.0, station + 3.0)
        s_ego, d_ego = float(s_ego[0]), float(d_ego[0]) - self.wall_offset   # 벽 기준 횡위치
        replan = self.detour is None or self.detour_zone != zone
        if not replan:
            # 다시 만드는 경우: 실제 위치가 경로에서 0.8 m 넘게 벗어남, 또는 앞쪽 장애물 배치(개수·벽 쪽·
            # 위치 0.6 m)가 만들 때와 달라짐 (가려졌던 장애물이 보임). 매 주기 후보 탐색은 하지 않습니다
            planned = float(np.interp(s_ego, self.detour.s, self.detour.d))
            scene = read_obstacles(obs_s, obs_d, self.obstacle_zones.get(zone, zone), s_ego, self.cfg)
            ahead = [(a, side) for a, side in scene_signature(scene) if a > s_ego + 1.0]
            known = [(a, side) for a, side in self.detour.signature if a > s_ego + 1.0]
            same = len(ahead) == len(known) and all(
                side == old_side and abs(a - old_a) < 0.6 for (a, side), (old_a, old_side) in zip(ahead, known))
            replan = abs(planned - d_ego) > 0.8 or s_ego > self.detour.s[-1] - 1.0 or not same
        if replan:
            _, base_yaw, _, _ = self.reference.evaluate(np.array([s_ego]))
            slope = math.tan(max(-1.0, min(1.0, math.atan2(math.sin(ego.yaw - base_yaw[0]),
                                                           math.cos(ego.yaw - base_yaw[0])))))
            mission = self.obstacle_zones.get(zone, zone)
            self.detour = None
            # 여유를 줄인 우회 경로는 추종 오차를 줄이려 천천히 (감점 1회 = 점수 ×0.8, 30 s보다 큼).
            # 마지막은 여유 검사 없이 템플릿을 아주 천천히: 멈추면 시간 초과로 실패하므로 서지 않습니다
            margins = (self.cfg.margin, 0.5 * self.cfg.margin, 0.0)
            caps = {margins[0]: math.inf, margins[1]: self.detour_slow_speed, margins[2]: self.cfg.tight_speed}
            for check in (True, False):
                detour, reason = plan_detour(self.reference, s_ego, d_ego, slope, obs_s, obs_d, mission, zone[1],
                                             replace(self.cfg, max_lateral_accel=self.detour_lateral_accel),
                                             margins=margins, check=check)
                if detour is not None:
                    cap = caps[detour.margin] if check else self.cfg.creep_speed
                    self.detour, self.detour_zone, self.detour_cap = detour, zone, cap
                    self.get_logger().info(
                        f'우회 경로: s={s_ego:.1f} d={d_ego:+.2f}에서 장애물 {len(detour.targets)}개 '
                        + ', '.join(f'{a:.0f}~{b:.0f} m d={t:+.2f}' for a, b, t in detour.targets)
                        + f', margin {detour.margin:.2f}{"" if check else " (여유 검사 없음)"}, '
                        + f'남는 여유 {detour.clearance:.2f} m, 최저 {detour.speed.min():.1f} m/s, 상한 {cap:.1f}')
                    break
            else:
                return False
        detour = self.detour
        k0 = max(0, int(np.searchsorted(detour.s, s_ego - self.behind_m)))
        k1 = int(np.searchsorted(detour.s, s_ego + self.ahead_m))
        path = Path(header=header)
        # 제어(Stanley)는 앞바퀴 축을 경로에 맞추므로, 우회 경로(뒤차축 궤적)를 그대로 주면 지그재그에서 뒤차축이
        # 곡선 안쪽으로 √(R²+L²)−R(κ 0.12에서 약 0.5 m)만큼 파고들어 장애물 옆 횡위치가 0.3 m 모자랐습니다.
        # 뒤차축이 계획대로 지날 때 앞바퀴 축이 있을 자리(축간거리만큼 앞)를 내보냅니다.
        normal = np.column_stack((-np.sin(detour.yaw), np.cos(detour.yaw)))
        trace = (detour.xy + self.wall_offset * normal
                 + self.wheelbase * np.column_stack((np.cos(detour.yaw), np.sin(detour.yaw))))
        heading = np.unwrap(np.arctan2(np.gradient(trace[:, 1]), np.gradient(trace[:, 0])))
        for (x, y), heading in zip(trace[k0:k1 + 1], heading[k0:k1 + 1]):
            ps = PoseStamped(header=header)
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(float(heading) / 2), math.cos(float(heading) / 2)
            path.poses.append(ps)
        if len(path.poses) < 2:
            return False
        self.pub.publish(path)
        route_speed = self.profile_speed(station, ego.speed)
        speed = min(route_speed, detour.speed_at(s_ego, max(ego.speed, 0.0) * self.speed_preview_s + 1.0),
                    self.detour_cap)
        self.speed_pub.publish(Float32(data=speed))
        target = next((t for a, b, t in detour.targets if b >= s_ego), 0.0)
        self.publish_status('detour', offset=target, speed=speed, tight=detour.margin < self.cfg.margin)
        return True

    def plan(self):
        self.plan_started = time.perf_counter()     # 계획 계산 시간 측정 (status의 plan_ms)
        if not self.active:
            return
        if self.odom is None or self.reference is None:
            self.publish_stop('waiting_for_odom')
            return
        now = self.now()
        stamp = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
        if not -self.clock_tolerance - 1e-6 <= now - stamp <= self.odom_timeout or stamp < self.reset_time:
            self.publish_stop('stale_odom')
            return
        if self.now() - self.reset_time < SETTLE_S:
            self.route.reset()
        p = self.odom.pose.pose.position
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        i = self.route.nearest(p.x, p.y)
        if self.route.past_end(p.x, p.y):
            self.publish_stop('route_end')
            return
        station = float(self.reference.stations[min(i, len(self.reference.stations) - 1)])
        route_speed = self.profile_speed(station, self.odom.twist.twist.linear.x)
        zone = next(((a, b) for a, b in self.avoidance_ranges or () if a <= station <= b), None)
        in_zone = self.avoidance_ranges is None or zone is not None
        self.current_zone = zone
        self.last_station = station
        if not in_zone:
            self.previous = None
        if not self.avoidance_enabled or not in_zone:
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
            self.speed_pub.publish(Float32(data=route_speed))
            self.publish_status('reference_only', speed=route_speed)
            return
        if (self.obstacles is None or self.observation_stamp is None
                or not -self.clock_tolerance - 1e-6 <= now - self.observation_stamp <= self.obstacle_timeout):
            self.publish_stop('sensor_unavailable')
            return
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        ego = Ego(p.x, p.y, yaw, self.odom.twist.twist.linear.x)
        if self.detour_enabled and zone is not None and self.follow_detour(zone, station, ego, path.header):
            self.previous = None
            return
        # 회피 구간 끝(check_end)까지만 관측물을 검사하고 그 전에 기준 경로로 돌아옵니다. 구간 뒤 좁은
        # 곡선 통로의 벽까지 검사하면 기준 경로조차 충돌로 판정되어 모든 후보가 탈락합니다.
        result = plan_avoidance(self.reference, ego, self.obstacles, self.cfg,
                                self.previous, float(self.reference.stations[i]),
                                check_end=zone[1] if zone is not None else None,
                                obstacle_zone=self.obstacle_zones.get(zone, zone),
                                start_level=self.plan_level if self.previous is not None else 0)
        self.plan_level = result.level
        self.publish_debug(path.header, result.candidates, result.selected)
        if result.selected is None:
            self.publish_stop(result.reason, result.candidates, clear_markers=False)
            return
        for (x, y), heading in zip(result.selected.xy, result.selected.yaw):
            ps = PoseStamped(header=path.header)
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z = math.sin(float(heading) / 2)
            ps.pose.orientation.w = math.cos(float(heading) / 2)
            path.poses.append(ps)
        self.pub.publish(path)
        # 속도 프로파일과 회피 경로의 곡률 속력(감속 한계로 미리 반영) 중 작은 값
        avoiding = np.max(np.abs(result.selected.d)) > 0.3
        tight = result.reason == 'tight'           # 여유를 줄여 찾은 경로: 회피 경로가 아니어도 천천히
        speed = min(route_speed, float(result.speed))
        self.speed_pub.publish(Float32(data=speed))
        self.previous = result.selected
        self.publish_status('avoiding' if avoiding else 'following',
                            result.candidates, result.selected.offset, speed, tight)

    def load_avoidance_ranges(self):
        """회피를 켤 경로 거리 구간 [(시작, 끝), ...]. self.obstacle_zones에 구간별 원래 미션 구간을 둡니다."""
        self.obstacle_zones = {}
        flat = [float(v) for v in self.get_parameter('avoidance_zones').value]
        if flat != [-1.0]:
            return [(flat[k], flat[k + 1]) for k in range(0, len(flat) - 1, 2)]
        path = self.get_parameter('course_json').value or str(COURSE_DIR / 'course.json')
        try:
            with open(path, encoding='utf-8') as f:
                missions = json.load(f)['course'].get('missions', [])
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().warn(f'코스 파일에서 장애물 구간을 읽지 못해 코스 전체에서 회피합니다: {error}')
            return [(0.0, self.reference.length)]
        types = set(self.get_parameter('avoidance_mission_types').value)
        try:
            with open(self.get_parameter('attempt_course_json').value, encoding='utf-8') as f:
                options = json.load(f).get('options', {})
            types = {t for t in types if options.get(t, True)}
        except (OSError, ValueError):
            pass                                   # 시도 파일이 없으면 모든 장애물 미션 구간에서 회피
        ranges = []
        for mission in missions:
            if mission.get('type') not in types or not mission.get('zone'):
                continue
            stations, _ = self.reference.project(np.asarray(mission['zone'], dtype=float))
            ranges.append((max(0.0, float(stations.min()) - self.avoidance_zone_before),
                           float(stations.max()) + self.avoidance_zone_after))
            self.obstacle_zones[ranges[-1]] = (float(stations.min()), float(stations.max()))
        return sorted(ranges)

    def build_profile(self):
        flat = [float(v) for v in self.get_parameter('speed_zones').value]
        zones = [tuple(flat[k:k + 3]) for k in range(0, len(flat) - 2, 3)]
        self.profile_s, self.profile_v = speed_profile(
            self.reference, self.cruise_speed, self.corner_lateral_accel, self.profile_accel, self.profile_decel,
            zones=zones)
        self.profile_reference = self.reference

    def profile_speed(self, station, speed):
        """속도 프로파일에서 지금부터 speed_preview_s 동안 지날 구간의 가장 낮은 제한."""
        if getattr(self, 'profile_reference', None) is not self.reference:
            self.build_profile()
        ahead = station + max(speed, 0.0) * self.speed_preview_s + 1.0
        lo, hi = np.searchsorted(self.profile_s, [station, ahead])
        lo = min(int(lo), len(self.profile_v) - 1)
        return float(np.min(self.profile_v[lo:max(hi, lo + 1)]))

    def publish_stop(self, reason, candidates=(), clear_markers=True):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(path)
        self.speed_pub.publish(Float32(data=0.0))
        self.publish_status(reason, candidates)
        if clear_markers:
            self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def publish_status(self, reason, candidates=(), offset=0.0, speed=0.0, tight=False):
        payload = {'state': reason, 'offset_m': float(offset), 'speed_limit_mps': float(speed), 'tight': bool(tight),
                   'candidate_results': dict(Counter(c.reason for c in candidates)),
                   'level': int(self.plan_level), 'plan_ms': round(1000 * (time.perf_counter() - getattr(self, 'plan_started', time.perf_counter())), 1)}
        self.status_pub.publish(String(data=json.dumps(payload)))
        label = reason + (' (여유 축소)' if tight else '')
        if label != self.last_status:
            self.get_logger().info(f'계획 상태: {label}')
            self.last_status = label

    def publish_debug(self, header, candidates, selected):
        markers = [Marker(action=Marker.DELETEALL)]
        if candidates:
            xy, yaw, _, _ = self.reference.evaluate(candidates[0].s)
            normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
            for i, offset in enumerate((0.0, -self.cfg.road_half_width, self.cfg.road_half_width)):
                reference = Marker(header=header, ns='reference_corridor', id=i, type=Marker.LINE_STRIP, action=Marker.ADD)
                reference.pose.orientation.w = 1.0
                reference.scale.x = 0.06
                reference.color.r = reference.color.g = reference.color.b = 0.7
                reference.color.a = 0.8
                reference.points = [Point(x=float(x), y=float(y), z=0.08) for x, y in xy + offset * normal]
                reference.lifetime = Duration(seconds=0.5).to_msg()
                markers.append(reference)
        for i, candidate in enumerate(candidates):
            if not np.isfinite(candidate.xy).all():
                continue
            marker = Marker(header=header, ns='candidates', id=2*i, type=Marker.LINE_STRIP, action=Marker.ADD)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.14 if candidate is selected else 0.04
            marker.color.r = 0.95 if candidate.reason != 'ok' else 0.1
            marker.color.g = 0.95 if candidate.reason == 'ok' else 0.2
            marker.color.b = 0.9 if candidate.reason == 'ok' and candidate is not selected else 0.1
            marker.color.a = 1.0 if candidate is selected else 0.5
            marker.points = [Point(x=float(x), y=float(y), z=0.15) for x, y in candidate.xy]
            marker.lifetime = Duration(seconds=0.5).to_msg()
            markers.append(marker)
            label = Marker(header=header, ns='candidate_reasons', id=2*i+1, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.orientation.w = 1.0
            k = min(len(candidate.xy) - 1, int((0.4 + 0.15 * (i % 3)) * len(candidate.xy)))
            label.pose.position.x, label.pose.position.y = map(float, candidate.xy[k])
            label.pose.position.z = 0.6 + (i % 3) * 0.5
            label.scale.z = 0.25
            label.color = marker.color
            label.text = f'd={candidate.offset:+.1f}, T={candidate.transition:.1f}: {candidate.reason}'
            label.lifetime = marker.lifetime
            markers.append(label)
        if selected is not None:
            body = Marker(header=header, ns='vehicle_footprints', id=0, type=Marker.LINE_LIST, action=Marker.ADD)
            body.pose.orientation.w = 1.0
            body.scale.x = 0.04
            body.color.g = body.color.a = 0.8
            corners = footprint_corners(selected.xy[::20], selected.yaw[::20], self.cfg)
            for rectangle in corners:
                for a, b in zip(rectangle, np.roll(rectangle, -1, axis=0)):
                    body.points.extend([Point(x=float(a[0]), y=float(a[1]), z=0.25),
                                        Point(x=float(b[0]), y=float(b[1]), z=0.25)])
            body.lifetime = Duration(seconds=0.5).to_msg()
            markers.append(body)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
