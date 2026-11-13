"""Reference-route following with static-obstacle Frenet avoidance.

Avoidance runs only inside avoidance zones (by default the obstacle mission zones of course.json, widened by
avoidance_zone_before/after). Elsewhere the corridor is only 4.39 m wide: its walls are not obstacles, and
checking the vehicle body against them with safety margins in the R≈6 m S-curves rejects every candidate.
Inside a zone, fresh map-frame obstacle clouds are required; missing perception or an infeasible plan
publishes an empty Path and zero speed.

Roundabout (roundabout.py): at load time the kinked entry/exit of route.csv is replaced by tangent arcs that the
vehicle can follow (published as the front-axle trace of the designed rear-axle path). NPCs on the ring are tracked
from the obstacle clouds; before the entry the planner yields at a stop point when the predicted merge gap is too
small, and on the ring it caps speed from the gap to the NPC ahead. A generic forward-collision speed cap for any
observed cluster on the path is applied inside collision zones (default: the roundabout section).
"""
from collections import Counter
import json
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial import cKDTree
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .avoidance import AvoidanceConfig, Ego, ReferencePath, footprint_corners, plan_avoidance, speed_profile
from .course import COURSE_DIR, declare_file_params, files_signature, load_route, Route
from .roundabout import NpcTracker, follow_speed, merge_conflicts, ring_reference, stop_point, travel_time, wrap
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
    'return_time': 2.5,     # 장애물을 지난 뒤 기준 경로로 돌아오는 시간 (s)
    'corner_lateral_accel': 4.0,   # 기준 경로 곡선에서 허용하는 횡가속도 (m/s²)
    'profile_accel': 2.0,          # 속도 프로파일 가속 한계 (m/s²)
    'profile_decel': 3.0,          # 속도 프로파일 감속 한계 (m/s²). 곡선 앞에서 미리 줄임
    'speed_preview_s': 0.6,        # 이 시간만큼 앞 구간의 제한까지 미리 반영 (구동 지연·제어 반응)
    'avoidance_zone_before': 15.0,  # 장애물 미션 구간 시작 이만큼(m) 앞부터 회피를 켬
    'avoidance_zone_after': 8.0,    # 장애물 미션 구간 끝 이만큼(m) 뒤까지 회피를 켬 (기준 경로로 돌아올 거리)
    'obstacle_timeout': 0.35,
    'odom_timeout': 0.35,
    'clock_tolerance': 0.05,
    # 회전교차로 (roundabout.py). 반지름·거리는 m, 각속도는 rad/s
    'roundabout_entry_radius': 7.25,  # 진입 원호 반지름 (뒤차축)
    'roundabout_exit_radius': 7.25,   # 진출 원호 반지름 (뒤차축)
    'roundabout_ring_offset': 0.0,    # 우리 링 반지름 = NPC 반지름 + 이 값
    'roundabout_smooth_m': 2.5,       # 곡률이 바뀌는 곳을 둥글리는 길이 (조향 속도)
    'roundabout_cut_bias': 1.8,       # 곡선 안쪽 파고듦 보상: 제어 경로를 바깥으로 이 값 × 곡률 (m², R 7.25 → 0.25 m)
    'roundabout_speed': 4.0,          # 정지 위치 ~ 진출 직선 속력 상한 (m/s)
    'roundabout_stop_radius': 13.5,   # 양보 정지 위치: 앞 범퍼가 링 중심에서 이 거리
    'npc_rate': 0.33,                 # NPC 각속도 사전값 (관측 rb_t1: 약 0.33 rad/s = 3.6 m/s)
    'npc_track_timeout': 2.5,         # 관측 없이 NPC 추적을 유지하는 시간 (s)
    'npc_gap_behind': 7.0,            # 합류 순간 뒤 NPC 중심이 합류점에서 이만큼(호 m) 뒤에 있어야 함
    'npc_gap_ahead': 7.5,             # 합류 순간 앞 NPC 중심이 합류점에서 이만큼 앞에 있어야 함
    'npc_follow_gap': 9.0,            # 링에서 앞 NPC 중심 ~ 우리 뒤차축 최소 거리 (호 m, 범퍼 간격 약 2.9 m)
    'merge_accel': 1.5,               # 합류 시각 예측에 쓰는 가속도 (m/s²)
    'yield_decel': 2.0,               # 정지 위치에 서는 감속도 (m/s²)
    'yield_timeout': 20.0,            # 이만큼 서 있어도 틈이 없으면 진입 (s, 허상 추적으로 영영 서는 것 방지)
    'collision_decel': 3.0,           # 경로 위 물체 앞 감속도 (m/s²)
    'collision_stop_gap': 3.0,        # 경로 위 물체와 앞 범퍼 사이에 남길 거리 (m)
    'collision_half_width': 1.2,      # 경로에서 이 거리 안의 관측점을 경로 위 물체로 봄 (m)
    'collision_horizon': 25.0,        # 이만큼 앞까지 검사 (m)
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
        self.declare_parameter('roundabout_enabled', True)
        self.roundabout_enabled = bool(self.get_parameter('roundabout_enabled').value)
        # 앞 물체 속력 제한을 켤 구간 [시작 s, 끝 s, ...]. [-1.0]이면 회전교차로 구간(정지 위치 30 m 앞 ~ 진출 10 m 뒤)
        self.declare_parameter('collision_zones', [-1.0])
        self.declare_parameter('collision_min_points', 3)
        self.collision_min_points = int(self.get_parameter('collision_min_points').value)
        self.ring = None                 # roundabout.RingGeometry (회전교차로가 없으면 None)
        self.ring_s = {}
        self.speed_reference = None      # 속도 프로파일용 뒤차축 경로 (회전교차로 구간을 바꿨을 때)
        self.collision_ranges = []
        self.npc = None
        self.reset_roundabout_state()
        self.avoidance_ranges = None     # 경로를 읽기 전(시험에서 reference를 직접 넣은 경우)은 코스 전체
        self.cfg = AvoidanceConfig(
            horizon=self.ahead_m, step=self.path_step, road_half_width=self.road_half_width,
            vehicle_length=self.vehicle_length, vehicle_width=self.vehicle_width,
            rear_overhang=self.rear_overhang, wheelbase=self.wheelbase,
            max_steer_deg=self.max_steer_deg, margin=self.safety_margin,
            surface_padding=self.surface_padding, max_lateral_accel=self.max_lateral_accel,
            cruise_speed=self.cruise_speed, avoidance_speed=self.avoidance_speed,
            offsets=tuple(self.get_parameter('lateral_offsets').value),
            transition_times=tuple(self.get_parameter('transition_times').value),
            return_time=self.return_time,
        )
        if (not all(math.isfinite(v) for v in (self.obstacle_timeout, self.odom_timeout, self.clock_tolerance))
                or self.obstacle_timeout <= 0 or self.odom_timeout <= 0 or self.clock_tolerance < 0):
            raise ValueError('Sensor timeouts must be positive')
        self.route = Route()
        self.files_loaded = None         # 지금 경로·프로파일을 만든 파일 (course.files_signature)
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

    def preload(self):
        """켤 때 경로·회전교차로·속도 프로파일을 미리 만듦 (시작 때 다시 만들지 않게)."""
        self.route = load_route(self)
        self.setup_roundabout()
        self.build_profile()
        self.files_loaded = files_signature(self)

    def on_reset(self, checkpoint, reload):
        signature = files_signature(self)
        if not self.route.points or (reload and signature != self.files_loaded):
            self.route = load_route(self)
            self.setup_roundabout()
            self.build_profile()
            self.files_loaded = signature
        if reload or self.avoidance_ranges is None:
            self.avoidance_ranges = self.load_avoidance_ranges()
            self.get_logger().info(f'전역 경로 {len(self.route.points)}점, 속도 프로파일 '
                                   f'{self.profile_v.min():.1f}~{self.profile_v.max():.1f} m/s, 회피 구간 '
                                   + ', '.join(f'{a:.0f}~{b:.0f} m' for a, b in self.avoidance_ranges))
        self.route.reset()
        self.odom = None
        self.obstacles = self.observation_stamp = self.previous = None
        self.reset_roundabout_state()
        self.reset_time = self.now()
        self.active = True

    # ---- 회전교차로 -------------------------------------------------------------------------------------------
    def reset_roundabout_state(self):
        """시작·복귀: NPC 추적과 양보 상태를 비움."""
        if self.npc is not None:
            self.npc.reset()
        self.committed = False          # 정지 위치를 지나 진입하기로 함 (다시 서지 않음)
        self.yield_since = None
        self.dynamic_info = {}

    def setup_roundabout(self):
        """course.json의 회전교차로 미션으로 기준 경로를 바꾸고 주요 지점의 경로 거리를 구함."""
        self.ring, self.ring_s, self.speed_reference, self.npc = None, {}, None, None
        points = np.asarray(self.route.points, dtype=float)
        mission = None
        if self.roundabout_enabled:
            path = self.get_parameter('course_json').value or str(COURSE_DIR / 'course.json')
            try:
                with open(path, encoding='utf-8') as f:
                    mission = next((m for m in json.load(f)['course'].get('missions', [])
                                    if m.get('type') == 'roundabout' and m.get('center')), None)
            except (OSError, ValueError, KeyError) as error:
                self.get_logger().warn(f'코스 파일에서 회전교차로를 읽지 못함: {error}')
        geometry = None
        if mission is not None:
            radius = float(mission.get('npc_radius', 11.0))
            out, rear, geometry = ring_reference(
                points, mission['center'], radius, radius + self.roundabout_ring_offset,
                self.roundabout_entry_radius, self.roundabout_exit_radius, self.wheelbase,
                smooth_m=self.roundabout_smooth_m, cut_bias=self.roundabout_cut_bias)
        if geometry is None:
            if mission is not None:
                self.get_logger().warn('회전교차로 기준 경로를 만들지 못해 route.csv를 그대로 씁니다')
            self.reference = ReferencePath(self.route.points)
            self.collision_ranges = self.load_collision_ranges()
            return
        self.route.points = [(float(x), float(y)) for x, y in out]
        self.reference = ReferencePath(self.route.points)
        self.speed_reference = ReferencePath(rear)
        self.ring = geometry
        stations, pts = self.reference.stations, self.reference.points

        def station_of(xy):             # 차의 경로 거리와 같은 방법(뒤차축에서 가장 가까운 경로 점)
            return float(stations[int(np.argmin(np.linalg.norm(pts - np.asarray(xy), axis=1)))])

        front = self.vehicle_length - self.rear_overhang
        self.ring_s = {'stop': station_of(stop_point(geometry, self.roundabout_stop_radius, front)),
                       'entry': station_of(geometry.entry_point), 'merge': station_of(geometry.merge_point),
                       'leave': station_of(geometry.leave_point), 'exit': station_of(geometry.exit_point)}
        self.npc = NpcTracker(geometry.center, geometry.npc_radius, geometry.direction,
                              nominal_rate=self.npc_rate, timeout=self.npc_track_timeout)
        self.collision_ranges = self.load_collision_ranges()
        self.get_logger().info(
            '회전교차로 기준 경로: 진입 원호 R %.2f, 링 R %.2f, 진출 원호 R %.2f, 합류 %.0f° → 이탈 %.0f°, 경로 거리 %s' % (
                self.roundabout_entry_radius, geometry.radius, self.roundabout_exit_radius,
                math.degrees(geometry.entry_angle), math.degrees(geometry.exit_angle),
                ', '.join(f'{k} {v:.1f}' for k, v in self.ring_s.items())))

    def load_collision_ranges(self):
        flat = [float(v) for v in self.get_parameter('collision_zones').value]
        if flat != [-1.0]:
            return [(flat[k], flat[k + 1]) for k in range(0, len(flat) - 1, 2)]
        if not self.ring_s:
            return []
        return [(self.ring_s['stop'] - 30.0, self.ring_s['exit'] + 10.0)]

    def fresh_clusters(self, now):
        if (self.obstacles is None or self.observation_stamp is None
                or not -self.clock_tolerance - 1e-6 <= now - self.observation_stamp <= self.obstacle_timeout):
            return None
        return self.obstacles

    def dynamic_limit(self, station, x, y, speed, now):
        """회전교차로 양보·차간 거리와 경로 위 물체에 따른 속력 상한 (m/s, 없으면 inf)."""
        limit, info = math.inf, {}
        clusters = self.fresh_clusters(now)
        ring, s = self.ring, self.ring_s
        if ring is not None and s:
            tracks = self.npc.confirmed()
            radius, direction = ring.npc_radius, ring.direction
            info['npc'] = [[round(math.degrees(k.predict(now)), 1), round(k.rate, 3)] for k in tracks]
            # 1) 진입 양보: 정지 위치 30 m 앞부터 합류점 전까지
            if s['stop'] - 30.0 <= station < s['merge'] and not self.committed:
                if station > s['stop'] + 1.5:          # 정지 위치를 지나쳤으면 서지 않고 진입
                    self.committed = True
                else:
                    t_go = travel_time(s['merge'] - station, speed, self.merge_accel, self.roundabout_speed)
                    blocking = merge_conflicts(tracks, now, t_go, ring.entry_angle, radius, direction,
                                               self.npc_gap_behind, self.npc_gap_ahead)
                    waited = 0.0 if self.yield_since is None else now - self.yield_since
                    if (blocking or clusters is None) and waited < self.yield_timeout:
                        cap = math.sqrt(2 * self.yield_decel * max(0.0, s['stop'] - station))
                        limit = min(limit, cap if cap > 0.3 else 0.0)
                        if self.yield_since is None and speed < 0.3 and s['stop'] - station < 1.0:
                            self.yield_since = now
                        info['roundabout'] = 'yield'
                        info['t_go'] = round(t_go, 2)
                        info['blocking'] = [round(a, 1) for a, _ in blocking]
                    else:
                        info['roundabout'] = 'go'
                        if station >= s['stop'] - 1.0:
                            self.committed = True
            # 2) 링 안 차간 거리: 합류 10 m 앞 ~ 이탈 3 m 뒤, 우리가 지날 링 구간에 있는 앞 NPC
            if s['merge'] - 10.0 <= station <= s['leave'] + 3.0:
                # 합류 전: 경로로 합류점까지 거리 + 합류점에서 NPC까지 호 길이, 링 안: 우리 각도에서 NPC까지 호 길이
                before = station < s['merge']
                ego = ring.entry_angle if before else math.atan2(y - ring.center[1], x - ring.center[0])
                end = ring.exit_angle + math.radians(25.0)
                for k in tracks:
                    angle = k.predict(now)
                    arc = direction * wrap(angle - ego) * radius
                    gap = arc + (s['merge'] - station if before else 0.0)
                    if arc > -1.0 and 0.0 < gap < 25.0 and direction * wrap(angle - end) < 0:
                        cap = follow_speed(gap, abs(k.rate) * radius, self.npc_follow_gap, self.collision_decel)
                        if cap < limit:
                            limit = cap
                            info['follow_gap'] = round(gap, 1)
        # 3) 경로 위 물체 (collision_zones 안, 기본 회전교차로 구간)
        if clusters and any(a <= station <= b for a, b in self.collision_ranges):
            gap = self.path_obstacle_gap(clusters, station)
            if gap is not None:
                cap = math.sqrt(2 * self.collision_decel * max(0.0, gap - self.collision_stop_gap))
                if cap < limit:
                    limit = cap if cap > 0.3 else 0.0
                    info['obstacle_gap'] = round(gap, 1)
        if math.isfinite(limit):
            info['limit'] = round(limit, 2)
        self.dynamic_info = info
        return limit

    def path_obstacle_gap(self, clusters, station):
        """경로(앞으로 collision_horizon) 위 관측점과 앞 범퍼 사이 거리 (m), 없으면 None."""
        front = self.vehicle_length - self.rear_overhang
        lo, hi = station, min(station + front + self.collision_horizon, self.reference.length)
        if hi - lo < 1.0:
            return None
        xy, _, _, _ = self.reference.evaluate(np.linspace(lo, hi, 9))
        best = None
        for points in clusters:
            if len(points) < self.collision_min_points:
                continue
            # 경로 근처(샘플 점에서 4 m 안)의 점만 투영
            near = points[np.min(np.linalg.norm(points[:, None, :] - xy[None], axis=2), axis=1)
                          < (hi - lo) / 16 + self.collision_half_width + 1.0]
            if len(near) < self.collision_min_points:
                continue
            s, d = self.reference.project(near, lo, hi)
            on = (np.abs(d) < self.collision_half_width) & (s > lo + front - 0.5) & (s < hi - 0.05)
            if np.count_nonzero(on) >= self.collision_min_points:
                gap = float(np.min(s[on])) - (station + front)
                best = gap if best is None else min(best, gap)
        return best

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
        if self.npc is not None:
            self.npc.update(stamp, self.obstacles)

    def plan(self):
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
        route_speed = min(route_speed, self.dynamic_limit(station, p.x, p.y, self.odom.twist.twist.linear.x, now))
        in_zone = self.avoidance_ranges is None or any(a <= station <= b for a, b in self.avoidance_ranges)
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
        result = plan_avoidance(self.reference, ego, self.obstacles, self.cfg,
                                self.previous, float(self.reference.stations[i]))
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
        # 기준 경로를 따를 때는 속도 프로파일, 옆으로 비킬 때는 회피 경로의 곡률·회피 속력 제한도 함께
        avoiding = np.max(np.abs(result.selected.d)) > 0.3
        tight = result.reason == 'tight'           # 여유를 줄여 찾은 경로: 회피 경로가 아니어도 천천히
        speed = min(route_speed, float(result.speed)) if avoiding or tight else route_speed
        self.speed_pub.publish(Float32(data=speed))
        self.previous = result.selected
        self.publish_status('avoiding' if avoiding else 'following',
                            result.candidates, result.selected.offset, speed, tight)

    def load_avoidance_ranges(self):
        """회피를 켤 경로 거리 구간 [(시작, 끝), ...]."""
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
        return sorted(ranges)

    def build_profile(self):
        flat = [float(v) for v in self.get_parameter('speed_zones').value]
        zones = [tuple(flat[k:k + 3]) for k in range(0, len(flat) - 2, 3)]
        if self.speed_reference is not None and self.ring_s:
            # 회전교차로 구간을 바꾼 경우: 곡률은 뒤차축 설계 경로에서 읽고(앞바퀴 축 궤적은 조향이 바뀌는 곳에서
            # 곡률이 커 보임), 경로 거리는 제어 경로 기준으로 옮김. 차의 경로 거리처럼 가장 가까운 점으로 짝지음
            rear = self.speed_reference
            _, index = cKDTree(self.reference.points).query(rear.points)
            pair = np.maximum.accumulate(self.reference.stations[index])
            zones.append((float(np.interp(self.ring_s['stop'] - 3.0, pair, rear.stations)),
                          float(np.interp(self.ring_s['exit'] + 2.0, pair, rear.stations)), self.roundabout_speed))
            stations, self.profile_v = speed_profile(
                rear, self.cruise_speed, self.corner_lateral_accel, self.profile_accel, self.profile_decel,
                zones=zones)
            self.profile_s = np.interp(stations, rear.stations, pair)
        else:
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
                   'candidate_results': dict(Counter(c.reason for c in candidates))}
        payload.update(self.dynamic_info)
        self.status_pub.publish(String(data=json.dumps(payload)))
        label = reason + (' (여유 축소)' if tight else '')
        if self.dynamic_info.get('roundabout'):
            label += ' / 회전교차로 ' + self.dynamic_info['roundabout']
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
    node.preload()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
