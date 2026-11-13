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

Parking (parking.py): while approaching an enabled parking mission zone, map-frame LiDAR points near the zone are
accumulated; the opening in the slot-side wall gives the seed slide. A maneuver (forward → stop → reverse → hold →
forward exit) is then spliced into the GLOBAL route. Each route point carries a gear (+1/-1) and the route is split
into segments whose ends are stop points (cusps, hold). Only the current segment is published, the nearest-point
search never crosses its end, and /hk/plan/gear (std_msgs/Int8) tells control how to follow the published path:
  0      normal driving (forward; the path continues — previous behaviour),
  +1/+2  parking segment, forward (2: the published path ends at a stop point),
  -1/-2  parking segment, reverse (2: ends at a stop point); poses are rear-axle poses, orientation = vehicle yaw.
Forward parking paths are the front-axle trace of the designed rear-axle poses (control tracks the front axle).
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
from std_msgs.msg import Float32, Int8, String
from visualization_msgs.msg import Marker, MarkerArray

from .avoidance import AvoidanceConfig, Ego, ReferencePath, footprint_corners, plan_avoidance, speed_profile
from .course import COURSE_DIR, declare_file_params, files_signature, load_route, Route
from .parking import (FORWARD, REVERSE, Maneuver, Mission, ParkingConfig, PathBuilder, Segment, SlotFrame,
                      body_corners, detect_slot, free_space_ok, in_slot, parallel_exit, plan_maneuver, road_lateral,
                      splice_route)
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
    # 주차 (parking.py). 거리는 m, 속력 m/s
    'parking_speed': 3.0,             # 주차 전진 구간 속력 상한
    'parking_reverse_speed': 2.0,     # 후진 구간 속력 상한
    'parking_lateral_accel': 2.0,     # 주차 원호 횡가속도 한계 (m/s²)
    'parking_stop_decel': 1.2,        # 정지 지점 앞 감속도 (m/s², 제어도 같은 값으로 50 Hz 정지)
    'parking_approach_decel': 1.2,    # 동작 안 원호 속력 상한까지 미리 줄이는 감속도 (m/s², 제어 PI 지연 여유)
    'parking_stop_tolerance': 0.2,    # 정지 지점까지 남은 거리가 이보다 작고 멈췄으면 도착
    'parking_accept_short': 0.6,      # 이 안에서 parking_short_wait 동안 멈춰 있으면 도착으로 봄
    'parking_short_wait': 1.5,
    'parking_hold_s': 3.4,            # 칸 안에서 서 있는 시간 (규정 3 s)
    'parking_stopped_speed': 0.03,    # 이보다 느리면 멈춤
    'parking_detect_range': 45.0,     # 존 중심까지 도로 방향 거리가 이 안이면 관측점을 모으고 칸을 찾음
    'parking_commit_distance': 12.0,  # 칸을 못 찾았으면 (존 중심 - slide 범위 - 이 거리)에 정지해 기다림
    'parking_commit_decel': 2.5,
    'parking_stable_scans': 3.0,      # 이만큼 연속으로 같은 slide(± parking_stable_tol)가 나오면 확정
    'parking_stable_tol': 0.3,
    'parking_refine_until': 4.0,      # 동작 시작 이만큼(m) 전까지 칸을 다시 찾아 동작을 고침
    'parking_refine_tol': 0.08,       # slide가 이만큼 넘게 바뀌면 다시 만듦 (m)
    'parking_clearance': 0.3,         # 계획 경로의 차체 ~ 벽 모형·관측점 최소 거리
    'parking_slot_margin': 0.15,      # 계획한 최종 차체가 칸 경계 안쪽으로 들어온 거리 (규정 0.05)
    'parking_radius_margin': 1.05,    # 최소 회전반경 × 이 값 이상 원호만
    'parking_voxel': 0.1,             # 관측점 누적 격자 (m)
    'parking_check_margin': 0.05,     # 유지 전 자체 검사: 추정 꼭짓점이 칸 경계 안쪽 이 거리(m) 이상
    'parking_check_yaw_deg': 12.0,    # 자체 검사 방향 오차 한계 (도, 규정 20°)
    'parking_max_retries': 2.0,       # 검사 실패 시 다시 들어가기 최대 횟수
    'parking_retry_pull': 4.0,        # 후면: 다시 들어가기 전 앞으로 갈 거리 (m)
    'parking_reverse_lateral_accel': 1.2,  # 후진 원호 횡가속도 한계 (m/s²)
    'parking_point_window': 3.0,      # 칸 검출에 이 시간(s) 안에 본 점만 씀
    'rear_back_wall_x': -2.8,         # 후면주차 방 북쪽 벽 (칸 좌표계 x, 칸 끝에서 0.1 m 뒤). 후진 정지 지점을 이 벽 기준으로
    'parking_wall_search': 0.6,       # 예상 위치 ± 이 거리(m)에서 벽 점을 찾고, 정지 지점을 최대 이만큼 옮김
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
        # 주차: 켤지, 미션 종류, 칸 좌표계 자유 공간 직사각형 [x0, x1, y0, y1, ...], 후보 동작 값 (bringup.yaml 설명)
        self.declare_parameter('parking_enabled', True)
        self.parking_enabled = bool(self.get_parameter('parking_enabled').value)
        self.declare_parameter('parking_mission_types', ['parallel_parking', 'rear_parking'])
        self.declare_parameter('parallel_parking_free', [-4.0, 4.0, -2.05, 2.5])
        self.declare_parameter('parallel_opening_length', 8.0)    # 칸 쪽 벽 입구 길이 (m, 0 = 관측 양 끝 중심)
        self.declare_parameter('rear_opening_length', 5.0)
        self.declare_parameter('rear_parking_free', [4.2, 9.2, -11.0, -2.0, -2.8, 16.2, -2.475, 2.525])
        self.declare_parameter('parallel_radius_scales', [1.0, 1.1])
        self.declare_parameter('parallel_final_lateral', [0.6, 0.5])
        self.declare_parameter('parallel_final_back', [1.1, 1.0, 1.2])
        self.declare_parameter('parallel_start_lateral', [0.0, -0.5, 0.5])
        self.declare_parameter('parallel_exit_radius_scale', 1.0)
        self.declare_parameter('rear_entry_radius_scales', [1.0])
        self.declare_parameter('rear_turn_radii', [5.0, 4.7])
        self.declare_parameter('rear_passage_offsets', [-0.75, -0.5])
        self.declare_parameter('rear_final_lateral', -0.2)
        self.declare_parameter('rear_final_back', [0.05])
        self.declare_parameter('rear_exit_turn_radius_scale', 1.0)
        self.declare_parameter('rear_exit_radius_scale', 1.0)
        self.declare_parameter('rear_exit_passage_offset', -0.25)
        self.park_missions = []          # [Mission] 이번 시도에서 켠 주차 미션 (경로 순서)
        self.park_cfg = ParkingConfig(
            wheelbase=self.wheelbase, vehicle_length=self.vehicle_length, vehicle_width=self.vehicle_width,
            rear_overhang=self.rear_overhang, max_steer_deg=self.max_steer_deg,
            radius_margin=self.parking_radius_margin, clearance=self.parking_clearance,
            slot_margin=self.parking_slot_margin)
        self.base_points = None          # 주차 동작을 넣기 전 경로 (복귀하면 되돌림)
        self.park_done = set()           # 끝낸(또는 포기한) 주차 미션 id
        self.reset_parking_state()
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
        # 경로를 따르는 방법: 0 일반 주행, +1 주차 전진(경로 끝 = 정지 지점), -1 주차 후진(경로 끝 = 정지 지점)
        self.gear_pub = self.create_publisher(Int8, '/hk/plan/gear', 1)
        self.park_marker_pub = self.create_publisher(MarkerArray, '/hk/plan/parking_markers', 1)
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
            self.base_points = None
        if self.base_points is None:
            self.base_points = list(self.route.points)
            self.base_reference = self.reference
        elif self.route_spliced:                      # 시작·복귀: 주차 동작을 넣은 경로를 원래대로
            self.route.points = list(self.base_points)
            self.reference = self.base_reference
        if reload:
            self.park_missions = self.load_parking_missions()
            self.park_done = set()
        self.reset_parking_state()
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

    # ---- 주차 -------------------------------------------------------------------------------------------------
    def reset_parking_state(self):
        """시작·복귀: 모은 관측점, 검출 기록, 진행 중인 주차 동작을 비움 (끝낸 미션 기록은 유지)."""
        if not hasattr(self, 'park_done'):
            self.park_done = set()
        self.park_plan = None            # dict: mission, spliced, segments ... (경로에 넣은 동작)
        self.park_seg = 0
        self.park_voxels = {}            # 격자 칸 → 마지막으로 본 시각
        self.park_history = []
        self.park_wait_since = None
        self.park_stop_since = None
        self.park_hold_since = None
        self.park_info = {}
        self.park_station = None
        self.park_still_since = None     # 차가 멈춘 시각 (odom), 감시 중 점 모으기에 씀
        self.park_offsets = []           # [(끼운 경로의 경로 거리, 원래 경로보다 늘어난 길이)] 속도 프로파일 대응용
        self.route_spliced = False       # 경로에 주차 동작을 넣었음 (복귀하면 base_points로 되돌림)
        self.route_yaws = self.route_gears = None

    def load_parking_missions(self):
        """course.json의 주차 미션 중 이번 시도에서 켠 것 (경로 순서)."""
        if not self.parking_enabled:
            return []
        types = set(self.get_parameter('parking_mission_types').value)
        path = self.get_parameter('course_json').value or str(COURSE_DIR / 'course.json')
        try:
            with open(path, encoding='utf-8') as f:
                raw = [m for m in json.load(f)['course'].get('missions', []) if m.get('type') in types and m.get('slot')]
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().warn(f'코스 파일에서 주차 미션을 읽지 못함: {error}')
            return []
        try:
            with open(self.get_parameter('attempt_course_json').value, encoding='utf-8') as f:
                options = json.load(f).get('options', {})
            raw = [m for m in raw if options.get(m['type'], True)]
        except (OSError, ValueError):
            pass                                   # 시도 파일이 없으면 모든 주차 미션을 함
        out = []
        route = np.asarray(self.route.points, dtype=float)
        for m in raw:
            mission = Mission.from_json(m)
            y_road, spread = road_lateral(mission, route)
            if y_road is None or spread > 0.3:
                self.get_logger().warn(f'{mission.id}: 칸 근처 경로가 직선이 아니어서 주차를 하지 않음')
                continue
            mission.y_road = y_road
            mission.station = float(self.reference.project(mission.zone_center[None])[0][0])
            out.append(mission)
        out.sort(key=lambda m: m.station)
        if out:
            self.get_logger().info('주차 미션: ' + ', '.join(f'{m.id}({m.type}, 존 중심 s={m.station:.0f} m, '
                                                          f'도로 중심 y={m.y_road:+.2f})' for m in out))
        return out

    def active_parking(self):
        return next((m for m in self.park_missions if m.id not in self.park_done), None)

    def collect_parking_points(self, clusters):
        """진행할 주차 미션 존 근처의 관측점을 격자(parking_voxel)로 모음."""
        mission = self.active_parking()
        if mission is None or self.odom is None or not clusters or (self.park_plan is not None and self.park_seg > 0):
            return
        frame = SlotFrame(mission.slot_center, mission.slot_yaw)
        a0 = float(frame.to_local(mission.zone_center)[0])
        station = getattr(self, 'park_station', None)       # plan()에서 구한 차의 경로 거리
        if station is None or mission.station - station > self.parking_detect_range + 5.0:
            return
        local = frame.to_local(np.vstack(clusters))
        keep = ((np.abs(local[:, 0] - a0) < mission.slide_range + 20.0)
                & (np.abs(local[:, 1] - mission.y_road) < 18.0))
        if not keep.any():
            return
        if (self.park_wait_since is not None and self.observation_stamp - self.park_wait_since > 4.0
                and (self.park_still_since is None or self.observation_stamp - self.park_still_since < 4.0)):
            return                                       # 감시 중: 오래 서 있을 때 찍은 점만
        cells = np.round(local[keep] / self.parking_voxel).astype(np.int64)
        stamp = self.observation_stamp
        self.park_voxels.update(dict.fromkeys(map(tuple, cells), stamp))
        if len(self.park_voxels) > 50000:            # 오래된 칸 정리
            old = stamp - self.parking_point_window
            self.park_voxels = {c: t for c, t in self.park_voxels.items() if t >= old}

    def parking_points(self, mission):
        """최근 parking_point_window(s) 동안 본 관측점 (map, (N,2)). 위치 추정 오차가 천천히 바뀌므로 오래된 점을
        빼서 지금 자기 위치와 같은 기준의 점만 씀 (GNSS 노이즈 시험에서 누적 점의 slide가 0.3 m 흘렀음)."""
        if not self.park_voxels or self.observation_stamp is None:
            return np.zeros((0, 2))
        old = self.observation_stamp - self.parking_point_window
        cells = [c for c, t in self.park_voxels.items() if t >= old]
        if not cells:
            return np.zeros((0, 2))
        frame = SlotFrame(mission.slot_center, mission.slot_yaw)
        return frame.to_map(np.asarray(cells, dtype=float) * self.parking_voxel)

    def parking_candidate_params(self):
        r = self.park_cfg.min_radius
        g = lambda name: self.get_parameter(name).value
        return {
            'parallel_parking': dict(
                radii=[r * float(v) for v in g('parallel_radius_scales')],
                final_lateral=[float(v) for v in g('parallel_final_lateral')],
                final_back=[float(v) for v in g('parallel_final_back')],
                start_lateral=[float(v) for v in g('parallel_start_lateral')],
                shift_radius=12.0, straight_before_stop=0.0, exit_radius=r * float(g('parallel_exit_radius_scale')),
                hold_s=self.parking_hold_s, cost_per_radius=0.0),
            'rear_parking': dict(
                entry_radii=[r * float(v) for v in g('rear_entry_radius_scales')],
                turn_radii=[max(r, float(v)) for v in g('rear_turn_radii')],
                final_back=[float(v) for v in g('rear_final_back')],
                passage_offsets=[float(v) for v in g('rear_passage_offsets')],
                final_lateral=float(g('rear_final_lateral')), straight_before=0.0, straight_after_turn=0.0,
                exit_turn_radius=r * float(g('rear_exit_turn_radius_scale')),
                exit_radius=r * float(g('rear_exit_radius_scale')),
                exit_passage_offset=float(g('rear_exit_passage_offset')), hold_s=self.parking_hold_s),
        }

    def parking_detect(self, station, speed, now):
        """칸을 찾고 확정되면 동작을 경로에 넣음. 반환: 아직 칸을 못 찾았을 때의 속력 상한 (없으면 inf)."""
        mission = self.active_parking()
        if mission is None or self.park_plan is not None:
            return math.inf
        frame = SlotFrame(mission.slot_center, mission.slot_yaw)
        # 거리는 경로 거리로 잼 (칸 좌표계 x는 칸 근처 직선 도로에서만 뜻이 있음: 출발 쪽에서는 -120 m로 보였음)
        a0, car = mission.station, station
        if a0 - car > self.parking_detect_range or car > a0 + mission.slide_range + 15.0:
            return math.inf
        det, points = self.stable_detection(mission)
        if det is not None and self.start_parking(mission, det, points):
            return math.inf
        if now - getattr(self, 'park_log_time', -1e9) > 1.0:          # 칸을 찾는 중: 1 s마다 상태 기록
            self.park_log_time = now
            self.get_logger().info(f'{mission.id}: 칸 찾는 중 (존 중심까지 {a0 - car:.1f} m) {self.park_info} '
                                   f'기록 {[round(v, 2) for v in self.park_history]}')
        # 칸을 확정하기 전에는 (존 중심 - slide 범위 - commit 거리)에서 멈춰 기다림. 오래 못 찾으면 천천히 다가감
        gap = a0 - mission.slide_range - self.parking_commit_distance - car
        cap = math.sqrt(2 * self.parking_commit_decel * max(0.0, gap))
        if cap < 0.3:
            cap = 0.0
            if self.park_wait_since is None and abs(speed) < 0.1:
                self.park_wait_since = now
            # 감시: 4 s 넘게 못 찾으면 1.5 s 기어가고 6 s 서기를 되풀이. 서 있는 동안 인지의 '정지 중 최신 TF' 대체가
            # 동작하고(TF가 몇 초 늦을 때), 움직이면 시점이 바뀜. 이 동안은 4 s 넘게 서 있을 때 찍은 점만 모음
            # (TF가 늦으면 움직이는 중 점이 엉뚱한 자리에 찍힘). 존 중심 - slide 범위 - 6 m 앞까지만 기어감
            if self.park_wait_since is not None and now - self.park_wait_since > 4.0:
                phase = (now - self.park_wait_since - 4.0) % 7.5
                cap = 1.0 if a0 - car > mission.slide_range + 6.0 and phase < 1.5 else 0.0
        self.park_info['wait_cap'] = round(cap, 2)
        return cap

    def stable_detection(self, mission):
        """모은 점으로 칸을 찾아 기록에 넣고, 연속 parking_stable_scans번 slide가 parking_stable_tol 안이면 그 중앙값."""
        points = self.parking_points(mission)
        length = float(self.get_parameter('parallel_opening_length' if mission.type == 'parallel_parking'
                                          else 'rear_opening_length').value)
        det = detect_slot(mission, points, mission.y_road, self.park_cfg, length if length > 0 else None)
        info = {'parking': mission.id, 'points': len(points)}
        if det is None:
            self.park_info.update(info)
            return None, points
        if getattr(self, 'park_history_id', None) != mission.id:     # 미션이 바뀌면 기록을 새로
            self.park_history, self.park_history_id = [], mission.id
        self.park_history = (self.park_history + [det['slide']])[-int(self.parking_stable_scans):]
        info.update(slide=round(det['slide'], 2), gap=[round(v, 2) for v in det['gap']], wall=round(det['wall'], 2))
        self.park_info.update(info)
        if (len(self.park_history) < int(self.parking_stable_scans)
                or max(self.park_history) - min(self.park_history) > self.parking_stable_tol):
            return None, points
        det['slide'] = float(np.median(self.park_history))
        det['slot_center'] = mission.slot_center + det['slide'] * mission.slide_axis
        return det, points

    def refine_parking(self, i):
        """동작 시작 parking_refine_until(m) 전까지는 칸을 계속 찾아, slide가 바뀌면 동작을 다시 만들어 넣음
        (멀리서는 입구 먼 쪽 끝이 비스듬히 보여 덜 정확함)."""
        plan = self.park_plan
        sp = plan['spliced']
        if self.park_seg != 0 or i >= sp.first:
            return
        if self.reference.stations[sp.first] - self.reference.stations[i] < self.parking_refine_until:
            return
        mission = plan['mission']
        det, points = self.stable_detection(mission)
        if det is None or abs(det['slide'] - plan['detection']['slide']) < self.parking_refine_tol:
            return
        old = plan['detection']['slide']
        points_in, yaws_in, gears_in = plan['route_in']
        self.route.points, self.route_yaws, self.route_gears = list(points_in), yaws_in, gears_in
        self.route_spliced = yaws_in is not None
        offsets = self.park_offsets
        self.park_offsets = list(plan['offsets_in'])
        self.reference = ReferencePath(self.route.points)
        if not self.start_parking(mission, det, points):
            self.get_logger().warn(f'{mission.id}: 다시 만든 동작이 없어 이전 동작 유지')
            self.park_done.discard(mission.id)
            self.route.points = [(float(px), float(py)) for px, py in sp.points]
            self.route_yaws, self.route_gears = sp.yaws, sp.gears
            self.reference = ReferencePath(self.route.points)
            self.route_spliced = True
            self.park_offsets = offsets
            self.park_plan = plan
        else:
            self.get_logger().info(f'{mission.id}: slide {old:+.2f} → {det["slide"]:+.2f} m로 동작 다시 넣음')

    def start_parking(self, mission, det, points):
        cfg = self.park_cfg
        params = self.parking_candidate_params()[mission.type]
        free = [float(v) for v in self.get_parameter(
            'parallel_parking_free' if mission.type == 'parallel_parking' else 'rear_parking_free').value]
        rects = [tuple(free[k:k + 4]) for k in range(0, len(free) - 3, 4)]
        maneuver = plan_maneuver(mission, det, mission.y_road, cfg, params, rects, points,
                                 log=self.get_logger().warn)
        if maneuver is None:
            # 관측점 검사를 빼고 다시 (LiDAR 잡음 점이 자유 공간 안에 있는 경우)
            maneuver = plan_maneuver(mission, det, mission.y_road, cfg, params, rects, None,
                                     log=self.get_logger().warn)
            if maneuver is None:
                # 포기하지 않음: 주차를 안 하고 다음 체크포인트를 지나면 시도 전체가 실패. 다음 주기에 다시 찾음
                self.get_logger().error(f'{mission.id}: 주차 동작을 만들지 못함 (slide {det["slide"]:+.2f}), 다시 찾음')
                self.park_history = []
                return False
            self.get_logger().warn(f'{mission.id}: 관측점 검사를 통과한 후보가 없어 벽 모형만으로 고름')
        route_in = (list(self.route.points), self.route_yaws if self.route_spliced else None,
                    self.route_gears if self.route_spliced else None)
        spliced = splice_route(route_in[0], maneuver, mission, self.wheelbase, route_in[1], route_in[2])
        if spliced is None or (self.route.idx is not None and spliced.first <= self.route.idx + 2):
            self.get_logger().error(f'{mission.id}: 주차 동작을 경로에 넣지 못함 (이미 지나침)')
            self.park_done.add(mission.id)
            return False
        self.route.points = [(float(px), float(py)) for px, py in spliced.points]
        self.route_yaws, self.route_gears = spliced.yaws, spliced.gears
        self.reference = ReferencePath(self.route.points)
        self.route_spliced = True
        offsets_in = list(self.park_offsets)
        self.park_offsets = offsets_in + [splice_offset(route_in[0], spliced)]
        self.park_plan = {'mission': mission, 'spliced': spliced, 'maneuver': maneuver, 'route_in': route_in,
                          'detection': det, 'rects': rects, 'vmax': self.parking_vmax(spliced),
                          'offsets_in': offsets_in}
        self.park_seg = 0
        self.park_stop_since = self.park_hold_since = None
        self.get_logger().info(
            f'{mission.id}: 칸 slide {det["slide"]:+.2f} m (입구 {det["gap"][0]:+.1f}~{det["gap"][1]:+.1f}, '
            f'벽 {det["wall"]:.2f} m), 칸 중심 ({det["slot_center"][0]:.2f}, {det["slot_center"][1]:.2f}), '
            f'동작 {maneuver.info}, 구간 ' + ', '.join(f'{s[5]}({"R" if s[2] < 0 else "D"}) {s[0]}~{s[1]}'
                                                     for s in spliced.segments))
        self.publish_parking_markers()
        return True

    def plan_parking(self, path, p, yaw, speed, now):
        """주차 동작 진행: 지금 구간만 발행, 정지 지점에서 멈추면 다음 구간으로."""
        plan = self.park_plan
        sp = plan['spliced']
        segs = sp.segments
        a, b, gear, stop, hold, name, end_pose = segs[self.park_seg]
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        rear = np.array([p.x, p.y])
        ref = rear if gear == REVERSE or a > sp.last else rear + self.wheelbase * heading
        if self.route.idx is None or self.route.idx < a:
            self.route.idx = a
        i = self.route.nearest(float(ref[0]), float(ref[1]), end=b + 1)
        if self.park_seg == 0 and i < sp.first:
            self.refine_parking(i)
            if self.park_plan is not plan:
                return self.plan_parking(path, p, yaw, speed, now) if self.park_plan is not None else None
            plan, sp = self.park_plan, self.park_plan['spliced']
            segs = sp.segments
            a, b, gear, stop, hold, name, end_pose = segs[self.park_seg]
        moving = abs(speed) >= self.parking_stopped_speed
        if moving:
            self.park_stop_since = None
        elif self.park_stop_since is None:
            self.park_stop_since = now
        rem = math.inf
        end = sp.points[b]
        if gear == REVERSE and stop and plan['mission'].type == 'rear_parking':
            end = end + self.back_wall_shift(plan, now)
        if stop:
            direction = sp.yaws[b] + (math.pi if gear == REVERSE else 0.0)
            rem = float(np.dot(end - ref, [math.cos(direction), math.sin(direction)]))
            stopped_for = 0.0 if self.park_stop_since is None else now - self.park_stop_since
            arrived = rem < self.parking_stop_tolerance or (rem < self.parking_accept_short
                                                            and stopped_for > self.parking_short_wait)
            if arrived and not moving:
                if hold > 0:
                    if self.park_hold_since is None and not self.check_in_slot(p, yaw):
                        if self.corrective_maneuver(p, yaw):
                            return self.plan_parking(path, p, yaw, speed, now)
                    if self.park_hold_since is None:
                        self.park_hold_since = now
                        self.get_logger().info(f'{plan["mission"].id}: 칸 도착 (남은 거리 {rem:+.2f} m), {hold:.1f} s 유지')
                        self.replan_exit(p, yaw)
                if hold <= 0 or now - self.park_hold_since >= hold:
                    self.get_logger().info(f'{plan["mission"].id}: {name} 끝 (남은 거리 {rem:+.2f} m)')
                    self.park_seg += 1
                    self.route.idx = segs[self.park_seg][0]
                    self.park_stop_since = self.park_hold_since = None
                    return self.plan_parking(path, p, yaw, speed, now)
            elif moving:
                self.park_hold_since = None
        # 동작을 다 끝내고 기존 경로로 돌아옴
        if self.park_seg == len(segs) - 1 and i > sp.last:
            self.park_done.add(plan['mission'].id)
            self.get_logger().info(f'{plan["mission"].id}: 주차 동작 끝, 기존 경로로')
            self.park_plan = None
            self.park_history = []                      # 다음 미션의 검출 기록은 새로 (앞 미션 slide가 섞이지 않게)
            self.park_voxels = {}
            self.park_info = {}
            return None
        # 발행: 지금 구간 안에서만 (cusp 너머로 이어지지 않게)
        step = max(1e-3, float(np.median(np.linalg.norm(np.diff(sp.points[a:b + 1], axis=0), axis=1)))) \
            if b > a else 0.25
        first = max(a, i - int(self.behind_m / step))
        last = min(b, i + int(self.ahead_m / step))
        if last - first < 1:
            first = max(a, last - 1)
        for k in range(first, last + 1):
            ps = PoseStamped(header=path.header)
            ps.pose.position.x, ps.pose.position.y = map(float, end if k == b else sp.points[k])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(sp.yaws[k] / 2), math.cos(sp.yaws[k] / 2)
            path.poses.append(ps)
        # 속력: 기본 상한, 원호 횡가속도, 정지 지점 감속, 유지
        # 이 구간의 점별 상한(기본·원호 횡가속도)을 감속도 parking_stop_decel로 앞에서부터 미리 반영
        vmax, stations = plan['vmax'], self.reference.stations
        hi = b + 1 if b < sp.last else sp.last + 1
        ahead = np.arange(i, max(i + 1, hi))
        cap = float(np.min(np.sqrt(vmax[ahead] ** 2 + 2 * self.parking_approach_decel
                                   * np.maximum(0.0, stations[ahead] - stations[i]))))
        if i < sp.first or i > sp.last:   # 기존 경로 위: 경로 속도 프로파일도
            cap = min(cap, self.profile_speed(float(stations[i]), speed))
        if stop:
            ramp = math.sqrt(2 * self.parking_stop_decel * max(0.0, rem - 0.5 * self.parking_stop_tolerance))
            cap = min(cap, ramp if ramp > 0.25 else 0.0)
        if self.park_hold_since is not None:
            cap = 0.0
        cap = max(0.0, cap)
        # 부호 = 방향, 크기 2 = 발행한 경로의 끝이 정지 지점, 1 = 경로가 이어짐. 동작 시작 전(기존 경로 위)은 0
        mode = (-1 if gear == REVERSE else 1) * (2 if stop and last == b else 1)
        if i < sp.first and mode == 1:
            mode = 0
        self.pub.publish(path)
        self.speed_pub.publish(Float32(data=float(cap)))
        self.gear_pub.publish(Int8(data=mode))
        self.park_info = {'parking': plan['mission'].id, 'segment': name, 'seg': self.park_seg,
                          'gear': int(gear), 'remaining': round(rem, 2) if math.isfinite(rem) else None,
                          'hold': round(now - self.park_hold_since, 1) if self.park_hold_since else None}
        self.publish_status('parking_' + name, speed=cap)
        return True

    def back_wall_shift(self, plan, now):
        """후면주차 후진: 방 북쪽 벽(칸 좌표계 x = rear_back_wall_x)을 지금 스캔에서 직접 재서 후진 정지 지점을
        옮길 거리 (map 벡터). 위치 추정 오차가 있어도 벽에 대한 상대 위치로 서게 함."""
        mission, maneuver = plan['mission'], plan['maneuver']
        clusters = self.fresh_clusters(now)
        shift = plan.get('wall_shift', 0.0)
        if clusters:
            frame = SlotFrame(maneuver.slot_center, mission.slot_yaw)
            pts = frame.to_local(np.vstack(clusters))
            x_wall = self.rear_back_wall_x
            sel = (np.abs(pts[:, 1]) < 1.6) & (np.abs(pts[:, 0] - x_wall) < self.parking_wall_search)
            if np.count_nonzero(sel) >= 10:
                measured = float(np.median(pts[sel, 0])) - x_wall
                measured = max(-self.parking_wall_search, min(self.parking_wall_search, measured))
                shift = measured if 'wall_shift' not in plan else 0.7 * shift + 0.3 * measured
                if abs(shift - plan.get('wall_logged', 99.0)) > 0.05:
                    plan['wall_logged'] = shift
                    self.get_logger().info(f'{mission.id}: 뒤 벽 {x_wall + measured:+.2f} m (예상 {x_wall:+.2f}), '
                                           f'후진 정지 지점 {shift:+.2f} m 옮김')
                plan['wall_shift'] = shift
        return shift * np.array([math.cos(mission.slot_yaw), math.sin(mission.slot_yaw)])

    def parking_vmax(self, sp):
        """경로 점별 속력 상한: 동작 안은 전진 parking_speed·후진 parking_reverse_speed와 원호 횡가속도
        (차 방향 변화 / 거리, 앞뒤 1 m 최대), 동작 밖은 제한 없음(경로 속도 프로파일이 따로 적용)."""
        n = len(sp.points)
        vmax = np.full(n, np.inf)
        a, b = sp.first, sp.last
        reverse = sp.gears[a:b + 1] < 0
        base = np.where(reverse, self.parking_reverse_speed, self.parking_speed)
        lateral = np.where(reverse, self.parking_reverse_lateral_accel, self.parking_lateral_accel)
        d = np.maximum(np.linalg.norm(np.diff(sp.points[a:b + 2], axis=0), axis=1), 1e-3)
        dyaw = np.abs(np.diff(np.unwrap(sp.yaws[a:b + 2])))
        kappa = np.where(d > 1.0, 0.0, dyaw / d)        # 정지 지점(앞바퀴 축 ↔ 뒤차축 점 사이 3 m 건너뜀)은 제외
        kappa = np.maximum.reduce([np.roll(kappa, k) for k in range(-4, 5)])
        vmax[a:b + 1] = np.minimum(base, np.sqrt(lateral / np.maximum(kappa, 1e-6)))
        return vmax

    def check_in_slot(self, p, yaw):
        """유지 전에 추정 자세로 칸 안인지 스스로 검사: 네 꼭짓점이 칸 경계 안쪽 parking_check_margin 이상,
        방향 오차 parking_check_yaw_deg 이하."""
        plan = self.park_plan
        mission = plan['mission']
        frame = SlotFrame(plan['maneuver'].slot_center, mission.slot_yaw)
        local = frame.to_local(np.array([p.x, p.y]))
        dyaw = (yaw - mission.slot_yaw + math.pi) % (2 * math.pi) - math.pi
        corners = body_corners((local[0], local[1], dyaw), self.park_cfg)
        margin = min(mission.slot_length / 2 - np.abs(corners[:, 0]).max(),
                     mission.slot_width / 2 - np.abs(corners[:, 1]).max())
        ok = margin >= self.parking_check_margin and abs(math.degrees(dyaw)) <= self.parking_check_yaw_deg
        if not ok:
            self.get_logger().warn(f'{mission.id}: 칸 안 자체 검사 실패 (꼭짓점 여유 {margin:+.2f} m, '
                                   f'방향 {math.degrees(dyaw):+.1f}°, 자세 {np.round(local, 2).tolist()})')
        return ok

    def corrective_maneuver(self, p, yaw):
        """칸 안 검사 실패: 앞으로 빠져나왔다가 다시 후진해 들어가는 구간을 지금 구간 뒤에 넣음 (최대
        parking_max_retries번). 평행: 실제 자세에서 전진 S자로 도로 중심 → 정지 → 후진 S자 다시.
        후면: 앞으로 parking_retry_pull m 직진 → 정지 → 계획한 최종 자세로 곧게 후진."""
        plan = self.park_plan
        mission, maneuver = plan['mission'], plan['maneuver']
        if plan.get('retries', 0) >= int(self.parking_max_retries):
            self.get_logger().error(f'{mission.id}: 다시 들어가기 {plan.get("retries", 0)}번 했으나 실패, 그대로 유지')
            return False
        frame = SlotFrame(maneuver.slot_center, mission.slot_yaw)
        local = frame.to_local(np.array([p.x, p.y]))
        pose = (float(local[0]), float(local[1]), float((yaw - mission.slot_yaw + math.pi) % (2 * math.pi) - math.pi))
        wall = plan['detection'].get('wall', self.park_cfg.wall_default)
        if abs(wall - self.park_cfg.wall_default) > 0.25:
            wall = self.park_cfg.wall_default
        rects = [(-80.0, 80.0, mission.y_road - wall, mission.y_road + wall)] + plan['rects']
        target = maneuver.segments[1].poses[-1]
        target = (float(frame.to_local(target[:2])[0]), float(frame.to_local(target[:2])[1]), 0.0)
        r = self.park_cfg.min_radius
        segs = None
        for clearance in (self.park_cfg.clearance, 0.5 * self.park_cfg.clearance):
            cfg = ParkingConfig(**{**self.park_cfg.__dict__, 'clearance': clearance})
            if mission.type == 'parallel_parking':
                side = -1.0 if mission.y_road > 0 else 1.0
                for k in (1.0, 1.1, 1.2, 1.35):
                    out = parallel_exit(pose, mission.y_road, r * k)
                    if out is None or not free_space_ok(out, rects, cfg)[0]:
                        continue
                    e = out[-1]
                    d = abs(e[1] - target[1])
                    th = math.acos(max(-1.0, 1.0 - d / (2 * r)))
                    b = PathBuilder(tuple(e), self.park_cfg.step)
                    b.turn(REVERSE, r, th, side > 0).turn(REVERSE, r, th, side < 0)
                    back = b.result()
                    if (in_slot(back[-1], mission.slot_length, mission.slot_width, self.park_cfg.slot_margin, cfg)
                            and free_space_ok(back, rects, cfg)[0]):
                        segs = (out, back)
                        break
            else:
                out = PathBuilder(pose, self.park_cfg.step).go(FORWARD, self.parking_retry_pull).result()
                end = out[-1]
                n = max(2, int(math.ceil(abs(end[0] - target[0]) / self.park_cfg.step)))
                xs = np.linspace(end[0], target[0], n + 1)
                ys = np.linspace(end[1], target[1], n + 1)
                back = np.column_stack((xs, ys, np.full_like(xs, math.atan2(end[1] - target[1], end[0] - target[0]))))
                if free_space_ok(out, rects, cfg)[0] and free_space_ok(back, rects, cfg)[0]:
                    segs = (out, back)
            if segs is not None:
                break
        if segs is None:
            self.get_logger().error(f'{mission.id}: 다시 들어가는 경로를 못 만듦, 그대로 유지')
            return False
        out, back = segs
        old = maneuver.segments
        k = self.park_seg
        hold = old[k].hold
        new_segs = old[:k] + [Segment(old[k].gear, old[k].poses, True, 0.0, old[k].name),
                              Segment(FORWARD, frame.poses_to_map(out), True, 0.0, 'retry_out'),
                              Segment(REVERSE, frame.poses_to_map(back), True, hold, 'retry_in')]
        ex = parallel_exit(back[-1], mission.y_road, r) if mission.type == 'parallel_parking' else None
        exit_poses = frame.poses_to_map(ex) if ex is not None else old[-1].poses
        new_segs.append(Segment(FORWARD, exit_poses, False, 0.0, 'exit'))
        new = Maneuver(new_segs, maneuver.slot_center, maneuver.slide, maneuver.start_x,
                       float(frame.to_local(exit_poses[-1, :2])[0]), maneuver.info)
        points, yaws, gears = plan['route_in']
        spliced = splice_route(points, new, mission, self.wheelbase, yaws, gears)
        if spliced is None or spliced.segments[k][0] != plan['spliced'].segments[k][0]:
            self.get_logger().error(f'{mission.id}: 다시 들어가는 경로를 끼우지 못함')
            return False
        self.route.points = [(float(px), float(py)) for px, py in spliced.points]
        self.route_yaws, self.route_gears = spliced.yaws, spliced.gears
        self.reference = ReferencePath(self.route.points)
        self.park_offsets = plan['offsets_in'] + [splice_offset(points, spliced)]
        plan['spliced'], plan['maneuver'] = spliced, new
        plan['vmax'] = self.parking_vmax(spliced)
        plan['retries'] = plan.get('retries', 0) + 1
        plan.pop('wall_shift', None)
        self.get_logger().info(f'{mission.id}: 앞으로 나갔다가 다시 후진 ({plan["retries"]}번째)')
        self.publish_parking_markers()
        return True

    def replan_exit(self, p, yaw):
        """평행주차: 칸에 선 실제 자세에서 나가는 전진 S자를 다시 만들어 경로의 마지막 구간을 바꿈."""
        plan = self.park_plan
        mission, maneuver = plan['mission'], plan['maneuver']
        if mission.type != 'parallel_parking':
            return
        frame = SlotFrame(maneuver.slot_center, mission.slot_yaw)
        local = frame.to_local(np.array([p.x, p.y]))
        pose = (float(local[0]), float(local[1]), float((yaw - mission.slot_yaw + math.pi) % (2 * math.pi) - math.pi))
        wall = plan['detection'].get('wall', self.park_cfg.wall_default)
        if abs(wall - self.park_cfg.wall_default) > 0.25:
            wall = self.park_cfg.wall_default
        rects = [(-80.0, 80.0, mission.y_road - wall, mission.y_road + wall)] + plan['rects']
        scale = float(self.get_parameter('parallel_exit_radius_scale').value)
        for clearance in (self.park_cfg.clearance, 0.5 * self.park_cfg.clearance):
            cfg = ParkingConfig(**{**self.park_cfg.__dict__, 'clearance': clearance})
            for k in (1.0, 1.1, 1.2, 1.35):
                poses = parallel_exit(pose, mission.y_road, self.park_cfg.min_radius * scale * k)
                if poses is None or not free_space_ok(poses, rects, cfg)[0]:
                    continue
                exit_segment = Segment(FORWARD, frame.poses_to_map(poses), False, 0.0, 'exit')
                new = Maneuver(maneuver.segments[:-1] + [exit_segment], maneuver.slot_center, maneuver.slide,
                               maneuver.start_x, float(poses[-1][0]), maneuver.info)
                points, yaws, gears = plan['route_in']
                spliced = splice_route(points, new, mission, self.wheelbase, yaws, gears)
                if spliced is None or spliced.segments[-1][0] != plan['spliced'].segments[-1][0]:
                    continue
                self.route.points = [(float(px), float(py)) for px, py in spliced.points]
                self.route_yaws, self.route_gears = spliced.yaws, spliced.gears
                self.reference = ReferencePath(self.route.points)
                self.park_offsets = plan['offsets_in'] + [splice_offset(points, spliced)]
                plan['spliced'], plan['maneuver'] = spliced, new
                plan['vmax'] = self.parking_vmax(spliced)
                self.get_logger().info(f'{mission.id}: 실제 자세 {np.round(pose, 2).tolist()}에서 나가는 경로 다시 만듦 '
                                       f'(R {self.park_cfg.min_radius * scale * k:.2f}, 여유 {clearance:.2f})')
                self.publish_parking_markers()
                return
        self.get_logger().warn(f'{mission.id}: 실제 자세 {np.round(pose, 2).tolist()}에서 나가는 경로를 못 만들어 계획대로 나감')

    def publish_parking_markers(self):
        if self.park_plan is None:
            return
        sp = self.park_plan['spliced']
        header = Path().header
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()
        markers = [Marker(action=Marker.DELETEALL)]
        for n, (a, b, gear, *_rest) in enumerate(sp.segments):
            m = Marker(header=header, ns='parking_segments', id=n, type=Marker.LINE_STRIP, action=Marker.ADD)
            m.pose.orientation.w = 1.0
            m.scale.x = 0.08
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = (0.9, 0.3, 0.1) if gear == REVERSE else (0.1, 0.6, 0.95)
            m.points = [Point(x=float(x), y=float(y), z=0.2) for x, y in sp.points[a:b + 1]]
            markers.append(m)
        slot = Marker(header=header, ns='parking_slot', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
        mission, centre = self.park_plan['mission'], self.park_plan['maneuver'].slot_center
        frame = SlotFrame(centre, mission.slot_yaw)
        hl, hw = mission.slot_length / 2, mission.slot_width / 2
        corners = frame.to_map(np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw], [hl, hw]]))
        slot.pose.orientation.w = 1.0
        slot.scale.x = 0.08
        slot.color.g = slot.color.a = 1.0
        slot.points = [Point(x=float(x), y=float(y), z=0.2) for x, y in corners]
        markers.append(slot)
        self.park_marker_pub.publish(MarkerArray(markers=markers))

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
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if abs(m.twist.twist.linear.x) > 0.05:
                self.park_still_since = None
            elif getattr(self, 'park_still_since', None) is None:
                self.park_still_since = stamp

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
        self.collect_parking_points(self.obstacles)
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
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        speed = self.odom.twist.twist.linear.x
        if self.park_plan is not None and self.plan_parking(path, p, yaw, speed, now):
            return
        i = self.route.nearest(p.x, p.y)
        if self.route.past_end(p.x, p.y):
            self.publish_stop('route_end')
            return
        station = float(self.reference.stations[min(i, len(self.reference.stations) - 1)])
        route_speed = self.profile_speed(station, self.odom.twist.twist.linear.x)
        route_speed = min(route_speed, self.dynamic_limit(station, p.x, p.y, self.odom.twist.twist.linear.x, now))
        self.park_station = self.base_station(station)     # 주차 미션 거리(원래 경로 기준)와 비교
        route_speed = min(route_speed, self.parking_detect(self.park_station, speed, now))
        if self.park_plan is not None and self.plan_parking(path, p, yaw, speed, now):
            return
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
            self.gear_pub.publish(Int8(data=0))
            self.publish_status('reference_only', speed=route_speed)
            return
        if (self.obstacles is None or self.observation_stamp is None
                or not -self.clock_tolerance - 1e-6 <= now - self.observation_stamp <= self.obstacle_timeout):
            self.publish_stop('sensor_unavailable')
            return
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
        self.gear_pub.publish(Int8(data=0))
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

    def base_station(self, station):
        """끼운 경로의 경로 거리 → 동작을 끼우기 전 경로의 경로 거리 (동작 뒤 구간만 옮겨짐)."""
        for threshold, delta in reversed(getattr(self, 'park_offsets', [])):
            if station >= threshold:
                station -= delta
        return station

    def profile_speed(self, station, speed):
        """속도 프로파일에서 지금부터 speed_preview_s 동안 지날 구간의 가장 낮은 제한."""
        if self.route_spliced:
            # 동작을 끼운 경로: 원래 경로의 속도 프로파일을 그대로 쓰고 경로 거리만 원래 경로 기준으로 옮김
            # (끼운 경로 전체로 다시 만들면 정지 지점의 스플라인 꼬임이 동작 뒤 구간까지 느리게 만들었음)
            station = self.base_station(station)
        elif getattr(self, 'profile_reference', None) is not self.reference:
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
        payload.update(self.park_info)
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


def splice_offset(points_in, spliced):
    """끼운 경로에서 동작 뒤 첫 점의 경로 거리와, 그 점의 원래(끼우기 전) 경로 거리와의 차이."""
    points_in = np.asarray(points_in, dtype=float)
    j = spliced.last + 1
    s_new = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(spliced.points[:j + 1], axis=0), axis=1))]
    i1 = int(np.argmin(np.linalg.norm(points_in - spliced.points[j], axis=1)))
    s_in = float(np.sum(np.linalg.norm(np.diff(points_in[:i1 + 1], axis=0), axis=1)))
    return float(s_new[j]), float(s_new[j] - s_in)


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
