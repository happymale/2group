import sys
p = sys.argv[1]
s = open(p, encoding='utf-8').read()


def rep(old, new, cnt=1):
    global s
    assert s.count(old) == cnt, (old, s.count(old))
    s = s.replace(old, new)


rep('''Inside a zone, fresh map-frame obstacle clouds are required; missing perception or an infeasible plan
publishes an empty Path and zero speed.
"""''', '''Inside a zone, fresh map-frame obstacle clouds are required; missing perception or an infeasible plan
publishes an empty Path and zero speed.

Roundabout (roundabout.py): at load time the kinked entry/exit of route.csv is replaced by tangent arcs that the
vehicle can follow (published as the front-axle trace of the designed rear-axle path). NPCs on the ring are tracked
from the obstacle clouds; before the entry the planner yields at a stop point when the predicted merge gap is too
small, and on the ring it caps speed from the gap to the NPC ahead. A generic forward-collision speed cap for any
observed cluster on the path is applied inside collision zones (default: the roundabout section).
"""''')
rep('''from .course import COURSE_DIR, declare_file_params, load_route, Route
''', '''from .course import COURSE_DIR, declare_file_params, load_route, Route
from .roundabout import NpcTracker, follow_speed, merge_conflicts, ring_reference, stop_point, travel_time, wrap
''')
rep('''    'obstacle_timeout': 0.35,
    'odom_timeout': 0.35,
    'clock_tolerance': 0.05,
}''', '''    'obstacle_timeout': 0.35,
    'odom_timeout': 0.35,
    'clock_tolerance': 0.05,
    # 회전교차로 (roundabout.py). 반지름·거리는 m, 각속도는 rad/s
    'roundabout_entry_radius': 7.25,  # 진입 원호 반지름 (뒤차축)
    'roundabout_exit_radius': 7.25,   # 진출 원호 반지름 (뒤차축)
    'roundabout_ring_offset': 0.0,    # 우리 링 반지름 = NPC 반지름 + 이 값
    'roundabout_smooth_m': 2.5,       # 곡률이 바뀌는 곳을 둥글리는 길이 (조향 속도)
    'roundabout_speed': 4.0,          # 정지 위치 ~ 진출 직선 속력 상한 (m/s)
    'roundabout_stop_radius': 13.5,   # 양보 정지 위치: 앞 범퍼가 링 중심에서 이 거리
    'npc_rate': 0.33,                 # NPC 각속도 사전값 (관측 rb_t1: 약 0.33 rad/s = 3.6 m/s)
    'npc_track_timeout': 2.5,         # 관측 없이 NPC 추적을 유지하는 시간 (s)
    'npc_gap_behind': 8.0,            # 합류 순간 뒤 NPC 중심이 합류점에서 이만큼(호 m) 뒤에 있어야 함
    'npc_gap_ahead': 8.0,             # 합류 순간 앞 NPC 중심이 합류점에서 이만큼 앞에 있어야 함
    'npc_follow_gap': 9.0,            # 링에서 앞 NPC 중심 ~ 우리 뒤차축 최소 거리 (호 m, 범퍼 간격 약 2.9 m)
    'merge_accel': 1.2,               # 합류 시각 예측에 쓰는 가속도 (m/s²)
    'yield_decel': 2.0,               # 정지 위치에 서는 감속도 (m/s²)
    'yield_timeout': 20.0,            # 이만큼 서 있어도 틈이 없으면 진입 (s, 허상 추적으로 영영 서는 것 방지)
    'collision_decel': 3.0,           # 경로 위 물체 앞 감속도 (m/s²)
    'collision_stop_gap': 3.0,        # 경로 위 물체와 앞 범퍼 사이에 남길 거리 (m)
    'collision_half_width': 1.2,      # 경로에서 이 거리 안의 관측점을 경로 위 물체로 봄 (m)
    'collision_horizon': 25.0,        # 이만큼 앞까지 검사 (m)
}''')
rep('''        self.declare_parameter('attempt_course_json', '/data/current/course.json')
''', '''        self.declare_parameter('attempt_course_json', '/data/current/course.json')
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
''')
rep('''    def on_reset(self, checkpoint, reload):
        if reload or not self.route.points:
            self.route = load_route(self)
            self.reference = ReferencePath(self.route.points)
            self.build_profile()''', '''    def on_reset(self, checkpoint, reload):
        if reload or not self.route.points:
            self.route = load_route(self)
            self.setup_roundabout()
            self.build_profile()''')
rep('''        self.route.reset()
        self.odom = None
        self.obstacles = self.observation_stamp = self.previous = None
        self.reset_time = self.now()
        self.active = True
''', '''        self.route.reset()
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
                smooth_m=self.roundabout_smooth_m)
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
                ego = math.atan2(y - ring.center[1], x - ring.center[0])
                end = ring.exit_angle + math.radians(25.0)
                for k in tracks:
                    angle = k.predict(now)
                    gap = direction * wrap(angle - ego) * radius
                    if 0.0 < gap < 25.0 and direction * wrap(angle - end) < 0:
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
''')
rep('''            self.obstacles = [xy[rows['cluster_id'] == cluster_id] for cluster_id in np.unique(rows['cluster_id'])]
        except (ValueError, AssertionError, KeyError):
            self.obstacles = None
            return
        self.observation_stamp = stamp''', '''            self.obstacles = [xy[rows['cluster_id'] == cluster_id] for cluster_id in np.unique(rows['cluster_id'])]
        except (ValueError, AssertionError, KeyError):
            self.obstacles = None
            return
        self.observation_stamp = stamp
        if self.npc is not None:
            self.npc.update(stamp, self.obstacles)''')
rep('''        route_speed = self.profile_speed(station, self.odom.twist.twist.linear.x)
''', '''        route_speed = self.profile_speed(station, self.odom.twist.twist.linear.x)
        route_speed = min(route_speed, self.dynamic_limit(station, p.x, p.y, self.odom.twist.twist.linear.x, now))
''')
rep('''        payload = {'state': reason, 'offset_m': float(offset), 'speed_limit_mps': float(speed), 'tight': bool(tight),
                   'candidate_results': dict(Counter(c.reason for c in candidates))}''', '''        payload = {'state': reason, 'offset_m': float(offset), 'speed_limit_mps': float(speed), 'tight': bool(tight),
                   'candidate_results': dict(Counter(c.reason for c in candidates))}
        payload.update(self.dynamic_info)''')
rep('''        label = reason + (' (여유 축소)' if tight else '')''', '''        label = reason + (' (여유 축소)' if tight else '')
        if self.dynamic_info.get('roundabout'):
            label += ' / 회전교차로 ' + self.dynamic_info['roundabout']''')
rep('''    def build_profile(self):
        flat = [float(v) for v in self.get_parameter('speed_zones').value]
        zones = [tuple(flat[k:k + 3]) for k in range(0, len(flat) - 2, 3)]
        self.profile_s, self.profile_v = speed_profile(
            self.reference, self.cruise_speed, self.corner_lateral_accel, self.profile_accel, self.profile_decel,
            zones=zones)
        self.profile_reference = self.reference''', '''    def build_profile(self):
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
        self.profile_reference = self.reference''')
rep('''from std_msgs.msg import Float32, String''', '''from scipy.spatial import cKDTree
from std_msgs.msg import Float32, String''')
open(p, 'w', encoding='utf-8').write(s)
print('ok')
