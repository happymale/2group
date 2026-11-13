"""회전교차로: 따라갈 수 있는 기준 경로, NPC 추적, 진입 양보·차간 거리 판단 (ROS 독립, planner.py가 씀).

1) 기준 경로 (ring_reference)
   route.csv의 회전교차로 진입·진출부는 반지름 약 2.5 m로 꺾여 최소 회전 반지름(약 4.2 m)으로는 따라갈 수 없습니다.
   그 부분을 뒤차축 경로 = 직선 → 진입 원호(entry_radius, 링과 바깥 접함) → 링 원호(ring_radius) → 진출 원호
   (exit_radius) → 직선 으로 바꿉니다. 차체 네 모서리가 통로(route.csv ± 2.196 m) 안에 남도록 반지름을 고릅니다
   (뒤차축이 정확히 따르면 entry 7.0 / exit 7.5 / ring 11.0 에서 최소 여유 약 +0.4 m, tools_rb/geo_eval.py).
   control.py의 Stanley는 앞바퀴 축을 경로에 붙이므로, 곡선에서 뒤차축은 √(R²+L²)-R 만큼 안쪽을 지납니다.
   그래서 이 구간은 설계한 뒤차축 경로의 '앞바퀴 축 궤적'(뒤차축 + L·진행 방향)을 내보냅니다. 앞바퀴 축이 그
   궤적을 따르면 뒤차축은 설계 경로를 그대로 지납니다. 곡률이 한 번에 바뀌는 곳은 짧게 이동 평균해 둥글립니다.

2) NPC 추적 (NpcTracker)
   /hk/perception/obstacles 군집 중 링 위(중심에서 ring ± band)에 있는 것을 링 각도로 추적해 각속도를 추정합니다.
   LiDAR는 차의 보이는 면만 보므로 군집 중심 각도를 차 중심 각도로 씁니다(부분 관측이면 몇 도 어긋남 → 여유로 흡수).
   관측이 끊기면 track_timeout 동안 예측만으로 유지합니다.

3) 진입 판단 (merge_conflicts) · 차간 거리 (follow_speed)
   우리 뒤차축이 링 합류점에 닿는 예상 시각에, 각 NPC의 예측 위치가 합류점 뒤 gap_behind ~ 앞 gap_ahead (링 호 길이)
   안에 있으면 진입하지 않고 정지선에서 기다립니다. 링 안에서는 앞 NPC와의 거리로 속력 상한을 둡니다.
"""
from dataclasses import dataclass, field
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _arc(center, radius, a0, a1, step):
    """center 중심 원호를 각도 a0 → a1 로 (a1 - a0 부호가 방향) step 간격 점으로."""
    n = max(2, int(math.ceil(abs(a1 - a0) * radius / step)) + 1)
    a = np.linspace(a0, a1, n)
    return np.column_stack((center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)))


def _line_fit(points):
    """점들의 주 방향 직선 (지나는 점, 단위 방향). 방향은 점 순서를 따름."""
    mean = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - mean)
    u = vt[0]
    if np.dot(points[-1] - points[0], u) < 0:
        u = -u
    return mean, u


def _resample(points, step):
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-6]
    points = points[keep]
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    t = np.linspace(0.0, s[-1], max(2, int(round(s[-1] / step)) + 1))
    return np.column_stack((np.interp(t, s, points[:, 0]), np.interp(t, s, points[:, 1])))


def _last_before(proj, end, limit):
    """end 앞에서 proj < limit 인 마지막 연속 구간의 끝 번호."""
    k = end - 1
    while k >= 0 and proj[k] >= limit:
        k -= 1
    return k if k >= 0 else None


def _first_after(proj, start, end, limit):
    """start 뒤에서 proj > limit 인 첫 번호."""
    k = start + 1
    while k < end and proj[k] <= limit:
        k += 1
    return k if k < end else None


@dataclass
class RingGeometry:
    """바꾼 회전교차로 구간. 점은 모두 뒤차축 기준 지도 좌표 (planner가 경로 거리로 투영해 씀)."""
    center: tuple
    npc_radius: float       # NPC가 도는 반지름
    radius: float           # 우리 뒤차축 링 반지름
    direction: int          # +1 반시계, -1 시계 (NPC 진행 방향 = 우리 링 진행 방향)
    entry_angle: float      # 링 합류 각도 (rad, 진입 원호와 링의 접점)
    exit_angle: float       # 링 이탈 각도 (rad)
    approach: tuple         # 진입 직선 (지나는 점, 단위 방향)
    entry_point: tuple      # 진입 원호 시작 (직선 끝)
    merge_point: tuple      # 링 합류점
    leave_point: tuple      # 링 이탈점
    exit_point: tuple       # 진출 원호 끝 (직선 시작)


def ring_reference(points, center, npc_radius, ring_radius=None, entry_radius=7.0, exit_radius=7.5,
                   wheelbase=3.0, step=0.5, smooth_m=2.5, line_band=(3.5, 8.5),
                   cut_bias=0.0, profile_smooth_n=3, profile_smooth_span=80):
    """(제어기에 줄 경로, 뒤차축 설계 경로, RingGeometry). 회전교차로가 없거나 못 바꾸면 (points, points, None).

    진입·진출 직선은 링 중심에서 npc_radius + line_band 거리에 있는 경로 점들로 맞춥니다.
    바꾼 구간은 뒤차축 설계 경로의 앞바퀴 축 궤적(wheelbase만큼 앞)으로 내보냅니다. 뒤차축 설계 경로는 곡률로 속도 프로파일을 만드는 데 씁니다 (앞바퀴 축 궤적은 조향 변화 때문에
    곡률이 커 보임).
    cut_bias (m²): 제어기 경로를 곡선 바깥으로 cut_bias × 곡률(1/m) 만큼 옮김. 실주행에서 짧은 원호마다 곡선 안쪽으로
    0.2~0.5 m 파고드는 것(조향 지연 예측·과도 응답)을 미리 보상합니다 (R 7.25 원호에서 1.8 → 0.25 m).
    """
    pts = np.asarray(points, dtype=float)
    c = np.asarray(center, dtype=float)
    radius = float(ring_radius or npc_radius)
    d = np.linalg.norm(pts - c, axis=1)
    near = np.flatnonzero(d < npc_radius + 1.0)
    if len(near) < 3:
        return pts, pts, None
    first, last = int(near[0]), int(near[-1])
    lo, hi = npc_radius + line_band[0], npc_radius + line_band[1]
    before = [i for i in range(max(0, first - 60), first) if lo <= d[i] <= hi]
    after = [i for i in range(last + 1, min(len(pts), last + 61)) if lo <= d[i] <= hi]
    if len(before) < 4 or len(after) < 4:
        return pts, pts, None
    p0, u = _line_fit(pts[before])
    p1, w = _line_fit(pts[after])
    # 링 진행 방향: 링 위 경로 점들의 각도 변화 부호
    ang = np.unwrap(np.arctan2(pts[first:last + 1, 1] - c[1], pts[first:last + 1, 0] - c[0]))
    direction = 1 if ang[-1] > ang[0] else -1
    # 반시계 링이면 진입·진출 원호는 오른쪽으로 돎 (원호 중심이 진행 방향 오른쪽)
    side_u = -direction * np.array([-u[1], u[0]])
    side_w = -direction * np.array([-w[1], w[0]])
    h = float(np.dot(c - p0, side_u))
    h2 = float(np.dot(c - p1, side_w))
    reach = (radius + entry_radius) ** 2 - (entry_radius - h) ** 2
    reach2 = (radius + exit_radius) ** 2 - (exit_radius - h2) ** 2
    if reach <= 0 or reach2 <= 0:
        return pts, pts, None
    t_e = float(np.dot(c - p0, u)) - math.sqrt(reach)        # 진입 원호 중심은 링 중심보다 앞(상류)
    e = p0 + t_e * u + entry_radius * side_u
    t_x = float(np.dot(c - p1, w)) + math.sqrt(reach2)       # 진출 원호 중심은 링 중심보다 뒤(하류)
    x = p1 + t_x * w + exit_radius * side_w
    a_in = math.atan2(e[1] - c[1], e[0] - c[0])               # 링과 진입 원호의 접점 각도
    a_out = math.atan2(x[1] - c[1], x[0] - c[0])
    line_in, line_out = p0 + t_e * u, p1 + t_x * w
    proj_in, proj_out = (pts - p0) @ u, (pts - p1) @ w
    # 진입·진출 원호는 링과 반대 방향으로 돎
    b0 = math.atan2(line_in[1] - e[1], line_in[0] - e[0])
    db = wrap(math.atan2(c[1] - e[1], c[0] - e[0]) - b0)
    if db * direction > 0:
        db -= direction * 2 * math.pi
    da = wrap(a_out - a_in)
    if da * direction < 0:
        da += direction * 2 * math.pi
    g0 = math.atan2(c[1] - x[1], c[0] - x[0])
    dg = wrap(math.atan2(line_out[1] - x[1], line_out[0] - x[0]) - g0)
    if dg * direction > 0:
        dg -= direction * 2 * math.pi
    fine = 0.05
    arcs = np.vstack((line_in, _arc(e, entry_radius, b0, b0 + db, fine), _arc(c, radius, a_in, a_in + da, fine),
                      _arc(x, exit_radius, g0, g0 + dg, fine), line_out))
    # 앞바퀴 축 궤적으로 바꾸는 구간: 진입 직선 위 (L + smooth_m) 앞 ~ 진출 직선 위 smooth_m/2 뒤 (뒤차축 기준).
    # 그 밖(회전교차로 앞뒤의 완만한 곡선)은 원래 경로·원래 추종 그대로
    wheelbase = max(float(wheelbase), 0.0)
    lead_in, lead_out = wheelbase + smooth_m, smooth_m / 2
    poly = np.vstack((line_in - lead_in * u, arcs, line_out + lead_out * w))
    cum = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    rear = _resample(poly, fine)
    spacing = cum[-1] / (len(rear) - 1)
    n = int(round(smooth_m / spacing / 2))
    if n > 0:                                                 # 곡률이 한 번에 바뀌는 곳을 둥글림 (끝 직선은 그대로)
        kernel = np.ones(2 * n + 1) / (2 * n + 1)
        rear = np.column_stack([np.convolve(np.pad(rear[:, k], n, mode='reflect', reflect_type='odd'), kernel,
                                            mode='valid') for k in range(2)])
    direction_xy = np.gradient(rear, axis=0)
    direction_xy /= np.maximum(np.linalg.norm(direction_xy, axis=1, keepdims=True), 1e-9)
    front = rear + wheelbase * direction_xy
    if cut_bias:
        second = np.gradient(direction_xy, axis=0) / spacing
        kappa = direction_xy[:, 0] * second[:, 1] - direction_xy[:, 1] * second[:, 0]
        k = max(1, int(round(0.5 / spacing)))
        kappa = np.convolve(np.pad(kappa, k, mode='edge'), np.ones(2 * k + 1) / (2 * k + 1), mode='valid')
        normal = np.column_stack((-direction_xy[:, 1], direction_xy[:, 0]))          # 왼쪽
        front -= (cut_bias * kappa)[:, None] * normal                                  # 곡선 바깥으로
    # 원래 경로는 앞바퀴 축 궤적 시작점 앞, 끝점 뒤만 남김 (둘 다 진입·진출 직선 위)
    i_head = _last_before(proj_in, first, t_e - smooth_m - 0.1)
    i_tail = _first_after(proj_out, last, len(pts), t_x + lead_out + wheelbase + 0.1)
    if i_head is None or i_tail is None:
        return pts, pts, None
    out = np.vstack((pts[:i_head], _resample(np.vstack((pts[i_head], front, pts[i_tail])), step), pts[i_tail + 1:]))
    # 속도 프로파일용 뒤차축 경로: 같은 앞부분 + 설계 경로(진입 직선 위 부분부터) + 진출 직선 위 이후
    i_rear = _first_after(proj_out, last, len(pts), t_x + lead_out + 0.1)
    keep = (rear - p0) @ u > t_e - smooth_m
    rear_out = np.vstack((pts[:i_head], _resample(np.vstack((pts[i_head], rear[keep], pts[i_rear])), step),
                          pts[i_rear + 1:]))
    # route.csv 점은 곡선에서 들쭉날쭉해 스플라인 곡률이 ±0.3까지 튐(진출 뒤 R≈15 곡선이 3.6 m/s로 묶임).
    # 속도 프로파일용 경로만 회전교차로 앞뒤 profile_smooth_span 점 안에서 이동 평균
    if profile_smooth_n > 0:
        a = max(i_head - profile_smooth_span, profile_smooth_n)
        b = min(len(rear_out) - (len(pts) - i_rear) + profile_smooth_span, len(rear_out) - profile_smooth_n - 1)
        kernel = np.ones(2 * profile_smooth_n + 1) / (2 * profile_smooth_n + 1)
        window = rear_out[a - profile_smooth_n:b + profile_smooth_n + 1]
        rear_out = rear_out.copy()
        rear_out[a:b + 1] = np.column_stack([np.convolve(window[:, k], kernel, mode='valid') for k in range(2)])
    geometry = RingGeometry(
        center=(float(c[0]), float(c[1])), npc_radius=float(npc_radius), radius=radius, direction=direction,
        entry_angle=a_in, exit_angle=a_in + da, approach=(tuple(p0), tuple(u)),
        entry_point=tuple(line_in), merge_point=tuple(c + radius * np.array([math.cos(a_in), math.sin(a_in)])),
        leave_point=tuple(c + radius * np.array([math.cos(a_in + da), math.sin(a_in + da)])),
        exit_point=tuple(line_out))
    return out, rear_out, geometry


def stop_point(geometry, stop_radius, front_length):
    """진입 직선 위 정지 위치(뒤차축): 앞 범퍼가 링 중심에서 stop_radius 떨어지는 곳."""
    p0, u = np.asarray(geometry.approach[0]), np.asarray(geometry.approach[1])
    c = np.asarray(geometry.center)
    along = float(np.dot(c - p0, u))
    h2 = float(np.sum((c - p0) ** 2)) - along ** 2
    t = along - math.sqrt(max(stop_radius ** 2 - h2, 0.0)) - front_length
    return tuple(p0 + t * u)


# ---- NPC 추적 ---------------------------------------------------------------------------------------------
@dataclass
class NpcTrack:
    angle: float            # 링 각도 (rad)
    rate: float             # 각속도 (rad/s, 반시계 +)
    stamp: float            # 마지막 갱신(예측 기준) 시각
    seen: float             # 마지막 관측 시각
    hits: int = 1

    def predict(self, t):
        return wrap(self.angle + self.rate * (t - self.stamp))


@dataclass
class NpcTracker:
    center: tuple
    radius: float                   # NPC 링 반지름
    direction: int = 1
    nominal_rate: float = 0.33      # 관측 전 각속도 사전값 (rad/s, 진행 방향 크기)
    band_in: float = 2.0            # 링 안쪽으로 이만큼(m)까지 군집 중심을 NPC로 봄
    band_out: float = 1.6           # 링 바깥쪽
    max_extent: float = 6.5         # 이보다 큰 군집은 NPC가 아님 (벽·연석)
    min_points: int = 2
    gate: float = math.radians(20.0)
    merge_angle: float = math.radians(12.0)   # 한 차가 두 군집으로 나뉜 것을 합치는 각도
    timeout: float = 2.5            # 관측 없이 예측만으로 유지하는 시간 (s)
    gain: float = 0.5               # 각도 갱신 이득
    rate_gain: float = 0.05         # 각속도 갱신 이득
    rate_bounds: tuple = (0.5, 1.6)  # 각속도를 사전값의 이 배수 사이로 제한
    tracks: list = field(default_factory=list)

    def reset(self):
        self.tracks = []

    def measurements(self, clusters):
        """군집들 → NPC로 보이는 링 각도 목록."""
        c = np.asarray(self.center, dtype=float)
        angles = []
        for points in clusters:
            points = np.asarray(points, dtype=float)
            if len(points) < self.min_points or np.max(np.ptp(points, axis=0)) > self.max_extent:
                continue
            centroid = points.mean(axis=0)
            r = float(np.linalg.norm(centroid - c))
            if not self.radius - self.band_in <= r <= self.radius + self.band_out:
                continue
            angles.append(math.atan2(centroid[1] - c[1], centroid[0] - c[0]))
        angles.sort()
        merged = []
        for a in angles:
            if merged and abs(wrap(a - merged[-1][-1])) < self.merge_angle:
                merged[-1].append(a)
            else:
                merged.append([a])
        if len(merged) > 1 and abs(wrap(merged[0][0] - merged[-1][-1])) < self.merge_angle:
            merged[0] = merged.pop() + merged[0]
        return [math.atan2(np.mean(np.sin(g)), np.mean(np.cos(g))) for g in merged]

    def update(self, t, clusters):
        meas = self.measurements(clusters)
        for track in self.tracks:
            track.angle, track.stamp = track.predict(t), t
        free = list(range(len(meas)))
        for track in sorted(self.tracks, key=lambda k: -k.hits):
            if not free:
                break
            j = min(free, key=lambda k: abs(wrap(meas[k] - track.angle)))
            error = wrap(meas[j] - track.angle)
            if abs(error) > self.gate:
                continue
            free.remove(j)
            dt = max(t - track.seen, 0.05)
            track.angle = wrap(track.angle + self.gain * error)
            # 진행 방향 기준. 부분 관측(보이는 면이 바뀜)으로 군집 중심이 멈춘 듯 보일 때가 있어 사전값 근처로 묶음
            low, high = self.rate_bounds[0] * self.nominal_rate, self.rate_bounds[1] * self.nominal_rate
            rate = self.direction * track.rate + self.rate_gain * self.direction * error / dt
            track.rate = self.direction * min(max(rate, low), high)
            track.seen = t
            track.hits += 1
        for j in free:
            self.tracks.append(NpcTrack(meas[j], self.direction * self.nominal_rate, t, t))
        self.tracks = [k for k in self.tracks if t - k.seen <= self.timeout]
        # 같은 차를 가리키게 된 두 추적은 하나로
        self.tracks.sort(key=lambda k: -k.hits)
        kept = []
        for track in self.tracks:
            if all(abs(wrap(track.angle - other.angle)) > self.merge_angle for other in kept):
                kept.append(track)
        self.tracks = kept

    def confirmed(self, min_hits=2):
        return [k for k in self.tracks if k.hits >= min_hits]


# ---- 진입·차간 판단 ----------------------------------------------------------------------------------------
def travel_time(distance, speed, accel, max_speed):
    """distance(m)를 speed에서 accel로 max_speed까지 가속하며 가는 시간."""
    if distance <= 0:
        return 0.0
    speed = min(max(speed, 0.0), max_speed)
    d_acc = (max_speed ** 2 - speed ** 2) / (2 * accel)
    if distance <= d_acc:
        return (-speed + math.sqrt(speed ** 2 + 2 * accel * distance)) / accel
    return (max_speed - speed) / accel + (distance - d_acc) / max_speed


def merge_conflicts(tracks, t_now, t_arrive, merge_angle, radius, direction, gap_behind, gap_ahead, step=0.25):
    """합류점에 t_arrive 뒤 닿을 때 막는 NPC 목록 [(호 거리 m: + 합류점 앞, 추적), ...].

    합류 시각에 NPC 중심이 합류점 뒤 gap_behind ~ 앞 gap_ahead 안이면 충돌 위험. 지금부터 합류 시각까지 그
    사이를 지나가는 NPC(뒤에서 와서 앞으로 빠져나가는 중)도 앞 gap_ahead를 못 벌리면 위험으로 봅니다.
    """
    blocking = []
    for track in tracks:
        arc = direction * wrap(track.predict(t_now + t_arrive) - merge_angle) * radius
        if -gap_behind < arc < gap_ahead:
            blocking.append((arc, track))
    return blocking


def follow_speed(gap, lead_speed, min_gap, decel, time_gap=0.8):
    """앞 차와의 중심 간 거리 gap(m)에서 낼 수 있는 속력 상한: 앞차가 서도 min_gap 앞에서 설 수 있게."""
    room = gap - min_gap
    if room <= 0:
        return 0.0
    v = math.sqrt(max(lead_speed, 0.0) ** 2 + 2 * decel * room)
    return min(v, max(lead_speed, 0.0) + room / time_gap)
