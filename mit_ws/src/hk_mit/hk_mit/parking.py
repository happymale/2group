"""주차 (평행·후면): LiDAR로 칸 위치 검출 → 전역 경로에 넣을 주차 동작(전진·후진 구간) 생성 → 차체 검사.

ROS와 무관한 계산만 둡니다 (planner.py가 씁니다).

관측한 구조 (탐색 주행 pk_probe*, seed 0/1/2, 주차를 켜고 끔):
  - cp8~cp10 도로는 벽이 도로 중심에서 ±3.0 m (주차 구간은 도로 폭 단계로 좁히지 않음).
  - 평행주차: 오른쪽 벽에 8.0 × 4.0 m 홈(pocket). 홈 중심 = 칸 중심. 칸 표시는 LiDAR에 보이지 않음.
  - 후면주차: 왼쪽 벽에 폭 5.0 m 통로 → 통로 끝에 남북 19 × 5 m 방. 칸은 방 북쪽 끝(벽에서 0.1 m),
    통로 중심은 course.json 미션 존 중심과 같은 위치(칸 중심에서 도로 방향 6.7 m 앞).
  - seed는 칸·홈·통로·방을 함께 도로 방향으로 밀어 줌(slide). 미션을 끄면 slide 0 자리.
  → 칸 쪽 벽 선에서 '열린 구간'(홈 입구, 통로 입구)을 찾아 그 중심과 미션 존 중심의 차이를 slide로 씁니다.

좌표: 칸 좌표계 = 원점 칸 중심(검출), x = 칸 yaw 방향(= 도로 진행 방향), y = 왼쪽.
자세는 뒤차축 (x, y, yaw), yaw는 차가 바라보는 방향(후진해도 차의 방향).
성공 조건(규정): 차체 네 꼭짓점이 칸 안(여유 0.05 m), 방향 ±20°, 0.05 m/s 이하로 3 s.
"""
from dataclasses import dataclass, field
import math

import numpy as np

FORWARD, REVERSE = 1, -1


@dataclass
class ParkingConfig:
    wheelbase: float = 3.0
    vehicle_length: float = 4.635
    vehicle_width: float = 1.892
    rear_overhang: float = 0.79
    max_steer_deg: float = 35.4
    radius_margin: float = 1.05     # 최소 회전반경 × 이 값 이상으로만 경로를 만듦 (조향 지연·추종 오차 여유)
    step: float = 0.1               # 경로 점 간격 (m)
    clearance: float = 0.3          # 차체 ~ 벽(모형·관측점) 최소 거리 (m)
    slot_margin: float = 0.2        # 계획한 최종 차체가 칸 안쪽으로 이만큼 들어와야 함 (규정 0.05 + 추종 오차)
    wall_band: float = 0.3          # 벽 선에서 이 거리 안의 관측점을 벽으로 봄 (m)
    min_gap: float = 3.0            # 이보다 긴 벽 선의 빈 구간만 입구로 봄 (m)
    gap_tolerance: float = 1.0      # 입구 중심이 (존 중심 ± slide 범위 ± 이 값) 밖이면 버림
    wall_default: float = 3.0       # 벽을 못 찾았을 때 도로 중심 ~ 벽 거리 (m)

    @property
    def min_radius(self):
        return self.wheelbase / math.tan(math.radians(self.max_steer_deg)) * self.radius_margin


@dataclass
class Mission:
    id: str
    type: str
    slot_center: np.ndarray       # 존 json의 (slide 0) 칸 중심
    slot_yaw: float
    slot_length: float
    slot_width: float
    zone_center: np.ndarray
    slide_axis: np.ndarray
    slide_range: float

    @classmethod
    def from_json(cls, m):
        slot = m['slot']
        zone = np.asarray(m['zone'], dtype=float)
        slide = m.get('slide') or {'axis': [0.0, 0.0], 'range': 0.0}
        axis = np.asarray(slide['axis'], dtype=float)
        norm = np.linalg.norm(axis)
        return cls(m['id'], m['type'], np.asarray(slot['center'], dtype=float), float(slot['yaw']),
                   float(slot['length']), float(slot['width']), zone.mean(axis=0),
                   axis / norm if norm > 0 else axis, float(slide['range']))


@dataclass
class Segment:
    gear: int                      # FORWARD / REVERSE
    poses: np.ndarray              # (N, 3) 뒤차축 x, y, yaw (차 방향)
    stop: bool = True              # 끝에서 정지(기어 전환·유지 지점)
    hold: float = 0.0              # 끝에서 이만큼(s) 서 있음 (주차 유지)
    name: str = ''


@dataclass
class Maneuver:
    segments: list
    slot_center: np.ndarray        # map
    slide: float
    start_x: float                 # 칸 좌표계에서 기존 경로를 떠나는 x / 돌아오는 x
    end_x: float
    info: dict = field(default_factory=dict)


class SlotFrame:
    """칸 좌표계 ↔ map."""

    def __init__(self, origin, yaw):
        self.o = np.asarray(origin, dtype=float)
        self.yaw = float(yaw)
        self.c, self.s = math.cos(yaw), math.sin(yaw)

    def to_local(self, xy):
        d = np.asarray(xy, dtype=float) - self.o
        return np.stack([d[..., 0] * self.c + d[..., 1] * self.s, -d[..., 0] * self.s + d[..., 1] * self.c], axis=-1)

    def to_map(self, xy):
        xy = np.asarray(xy, dtype=float)
        return np.stack([self.o[0] + xy[..., 0] * self.c - xy[..., 1] * self.s,
                         self.o[1] + xy[..., 0] * self.s + xy[..., 1] * self.c], axis=-1)

    def poses_to_map(self, poses):
        out = np.empty_like(poses)
        out[:, :2] = self.to_map(poses[:, :2])
        out[:, 2] = poses[:, 2] + self.yaw
        return out


# ---- 기본 도형 -------------------------------------------------------------------------------------------
def drive(pose, gear, length, kappa, step=0.1):
    """뒤차축 자세에서 gear 방향으로 length(≥0)만큼, 곡률 kappa(+ 왼쪽 조향)로 이동한 자세 목록 (첫 점 제외)."""
    n = max(1, int(math.ceil(length / step - 1e-9)))
    s = gear * length * np.arange(1, n + 1) / n
    x, y, th = pose
    if abs(kappa) < 1e-9:
        return np.column_stack((x + s * math.cos(th), y + s * math.sin(th), np.full(n, th)))
    t = th + s * kappa
    return np.column_stack((x + (np.sin(t) - math.sin(th)) / kappa, y - (np.cos(t) - math.cos(th)) / kappa, t))


class PathBuilder:
    def __init__(self, pose, step):
        self.poses = [np.asarray([pose], dtype=float)]
        self.step = step

    @property
    def end(self):
        return self.poses[-1][-1]

    def go(self, gear, length, kappa=0.0):
        if length > 1e-6:
            self.poses.append(drive(self.end, gear, length, kappa, self.step))
        return self

    def turn(self, gear, radius, angle, left):
        """반지름 radius, 각 angle(rad, ≥0)만큼. left = 왼쪽 조향."""
        return self.go(gear, radius * angle, (1.0 if left else -1.0) / radius)

    def result(self):
        return np.vstack(self.poses)


# ---- 칸 검출 ---------------------------------------------------------------------------------------------
def road_lateral(mission, route_xy):
    """칸 좌표계(slide 0)에서 기준 경로(도로 중심)의 y와 그 직선성 (존 근처 경로 점의 y 표준편차)."""
    frame = SlotFrame(mission.slot_center, mission.slot_yaw)
    local = frame.to_local(route_xy)
    reach = mission.slide_range + 12.0
    near = local[(np.abs(local[:, 0] - frame.to_local(mission.zone_center)[0]) < reach)
                 & (np.abs(local[:, 1]) < 30.0)]
    if len(near) < 5:
        return None, None
    return float(np.median(near[:, 1])), float(np.std(near[:, 1]))


def detect_slot(mission, points, y_road, cfg, opening_length=None):
    """관측점(map, (N,2))에서 칸 쪽 벽 선의 열린 구간을 찾아 slide를 추정.

    opening_length(m)를 주면 입구 중심 = 먼 쪽 끝 - 길이/2. 다가가며 보면 가까운 쪽 모서리 직전 벽은 비스듬히 보여
    점이 듬성해 0.2~0.8 m 일찍 끝나는 것으로 보이고(탐색 주행), 먼 쪽 끝은 0.1 m 안으로 맞기 때문.
    반환: dict(slide, slot_center(map), gap=(시작, 끝) 존 중심 기준 x, wall=벽 거리) 또는 None."""
    if points is None or len(points) == 0:
        return None
    frame = SlotFrame(mission.slot_center, mission.slot_yaw)
    local = frame.to_local(points)
    side = -1.0 if y_road > 0 else 1.0             # 칸이 도로의 오른쪽(-y)이면 -1
    offset = (local[:, 1] - y_road) * side          # 도로 중심에서 칸 쪽으로 잰 거리
    a0 = float(frame.to_local(mission.zone_center)[0])
    span = mission.slide_range + 14.0
    sel = (np.abs(local[:, 0] - a0) < span)
    # 벽 선: 칸 쪽 2.6~4.5 m의 관측점 거리 분포에서 가장 많은 곳. 칸 근처 직선 구간 점만
    # (전체 경로 시험에서 굽은 길 앞 벽 점 때문에 2.35 m로 잡혀 모든 후보가 탈락한 적 있음)
    near = np.abs(local[:, 0] - a0) < mission.slide_range + 8.0
    cand = offset[sel & near & (offset > 2.6) & (offset < 4.5)]
    wall = cfg.wall_default
    if len(cand) >= 20:
        hist, edges = np.histogram(cand, bins=np.arange(2.3, 4.55, 0.1))
        k = int(np.argmax(hist))
        on = cand[(cand >= edges[k] - 0.1) & (cand < edges[k + 1] + 0.1)]
        wall = float(np.median(on))
    xs = np.sort(local[sel & (np.abs(offset - wall) < cfg.wall_band), 0])
    if len(xs) < 10:
        return None
    gaps = np.diff(xs)
    best = None
    for k in np.flatnonzero(gaps > cfg.min_gap):
        lo, hi = xs[k], xs[k + 1]
        # 가까운 쪽 끝 3 m 안에 벽 점이 충분하고, 먼 쪽은 벽 점 2개(4 m 안) 또는 입구 안쪽 면 점 5개 (멀어서 듬성한 빈틈 배제)
        far_face = np.count_nonzero(sel & (offset > wall + 0.3) & (offset < wall + 1.5)
                                    & (local[:, 0] > hi - 1.0) & (local[:, 0] < hi + 0.3))
        if (np.count_nonzero((xs > lo - 3.0) & (xs <= lo)) < 4
                or (np.count_nonzero((xs >= hi) & (xs < hi + 4.0)) < 2 and far_face < 5)):
            continue
        # 먼 쪽 끝 다듬기: 입구 안쪽 면(도로 쪽을 향한 벽 면)과 모서리 점들 중 가까운 쪽 10 %
        face = local[sel & (offset > wall - 0.3) & (offset < wall + 1.5)
                     & (local[:, 0] > hi - 1.0) & (local[:, 0] < hi + 0.3), 0]
        if len(face) >= 5:
            hi = min(hi, float(np.percentile(face, 10)))
        centre = 0.5 * (lo + hi)
        # 가까운 쪽 모서리는 일찍 끝나 보일 뿐(입구가 길어 보임) 늦게 보이지는 않으므로 긴 쪽으로 넉넉히 허용
        if opening_length and -1.0 < (hi - lo) - opening_length < 3.0:
            centre = hi - 0.5 * opening_length
        if abs(centre - a0) > mission.slide_range + cfg.gap_tolerance:
            continue
        if best is None or abs(centre - a0) < abs(best[2] - a0):
            best = (lo, hi, centre)
    if best is None:
        return None
    lo, hi, centre = best
    shift = (centre - a0) * np.array([frame.c, frame.s])      # map에서 칸이 밀린 벡터
    slide = float(np.dot(shift, mission.slide_axis)) if np.linalg.norm(mission.slide_axis) > 0 else 0.0
    return {'slide': slide, 'slot_center': mission.slot_center + slide * mission.slide_axis,
            'gap': (float(lo - a0), float(hi - a0)), 'gap_length': float(hi - lo), 'wall': wall}


# ---- 차체 검사 ---------------------------------------------------------------------------------------------
def body_outline(cfg, inflate=0.0, step=0.15):
    front = cfg.vehicle_length - cfg.rear_overhang + inflate
    rear = -cfg.rear_overhang - inflate
    half = cfg.vehicle_width / 2 + inflate
    xs = np.linspace(rear, front, max(2, int(math.ceil((front - rear) / step)) + 1))
    ys = np.linspace(-half, half, max(2, int(math.ceil(2 * half / step)) + 1))
    return np.vstack([np.column_stack((xs, np.full_like(xs, -half))), np.column_stack((xs, np.full_like(xs, half))),
                      np.column_stack((np.full_like(ys, rear), ys)), np.column_stack((np.full_like(ys, front), ys))])


def body_corners(pose, cfg):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = []
    for lx in (-cfg.rear_overhang, cfg.vehicle_length - cfg.rear_overhang):
        for ly in (-cfg.vehicle_width / 2, cfg.vehicle_width / 2):
            out.append((x + lx * c - ly * s, y + lx * s + ly * c))
    return np.asarray(out)


def outline_points(poses, outline):
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    x = poses[:, 0][:, None] + outline[:, 0][None] * c - outline[:, 1][None] * s
    y = poses[:, 1][:, None] + outline[:, 0][None] * s + outline[:, 1][None] * c
    return x, y


def free_space_ok(poses, rects, cfg):
    """차체 외곽(여유 clearance 포함)의 모든 점이 자유 공간 직사각형들의 합집합 안에 있으면 True.
    rects: [(x0, x1, y0, y1)] 칸 좌표계. 반환 (ok, 처음 걸린 자세 번호)."""
    x, y = outline_points(poses, body_outline(cfg, cfg.clearance))
    inside = np.zeros(x.shape, dtype=bool)
    for x0, x1, y0, y1 in rects:
        inside |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    bad = ~inside.all(axis=1)
    return (not bad.any()), (int(np.argmax(bad)) if bad.any() else -1)


def points_clear(poses, points, cfg):
    """관측점(칸 좌표계)이 차체(+clearance) 안에 하나도 없으면 True. 반환 (ok, 걸린 자세 번호, 최소 여유)."""
    if points is None or len(points) == 0:
        return True, -1, math.inf
    front = cfg.vehicle_length - cfg.rear_overhang
    half = cfg.vehicle_width / 2
    best, first = math.inf, -1
    # 경로 근처 점만
    lo, hi = poses[:, :2].min(axis=0) - 6.0, poses[:, :2].max(axis=0) + 6.0
    pts = points[(points[:, 0] > lo[0]) & (points[:, 0] < hi[0]) & (points[:, 1] > lo[1]) & (points[:, 1] < hi[1])]
    if len(pts) == 0:
        return True, -1, math.inf
    for k in range(0, len(poses), 2):
        x, y, th = poses[k]
        c, s = math.cos(th), math.sin(th)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        bx, by = dx * c + dy * s, -dx * s + dy * c
        ex = np.maximum(np.maximum(-cfg.rear_overhang - bx, bx - front), 0.0)
        ey = np.maximum(np.abs(by) - half, 0.0)
        d = float(np.min(np.hypot(ex, ey)))
        if d < best:
            best = d
        if d < cfg.clearance and first < 0:
            first = k
    return first < 0, first, best


def in_slot(pose, length, width, margin, cfg):
    """칸 좌표계 최종 자세가 칸 안(여유 margin)이고 방향 ±20° 안이면 True."""
    corners = body_corners(pose, cfg)
    yaw = (pose[2] + math.pi) % (2 * math.pi) - math.pi
    return (abs(yaw) <= math.radians(20.0) and np.all(np.abs(corners[:, 0]) <= length / 2 - margin)
            and np.all(np.abs(corners[:, 1]) <= width / 2 - margin))


# ---- 동작 후보 ---------------------------------------------------------------------------------------------
def two_arc(builder, gear, radius, shift, first_left):
    """같은 반지름 두 원호 S자로 옆으로 shift(>0)만큼 이동하고 같은 방향으로 끝남. 진행 거리 = 2R sinθ."""
    th = math.acos(max(-1.0, 1.0 - shift / (2 * radius)))
    builder.turn(gear, radius, th, first_left).turn(gear, radius, th, not first_left)
    return 2 * radius * math.sin(th)


def parallel_candidates(y_road, cfg, p):
    """평행주차: (기존 경로) → [옆 이동] → 칸을 지나 정지 → 후진 S자(오른쪽·왼쪽 조향)로 칸 뒤쪽에 → 유지 →
    전진 S자로 도로 중심에 복귀. 칸이 도로 오른쪽(y<0)인 경우의 식이며, 왼쪽이면 y를 뒤집어 씁니다."""
    side = -1.0 if y_road > 0 else 1.0                   # 칸 쪽 (오른쪽 -1)
    yr = abs(y_road)                                      # 도로 중심까지 (칸 쪽을 -y로 정규화)
    back = cfg.vehicle_length / 2 - cfg.rear_overhang     # 뒤차축 → 차체 중심
    out = []
    for radius in p['radii']:
        for y_f in p['final_lateral']:                   # 칸 중심에서 도로 쪽(+)으로 뒤차축 옆 위치
            for x_back in p['final_back']:               # 차체 중심을 칸 중심에서 뒤로(-x) 이만큼
                for y_s in p['start_lateral']:            # 후진 시작점: 도로 중심에서 칸 쪽(-)/반대(+) m
                    x_f = -x_back - back
                    ys = yr + y_s
                    d = ys - y_f
                    if d <= 0 or d >= 4 * radius:
                        continue
                    th = math.acos(1 - d / (2 * radius))
                    x_s = x_f + 2 * radius * math.sin(th)
                    # 전진: 도로 중심 → (옆 이동 S자) → 시작점
                    shift = ys - yr
                    if abs(shift) > 1e-3:
                        sh_r = p['shift_radius']
                        th_s = math.acos(1 - abs(shift) / (2 * sh_r))
                        length = 2 * sh_r * math.sin(th_s)
                    else:
                        length = 0.0
                    x0 = x_s - length - p['straight_before_stop']
                    a = PathBuilder((x0, yr, 0.0), cfg.step)
                    if length:
                        two_arc(a, FORWARD, sh_r, abs(shift), shift > 0)
                    a.go(FORWARD, x_s - a.end[0])
                    fwd = a.result()
                    r = PathBuilder(tuple(fwd[-1]), cfg.step)
                    r.turn(REVERSE, radius, th, False).turn(REVERSE, radius, th, True)
                    rev = r.result()
                    # 복귀: 전진 S자 (왼쪽·오른쪽)
                    e = PathBuilder(tuple(rev[-1]), cfg.step)
                    ex_r = p['exit_radius']
                    dist = two_arc(e, FORWARD, ex_r, yr - rev[-1][1], True)
                    ext = e.result()
                    segs = [Segment(FORWARD, fwd, True, 0.0, 'approach'),
                            Segment(REVERSE, rev, True, p['hold_s'], 'reverse_in'),
                            Segment(FORWARD, ext, False, 0.0, 'exit')]
                    if side > 0:                         # 칸이 왼쪽: y 뒤집기
                        for sg in segs:
                            sg.poses = sg.poses * np.array([1.0, -1.0, -1.0])
                    cost = (len(fwd) + 1.6 * len(rev) + len(ext)) * cfg.step + p['cost_per_radius'] * radius
                    out.append((cost, segs, {'radius': radius, 'theta_deg': math.degrees(th), 'y_f': y_f,
                                             'x_back': x_back, 'y_s': y_s, 'exit_dist': dist}))
    out.sort(key=lambda c: c[0])
    return out


def rear_candidates(y_road, x_passage, cfg, p):
    """후면주차: 도로 → 왼쪽 90° 원호로 통로 진입 → 직진 → 오른쪽 90° 원호로 방 안에서 남쪽(칸 방향)을 봄 →
    정지 → 직진 후진으로 칸에 → 유지 → 전진 직진 → 오른쪽 90° → 통로 직진 → 왼쪽 90°로 도로 복귀.
    칸이 도로 왼쪽(y>0)인 경우의 식이며, 오른쪽이면 y를 뒤집습니다."""
    side = 1.0 if y_road < 0 else -1.0
    yr = -abs(y_road)
    back = cfg.vehicle_length / 2 - cfg.rear_overhang
    out = []
    for ra in p['entry_radii']:
        for rb in p['turn_radii']:
            for x_back in p['final_back']:
                for dx in p['passage_offsets']:          # 통로 중심에서 진입 선 옮김 (+ 앞쪽)
                    xp = x_passage + dx
                    y_f = p['final_lateral']
                    x_f = -back + x_back                  # x_back: 차체 중심을 칸 중심에서 +x(앞)로
                    a = PathBuilder((xp - ra - p['straight_before'], yr, 0.0), cfg.step)
                    a.go(FORWARD, p['straight_before'])
                    a.turn(FORWARD, ra, math.pi / 2, True)
                    a.go(FORWARD, (y_f - rb) - a.end[1])
                    a.turn(FORWARD, rb, math.pi / 2, False)
                    a.go(FORWARD, p['straight_after_turn'])
                    fwd = a.result()
                    if fwd[-1][0] <= x_f + 1.0:
                        continue
                    r = PathBuilder(tuple(fwd[-1]), cfg.step)
                    r.go(REVERSE, fwd[-1][0] - x_f)
                    rev = r.result()
                    e = PathBuilder(tuple(rev[-1]), cfg.step)
                    rc, rd = p['exit_turn_radius'], p['exit_radius']
                    xe = x_passage + p['exit_passage_offset']
                    e.go(FORWARD, (xe - rc) - e.end[0])
                    e.turn(FORWARD, rc, math.pi / 2, False)
                    e.go(FORWARD, e.end[1] - (yr + rd))
                    e.turn(FORWARD, rd, math.pi / 2, True)
                    ext = e.result()
                    if (xe - rc) < x_f:
                        continue
                    segs = [Segment(FORWARD, fwd, True, 0.0, 'approach'),
                            Segment(REVERSE, rev, True, p['hold_s'], 'reverse_in'),
                            Segment(FORWARD, ext, False, 0.0, 'exit')]
                    if side < 0:
                        for sg in segs:
                            sg.poses = sg.poses * np.array([1.0, -1.0, -1.0])
                    cost = (len(fwd) + 1.6 * len(rev) + len(ext)) * cfg.step
                    out.append((cost, segs, {'entry_radius': ra, 'turn_radius': rb, 'x_back': x_back,
                                             'passage_offset': dx, 'reverse_m': float(fwd[-1][0] - x_f)}))
    out.sort(key=lambda c: c[0])
    return out


def plan_maneuver(mission, detection, y_road, cfg, params, free_rects, points_map=None, log=None):
    """검출한 칸에 맞춰 후보를 만들고, 자유 공간 모형과 관측점에 대해 검사해 가장 싼 것을 고름.

    free_rects: 칸 좌표계 자유 공간 직사각형 (도로 띠는 y_road ± wall로 여기서 추가).
    반환 Maneuver(map 좌표) 또는 None."""
    slot = detection['slot_center']
    frame = SlotFrame(slot, mission.slot_yaw)
    wall = detection.get('wall', cfg.wall_default)
    if abs(wall - cfg.wall_default) > 0.25:          # 터무니없는 벽 거리는 기본값 (관측점 검사는 따로 함)
        wall = cfg.wall_default
    rects = [(-80.0, 80.0, y_road - wall, y_road + wall)] + [tuple(r) for r in free_rects]
    pts = frame.to_local(points_map) if points_map is not None and len(points_map) else None
    if mission.type == 'parallel_parking':
        cands = parallel_candidates(y_road, cfg, params)
    elif mission.type == 'rear_parking':
        x_passage = float(frame.to_local(mission.zone_center + detection['slide'] * mission.slide_axis)[0])
        cands = rear_candidates(y_road, x_passage, cfg, params)
    else:
        return None
    reasons = {}
    for cost, segs, info in cands:
        final = segs[1].poses[-1] if len(segs) > 1 else segs[0].poses[-1]
        if not in_slot(final, mission.slot_length, mission.slot_width, cfg.slot_margin, cfg):
            reasons['slot'] = reasons.get('slot', 0) + 1
            continue
        allp = np.vstack([s.poses for s in segs])
        ok, _ = free_space_ok(allp, rects, cfg)
        if not ok:
            reasons['free_space'] = reasons.get('free_space', 0) + 1
            continue
        ok, _, gap = points_clear(allp, pts, cfg)
        if not ok:
            reasons['points'] = reasons.get('points', 0) + 1
            continue
        out = []
        for s in segs:
            out.append(Segment(s.gear, frame.poses_to_map(s.poses), s.stop, s.hold, s.name))
        final = frame.poses_to_map(np.asarray([final]))[0]
        info['final_pose'] = [round(float(v), 3) for v in final]
        info = dict(info, cost=round(cost, 2), point_gap=round(gap, 2) if math.isfinite(gap) else None,
                    rejected=reasons)
        return Maneuver(out, slot, detection['slide'], float(segs[0].poses[0][0]), float(segs[-1].poses[-1][0]), info)
    if log:
        log(f'주차 후보 {len(cands)}개 모두 탈락: {reasons}')
    return None


@dataclass
class Spliced:
    points: np.ndarray       # (M, 2) 따라갈 점: 전진 동작은 앞바퀴 축 궤적, 후진 동작은 뒤차축, 기존 경로는 그대로
    yaws: np.ndarray         # (M,) 차 방향 (후진이면 진행 방향 + π)
    gears: np.ndarray        # (M,) +1 / -1
    segments: list           # [(시작 번호, 끝 번호, gear, stop, hold, 이름, 끝 뒤차축 자세(map))]
    first: int               # 동작의 첫 점 번호 (그 앞은 기존 경로)
    last: int                # 동작의 마지막 점 번호 (그 뒤는 기존 경로)


def splice_route(route_xy, maneuver, mission, lead, route_yaws=None, route_gears=None, step=0.25):
    """기존 경로 점(map, (N,2))에 주차 동작을 끼움. lead = 축간 거리 (전진 동작을 앞바퀴 축 궤적으로 바꿈).

    처음 구간 = 기존 경로 앞부분 + 첫 동작, 마지막 구간 = 마지막 동작 + 기존 경로 뒷부분.
    구간 경계(정지 지점)에서 점은 겹치지 않음: 전진 구간 끝은 앞바퀴 축, 후진 구간 처음은 뒤차축."""
    route_xy = np.asarray(route_xy, dtype=float)
    frame = SlotFrame(maneuver.slot_center, mission.slot_yaw)
    local = frame.to_local(route_xy)
    segs_in = maneuver.segments

    def track(s):
        poses = s.poses[::max(1, int(round(step / 0.1)))]
        if not np.allclose(poses[-1], s.poses[-1]):
            poses = np.vstack([poses, s.poses[-1]])
        xy = poses[:, :2].copy()
        if s.gear == FORWARD:
            xy += lead * np.column_stack((np.cos(poses[:, 2]), np.sin(poses[:, 2])))
        return xy, poses[:, 2]

    tracks = [track(s) for s in segs_in]
    x0 = frame.to_local(tracks[0][0][0])[0]
    x1 = frame.to_local(tracks[-1][0][-1])[0]
    # 칸 근처(옆 30 m 안)의 경로에서 x0 앞 마지막 점, x1 뒤 첫 점
    idx = np.flatnonzero(np.abs(local[:, 1]) < 30.0)
    before = idx[local[idx, 0] < x0 - 0.05]
    if len(before) == 0:
        return None
    i0 = int(before.max())
    after = idx[(local[idx, 0] > x1 + 0.05) & (idx > i0)]
    if len(after) == 0:
        return None
    i1 = int(after.min())

    def route_yaw(k):
        if route_yaws is not None:
            return float(route_yaws[k])
        j = max(0, min(k, len(route_xy) - 2))
        d = route_xy[j + 1] - route_xy[j]
        return math.atan2(d[1], d[0])

    pts = [route_xy[k] for k in range(i0 + 1)]
    yaws = [route_yaw(k) for k in range(i0 + 1)]
    gears = [int(route_gears[k]) if route_gears is not None else FORWARD for k in range(i0 + 1)]
    segs = []
    for n, (s, (xy, th)) in enumerate(zip(segs_in, tracks)):
        begin = 0 if n == 0 else len(pts)
        for p, t in zip(xy, th):
            if np.linalg.norm(p - pts[-1]) < 1e-3:
                continue
            pts.append(p); yaws.append(float(t)); gears.append(s.gear)
        segs.append([begin, len(pts) - 1, s.gear, s.stop, s.hold, s.name, s.poses[-1].copy()])
    last = len(pts) - 1
    for k in range(i1, len(route_xy)):
        pts.append(route_xy[k]); yaws.append(route_yaw(k))
        gears.append(int(route_gears[k]) if route_gears is not None else FORWARD)
    segs[-1][1] = len(pts) - 1
    return Spliced(np.asarray(pts), np.asarray(yaws), np.asarray(gears), [tuple(s) for s in segs], i0 + 1, last)


def parallel_exit(pose, y_road, radius, step=0.1):
    """평행주차 칸에서 실제로 선 자세(칸 좌표계)로부터 전진 S자(칸 반대쪽 조향 → 되돌림)로 도로 중심(y_road,
    방향 0)에 합류하는 자세 목록. 첫 원호는 방향 ψ0 → φ, 둘째는 φ → 0:
        옆 이동 = R(cos ψ0 + 1 - 2 cos φ),  cos φ = (cos ψ0 + 1 - 옆 이동 / R) / 2."""
    side = -1.0 if y_road > 0 else 1.0                   # 칸 쪽 (오른쪽 -1). 칸이 오른쪽인 식으로 정규화
    x0, y0, th0 = pose[0], pose[1] * -side, pose[2] * -side
    shift = abs(y_road) - y0
    c = (math.cos(th0) + 1.0 - shift / radius) / 2.0
    if not -1.0 <= c <= 1.0 or shift <= 0:
        return None
    phi = math.acos(c)
    if phi < th0:
        return None
    b = PathBuilder((x0, y0, th0), step)
    b.turn(FORWARD, radius, phi - th0, True).turn(FORWARD, radius, phi, False)
    out = b.result()
    return out * np.array([1.0, -side, -side])
