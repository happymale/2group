#!/usr/bin/env python3
"""HEVEN 2026 주차 기하 계산기 (독립 실행, 표준 라이브러리만 사용, ROS 불필요).

좌표계 (평면 주차 장면을 정규화한 로컬 좌표):
  - 도로는 +x 방향으로 진행, 도로 영역 y in [0, road_w].
  - 주차 칸은 도로 오른쪽(y<0)에 붙어 있음: x in [x0, x0+slot_len], y in [-slot_w, 0]
    (평행주차) 또는 x in [x0, x0+slot_w], y in [-slot_len, 0] (후진 직각주차).
  - 차량 자세는 뒤차축 중심 (x, y, yaw). 차체 [-rear_oh, L-rear_oh] x [-W/2, W/2].
실제 적용: 칸의 입구 모서리(도로쪽 두 꼭짓점)를 map 좌표에서 찾은 뒤
  T = [칸 입구 방향 회전 + 평행이동] 으로 이 로컬 좌표와 map을 오갑니다.

실행:  python parking_geometry.py            (기본 차량·칸 치수로 결과 출력)
       python parking_geometry.py --road 6.0  (주차 구역 도로 폭이 더 넓다면)
"""
import argparse
import math


def arange(a, b, step):
    n = int(math.floor((b - a) / step + 1e-9))
    return [a + i * step for i in range(max(0, n + 1)) if a + i * step < b + 1e-9]

# ---- 차량 ----
L_BODY, W_BODY, REAR_OH, WB = 4.635, 1.892, 0.79, 3.0
MAX_STEER = math.radians(35.4)
R_MIN = WB / math.tan(MAX_STEER)          # 뒤차축 최소 회전반경 ≈ 4.22 m
FRONT = L_BODY - REAR_OH                  # 뒤차축→앞범퍼 3.845 m


def body_outline(step=0.1):
    """뒤차축 좌표계의 차체 외곽 샘플점 (N,2)."""
    xs = arange(-REAR_OH, FRONT, step) + [FRONT]
    ys = arange(-W_BODY / 2, W_BODY / 2, step) + [W_BODY / 2]
    return ([(x, -W_BODY / 2) for x in xs] + [(x, W_BODY / 2) for x in xs]
            + [(-REAR_OH, y) for y in ys] + [(FRONT, y) for y in ys])


OUTLINE = body_outline()


def move(pose, ds, kappa):
    """뒤차축 자세를 부호 있는 호 길이 ds(후진이면 음수), 곡률 kappa로 정확히 이동."""
    x, y, th = pose
    if abs(kappa) < 1e-9:
        return (x + ds * math.cos(th), y + ds * math.sin(th), th)
    dth = ds * kappa
    r = 1.0 / kappa
    return (x + r * (math.sin(th + dth) - math.sin(th)),
            y - r * (math.cos(th + dth) - math.cos(th)), th + dth)


def sample_segment(pose, length, kappa, gear, step=0.1):
    """세그먼트를 따라 자세 목록. gear=+1 전진, -1 후진. length>=0."""
    n = max(1, int(math.ceil(length / step)))
    return [move(pose, gear * length * i / n, kappa) for i in range(n + 1)]


def world_points(pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in OUTLINE]


class Scene:
    """허용 영역 = 도로 띠 ∪ 칸 기둥(칸 x범위로 도로까지 이어진 영역). margin만큼 안쪽으로 축소."""

    def __init__(self, road_w, slot_x0, slot_x1, slot_depth, margin):
        self.road_w, self.x0, self.x1, self.depth, self.m = road_w, slot_x0, slot_x1, slot_depth, margin

    def clearance_ok(self, pts):
        m = self.m
        for x, y in pts:
            if y > self.road_w - m:
                return False
            if y >= m:
                continue
            if not (self.x0 + m <= x <= self.x1 - m and y >= -self.depth + m):
                return False
        return True

    def path_ok(self, poses):
        return all(self.clearance_ok(world_points(p)) for p in poses)


# ------------------------------------------------------------------ 평행 주차
def parallel_plan(road_w=4.392, slot_len=7.4, slot_w=3.6, margin=0.15, start_y=None):
    """후진 2호(S자) 평행주차. 오른쪽 칸 x in [0, slot_len].

    최종 자세 F=(xf, yf=-slot_w/2, 0). 1호: 시작점 S=(xs, ys, 0)에서 오른쪽 조향 후진,
    2호: 왼쪽 조향 후진. 같은 반경 R이면
        d  = ys - yf,  cos(theta) = 1 - d/(2R),  Δx = xs - xf = 2R sin(theta),
    각 호 길이 R*theta. 전환점은 두 원 중심의 중점 (yaw가 theta가 되는 순간).
    """
    yf = -slot_w / 2
    ys_list = [start_y] if start_y is not None else [road_w / 2 - 0.1 * i for i in range(int((road_w / 2 - 0.9) / 0.1) + 1)] + \
        arange(road_w / 2 + 0.1, road_w - 0.9, 0.1)
    best = None
    for R in arange(R_MIN, R_MIN + 3.01, 0.25):
        for xf in arange(REAR_OH + margin, slot_len - FRONT - margin + 1e-9, 0.05):
            for ys in ys_list:
                d = ys - yf
                if d <= 0 or d >= 4 * R:
                    continue
                th = math.acos(1 - d / (2 * R))
                xs = xf + 2 * R * math.sin(th)
                scene = Scene(road_w, 0.0, slot_len, slot_w, margin)
                S = (xs, ys, 0.0)
                seg1 = sample_segment(S, R * th, -1.0 / R, -1)   # 후진 + 오른쪽 조향(곡률 음수)
                mid = seg1[-1]
                seg2 = sample_segment(mid, R * th, 1.0 / R, -1)   # 후진 + 왼쪽 조향
                if not scene.path_ok(seg1 + seg2):
                    continue
                cost = 2 * R * th
                if best is None or cost < best['length']:
                    best = dict(R=R, theta_deg=math.degrees(th), xs=xs, ys=ys, xf=xf, yf=yf,
                                switch=mid, end=seg2[-1], length=cost,
                                steer_cmd=math.atan(WB / R) / MAX_STEER)
    return best


# ------------------------------------------------------------------ 후진 직각 주차
def perpendicular_plan(road_w=4.392, slot_len=5.4, slot_w=3.6, margin=0.15):
    """3동작 후진 직각주차. 칸은 오른쪽, x in [0, slot_w], y in [-slot_len, 0].

    실제 순서: (A) 칸을 지나치며 왼쪽 조향 전진 호 alpha (머리를 도로 바깥쪽으로 돌려 놓음)
               (B) 오른쪽 최대조향 후진 호 phi  (꼬리를 칸으로)
               (C) 직진 후진 s0 (칸 안으로)
    계산은 최종 자세(뒤차축이 칸 깊숙이, yaw=+90°)에서 시간 역순으로 적분합니다.
    yaw 조건: 90° - phi - alpha = 0  → alpha = 90° - phi.
    """
    best = None
    xc = slot_w / 2
    scene = Scene(road_w, 0.0, slot_w, slot_len, margin)
    yf = -slot_len + REAR_OH + margin                     # 뒤범퍼가 칸 끝에서 margin
    for R in arange(R_MIN, R_MIN + 2.01, 0.25):
        for phi_deg in arange(30, 90.1, 2.5):
            phi = math.radians(phi_deg)
            alpha = math.pi / 2 - phi
            for s0 in arange(0.0, slot_len + 2.0, 0.1):
                F = (xc, yf, math.pi / 2)
                c_seg = sample_segment(F, s0, 0.0, +1)                      # C 역순: 전진 직진
                b_start = c_seg[-1]
                b_seg = sample_segment(b_start, R * phi, -1.0 / R, +1)      # B 역순: 오른쪽 조향 전진
                a_start = b_seg[-1]
                a_seg = sample_segment(a_start, R * alpha, 1.0 / R, -1)     # A 역순: 왼쪽 조향 후진
                end = a_seg[-1]
                if not (1.0 <= end[1] <= road_w - 1.0):                     # 도로 위 진입 자세여야 함
                    continue
                if not scene.path_ok(c_seg + b_seg + a_seg):
                    continue
                cost = s0 + R * phi + R * alpha
                if best is None or cost < best['length']:
                    best = dict(R=R, phi_deg=phi_deg, alpha_deg=math.degrees(alpha), s0=s0,
                                approach_pose=end, cusp_pose=a_start, straight_start=b_start,
                                final=F, length=cost)
    return best


def multi_cusp_search(final, scene, road_w, prim_len=0.5, cusp_cost=2.0, steer_change_cost=0.6,
                      max_expand=150000):
    """작은 hybrid A* (시간 역순): 최종 자세에서 출발해 '도로 위, yaw≈0' 자세에 도달하는
    최단(길이 + 전환 벌점) 전후진 조합을 찾습니다. 동작 = {전진,후진} x {좌최대, 직진, 우최대}.
    반환되는 목록은 '실제 주행 순서'의 (gear, steer_cmd, length) 세그먼트입니다.
    고정 장면이므로 오프라인(시작 전) 또는 칸 인식 직후 1회만 돌리면 됩니다.
    """
    import heapq
    k = 1.0 / R_MIN
    prims = [(g, kap) for g in (+1, -1) for kap in (k, 0.0, -k)]

    def key(p):
        return (round(p[0] / 0.2), round(p[1] / 0.2), round(math.degrees(p[2]) / 5.0) % 72)

    def h(p):
        yaw = (p[2] + math.pi) % (2 * math.pi) - math.pi
        return abs(yaw) * R_MIN

    heap = [(h(final), 0.0, 0, final, None, ())]
    seen = {}
    n = 0
    while heap and n < max_expand:
        f, g, _, p, last_gear, hist = heapq.heappop(heap)
        yaw = (p[2] + math.pi) % (2 * math.pi) - math.pi
        if abs(yaw) < math.radians(3) and 1.0 <= p[1] <= road_w - 1.0:
            # 시간 역순 기록 → 실제 순서: 뒤집고 gear 부호 반전
            segs = []
            for gear, kap in reversed(hist):
                steer = 0.0 if kap == 0 else math.copysign(1.0, kap)
                if segs and segs[-1][0] == -gear and segs[-1][1] == steer:
                    segs[-1] = (segs[-1][0], steer, segs[-1][2] + prim_len)
                else:
                    segs.append((-gear, steer, prim_len))
            return dict(approach_pose=p, length=g, segments=segs, expanded=n)
        kk = key(p)
        if seen.get(kk, 1e9) <= g:
            continue
        seen[kk] = g
        n += 1
        for gear, kap in prims:
            poses = sample_segment(p, prim_len, kap, gear, step=0.25)
            if not scene.path_ok(poses[1:]):
                continue
            q = poses[-1]
            g2 = g + prim_len + (cusp_cost if last_gear is not None and gear != last_gear else 0.0)
            if hist and hist[-1][1] != kap:
                g2 += steer_change_cost        # 조향 반전마다 조향 지연(0.25 s)만큼 손해 → 벌점
            heapq.heappush(heap, (g2 + h(q), g2, n * 7 + len(heap), q, gear, hist + ((gear, kap),)))
    return None


def perpendicular_fallback(road_w=4.392, slot_len=5.4, slot_w=3.6, margin=0.15):
    scene = Scene(road_w, 0.0, slot_w, slot_len, margin)
    final = (slot_w / 2, -slot_len + REAR_OH + margin, math.pi / 2)
    return multi_cusp_search(final, scene, road_w)


# ------------------------------------------------------------------ 실제 course.json 배치
class RectScene:
    """허용 영역 = 축 정렬 직사각형들의 합집합 (각각 margin만큼 축소)."""

    def __init__(self, rects, margin):
        self.rects = [(x0 + margin, x1 - margin, y0 + margin, y1 - margin) for x0, x1, y0, y1 in rects]

    def clearance_ok(self, pts):
        for x, y in pts:
            if not any(a <= x <= b and c <= y <= d for a, b, c, d in self.rects):
                return False
        return True

    def path_ok(self, poses):
        return all(self.clearance_ok(world_points(p)) for p in poses)


def hybrid_search(final, scene, goal_fn, prim_len=0.5, cusp_cost=2.0, steer_change_cost=0.6,
                  max_expand=200000, h=lambda p: 0.0):
    """multi_cusp_search의 일반형: goal_fn(pose)->bool 인 진입 자세까지 시간 역순 탐색."""
    import heapq
    k = 1.0 / R_MIN
    prims = [(g, kap) for g in (+1, -1) for kap in (k, 0.0, -k)]

    def key(p):
        return (round(p[0] / 0.25), round(p[1] / 0.25), round(math.degrees(p[2]) / 5.0) % 72)

    heap = [(0.0, 0.0, 0, final, None, ())]
    seen = {}
    n = 0
    while heap and n < max_expand:
        _, g, _, p, last_gear, hist = heapq.heappop(heap)
        if goal_fn(p):
            segs = []
            for gear, kap in reversed(hist):
                steer = 0.0 if kap == 0 else math.copysign(1.0, kap)
                if segs and segs[-1][0] == -gear and segs[-1][1] == steer:
                    segs[-1] = (segs[-1][0], steer, segs[-1][2] + prim_len)
                else:
                    segs.append((-gear, steer, prim_len))
            return dict(approach_pose=p, cost=g, segments=segs, expanded=n)
        kk = key(p)
        if seen.get(kk, 1e9) <= g:
            continue
        seen[kk] = g
        n += 1
        for gear, kap in prims:
            poses = sample_segment(p, prim_len, kap, gear, step=0.25)
            if not scene.path_ok(poses[1:]):
                continue
            g2 = g + prim_len + (cusp_cost if last_gear is not None and gear != last_gear else 0.0)
            if hist and hist[-1][1] != kap:
                g2 += steer_change_cost
            heapq.heappush(heap, (g2 + h(poses[-1]), g2, n * 7 + len(heap), poses[-1], gear, hist + ((gear, kap),)))
    return None


# 두 주차 미션 모두 도로는 x_map = -13.1647 을 따라 남쪽(-y)으로 진행.
# 로컬: x_loc = Y0 - y_map (진행 방향 = +x), y_loc = x_map + 13.1647 (왼쪽 = 동쪽 = +y).
ROAD_X, ROAD_HALF = -13.1647, 4.392 / 2


def to_local(xm, ym, y0):
    return (y0 - ym, xm - ROAD_X)


def course_scene(kind, slide=0.0, margin=0.15, strict=False):
    """kind='ppark' | 'rpark'. slide: seed에 따른 칸 이동(+면 남쪽, m). strict=True면 존 전체가 아니라
    도로 + 칸 + 칸과 도로 사이 띠만 허용 (칸 앞뒤에 차량·벽이 있다고 가정한 보수적 경우)."""
    if kind == 'ppark':
        y0 = 128.2754                                     # 존 북쪽 끝 = x_loc 0
        zx0, zx1 = 0.0, 15.0
        zy0, zy1 = -20.6647 - ROAD_X, -15.6647 - ROAD_X     # [-7.5, -2.5]
        cx, cy = to_local(-18.1147, 120.7754, y0)
        cx += slide
        slot = (cx - 3.7, cx + 3.7, cy - 1.8, cy + 1.8)
        final_yaw = 0.0                                   # 칸 yaw -90° = 진행 방향(남)
    else:
        y0 = 89.2754
        zx0, zx1 = 0.0, 26.0
        zy0, zy1 = -10.6647 - ROAD_X, 3.3353 - ROAD_X       # [2.5, 16.5]
        cx, cy = to_local(0.3353, 82.9754, y0)
        cx += slide
        slot = (cx - 2.7, cx + 2.7, cy - 1.8, cy + 1.8)    # 길이 5.4가 x(남북) 방향
        final_yaw = 0.0
    road = (-30.0, 60.0, -ROAD_HALF, ROAD_HALF)
    if strict:
        bridge = (slot[0], slot[1], min(slot[2], -ROAD_HALF), max(slot[3], ROAD_HALF))
        rects = [road, slot, bridge]
    else:
        # 도로 가장자리와 존 사이 0.3 m 띠도 주행 가능하다고 가정
        rects = [road, (zx0, zx1, min(zy0, -ROAD_HALF), max(zy1, ROAD_HALF))]
    return RectScene(rects, margin), slot, final_yaw


def _two_arc(start, R, theta, first_kappa_sign, gear):
    """같은 반경·같은 각도의 S자 두 호. 반환: (전체 자세 목록, 전환 자세)."""
    a = sample_segment(start, R * theta, first_kappa_sign / R, gear)
    b = sample_segment(a[-1], R * theta, -first_kappa_sign / R, gear)
    return a + b[1:], a[-1]


def course_plan(kind, slide=0.0, margin=0.15, strict=False):
    """실제 배치에서 해석해(analytic) 후보를 만들고 충돌 검사 후 가장 짧은 것을 고릅니다.
    ppark: (A) 후진 S자 2호  (B) 전진 S자 진입(규정상 전진 진입이 허용될 때만)
    rpark: (C) 전진 S자로 칸 축(남쪽 너머)으로 이동 → 정지 → 직진 후진
    최종 자세는 칸 중앙(차체 중심 = 칸 중심), yaw = 진행 방향(남, 로컬 0)으로 가정."""
    scene, slot, yaw = course_scene(kind, slide, margin, strict)
    cx, cy = (slot[0] + slot[1]) / 2, (slot[2] + slot[3]) / 2
    back = L_BODY / 2 - REAR_OH
    out = []
    for dx in arange(-0.4, 0.41, 0.1):                   # 칸 안 앞뒤 위치 미세 조정
        xf = cx - back + dx
        final = (xf, cy, 0.0)
        if not scene.clearance_ok(world_points(final)):
            continue
        for R in arange(R_MIN, R_MIN + 6.0, 0.25):
            for ys in arange(-0.6, 0.61, 0.3):           # 도로 중심선 근처에서 시작
                d = ys - cy
                if kind == 'ppark':
                    if d >= 4 * R:
                        continue
                    th = math.acos(1 - d / (2 * R))
                    # (A) 후진: 시작 S는 최종보다 Δx 앞(+x). 1호 오른쪽조향(곡률 -), 2호 왼쪽
                    S = (xf + 2 * R * math.sin(th), ys, 0.0)
                    poses, mid = _two_arc(S, R, th, -1.0, -1)
                    if scene.path_ok(poses):
                        out.append(('A 후진S', R * 2 * th, R, th, S, mid, final, None))
                    # (B) 전진: 시작은 최종보다 Δx 뒤(-x). 1호 오른쪽(곡률 -), 2호 왼쪽
                    S2 = (xf - 2 * R * math.sin(th), ys, 0.0)
                    poses, mid = _two_arc(S2, R, th, -1.0, +1)
                    if scene.path_ok(poses):
                        out.append(('B 전진S', R * 2 * th, R, th, S2, mid, final, None))
                else:
                    dd = cy - ys                           # 왼쪽으로 이동할 거리 (양수)
                    if dd >= 2 * R:                        # θ ≤ 90° 로 제한
                        continue
                    th = math.acos(1 - dd / (2 * R))
                    for over in arange(0.5, 8.01, 0.5):    # 칸을 지나친 거리 → 이후 직진 후진
                        xa = xf + over
                        S = (xa - 2 * R * math.sin(th), ys, 0.0)
                        poses, mid = _two_arc(S, R, th, +1.0, +1)
                        rev = sample_segment(poses[-1], over, 0.0, -1)
                        if scene.path_ok(poses + rev):
                            out.append(('C 전진S+직진후진', R * 2 * th + 2 * over, R, th, S, mid, final, over))
                            break
    out.sort(key=lambda c: c[1])
    return out


def fmt(p):
    return '(%.2f, %.2f, %.1f°)' % (p[0], p[1], math.degrees(p[2]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--road', type=float, default=4.392, help='주차 구역 앞 도로(허용 영역) 폭 m')
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--course', action='store_true', help='course.json 실제 배치로 탐색')
    a = ap.parse_args()
    if a.course:
        for kind in ('ppark', 'rpark'):
            for strict in (False, True):
                for slide in (-3.0, 0.0, 3.0):
                    res = course_plan(kind, slide, a.margin, strict)
                    tag = '%s strict=%s slide=%+.0f' % (kind, strict, slide)
                    if not res:
                        print(tag, ': 해석해 후보 없음 → hybrid_search 사용')
                        continue
                    seen = set()
                    for name, length, R, th, S, mid, final, over in res:
                        if name in seen:
                            continue
                        seen.add(name)
                        print('%s %s: 길이 %.1f m(후진 포함 가중), R=%.2f, 각 %.1f°, 시작 %s, 전환 %s, 최종 %s%s' % (
                            tag, name, length, R, math.degrees(th), fmt(S), fmt(mid), fmt(final),
                            '' if over is None else ', 지나친 뒤 직진후진 %.1f m' % over))
        return
    print('R_min(뒤차축) = %.3f m, 바깥 앞모서리 반경 = %.3f m, 안쪽 측면 반경 = %.3f m' % (
        R_MIN, math.hypot(R_MIN + W_BODY / 2, FRONT), R_MIN - W_BODY / 2))
    print('\n[평행주차 7.4 x 3.6, 도로폭 %.3f, 여유 %.2f]' % (a.road, a.margin))
    p = parallel_plan(a.road, margin=a.margin)
    if p:
        print(' R=%.2f theta=%.1f°  시작 S=(%.2f, %.2f)  전환점 %s  최종 %s  총 후진 %.2f m' % (
            p['R'], p['theta_deg'], p['xs'], p['ys'], fmt(p['switch']), fmt(p['end']), p['length']))
        print(' 조향 명령: 1호 -%.2f, 2호 +%.2f (R=R_min이면 ±1.0)' % (p['steer_cmd'], p['steer_cmd']))
    else:
        print(' 2호 단일 후진으로는 불가 → 여유(margin) 축소, 도로폭 확인, 또는 전진 보정 1회 추가')
    print('\n[후진 직각주차 5.4 x 3.6, 도로폭 %.3f, 여유 %.2f]' % (a.road, a.margin))
    q = perpendicular_plan(a.road, margin=a.margin)
    if q:
        print(' R=%.2f  A:전진 왼쪽조향 %.1f°  B:후진 오른쪽조향 %.1f°  C:직진후진 %.2f m  총 %.2f m' % (
            q['R'], q['alpha_deg'], q['phi_deg'], q['s0'], q['length']))
        print(' 진입자세 %s → 전환(정지·기어R) %s → 직진시작 %s → 최종 %s' % (
            fmt(q['approach_pose']), fmt(q['cusp_pose']), fmt(q['straight_start']), fmt(q['final'])))
    else:
        print(' 3동작으로 불가 → 전후진 반복 탐색(hybrid A*)으로 대체')
        r = perpendicular_fallback(a.road, margin=a.margin)
        if r:
            print(' 진입자세 %s, 총 %.2f m(벌점 포함), 확장 %d' % (fmt(r['approach_pose']), r['length'], r['expanded']))
            for gear, steer, length in r['segments']:
                print('   %s  steer=%+.0f  %.1f m' % ('전진' if gear > 0 else '후진', steer, length))
        else:
            print(' 탐색 실패: 이 여유/도로폭에서는 해 없음')


if __name__ == '__main__':
    main()
