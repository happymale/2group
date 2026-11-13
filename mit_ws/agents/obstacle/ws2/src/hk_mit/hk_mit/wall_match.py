"""LiDAR 통로 벽으로 횡위치·방향을 재는 의사측정 (localization.py EKF에 넣음).

research/localization/wall_lateral_update.py 스케치를 옮겨 실측에 맞춘 것입니다.
  * 지도: route.csv 중심선 ± 반폭(= corridor_width / 2). 실측(LiDAR + ground truth, 소음 켬)으로 확인한 벽 위치는
    통로 구간에서 경로 ±2.20 m (점 10~90 % 범위 ±0.03 m = LiDAR 잡음)입니다. 장애물 구간(±3.75), 주차 구간·회전교차로
    (±3.0, 한쪽 열림)와 그 앞뒤 폭이 바뀌는 곳(약 8 m)은 모형이 맞지 않아 벽을 쓰지 않습니다(미션 구간 ± wall_zone_margin).
  * 스캔 점을 스캔 시각의 추정 자세로 지도에 올려 중심선에 투영(d = 왼쪽 + 횡거리) → 왼쪽 벽 점은 d ≈ +w, 오른쪽 벽은
    d ≈ -w. 잔차를 자세 보정 (δx, δy, δyaw)로 설명하는 가중 최소제곱(측별 최빈값 → 좁은 게이트 → Huber IRLS).
  * 직선에서는 진행 방향 위치가 관측되지 않으므로 정보행렬 고유분해로 관측되는 방향만 측정으로 냅니다.
  * 출력 (z, H, R): 상태 [x, y, yaw]에 대한 혁신 z = H δ, 측정 공분산 R.
의존성: numpy만 (ROS 없이 시험 가능).
"""
import json
import math

import numpy as np

LIDAR_X = 1.15          # 뒤차축 기준 LiDAR 위치 (config/sensor_mounts.yaml)
NO_WALL_TYPES = ('obstacle1', 'obstacle3', 'parallel_parking', 'rear_parking')


class CorridorMap:
    """route 중심선(0.5 m 간격)과 측별 기대 벽 반폭. nan이면 그쪽 벽을 쓰지 않음."""

    def __init__(self, route_xy, half_width=2.196, kappa_w_max=0.6):
        p = np.asarray(route_xy, float)
        self.p = p
        seg = np.diff(p, axis=0)
        self.s = np.r_[0.0, np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))]
        t = np.vstack([seg[:1], seg[:-1] + seg[1:], seg[-1:]])       # 꼭짓점 접선: 앞뒤 선분 평균
        t /= np.hypot(t[:, 0], t[:, 1])[:, None]
        self.t = t
        self.n = np.c_[-t[:, 1], t[:, 0]]                              # 왼쪽 법선
        heading = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
        self.kappa = np.gradient(heading, self.s)
        self.wL = np.full(len(p), float(half_width))
        self.wR = np.full(len(p), float(half_width))
        # 안쪽 벽 반경(R - w)이 작아 투영이 모호한 곳(회전교차로 진입·진출 꺾임)은 그쪽 벽을 끔
        k = self.kappa
        self.wL[k * half_width > kappa_w_max] = np.nan
        self.wR[-k * half_width > kappa_w_max] = np.nan

    def disable(self, s0, s1):
        """경로 거리 s0~s1 구간의 벽을 끔."""
        off = (self.s >= s0) & (self.s <= s1)
        self.wL[off] = np.nan
        self.wR[off] = np.nan

    def nearest_index(self, x, y, hint=None, window=200):
        i0, i1 = (0, len(self.p)) if hint is None else (max(0, hint - window), min(len(self.p), hint + window))
        d2 = (self.p[i0:i1, 0] - x) ** 2 + (self.p[i0:i1, 1] - y) ** 2
        return i0 + int(np.argmin(d2))

    def station_range(self, polygon_or_center, radius=None, pad=3.0):
        """미션 구간(다각형의 외접 사각형을 pad만큼 넓힘) 또는 원(중심, 반지름) 안에 드는 경로 거리 (최소, 최대).
        주차 구간은 도로 옆에 있어 경로가 구간 밖 2.5 m를 지나므로 pad가 필요. 없으면 None."""
        if radius is None:
            zone = np.asarray(polygon_or_center, float)
            lo, hi = zone.min(axis=0) - pad, zone.max(axis=0) + pad
            inside = np.all((self.p >= lo) & (self.p <= hi), axis=1)
        else:
            inside = np.hypot(*(self.p - np.asarray(polygon_or_center, float)).T) < radius
        return (float(self.s[inside].min()), float(self.s[inside].max())) if inside.any() else None

    def project(self, pts, i0, i1, j_prev=None):
        """pts (P,2)를 꼭짓점 i0..i1-1 폴리라인에 투영 → s, d(왼쪽 +), 법선 (P,2), 가장 가까운 꼭짓점 번호."""
        px, py = pts[:, 0], pts[:, 1]
        if j_prev is None:
            P = self.p[i0:i1]
            d2 = (px[:, None] - P[None, :, 0]) ** 2 + (py[:, None] - P[None, :, 1]) ** 2
            j = i0 + np.argmin(d2, axis=1)
        else:                                       # 반복 중에는 보정이 작으므로 이전 꼭짓점 ±2만 봄
            cand = np.clip(j_prev[:, None] + np.arange(-2, 3)[None, :], 0, len(self.p) - 1)
            d2 = (px[:, None] - self.p[cand, 0]) ** 2 + (py[:, None] - self.p[cand, 1]) ** 2
            j = cand[np.arange(len(pts)), np.argmin(d2, axis=1)]
        best = np.full(len(pts), np.inf)
        out_s, out_d, out_n = np.zeros(len(pts)), np.zeros(len(pts)), np.zeros((len(pts), 2))
        for off in (-1, 0):                        # (j-1, j), (j, j+1) 두 선분 중 가까운 쪽
            a = np.clip(j + off, 0, len(self.p) - 2)
            A, ab = self.p[a], self.p[a + 1] - self.p[a]
            L2 = (ab ** 2).sum(1)
            u = np.clip(((pts - A) * ab).sum(1) / L2, 0.0, 1.0)
            foot = A + u[:, None] * ab
            dist = np.hypot(*(pts - foot).T)
            tv = ab / np.sqrt(L2)[:, None]
            nv = np.stack((-tv[:, 1], tv[:, 0]), axis=1)
            better = dist < best
            best = np.where(better, dist, best)
            out_s = np.where(better, self.s[a] + u * np.sqrt(L2), out_s)
            out_d = np.where(better, ((pts - foot) * nv).sum(1), out_d)
            out_n = np.where(better[:, None], nv, out_n)
        return out_s, out_d, out_n, j


def corridor_map(route_xy, course_json=None, corridor_width=4.392, margin=18.0):
    """경로 + 코스 파일 미션으로 지도를 만듦. 장애물·주차 구간과 회전교차로(링 반지름 + 12 m 원)는 앞뒤 margin(m)까지 벽 끔
    (실측: 폭이 바뀌는 구간이 미션 구간 바깥 8~15 m까지 이어짐)."""
    cmap = CorridorMap(route_xy, corridor_width / 2.0)
    if course_json:
        try:
            with open(course_json, encoding='utf-8') as f:
                missions = json.load(f)['course'].get('missions', [])
        except (OSError, ValueError, KeyError):
            missions = []
        for m in missions:
            rng = None
            if m.get('type') in NO_WALL_TYPES and m.get('zone'):
                rng = cmap.station_range(m['zone'])
            elif m.get('type') == 'roundabout' and m.get('center'):
                rng = cmap.station_range(m['center'], float(m.get('npc_radius', 11.0)) + 12.0)
            if rng:
                cmap.disable(rng[0] - margin, rng[1] + margin)
    return cmap


def scan_points(ranges, angle_min, angle_increment, range_min=0.5, max_range=30.0, stride=2):
    """LaserScan 거리 → 뒤차축 좌표 점 (P,2). 시뮬레이터 스캔은 한 순간 값(time_increment 0)이라 역왜곡은 하지 않음."""
    r = np.asarray(ranges, float)
    a = angle_min + angle_increment * np.arange(len(r))
    r, a = r[::stride], a[::stride]
    ok = np.isfinite(r) & (r >= range_min) & (r <= max_range)
    r, a = r[ok], a[ok]
    return np.stack((LIDAR_X + r * np.cos(a), r * np.sin(a)), axis=1)


def wall_measurement(cmap, q_base, pose, lat_sigma_prior=0.5, hint=None, behind=5.0, ahead=25.0, gate_fine=0.20,
                     huber=0.06, sigma_pt=0.05, inflation=4.0, min_pts=15, min_span=4.0, iters=3,
                     floor=(0.03, 0.003), max_dwidth=0.4):
    """q_base: 뒤차축 좌표 점, pose: 스캔 시각의 추정 (x, y, yaw). 반환 dict(z, H, R, lat, ...) 또는 None.

    sigma_pt: 점 하나 잔차 σ (LiDAR 0.03 + 모형 오차), inflation: 점들 사이 상관을 고려해 R에 곱하는 값,
    floor: R 하한 σ (횡 m, 방향 rad), max_dwidth: 양쪽 벽으로 본 반폭 오차가 이보다 크면 한쪽만 씀."""
    x0, y0, th0 = pose
    i_veh = cmap.nearest_index(x0, y0, hint)
    s_veh = cmap.s[i_veh]
    i0 = int(np.searchsorted(cmap.s, s_veh - behind - 3.0))
    i1 = min(int(np.searchsorted(cmap.s, s_veh + ahead + 3.0)) + 1, len(cmap.p))
    if i1 - i0 < 3:
        return None
    q_base = q_base[np.hypot(*q_base.T) < ahead + 5.0]
    if len(q_base) < min_pts:
        return None
    delta, dw = np.zeros(3), 0.0
    gate_coarse = min(max(3.0 * lat_sigma_prior + 0.3, 0.5), 1.6)
    qx, qy = q_base[:, 0], q_base[:, 1]
    j = sel = None
    for it in range(iters + 1):
        th = th0 + delta[2]
        cc, ss = math.cos(th), math.sin(th)
        t = np.array([x0 + delta[0], y0 + delta[1]])
        pts = np.stack((t[0] + cc * qx - ss * qy, t[1] + ss * qx + cc * qy), axis=1)
        sp, d, nrm, j = cmap.project(pts, i0, i1, j)
        wl, wr = cmap.wL[j], cmap.wR[j]
        in_s = (sp > s_veh - behind) & (sp < s_veh + ahead)
        left = in_s & (d > 0) & np.isfinite(wl)
        right = in_s & (d < 0) & np.isfinite(wr)
        r = d - np.where(d > 0, np.nan_to_num(wl) + dw, -(np.nan_to_num(wr) + dw))
        if it == 0:
            # 1단계: 측별 잔차 최빈값 (0.04 m 칸) → 장애물·NPC·열린 곳 점에 강함
            modes = {}
            for name, m in (('L', left), ('R', right)):
                rr = r[m & (np.abs(r) < gate_coarse)]
                if len(rr) >= min_pts // 2:
                    hist, edges = np.histogram(rr, bins=np.arange(-gate_coarse, gate_coarse + 0.04, 0.04))
                    k = int(np.argmax(np.convolve(hist, [1, 2, 3, 2, 1], 'same')))
                    mode = 0.5 * (edges[k] + edges[k + 1])
                    near = rr[np.abs(rr - mode) < 0.1]
                    modes[name] = float(np.median(near)) if len(near) else mode
            if not modes:
                return None
            if len(modes) == 2:
                e_lat, dw = -(modes['L'] + modes['R']) / 2, (modes['L'] - modes['R']) / 2
                if abs(dw) > max_dwidth:            # 폭이 지도와 다름 → 점이 많은 쪽만
                    keep = 'L' if left.sum() >= right.sum() else 'R'
                    e_lat, dw = -modes[keep], 0.0
            else:
                e_lat = -next(iter(modes.values()))
            delta[:2] = e_lat * cmap.n[i_veh]
            continue
        # 2단계: 좁은 게이트 + Huber IRLS (Gauss-Newton, 재투영 포함)
        sel = (left | right) & (np.abs(r) < gate_fine)
        nL, nR = int((sel & left).sum()), int((sel & right).sum())
        if nL + nR < min_pts:
            return None
        rs, ns, rel = r[sel], nrm[sel], pts[sel] - t
        both = nL >= min_pts // 3 and nR >= min_pts // 3
        cols = [ns[:, 0], ns[:, 1], -ns[:, 0] * rel[:, 1] + ns[:, 1] * rel[:, 0]]
        if both:
            cols.append(-np.where(d[sel] > 0, 1.0, -1.0))   # ∂r/∂Δw
        J = np.stack(cols, axis=1)
        wts = np.where(np.abs(rs) < huber, 1.0, huber / np.maximum(np.abs(rs), 1e-9))
        A = (J * wts[:, None]).T @ J
        g = (J * wts[:, None]).T @ rs
        reg = np.diag([1e-4, 1e-4, 1e-3] + ([1.0] if both else []))   # 직선 진행 방향 특이성 방지 (아주 약한 사전)
        step = -np.linalg.solve(A + reg, g)
        delta += step[:3]
        if both:
            dw += step[3]
    rs_fin = rs + J @ step
    sig2 = max(sigma_pt ** 2, float(np.average(rs_fin ** 2, weights=wts)))
    info = A / sig2 / inflation
    if both:                                       # Δw 소거 (Schur complement)
        info = info[:3, :3] - np.outer(info[:3, 3], info[3, :3]) / (info[3, 3] + 1.0 / 0.3 ** 2)
    span = float(np.ptp(sp[sel])) if sel.any() else 0.0
    Ls = 5.0                                       # yaw를 5 m 길이로 정규화해 고유분해
    S = np.diag([1.0, 1.0, Ls])
    Sinv = np.linalg.inv(S)
    lam, U = np.linalg.eigh(Sinv @ info @ Sinv)
    keep = lam > 1.0 / 0.5 ** 2                    # σ < 0.5 (정규화 단위)인 방향만
    if span < min_span:
        keep &= np.abs(U[2, :]) < 0.5              # 진행 방향 분포가 짧으면 방향 정보는 버림
    if not keep.any():
        return None
    H = U[:, keep].T @ S
    R = np.diag(1.0 / lam[keep]) + np.eye(int(keep.sum())) * min(floor[0] ** 2, (floor[1] * Ls) ** 2)
    n0 = cmap.n[i_veh]
    return {'z': H @ delta, 'H': H, 'R': R, 'delta': delta, 'lat': float(n0 @ delta[:2]),
            'dwidth': float(dw) if both else None, 'n_left': nL, 'n_right': nR, 'index': i_veh,
            'rms': math.sqrt(sig2)}
