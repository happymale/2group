"""LiDAR 벽 기반 횡위치·방향 의사측정 (HEVEN 2026 MIT, research sketch — src/는 건드리지 않음).

아이디어 (curb/lane-keeping localization, ICP point-to-line 의 '지도 = 경로 중심선 ± 반폭' 버전)
  * 지도: route.csv 중심선 + 반폭 w(s) (기본 corridor_width/2, 장애물 구간 3.75, 주차·로터리는 벽 없음 처리).
  * 스캔 점을 '현재 추정 자세'로 map에 올리고, 각 점을 중심선에 투영해 (s, d) (d = 왼쪽 + 부호 횡거리)를 구함.
  * 왼쪽 벽 점은 d ≈ +wL, 오른쪽 벽 점은 d ≈ -wR 이어야 함. 차가 추정보다 e만큼 오른쪽에 있으면 모든 벽 점의 d가
    +e 만큼 밀려 보임 → 잔차 r = d - (±w)를 자세 보정 δ=(δx, δy, δθ)로 설명하는 가중 최소제곱(Huber IRLS).
  * 양쪽 벽이 다 보이면 통로 폭 오차 Δw도 같이 추정 (도로 폭 단계 0~5로 폭이 바뀌어도 횡위치는 (dL+dR)/2라 영향 없음).
  * 직선에서는 진행 방향(along-track)이 관측되지 않음 → 정보행렬 A의 고유분해로 '관측되는 방향만' 측정으로 냄.
  * 출력: EKF에 그대로 넣는 (z, H, R) — 상태 [x, y, yaw] 부분만. 횡/방향 요약(lat, dyaw, σ)도 같이.

이상치 처리
  * 1단계: 넓은 게이트(EKF 횡 σ 기반) 안의 점으로 측별 잔차 히스토그램 최빈값 → 장애물·NPC·열린 곳에 강함.
  * 2단계: 최빈값 ±gate_fine 안의 점만 Huber IRLS 3회 (재투영 포함, ICP식).
  * 곡률 |κ|·w > 0.6 (로터리 진입 R≈2.5 등) 근처, 차 뒤 5 m 이전, 앞 max_ahead 이후 점은 버림.
  * 점 수/진행방향 분포가 부족하면 None.

의존성: numpy 만. 실행: python wall_lateral_update.py  (합성 자체 시험 + 시간 측정)
라이선스: 직접 작성. 참고한 알고리즘(PL-ICP, curb 기반 횡위치, likelihood field)은 논문 아이디어만 사용.
"""
from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

LIDAR_X = 1.15          # 후륜 차축 기준 LiDAR 위치 (sensor_mounts.yaml)


# ---------------------------------------------------------------------------------------------------
# 지도: 중심선 + 측별 반폭
# ---------------------------------------------------------------------------------------------------
def _point_in_poly(px, py, poly):
    """짝수-홀수 규칙. px, py: 배열, poly: (K,2)."""
    poly = np.asarray(poly, float)
    inside = np.zeros(np.shape(px), bool)
    xj, yj = poly[-1]
    for xi, yi in poly:
        cond = ((yi > py) != (yj > py)) & (px < (xj - xi) * (py - yi) / (yj - yi + 1e-300) + xi)
        inside ^= cond
        xj, yj = xi, yi
    return inside


class CorridorMap:
    """route 중심선(0.5 m 간격)과 측별 기대 벽 반폭. wL/wR가 nan이면 그 쪽 벽을 기대하지 않음."""

    def __init__(self, route_xy, half_width=2.196, kappa_w_max=0.6):
        p = np.asarray(route_xy, float)
        self.p = p
        seg = np.diff(p, axis=0)
        L = np.hypot(seg[:, 0], seg[:, 1])
        self.s = np.r_[0.0, np.cumsum(L)]
        # 꼭짓점 접선: 앞뒤 선분 평균 (끝점은 한쪽)
        t = np.vstack([seg[:1], seg[:-1] + seg[1:], seg[-1:]])
        t /= np.hypot(t[:, 0], t[:, 1])[:, None]
        self.t = t
        self.n = np.c_[-t[:, 1], t[:, 0]]                  # 왼쪽 법선
        h = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
        self.kappa = np.gradient(h, self.s)
        self.wL = np.full(len(p), float(half_width))
        self.wR = np.full(len(p), float(half_width))
        self.kappa_w_max = kappa_w_max
        self._mask_tight_curves()

    def _mask_tight_curves(self):
        """안쪽 벽 반경(R - w)이 작아 투영이 모호한 곳(로터리 진입·진출 꺾임)은 그 쪽 벽을 끔."""
        k = self.kappa
        # 왼쪽 굽음(κ>0)이면 왼쪽 벽이 안쪽
        self.wL[(k > 0) & (k * np.nan_to_num(self.wL, nan=0) > self.kappa_w_max)] = np.nan
        self.wR[(k < 0) & (-k * np.nan_to_num(self.wR, nan=0) > self.kappa_w_max)] = np.nan

    def set_zone(self, polygon, half_width=None, sides='both'):
        """polygon 안의 경로 구간 반폭을 바꿈. half_width=None이면 벽 없음(nan)."""
        inside = _point_in_poly(self.p[:, 0], self.p[:, 1], polygon)
        v = np.nan if half_width is None else float(half_width)
        if sides in ('both', 'left'):
            self.wL[inside] = v
        if sides in ('both', 'right'):
            self.wR[inside] = v
        self._mask_tight_curves()

    def disable_disc(self, center, radius):
        d = np.hypot(self.p[:, 0] - center[0], self.p[:, 1] - center[1])
        self.wL[d < radius] = np.nan
        self.wR[d < radius] = np.nan

    @classmethod
    def from_files(cls, route_csv, course_json=None, corridor_width=4.392, obstacle_half_width=3.75):
        with open(route_csv, encoding='utf-8') as f:
            pts = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
        m = cls(pts, corridor_width / 2.0)
        if course_json:
            with open(course_json, encoding='utf-8') as f:
                c = json.load(f)['course']
            for mi in c.get('missions', []):
                typ = mi.get('type', '')
                if typ.startswith('obstacle') and 'zone' in mi:
                    m.set_zone(mi['zone'], obstacle_half_width)        # course.json: 벽 ±3.75
                elif typ in ('parallel_parking', 'rear_parking') and 'zone' in mi:
                    m.set_zone(_inflate(mi['zone'], 3.0), None)         # 한쪽이 열림 → 보수적으로 양쪽 끔
                elif typ == 'roundabout' and 'center' in mi:
                    m.disable_disc(mi['center'], mi.get('npc_radius', 11.0) + 12.0)
                elif typ == 'low_friction':
                    pass   # 벽 구조 미확인: 소음 OFF 로그로 잔차를 보고 결정 (README 참고)
        return m

    # -----------------------------------------------------------------------------------------------
    def nearest_index(self, x, y, hint=None, window=200):
        if hint is None:
            i0, i1 = 0, len(self.p)
        else:
            i0, i1 = max(0, hint - window), min(len(self.p), hint + window)
        d2 = (self.p[i0:i1, 0] - x) ** 2 + (self.p[i0:i1, 1] - y) ** 2
        return i0 + int(np.argmin(d2))

    def project(self, pts, i0, i1, j_prev=None):
        """pts (P,2)를 꼭짓점 i0..i1-1 구간 폴리라인에 투영 → s, d(왼쪽+), 법선 n (P,2), 꼭짓점 번호.
        가장 가까운 꼭짓점 j를 찾고 (j-1,j), (j,j+1) 두 선분 중 가까운 쪽에 정확히 투영.
        j_prev(이전 반복의 j)를 주면 전수 탐색 대신 j_prev-2..j_prev+2만 봄 (ICP 반복에서 보정이 작으므로)."""
        px, py = pts[:, 0], pts[:, 1]
        if j_prev is None:
            P = self.p[i0:i1]
            d2 = (px[:, None] - P[None, :, 0]) ** 2 + (py[:, None] - P[None, :, 1]) ** 2   # (P, M)
            j = i0 + np.argmin(d2, axis=1)
        else:
            cand = np.clip(j_prev[:, None] + np.arange(-2, 3)[None, :], 0, len(self.p) - 1)
            d2 = (px[:, None] - self.p[cand, 0]) ** 2 + (py[:, None] - self.p[cand, 1]) ** 2
            j = cand[np.arange(len(pts)), np.argmin(d2, axis=1)]
        best_d = np.full(len(pts), np.inf)
        out_s = np.zeros(len(pts))
        out_lat = np.zeros(len(pts))
        out_n = np.zeros((len(pts), 2))
        for off in (-1, 0):
            a = np.clip(j + off, 0, len(self.p) - 2)
            A, B = self.p[a], self.p[a + 1]
            ab = B - A
            L2 = (ab ** 2).sum(1)
            u = np.clip(((pts - A) * ab).sum(1) / L2, 0.0, 1.0)
            foot = A + u[:, None] * ab
            dist = np.hypot(*(pts - foot).T)
            tvec = ab / np.sqrt(L2)[:, None]
            nvec = np.stack((-tvec[:, 1], tvec[:, 0]), axis=1)
            lat = ((pts - foot) * nvec).sum(1)
            better = dist < best_d
            best_d = np.where(better, dist, best_d)
            out_s = np.where(better, self.s[a] + u * np.sqrt(L2), out_s)
            out_lat = np.where(better, lat, out_lat)
            out_n = np.where(better[:, None], nvec, out_n)
        return out_s, out_lat, out_n, j


# ---------------------------------------------------------------------------------------------------
# 측정
# ---------------------------------------------------------------------------------------------------
@dataclass
class WallMeasurement:
    delta: np.ndarray            # (3,) 추정 자세에 더할 보정 (δx, δy, δθ) — 관측 안 되는 방향은 0에 가까움
    z: np.ndarray                # (k,) = H @ delta  (EKF 혁신값으로 그대로 사용)
    H: np.ndarray                # (k,3) 상태 [x, y, yaw]에 대한 행
    R: np.ndarray                # (k,k)
    lat: float                   # 차 위치 경로 법선 방향 보정 (m, 왼쪽 +)
    lat_sigma: float
    dyaw: float                  # 방향 보정 (rad)
    dyaw_sigma: float
    dwidth: float | None         # 양쪽 벽이 보일 때 반폭 오차 추정 (m), 아니면 None
    n_left: int
    n_right: int
    info: dict = field(default_factory=dict)


def scan_to_base(ranges, angles, rmin=0.5, rmax=100.0, max_range=30.0, v=0.0, w=0.0, dt_beam=None, stride=1):
    """LaserScan → 후륜 차축(base) 좌표 점. v, w, dt_beam(빔별 스캔 시작 기준 시간)을 주면 스캔 시작 자세 기준으로
    역왜곡(deskew)함 — 10 Hz 스캔 동안 12 m/s면 1.2 m, 곡선 1 rad/s면 0.1 rad 움직이므로 곡선에서는 꼭 필요."""
    r = np.asarray(ranges, float)[::stride]
    a = np.asarray(angles, float)[::stride]
    if dt_beam is not None:
        dt_beam = np.asarray(dt_beam, float)[::stride]
    ok = np.isfinite(r) & (r >= rmin) & (r <= min(rmax, max_range))
    r, a = r[ok], a[ok]
    q = np.stack((LIDAR_X + r * np.cos(a), r * np.sin(a)), axis=1)
    if dt_beam is not None and (v or w):
        tb = np.asarray(dt_beam, float)[ok]
        th = w * tb                                   # 빔 시각의 차 자세(스캔 시작 기준, 일정 v, w)
        if abs(w) > 1e-6:
            px, py = v / w * np.sin(th), v / w * (1 - np.cos(th))
        else:
            px, py = v * tb, np.zeros_like(tb)
        c, s = np.cos(th), np.sin(th)
        q = np.c_[px + c * q[:, 0] - s * q[:, 1], py + s * q[:, 0] + c * q[:, 1]]
    return q


def wall_measurement(cmap: CorridorMap, q_base, pose, lat_sigma_prior=0.5, hint=None,
                     behind=5.0, ahead=25.0, gate_fine=0.20, huber=0.06, sigma_pt=0.05,
                     inflation=4.0, min_pts=15, min_span=4.0, iters=3, floor=(0.03, 0.003), max_dwidth=0.8):
    """q_base: (P,2) 차체 좌표 점, pose: (x, y, yaw) 추정(스캔 시각). 반환 WallMeasurement 또는 None.

    sigma_pt   : 점 하나의 '모형 포함' 잔차 σ (LiDAR σ0.03 + 벽/중심선 모형 오차)
    inflation  : 점들 사이 상관(같은 벽 모형 오차)을 고려해 R에 곱하는 값
    floor      : (횡 m, 방향 rad) R 하한 σ
    max_dwidth : 양쪽 벽 최빈값으로 본 반폭 오차가 이보다 크면 한쪽을 버림. 도로 폭 단계 0~5의 반폭 차가 최대 0.7 m.
                 주의: 한쪽 벽만 보일 때는 cmap 반폭이 맞아야 함 → 실제 폭 단계의 corridor_width를 넣을 것.
    """
    x0, y0, th0 = pose
    c, s = math.cos(th0), math.sin(th0)
    i_veh = cmap.nearest_index(x0, y0, hint)
    s_veh = cmap.s[i_veh]
    i0 = int(np.searchsorted(cmap.s, s_veh - behind - 3.0))
    i1 = int(np.searchsorted(cmap.s, s_veh + ahead + 3.0)) + 1
    i1 = min(i1, len(cmap.p))
    # 대략적 거리 필터: 차에서 ahead+5 m 이내 점만
    rr = np.hypot(*q_base.T)
    q_base = q_base[rr < ahead + 5.0]
    if len(q_base) < min_pts:
        return None

    delta = np.zeros(3)
    dw = 0.0
    gate_coarse = min(max(3.0 * lat_sigma_prior + 0.3, 0.5), 1.6)
    sel = None
    j = None
    qx, qy = q_base[:, 0], q_base[:, 1]
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
        exp = np.where(d > 0, np.nan_to_num(wl) + dw, -(np.nan_to_num(wr) + dw))
        r = d - exp
        if it == 0:
            # 1단계: 측별 최빈값 (히스토그램, 0.04 m bin) → 장애물·NPC 점에 강함
            modes = {}
            for name, m in (('L', left), ('R', right)):
                rr_ = r[m & (np.abs(r) < gate_coarse)]
                if len(rr_) >= min_pts // 2:
                    hist, edges = np.histogram(rr_, bins=np.arange(-gate_coarse, gate_coarse + 0.04, 0.04))
                    hist = np.convolve(hist, [1, 2, 3, 2, 1], 'same')
                    k = int(np.argmax(hist))
                    mode = 0.5 * (edges[k] + edges[k + 1])
                    near = rr_[np.abs(rr_ - mode) < 0.1]
                    modes[name] = float(np.median(near)) if len(near) else mode
            if not modes:
                return None
            if 'L' in modes and 'R' in modes:
                e_lat = -(modes['L'] + modes['R']) / 2     # 차 위치 보정 (법선 방향 +)
                dw = (modes['L'] - modes['R']) / 2
                if abs(dw) > max_dwidth:                  # 폭 단계 차(최대 0.7 m)보다 크면 한쪽이 벽이 아님 → 점 많은 쪽만
                    keep = 'L' if (left.sum() >= right.sum()) else 'R'
                    e_lat = -modes[keep]
                    dw = 0.0
                    modes = {keep: modes[keep]}
            else:
                (k_, v_), = modes.items()
                e_lat = -v_
            n0 = cmap.n[i_veh]
            delta[:2] = e_lat * n0
            continue
        # 2단계: 게이트 + Huber IRLS (Gauss-Newton 1 스텝/반복, 재투영 포함)
        sel = (left | right) & (np.abs(r) < gate_fine)
        nL, nR = int((sel & left).sum()), int((sel & right).sum())
        if nL + nR < min_pts:
            return None
        rs, ns, ps = r[sel], nrm[sel], pts[sel]
        sides = np.where(d[sel] > 0, 1.0, -1.0)
        rel = ps - t
        both = nL >= min_pts // 3 and nR >= min_pts // 3
        cols = [ns[:, 0], ns[:, 1], ns[:, 0] * -rel[:, 1] + ns[:, 1] * rel[:, 0]]
        if both:
            cols.append(-sides)                           # ∂r/∂Δw: 왼쪽 -1, 오른쪽 +1
        J = np.stack(cols, axis=1)
        a = np.abs(rs)
        wts = np.where(a < huber, 1.0, huber / np.maximum(a, 1e-9))
        A = (J * wts[:, None]).T @ J
        g = (J * wts[:, None]).T @ rs
        reg = np.diag([1e-4, 1e-4, 1e-3] + ([1.0] if both else []))   # 직선 along-track 특이성 방지 (아주 약한 사전)
        step = -np.linalg.solve(A + reg, g)
        delta += step[:3]
        if both:
            dw += step[3]
    # ---- 공분산: 마지막 반복의 정보행렬 ----
    rs_fin = rs + J @ step                                       # 마지막 스텝 적용 후 근사 잔차
    sig2 = max(sigma_pt ** 2, float(np.average(rs_fin ** 2, weights=wts)))
    Ainfo = A / sig2 / inflation
    if both:                                                     # Δw를 marginalize (Schur complement)
        Ainfo = Ainfo[:3, :3] - np.outer(Ainfo[:3, 3], Ainfo[3, :3]) / (Ainfo[3, 3] + 1.0 / 0.3 ** 2)
    # 진행 방향 분포가 짧으면 방향 정보는 신뢰하지 않음
    span = float(np.ptp(sp[sel])) if sel is not None and sel.any() else 0.0
    # 관측되는 방향만 측정으로: x,y는 m, yaw는 L=5 m 로 정규화해 고유분해
    Ls = 5.0
    S = np.diag([1.0, 1.0, Ls])
    Sinv = np.linalg.inv(S)
    An = Sinv @ Ainfo @ Sinv
    lam, U = np.linalg.eigh(An)
    keep = lam > 1.0 / (0.5 ** 2)                               # σ < 0.5 m (정규화 단위)인 방향만
    if span < min_span:
        # 방향 관측 불충분 → yaw 성분이 큰 고유벡터 제거
        keep &= np.abs(U[2, :]) < 0.5
    if not keep.any():
        return None
    Uk = U[:, keep]
    H = Uk.T @ S                                                # z = Ukᵀ (S δ): 정규화 보정의 관측 성분
    z = H @ delta
    Rk = np.diag(1.0 / lam[keep])
    # 하한: 횡 σ floor[0], 방향 σ floor[1] (정규화 단위에서 등방 하한으로 근사)
    Rk = Rk + np.eye(len(Rk)) * min(floor[0] ** 2, (floor[1] * Ls) ** 2)
    # 요약 (차 위치 법선 방향 / yaw)
    n0 = cmap.n[i_veh]
    Cov = np.linalg.pinv(H.T @ np.linalg.inv(Rk) @ H + np.diag([1e-6, 1e-6, 1e-6]))
    lat = float(n0 @ delta[:2])
    lat_sig = float(math.sqrt(max(n0 @ Cov[:2, :2] @ n0, 0.0)))
    return WallMeasurement(delta=delta, z=z, H=H, R=Rk, lat=lat, lat_sigma=lat_sig, dyaw=float(delta[2]),
                           dyaw_sigma=float(math.sqrt(max(Cov[2, 2], 0.0))),
                           dwidth=float(dw) if both else None, n_left=nL, n_right=nR,
                           info={'span': span, 'route_index': i_veh, 'eig': lam.tolist()})


# ---------------------------------------------------------------------------------------------------
# EKF 연결 예 (localization.py 의 self.update(innov, H, R) 형식에 맞춘 것; 여기서는 참고용 함수)
# ---------------------------------------------------------------------------------------------------
def ekf_apply(s, P, meas: WallMeasurement, idx=(0, 1, 2), gate_chi2=None):
    """s, P: EKF 상태·공분산 (localization.py: X, Y, YAW = 0, 1, 2). 측정 z = H δ 를 혁신값으로 사용.
    (측정 자세 = 추정 자세 + δ 이므로 혁신 = H(측정 - 추정) = H δ.)  반환 (s, P, accepted, nis)."""
    k = len(meas.z)
    Hf = np.zeros((k, len(s)))
    Hf[:, list(idx)] = meas.H
    Sm = Hf @ P @ Hf.T + meas.R
    nis = float(meas.z @ np.linalg.solve(Sm, meas.z))
    if gate_chi2 is None:
        gate_chi2 = {1: 10.8, 2: 13.8, 3: 16.3}[k]               # χ² 99.9 %
    if nis > gate_chi2:
        return s, P, False, nis
    K = P @ Hf.T @ np.linalg.inv(Sm)
    s = s + K @ meas.z
    s[idx[2]] = (s[idx[2]] + math.pi) % (2 * math.pi) - math.pi
    I_KH = np.eye(len(s)) - K @ Hf
    P = I_KH @ P @ I_KH.T + K @ meas.R @ K.T
    return s, P, True, nis


# ---------------------------------------------------------------------------------------------------
# 합성 자체 시험
# ---------------------------------------------------------------------------------------------------
def _raycast(origin, angles, segs, rmax=100.0):
    """origin (2,), angles (B,), segs (S,4) → ranges (B,). 벡터화한 광선-선분 교차."""
    d = np.c_[np.cos(angles), np.sin(angles)]                       # (B,2)
    a, b = segs[:, :2], segs[:, 2:]
    e = b - a                                                       # (S,2)
    ao = a - origin                                                 # (S,2)
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]          # (B,S)
    t = (ao[None, :, 0] * e[None, :, 1] - ao[None, :, 1] * e[None, :, 0]) / np.where(den == 0, 1e-12, den)
    u = (ao[None, :, 0] * d[:, None, 1] - ao[None, :, 1] * d[:, None, 0]) / np.where(den == 0, 1e-12, den)
    hit = (t > 0) & (u >= 0) & (u <= 1) & (np.abs(den) > 1e-12)
    t = np.where(hit, t, np.inf)
    r = t.min(1)
    r[r > rmax] = np.inf
    return r


def _walls(cmap, i0, i1, wl, wr, gap=None):
    P, N = cmap.p[i0:i1], cmap.n[i0:i1]
    L = P + N * wl
    R = P - N * wr
    segs = []
    for W in (L, R):
        for k in range(len(W) - 1):
            if gap is not None and gap[0] <= cmap.s[i0 + k] <= gap[1] and W is L:
                continue                                            # 왼쪽 벽 열린 곳
            segs.append(np.r_[W[k], W[k + 1]])
    return np.array(segs)


def _box(cx, cy, yaw, l, w):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(cx + c * dx - s * dy, cy + s * dx + c * dy) for dx, dy in
           ((l / 2, w / 2), (-l / 2, w / 2), (-l / 2, -w / 2), (l / 2, -w / 2))]
    return np.array([np.r_[pts[k], pts[(k + 1) % 4]] for k in range(4)])


def self_test(route_csv=None, seed=0, verbose=True, stride=1):
    rng = np.random.default_rng(seed)
    if route_csv and Path(route_csv).exists():
        with open(route_csv, encoding='utf-8') as f:
            route = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
        cases = [(30.0, 'straight'), (603.0, 'shadow S-curve'), (225.0, 'slope S-curve')]
    else:  # 합성 경로: 직선 30 m + R6 S자
        xs = np.arange(0, 30, 0.5)
        route = [(x, 0.0) for x in xs]
        th = np.arange(0, math.pi / 2, 0.5 / 6)
        route += [(30 + 6 * math.sin(a), 6 - 6 * math.cos(a)) for a in th]
        route += [(36 + 0.0, 6 + k * 0.5) for k in range(60)]
        cases = [(15.0, 'straight'), (35.0, 'curve')]
    cmap = CorridorMap(route, 2.196)
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    results = []
    for s_true, name in cases:
        i = int(np.searchsorted(cmap.s, s_true))
        lat_true = rng.uniform(-0.4, 0.4)
        xt, yt = cmap.p[i] + cmap.n[i] * lat_true
        yaw_t = math.atan2(cmap.t[i, 1], cmap.t[i, 0]) + rng.normal(0, 0.03)
        i0 = max(0, i - 120)
        i1 = min(len(cmap.p), i + 160)
        gap = (cmap.s[i] + 8.0, cmap.s[i] + 11.0)                  # 왼쪽 벽 3 m 열림
        segs = _walls(cmap, i0, i1, 2.196, 2.196, gap)
        # 장애물(NPC/콘) 하나: 앞 10 m, 오른쪽 0.8 m
        k = min(i + 20, len(cmap.p) - 1)
        ob = cmap.p[k] - cmap.n[k] * 0.8
        segs = np.vstack([segs, _box(ob[0], ob[1], 0.0, 1.0, 1.0)])
        lid = np.array([xt + LIDAR_X * math.cos(yaw_t), yt + LIDAR_X * math.sin(yaw_t)])
        rng_ = _raycast(lid, angles + yaw_t, segs)
        rng_ = rng_ + rng.normal(0, 0.03, rng_.shape)
        # 추정 자세 = 참값 + 오차 (횡 0.7 m, 진행 0.5 m, 방향 0.02 rad)
        e_lat, e_along, e_yaw = rng.choice([-1, 1]) * 0.7, 0.5, rng.normal(0, 0.02)
        xe = xt + cmap.n[i, 0] * e_lat + cmap.t[i, 0] * e_along
        ye = yt + cmap.n[i, 1] * e_lat + cmap.t[i, 1] * e_along
        ye_yaw = yaw_t + e_yaw
        q = scan_to_base(rng_, angles, stride=stride)
        t0 = time.perf_counter()
        m = wall_measurement(cmap, q, (xe, ye, ye_yaw), lat_sigma_prior=0.5, hint=i)
        dt_ms = (time.perf_counter() - t0) * 1e3
        if m is None:
            print(f'[{name}] no measurement')
            continue
        # EKF 1회 갱신 (P: 횡/진행 0.5 m, yaw 0.03 rad)
        s_ = np.array([xe, ye, ye_yaw])
        P = np.diag([0.5 ** 2, 0.5 ** 2, 0.03 ** 2])
        s2, P2, ok, nis = ekf_apply(s_, P, m)
        err_before = np.array([cmap.n[i] @ (np.r_[xe, ye] - [xt, yt]), ye_yaw - yaw_t])
        err_after = np.array([cmap.n[i] @ (s2[:2] - [xt, yt]), s2[2] - yaw_t])
        results.append((name, err_before, err_after, m, dt_ms))
        if verbose:
            print(f'[{name:15s}] lat err {err_before[0]:+.3f} -> {err_after[0]:+.3f} m | '
                  f'yaw err {math.degrees(err_before[1]):+.2f} -> {math.degrees(err_after[1]):+.2f} deg | '
                  f'meas lat {m.lat:+.3f}±{m.lat_sigma:.3f} dyaw {math.degrees(m.dyaw):+.2f}±'
                  f'{math.degrees(m.dyaw_sigma):.2f} deg | k={len(m.z)} nL={m.n_left} nR={m.n_right} '
                  f'dw={m.dwidth} NIS={nis:.1f} acc={ok} | {dt_ms:.1f} ms')
    # 시간 측정 (반복)
    if results:
        name, *_ = results[0]
        reps = 50
        t0 = time.perf_counter()
        for _ in range(reps):
            wall_measurement(cmap, q, (xe, ye, ye_yaw), lat_sigma_prior=0.5, hint=i)
        print(f'평균 처리 시간 (stride={stride}, 점 {len(q)}개): {(time.perf_counter() - t0) / reps * 1e3:.2f} ms / scan')
    return results


if __name__ == '__main__':
    here = Path(__file__).resolve()
    route = here.parents[2] / 'src' / 'hk_mit' / 'course' / 'route.csv'
    print('== 실제 route.csv ==' if route.exists() else '== 합성 경로 ==')
    for sd in range(3):
        self_test(str(route) if route.exists() else None, seed=sd)
    print('== 실제 route.csv, 빔 절반(stride=2) ==')
    self_test(str(route) if route.exists() else None, seed=5, stride=2)
    print('== 합성 경로 ==')
    self_test(None, seed=1)
