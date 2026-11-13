"""장애물 구간의 전역 경로 우회(detour): LiDAR로 모은 장애물 좌표로 기준 경로 구간 자체를 바꿉니다.

장애물 배치는 seed마다 다르므로 좌표는 실행 중 LiDAR 관측만으로 정합니다(코드에 좌표를 두지 않음).
  1) 구간 안 관측점(map 좌표 → 기준 경로 Frenet s, d)을 여러 스캔 모아 0.1 m 칸에 셉니다(DetourMap).
     두 번 이상 맞은 칸만 써서 LiDAR 잡음 점 하나로 가짜 장애물을 만들지 않습니다.
  2) 구간 안의 벽이 아닌 칸을 경로 거리로 묶어 장애물로 만들고, 장애물마다 빈 횡간격(벽 ±3.75 포함)에서
     지날 횡위치 d를 정해 끝까지 지킵니다(장애물을 지나는 동안 앞범퍼~뒤범퍼 전체가 그 d).
  3) 지금 자세 → 장애물별 d → 구간 끝 전 d=0 을 quintic d(s)로 잇고, 차체 여유·곡률을 검사합니다.
  4) 곡률로 속력 한계(sqrt(a/κ))를 만들고 감속 한계로 미리 반영합니다. planner는 이 경로를 기준 경로처럼
     내보내므로 매 주기 후보 선택이 없고, Stanley는 고정된 경로를 끝까지 같은 속력 계획으로 따라갑니다.
새 관측점이 지금 우회 경로와 부딪히면(가려졌던 장애물이 보임) 모은 점 전체로 지금 자세에서 다시 만듭니다.
"""
from dataclasses import dataclass, replace
import itertools
import math

import numpy as np
from scipy.spatial import cKDTree

from .avoidance import _frenet_clearance, _lateral_profile


DEBUG = []                  # 시험용: 마지막 계획에서 탈락한 후보와 이유
WALL_OFFSET = 0.05          # 코스 파일 벽(±3.75)보다 관측 벽면이 이만큼 바깥 (LiDAR 벽 점은 ±3.8 근처)
WALL_RELIEF = 0.15          # 구간 벽 쪽 필요 여유를 이만큼 덜 둠 (벽은 코스 파일로 정확: 지그재그에서 앞모서리가 벽 쪽으로 휨)


@dataclass
class Detour:
    s: np.ndarray           # 기준 경로 거리 (원래 route 기준, step 간격)
    d: np.ndarray           # 횡위치
    d_prime: np.ndarray
    xy: np.ndarray
    yaw: np.ndarray
    curvature: np.ndarray
    speed: np.ndarray       # 각 점의 속력 한계 (감속 한계 반영)
    targets: tuple          # 장애물별 (시작 s, 끝 s, 지날 d)
    clearance: float        # 필요 여유를 뺀 남는 최소 여유 (m)
    margin: float           # 이 경로를 만들 때 쓴 margin
    signature: tuple = ()   # 만들 때의 장애물 배치 (scene_signature)

    def speed_at(self, station, ahead):
        """station부터 ahead(m) 안의 가장 낮은 속력 한계."""
        lo, hi = np.searchsorted(self.s, [station, station + ahead])
        lo = min(int(lo), len(self.s) - 1)
        return float(np.min(self.speed[lo:max(int(hi), lo + 1)]))


class DetourMap:
    """구간 근처 관측점을 Frenet 칸(cell, 0.1 m)으로 모읍니다. 칸 = (round(s/cell), round(d/cell))."""

    def __init__(self, cell=0.1, min_hits=2):
        self.cell, self.min_hits = cell, min_hits
        self.counts = {}

    def clear(self):
        self.counts = {}

    def add(self, s, d):
        keys = np.unique(np.column_stack((np.round(s / self.cell), np.round(d / self.cell))).astype(int), axis=0)
        for key in map(tuple, keys):
            self.counts[key] = self.counts.get(key, 0) + 1

    def points(self):
        keys = [k for k, n in self.counts.items() if n >= self.min_hits]
        if not keys:
            return np.empty(0), np.empty(0)
        keys = np.asarray(keys, dtype=float) * self.cell
        return keys[:, 0], keys[:, 1]


def _transition_length(delta, speed, lateral_accel):
    """quintic 옆 이동 최대 곡률 ≈ 5.77·Δd/L² 를 a/v² 아래로: L = v·sqrt(5.77·Δd/a)."""
    return speed * math.sqrt(5.77 * abs(delta) / lateral_accel)


def _drop_wall_runs(obs_s, obs_d, mask, near_wall=2.0, gap=1.0, run=4.0):
    """mask에서 외딴 잡음 점과 벽으로 보이는 점(벽 가까이 경로 방향으로 길게 이어진 점)을 뺀 mask."""
    mask = mask.copy()
    # LiDAR 잡음으로 두 번 넘게 맞은 외딴 칸: 0.4 m 안에 다른 장애물 칸이 둘 미만이면 뺌
    idx = np.flatnonzero(mask)
    if len(idx):
        tree = cKDTree(np.column_stack((obs_s[idx], obs_d[idx])))
        counts = np.array([len(n) for n in tree.query_ball_point(tree.data, 0.4)])
        mask[idx[counts < 3]] = False
    for side in (1, -1):
        idx = np.flatnonzero(mask & (side * obs_d > near_wall))
        if not len(idx):
            continue
        idx = idx[np.argsort(obs_s[idx])]
        for piece in np.split(idx, np.flatnonzero(np.diff(obs_s[idx]) > gap) + 1):
            if obs_s[piece[-1]] - obs_s[piece[0]] > run:
                mask[piece] = False
    return mask


def complete_boxes(obs_s, obs_d, groups, cfg, box=(2.0, 4.3), wall_touch=2.0, spacing=0.2, mask=None):
    """규정상 장애물은 2 m × 4.3 m 상자(진행 방향 2 m, 가로 4.3 m)입니다. 앞 장애물에 가려 일부만 보인 상자를
    미리 채웁니다: 벽에 닿아 보이면 그 벽에서 가로 4.3 m까지, 깊이는 처음 본 앞면에서 2 m까지.
    그래야 처음 보았을 때부터 맞는 쪽·횡위치로 우회 경로를 정하고, 늦게 드러난 부분 때문에 급하게 다시 짜지 않습니다.
    반환: (점 s, 점 d, 늘린 무리 [(시작, 끝, 벽 쪽 ±1/0), ...]). 실제 관측이 상자보다 크면 관측을 따릅니다."""
    length, width = box
    wall = cfg.road_half_width
    extra_s, extra_d, grown = [], [], []
    for a, b in groups:
        sel = (obs_s >= a) & (obs_s <= b) & (np.abs(obs_d) < wall - 0.3)
        if mask is not None:
            sel &= mask
        low, high = float(np.min(obs_d[sel])), float(np.max(obs_d[sel]))
        side = 0                                    # +1: 왼쪽(+d) 벽에 붙음, -1: 오른쪽(-d) 벽, 0: 아님
        if high > wall - wall_touch and low < -wall + wall_touch:
            pass                                    # 양쪽 벽에 다 닿아 보임 (막힘): 그대로
        elif high > wall - wall_touch:
            low, side = min(low, wall - width), 1
        elif low < -wall + wall_touch:
            high, side = max(high, -wall + width), -1
        b = max(b, a + length)
        n_s = max(2, int(math.ceil((b - a) / spacing)) + 1)
        n_d = max(2, int(math.ceil((high - low) / spacing)) + 1)
        for edge_s in (a, b):                       # 앞·뒷면
            extra_s.append(np.full(n_d, edge_s))
            extra_d.append(np.linspace(low, high, n_d))
        for edge_d in (low, high):                  # 옆면
            extra_s.append(np.linspace(a, b, n_s))
            extra_d.append(np.full(n_s, edge_d))
        grown.append((a, b, side))
    if extra_s:
        obs_s = np.r_[obs_s, np.concatenate(extra_s)]
        obs_d = np.r_[obs_d, np.concatenate(extra_d)]
    return obs_s, obs_d, grown


def read_obstacles(obs_s, obs_d, zone, s_ego, cfg, box=(2.0, 4.3)):
    """모은 관측점 → 장애물(상자) 무리와 벽 쪽, 간격·여유 검사용 점.
    반환 dict: groups [(시작 s, 끝 s)], sides [±1/0], gap_s/gap_d, check_s/check_d."""
    # 구간 안의 벽이 아닌 점 → 장애물 무리
    inner = ((obs_s > zone[0] + 0.3) & (obs_s < zone[1] - 0.3) & (np.abs(obs_d) < cfg.road_half_width - 0.3)
             & (obs_s > s_ego - cfg.rear_overhang - 0.5))
    # 위치 추정 잡음(gnss_noise)으로 모은 벽 점이 안쪽(|d| < 3.45)으로 번지면 구간 전체가 장애물로 묶여
    # 우회가 막힙니다. 벽 가까이(|d| > 2.0) 점을 좌우별로 경로 방향 1 m 간격으로 이었을 때 4 m 넘게 이어지면
    # 벽으로 보고 뺍니다 (장애물 상자는 진행 방향 2 m). 벽에 붙은 장애물의 벽 쪽 부분도 빠지지만
    # complete_boxes가 벽까지 다시 채웁니다.
    inner = _drop_wall_runs(obs_s, obs_d, inner & (np.abs(obs_d) < cfg.road_half_width - 0.85))
    groups = []
    if np.any(inner):
        ordered = np.sort(obs_s[inner])
        cuts = np.flatnonzero(np.diff(ordered) > cfg.vehicle_length + 2.0)
        groups = [(float(a[0]), float(a[-1])) for a in np.split(ordered, cuts + 1)]
    sides = [0] * len(groups)
    gap_s, gap_d = obs_s[inner], obs_d[inner]     # 빈 간격은 장애물 점 + 채운 상자 + 코스 파일 벽(±R)으로
    if groups and box is not None:
        count = len(obs_s)
        obs_s, obs_d, grown = complete_boxes(obs_s, obs_d, groups, cfg, box, mask=inner)
        groups, sides = [(a, b) for a, b, _ in grown], [side for _, _, side in grown]
        gap_s, gap_d = np.r_[gap_s, obs_s[count:]], np.r_[gap_d, obs_d[count:]]
    # 여유 검사 점: 구간 안은 정리한 장애물 점 + 채운 상자 + 코스 파일 벽(±R) 선, 구간 밖(깔때기·좁은 통로)은
    # 관측점 그대로. 잡음으로 번진 벽·외딴 점이 구간 안의 여유를 깎지 않게 합니다
    outside = (obs_s[:len(inner)] < zone[0] + 0.3) | (obs_s[:len(inner)] > zone[1] - 0.3)
    # 계산 시간(시뮬레이터 3대가 CPU 공유): 벽 선은 0.5 m 간격, 구간 밖 관측점은 0.3 m 칸 하나씩만
    wall_s = np.arange(max(zone[0], s_ego - 5.0), zone[1], 0.5)
    outside &= (obs_s[:len(inner)] > s_ego - cfg.vehicle_length) & (np.abs(obs_d[:len(inner)]) < cfg.road_half_width + 0.3)
    if np.any(outside):
        _, first = np.unique(np.round(np.column_stack((obs_s[:len(inner)][outside], obs_d[:len(inner)][outside])) / 0.3),
                             axis=0, return_index=True)
        out_s, out_d = obs_s[:len(inner)][outside][first], obs_d[:len(inner)][outside][first]
    else:
        out_s, out_d = np.empty(0), np.empty(0)
    check_s = np.r_[out_s, gap_s, wall_s, wall_s]
    wall = cfg.road_half_width + WALL_OFFSET
    check_d = np.r_[out_d, gap_d, np.full(len(wall_s), wall), np.full(len(wall_s), -wall)]
    return {'groups': groups, 'sides': sides, 'gap_s': gap_s, 'gap_d': gap_d,
            'check_s': check_s, 'check_d': check_d}


def scene_signature(scene):
    """장애물 배치 요약: ((시작 s, 벽 쪽), ...). 0.5 m 단위로 반올림해 같은지 비교합니다."""
    return tuple((round(a * 2) / 2, side) for (a, _), side in zip(scene['groups'], scene['sides']))


def plan_detour(reference, s_ego, d_ego, slope, obs_s, obs_d, zone, s_exit, cfg, margins=None, step=0.4,
                box=(2.0, 4.3), check=True):
    """반환 (Detour 또는 None, 이유). zone = (시작 s, 끝 s) 장애물 미션 구간, s_exit까지 d=0으로 돌아옵니다.
    cfg는 avoidance.AvoidanceConfig (차체 크기, road_half_width, surface_padding, max_lateral_accel,
    cruise_speed, decel, return_gap, min_transition). check=False면 여유 검사 없이(곡률만) 템플릿을 씁니다:
    멈추면 끝내 실패하므로 마지막 수단으로 아주 천천히 지나가기 위함."""
    # margins: 큰 여유부터 시도할 margin 목록. 후보마다 여유를 한 번만 재서 만족하는 가장 큰 margin 단계를
    # 정합니다(단계마다 다시 찾으면 시뮬레이터 부하에서 0.5 s 넘게 걸려 제어가 멈췄음)
    margins = tuple(margins) if margins is not None else (cfg.margin,)
    cfg = replace(cfg, margin=min(margins))
    DEBUG.clear()
    need = min(margins) + cfg.surface_padding
    longitudinal = max(margins) + cfg.surface_padding + 0.1
    front = cfg.vehicle_length - cfg.rear_overhang
    kappa_max = math.tan(math.radians(cfg.max_steer_deg)) / cfg.wheelbase * 0.95
    s_exit = min(s_exit, reference.length - 1.0)
    if s_exit - s_ego < 2.0:
        return None, 'zone_passed'
    scene = read_obstacles(obs_s, obs_d, zone, s_ego, cfg, box)
    groups, sides = scene['groups'], scene['sides']
    gap_s, gap_d, check_s, check_d = scene['gap_s'], scene['gap_d'], scene['check_s'], scene['check_d']
    intervals = []
    for a, b in groups:
        gaps = _gap_intervals(gap_s, gap_d, a - 1.0, b + 1.0, cfg, need)
        if not gaps:
            return None, 'no_gap'
        intervals.append(gaps[:2])
    v = cfg.cruise_speed
    stations = np.arange(s_ego, s_exit + 5.0 + step, step)
    best, best_level = None, len(margins)
    # 간격 안쪽 여유 0.3 m: 추종 오차(5 m/s 지그재그에서 약 0.15 m)를 흡수. 간격이 좁으면 가운데.
    # 그 경로가 안 되면(좁은 지그재그) 안쪽 여유 0.1 m로 옆 이동을 줄인 순서도 시도
    key = lambda c: sum(abs(x - y) for x, y in zip((d_ego,) + c, c + (0.0,)))
    combos = []
    if groups and all(sides):
        # 규정 배치(모든 장애물이 한쪽 벽에 붙은 2 × 4.3 m 상자): 고정 템플릿. 반대쪽 빈 간격(3.2 m)의
        # 가운데 ∓2.15 m로 지나고, 지그재그가 조향 한계로 안 되면 옆 이동을 줄인 ∓1.95, ∓1.8 순서로
        combos = _template_sequences(sides, intervals)
    for extra in (max(cfg.gap_extra, 0.3), 0.1):
        combos += sorted(_target_sequences(intervals, d_ego, extra), key=key)[:4]
    combos = list(dict.fromkeys(combos))
    for combination in combos:
        # stretch: 옆 이동 길이 배율, late: 좁은 장애물 사이에서 옆 이동을 다음 장애물 앞면 뒤로 더 미는 거리
        # early: 무리 사이 옆 이동을 앞 장애물 끝보다 이만큼 일찍 시작 (뒤차축 뒤 차체는 꺾을 때 장애물 반대쪽으로 돎)
        for stretch, late, early in ((1.0, 0.0, 0.0), (0.7, 0.0, 0.0), (1.5, 0.0, 0.0), (0.45, 0.0, 0.0),
                                     (0.45, 1.0, 0.0), (0.45, 1.0, 1.0), (0.45, 2.0, 1.0)):
            segments, targets, cursor, previous = [], [], s_ego, d_ego
            feasible = True
            for (a, b), target in zip(groups, combination):
                hold_from, hold_to = a - front - 0.3, b + cfg.rear_overhang + 0.3
                if abs(target - previous) > 1e-6:
                    want = max(cfg.min_transition, stretch * _transition_length(target - previous, v,
                                                                                cfg.max_lateral_accel))
                    end = hold_from
                    if end - cursor < want:
                        # 무리 사이가 좁으면 뒤차축이 앞 장애물 끝을 지날 때 시작해 뒤범퍼가 다음 장애물 앞면에
                        # 올 때 끝내도 차체는 비켜 있음 (꺾을 때 뒤 오버행은 장애물 반대쪽으로 돎). 검사가 확인
                        cursor = max(targets[-1][1] - cfg.rear_overhang - early, s_ego) if targets else cursor
                        end = max(hold_from, min(a + cfg.rear_overhang + late, cursor + want + late))
                    room = end - cursor
                    if room <= 0.5:
                        if abs(target - previous) > 0.4:
                            feasible = False
                            break
                        target = previous          # 이미 장애물 옆이고 거의 그 횡위치: 그대로 유지
                    else:
                        length = min(max(want, 2.0), room)
                        segments.append((end - length, length, target))
                targets.append((a, hold_to, target))
                cursor, previous = max(cursor, hold_to), target
            if not feasible:
                DEBUG.append((combination, stretch, late, early, 'room'))
                continue
            if abs(previous) > 1e-6:
                start = cursor + (cfg.return_gap if targets else 0.0)
                want = max(cfg.min_transition, stretch * _transition_length(previous, v, cfg.max_lateral_accel))
                length = min(want, s_exit - 0.5 - start)
                if length < 3.0:
                    continue
                segments.append((start, length, 0.0))
            if not segments:
                segments = [(s_ego, cfg.min_transition, 0.0)]
            first = segments[0]
            # 첫 이동이 지금 위치에서 시작하면 지금 진행 방향(slope)에서 이어 붙임
            d, d_prime = _lateral_profile(stations, d_ego, slope if first[0] <= s_ego + 1e-6 else 0.0,
                                          [(max(first[0], s_ego),) + tuple(first[1:])] + segments[1:])
            candidate = _build(reference, stations, d, d_prime, targets, cfg, s_ego)
            if candidate is None or np.max(np.abs(candidate.curvature)) > kappa_max:
                DEBUG.append((combination, stretch, late, early, 'curvature'))
                continue
            clearance = _frenet_clearance(stations, d, d_prime, check_s, check_d, cfg, s_exit, longitudinal,
                                          wall_relief=WALL_RELIEF)
            if check and clearance < need - 0.02:
                DEBUG.append((combination, stretch, late, early, f'clearance {clearance - need:.2f}'))
                continue
            level = next((k for k, m in enumerate(margins)
                          if clearance >= m + cfg.surface_padding - 0.02), len(margins) - 1)
            candidate.clearance = clearance - margins[level] - cfg.surface_padding
            candidate.margin = margins[level]
            if best is None or level < best_level:
                best, best_level = candidate, level   # 선호 순서(긴 옆 이동부터)에서 가장 큰 여유 단계
            break
        if best is not None and best_level == 0:
            break
    if best is None:
        return None, 'no_feasible_detour'
    best.signature = scene_signature(scene)
    return best, 'ok'


def _gap_intervals(obs_s, obs_d, low, high, cfg, need):
    """[low, high]를 지날 때 차 뒤차축 횡위치가 있을 수 있는 구간 [(lo, hi), ...] (넓은 것 먼저)."""
    inside = (obs_s >= low) & (obs_s <= high) & (np.abs(obs_d) < cfg.road_half_width)
    walls = np.sort(np.r_[obs_d[inside], -cfg.road_half_width, cfg.road_half_width])
    half = 0.5 * cfg.vehicle_width + need
    gaps = [(b - a, a + half, b - half) for a, b in zip(walls[:-1], walls[1:]) if b - a >= 2 * half]
    return [(lo, hi) for _, lo, hi in sorted(gaps, reverse=True)]


TEMPLATE_OFFSETS = (2.4, 2.25, 2.1, 1.95, 1.8)   # 벽에 붙은 장애물 반대쪽으로 지날 |d|. 반대쪽 벽(위치가 코스 파일로 정확)에 붙어 장애물과 멀리


def _template_sequences(sides, intervals, limit=8):
    """장애물별 벽 쪽(±1)으로 정해지는 지날 d 순서 [(−side·2.15, ...), ...]. 그 여유 단계에서 빈 간격 안인 값만.
    가운데(2.15)를 많이 쓰는 순서부터."""
    options = []
    for side, gaps in zip(sides, intervals):
        values = [-side * value for value in TEMPLATE_OFFSETS
                  if any(lo - 1e-6 <= -side * value <= hi + 1e-6 for lo, hi in gaps)]
        if not values:
            return []
        options.append(values)
    ranked = sorted(itertools.product(*[range(len(v)) for v in options]), key=lambda idx: (sum(idx), max(idx)))
    return [tuple(options[k][i] for k, i in enumerate(idx)) for idx in ranked[:limit]]


def _target_sequences(intervals, d_ego, extra):
    """장애물마다 지날 d 순서 후보. 각 간격에서 (직전 d에 가장 가까운 곳: 지그재그 옆 이동 최소)과
    (기준 경로 0에 가장 가까운 곳) 두 가지. 간격 안쪽으로 extra만큼 더 들어갑니다(좁으면 가운데)."""
    sequences = [((), d_ego)]
    for gaps in intervals:
        grown = []
        for sequence, previous in sequences:
            options = []
            for lo, hi in gaps:
                spare = min(extra, 0.5 * (hi - lo))
                # 직전 d가 이미 간격 안이면 그대로(옆 이동 없음), 아니면 안쪽으로 spare만큼
                near = previous if lo <= previous <= hi else float(np.clip(previous, lo + spare, hi - spare))
                options += [near, float(np.clip(0.0, lo + spare, hi - spare))]
            for target in dict.fromkeys(round(t, 3) for t in options):
                grown.append((sequence + (target,), target))
        sequences = grown[:16]
    return [sequence for sequence, _ in sequences]


def _travel_time(detour):
    return float(np.sum(np.diff(detour.s) / np.maximum(detour.speed[:-1], 0.5)))


def _build(reference, stations, d, d_prime, targets, cfg, s_ego):
    base, base_yaw, base_curvature, base_speed = reference.evaluate(stations)
    denominator = base_speed * (1 - base_curvature * d)
    if np.any(denominator <= 0.1):
        return None
    xy = base + np.column_stack((-np.sin(base_yaw), np.cos(base_yaw))) * d[:, None]
    yaw = base_yaw + np.arctan2(d_prime, denominator)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    curvature = np.gradient(np.unwrap(yaw), np.maximum.accumulate(arc + np.arange(len(arc)) * 1e-9))
    # 곡률 속력 → 뒤에서부터 감속 한계로 거슬러 올라옴 (속도 프로파일과 같은 방식)
    speed = np.minimum(np.sqrt(cfg.max_lateral_accel / np.maximum(np.abs(curvature), 1e-6)), cfg.cruise_speed)
    for i in range(len(speed) - 2, -1, -1):
        speed[i] = min(speed[i], math.sqrt(speed[i + 1]**2 + 2 * cfg.decel * (arc[i + 1] - arc[i])))
    return Detour(stations, d, d_prime, xy, yaw, curvature, speed, tuple(targets), 0.0, cfg.margin)


def detour_clearance(detour, station, obs_s, obs_d, cfg):
    """지금 위치(station)부터 우회 경로와 관측점 사이의 남는 여유 (필요 여유를 뺀 값)."""
    keep = detour.s >= station - 0.5
    if np.sum(keep) < 2:
        return 10.0
    need = detour.margin + cfg.surface_padding
    gap = _frenet_clearance(detour.s[keep], detour.d[keep], detour.d_prime[keep], obs_s, obs_d, cfg,
                            float(detour.s[-1]), need + 0.1, wall_relief=WALL_RELIEF)
    return gap - need
