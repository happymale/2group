def _path_speed(candidate, s0, cfg):
    """경로 위 각 자세의 곡률 속력 sqrt(a/κ)을 감속 한계로 거슬러 올라온, 지금 낼 수 있는 최고 속력.
    멀리 있는 급한 곡률은 지금 속력을 덜 제한하므로 고정 회피 속력보다 빠르게 다가갈 수 있습니다."""
    curvature = np.abs(candidate.curvature)
    limit = np.minimum(np.sqrt(cfg.max_lateral_accel / np.maximum(curvature, 1e-6)), cfg.cruise_speed)
    reachable = np.sqrt(limit**2 + 2.0 * cfg.decel * np.maximum(candidate.s - s0, 0.0))
    return float(min(cfg.cruise_speed, np.min(reachable)))


def _selected_result(selected, candidates, cfg, s0=None):
    s0 = float(selected.s[0]) if s0 is None else s0
    speed = _path_speed(selected, s0, cfg)
    if np.max(np.abs(selected.d)) > 0.3:
        speed = min(speed, cfg.avoidance_speed)
    return PlanResult(selected, candidates, speed, 'ok')


def plan_avoidance(reference, ego, clusters, cfg, previous=None, station_hint=None, check_end=None):
    """check_end: 이 경로 거리(m)까지만 관측물·도로 폭 검사를 하고, 그 전에 기준 경로로 돌아온 후보만 씁니다
    (장애물 구간 뒤의 좁은 곡선 통로는 회피 대상이 아님. None이면 시야 전체 검사)."""
    result = _plan_avoidance(reference, ego, clusters, cfg, previous, station_hint, check_end)
    if result.selected is None and result.reason == 'no_safe_path' and cfg.margin > 0:
        # 좁은 틈에서 추종 오차로 현재 자세가 여유(margin) 안에 들어가면 모든 후보가 첫 자세부터
        # 충돌이라 영원히 멈춥니다. 관측 불확실성 여유(surface_padding)만 남기고 다시 찾되 천천히 갑니다.
        tight = replace(cfg, margin=0.0)
        retry = _plan_avoidance(reference, ego, clusters, tight, previous, station_hint, check_end)
        if retry.selected is not None:
            retry.speed = min(retry.speed, cfg.tight_speed)
            retry.reason = 'tight'
            return retry
    return result


def _lateral_profile(stations, d0, slope, segments):
    """segments [(시작 s, 길이, 목표 d), ...]를 차례로 잇는 quintic d(s), d'(s).
    첫 구간은 현재 횡위치·기울기에서, 나머지는 기울기 0에서 시작합니다."""
    d = np.full(len(stations), float(d0))
    derivative = np.zeros(len(stations))
    current, start_slope, previous_end = float(d0), slope, -math.inf
    for start, length, target in segments:
        start = max(start, previous_end)
        mask = stations >= start
        if np.any(mask):
            u = stations[mask] - start
            value, rate = _quintic(current, start_slope, target, length, u)
            done = u >= length
            value[done], rate[done] = target, 0.0
            d[mask], derivative[mask] = value, rate
        current, start_slope, previous_end = target, 0.0, start + length
    return d, derivative


def _frenet_clearance(stations, d, derivative, obs_s, obs_d, cfg, s_limit):
    """Frenet 근사 여유: 관측점 옆을 차체의 어느 길이 위치(u)가 지나는 자세의 d와 방향으로 옆 거리를 잽니다.
    정확 검사(collision_and_clearance)보다 수십 배 싸서 모든 후보를 거르는 데 쓰고, 고른 후보만 정확 검사합니다."""
    if not len(obs_s):
        return 10.0
    u = np.linspace(-cfg.rear_overhang, cfg.vehicle_length - cfg.rear_overhang, 9)
    pose_s = obs_s[:, None] - u[None, :]
    valid = (pose_s >= stations[0]) & (pose_s <= min(s_limit, stations[-1]))
    if not np.any(valid):
        return 10.0
    pose_s = pose_s[valid]
    along = np.broadcast_to(u[None, :], valid.shape)[valid]
    lateral = np.broadcast_to(obs_d[:, None], valid.shape)[valid]
    heading = np.arctan(np.interp(pose_s, stations, derivative))
    center = np.interp(pose_s, stations, d) + along * np.sin(heading)
    gap = np.abs(lateral - center) - 0.5 * cfg.vehicle_width * np.cos(heading)
    return float(np.min(gap))


def _gap_offsets(obs_s, obs_d, low, high, cfg, need):
    """[low, high] 구간을 지날 때 차체가 들어가는 빈 횡간격마다 목표 d 하나 (넓은 간격 먼저).
    넓은 간격은 기준 경로(0)에 가장 가까운 곳, 좁은 간격은 가운데."""
    inside = (obs_s >= low) & (obs_s <= high) & (np.abs(obs_d) < cfg.road_half_width)
    walls = np.sort(np.r_[obs_d[inside], -cfg.road_half_width, cfg.road_half_width])
    half = 0.5 * cfg.vehicle_width + need
    gaps = []
    for a, b in zip(walls[:-1], walls[1:]):
        if b - a < 2 * half:
            continue
        lo, hi = a + half, b - half
        spare = min(cfg.gap_extra, 0.5 * (hi - lo))
        gaps.append((b - a, float(np.clip(0.0, lo + spare, hi - spare))))
    return [target for _, target in sorted(gaps, reverse=True)]


def _plan_avoidance(reference, ego, clusters, cfg, previous=None, station_hint=None, check_end=None):
    if not np.isfinite([ego.x, ego.y, ego.yaw, ego.speed]).all() or ego.speed < -0.1:
        return PlanResult(None, reason='invalid_ego')
    low = max(0.0, station_hint - 3.0) if station_hint is not None else 0.0
    high = station_hint + 45.0 if station_hint is not None else reference.length
    s0, d0 = reference.project([[ego.x, ego.y]], low, high)
    s0, d0 = float(s0[0]), float(d0[0])
    end = min(s0 + cfg.horizon, reference.length)
    if end - s0 < 1.0:
        return PlanResult(None, reason='route_end')
    limit = end if check_end is None else max(min(end, check_end), s0 + cfg.step)
    stations = np.linspace(s0, end, int(math.ceil((end - s0) / cfg.step)) + 1)
    checked = stations <= limit + 1e-9
    base, base_yaw, base_curvature, base_speed = reference.evaluate(stations)
    heading_error = math.atan2(math.sin(ego.yaw - base_yaw[0]), math.cos(ego.yaw - base_yaw[0]))
    if abs(heading_error) > math.radians(65):
        return PlanResult(None, reason='heading_mismatch')
    slope = math.tan(heading_error) * base_speed[0] * (1 - base_curvature[0] * d0)
    front = cfg.vehicle_length - cfg.rear_overhang
    need = cfg.margin + cfg.surface_padding
    all_points = np.vstack(clusters) if clusters else np.empty((0, 2))
    near_s, near_d = np.empty(0), np.empty(0)
    groups = []
    if len(all_points):
        obs_s, obs_d = reference.project(all_points, max(0, s0 - 5), end + 5)
        # 충돌 검사는 후보 경로 근처 관측점만 씁니다 (45 m 안의 모든 벽 점은 계획 한 번에
        # 0.2 s 이상 걸려 odom·장애물 시간 초과를 부름). 잘린 곳에서 군집을 나눠 가짜 변을 만들지 않습니다.
        projected, _, _, _ = reference.evaluate(obs_s)
        near = (np.linalg.norm(all_points - projected, axis=1) <= cfg.road_half_width + 3.0) & (
            obs_s >= s0 - cfg.vehicle_length) & (obs_s <= limit + cfg.vehicle_length)
        near_s, near_d = obs_s[near], obs_d[near]
        kept, start = [], 0
        for cluster in clusters:
            mask = near[start:start + len(cluster)]
            start += len(cluster)
            edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
            kept += [np.asarray(cluster)[a:b] for a, b in zip(edges[::2], edges[1::2])]
        clusters = kept
        # 기준 경로를 막는 관측점을 경로 거리로 묶어 장애물 무리(group)로 만듭니다. 무리 사이가
        # 차 길이 + 2 m보다 가까우면 그 사이에서 옆을 바꿀 수 없으므로 한 무리로 봅니다.
        # 시야 끝(end) 바로 뒤의 장애물도 마지막 자세의 차체 앞부분과 겹치므로 포함합니다.
        blocking = (near_s > s0 - cfg.rear_overhang) & (near_s < limit + front) & (
            np.abs(near_d) < 0.5 * cfg.vehicle_width + need)
        if np.any(blocking):
            ordered = np.sort(near_s[blocking])
            cuts = np.flatnonzero(np.diff(ordered) > cfg.vehicle_length + 2.0)
            groups = [(float(a[0]), float(a[-1])) for a in np.split(ordered, cuts + 1)][:3]
    hold_until = groups[-1][1] + cfg.rear_overhang + need + cfg.return_gap if groups else s0
    geometry = obstacle_geometry(clusters)
    # 회피 길이는 지금 속력이 아니라 낼 수 있는 속력(avoidance_speed) 기준: 느릴 때 짧게 꺾으면
    # 곡률이 커져 다시 느려지는 악순환을 막습니다. 공간이 모자라면 아래에서 줄입니다.
    v_ref = max(ego.speed, cfg.avoidance_speed, 3.0)

    def return_segment(start):
        length = max(8.0, v_ref * cfg.return_time)
        if check_end is not None:
            length = min(length, max(cfg.min_transition, check_end - start - 0.5))
        return (start, length, 0.0)

    specifications = []           # (segments, 대표 offset, 전환 시간, 이어가기 여부)
    if previous is not None and previous.d_prime is not None:
        specifications.append((None, previous.offset, previous.transition, True))
    if not groups:
        for duration in sorted(cfg.transition_times, reverse=True):
            specifications.append(([return_segment(s0)[:1] + (max(cfg.min_transition, v_ref * duration), 0.0)]
                                   if check_end is None else [(s0, min(max(cfg.min_transition, v_ref * duration),
                                                                      return_segment(s0)[1]), 0.0)],
                                   0.0, duration, False))
    else:
        # 1) 무리마다 빈 간격의 목표 d (넓은 간격 2개까지)를 골라 이어 붙인 후보: 한 번 옆을 정하면
        #    장애물을 다 지날 때까지 유지하고, 좌우가 바뀌는 무리 사이에서만 옆을 바꿉니다.
        options = []
        for g0, g1 in groups:
            offsets = _gap_offsets(near_s, near_d, g0 - 1.0, g1 + 1.0, cfg, need)[:2]
            options.append(offsets or [None])
        first_room = groups[0][0] - front - 0.3 - s0
        lengths = sorted({round(max(cfg.min_transition, min(v_ref * t, max(first_room, cfg.min_transition))), 1)
                          for t in cfg.transition_times})
        for combination in itertools.product(*options):
            if None in combination:
                continue
            for length in lengths:
                segments = [(s0, length, combination[0])]
                for k in range(1, len(groups)):
                    if abs(combination[k] - combination[k - 1]) < 1e-6:
                        continue
                    start = groups[k - 1][1] + cfg.rear_overhang + 0.3
                    room = groups[k][0] - front - 0.3 - start
                    segments.append((start, max(cfg.min_transition, min(v_ref * 1.5, room)), combination[k]))
                segments.append(return_segment(max(hold_until, segments[-1][0] + segments[-1][1])))
                specifications.append((segments, combination[0], round(length / v_ref, 2), False))
        # 2) 고정 격자 offset 후보 (간격 계산이 놓친 경우 대비)
        for offset in cfg.offsets:
            for duration in cfg.transition_times:
                length = max(cfg.min_transition, v_ref * duration)
                specifications.append(([(s0, length, offset), return_segment(max(hold_until, s0 + length + 1.0))],
                                       offset, duration, False))
    candidates = []
    radius = math.hypot(front, cfg.vehicle_width / 2)
    distance = stations - s0
    for segments, offset, duration, continuation in specifications:
        if continuation:
            # Receding-horizon replanning must retain a feasible maneuver,
            # not move its lateral destination farther away each cycle.
            old = CubicHermiteSpline(previous.s, previous.d, previous.d_prime)
            covered = stations <= previous.s[-1]
            d, derivative = np.zeros(len(stations)), np.zeros(len(stations))
            d[covered], derivative[covered] = old(stations[covered]), old(stations[covered], 1)
            return_start, return_length = previous.return_start, previous.return_length
            extension = ~covered
            d[extension], derivative[extension] = offset, 0.0
            returning = extension & (stations > return_start)
            if np.any(returning):
                d[returning], derivative[returning] = _quintic(
                    offset, 0.0, 0.0, return_length, stations[returning] - return_start)
            finished = extension & (stations >= return_start + return_length)
            d[finished], derivative[finished] = 0.0, 0.0
            # Smoothly join measured ego position/heading to the retained
            # trajectory instead of assuming perfect path tracking.
            correction_length = max(5.0, ego.speed)
            correction, correction_prime = _quintic(d0 - d[0], slope - derivative[0],
                                                    0.0, correction_length, distance)
            correction[distance >= correction_length] = 0.0
            correction_prime[distance >= correction_length] = 0.0
            d += correction
            derivative += correction_prime
        else:
            d, derivative = _lateral_profile(stations, d0, slope, segments)
            return_start, return_length = segments[-1][0], segments[-1][1]
            if abs(segments[-1][2]) > 1e-9:
                return_start, return_length = math.inf, 8.0
        xy = base + np.column_stack((-np.sin(base_yaw), np.cos(base_yaw))) * d[:, None]
        denominator = base_speed * (1 - base_curvature * d)
        yaw = base_yaw + np.arctan2(derivative, denominator)
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        curvature = np.gradient(np.unwrap(yaw), np.maximum.accumulate(arc + np.arange(len(arc)) * 1e-9))
        candidate = Candidate(stations, d, xy, yaw, curvature, offset, duration)
        candidate.d_prime = derivative
        candidate.return_start, candidate.return_length = return_start, return_length
        candidates.append(candidate)
        if np.any(denominator <= 0.1) or not np.isfinite(xy).all():
            candidate.reason = 'invalid_geometry'
            continue
        if np.max(np.abs(curvature)) > math.tan(math.radians(cfg.max_steer_deg)) / cfg.wheelbase:
            candidate.reason = 'curvature'
            continue
        if check_end is not None and np.any(np.abs(d[stations >= check_end]) > cfg.exit_tolerance):
            candidate.reason = 'zone_exit'          # 구간 끝(좁은 통로) 전에 기준 경로로 돌아오지 못함
            continue
        clearance = _frenet_clearance(stations, d, derivative, near_s, near_d, cfg, limit)
        if clearance < need - 0.02:
            candidate.reason = 'collision'
            continue
        continuity = 0.0
        if previous is not None:
            overlap = (stations >= previous.s[0]) & (stations <= previous.s[-1])
            if np.any(overlap):
                old_d = np.interp(stations[overlap], previous.s, previous.d)
                continuity = 1.5 * float(np.mean((d[overlap] - old_d)**2))
            if previous.offset * offset < 0:
                continuity += 2.0
        candidate.speed = _path_speed(candidate, s0, cfg)
        candidate.cost = (float(np.mean(d**2)) + 15.0 * float(np.mean(curvature**2))
                          + 0.3 / (max(clearance - need, 0.0) + 0.2) + continuity
                          + cfg.speed_weight * (1.0 - candidate.speed / cfg.cruise_speed))
        if continuation:
            # Validate the retained maneuver first. If it is still collision-free,
            # keep it instead of spending the sensor deadline rebuilding every alternative.
            if _exact_check(candidate, reference, geometry, cfg, checked, radius):
                return _selected_result(candidate, candidates, cfg, s0)
            candidate.cost = math.inf
    feasible = sorted((c for c in candidates if c.reason == 'ok' and math.isfinite(c.cost)), key=lambda c: c.cost)
    for candidate in feasible[:cfg.max_exact_checks]:
        if _exact_check(candidate, reference, geometry, cfg, checked, radius):
            return _selected_result(candidate, candidates, cfg, s0)
    return PlanResult(None, candidates)


def _exact_check(candidate, reference, geometry, cfg, checked, radius):
    """차체 사각형과 관측 윤곽의 정확한 교차 검사와 설정 도로 폭 검사 (check_end까지의 자세만)."""
    xy, yaw, stations = candidate.xy[checked], candidate.yaw[checked], candidate.s[checked]
    road_padding = cfg.margin + 0.5 * float(np.max(np.abs(candidate.curvature))) * cfg.step**2 * (1 + radius)
    corners = footprint_corners(xy, yaw, cfg, road_padding)
    corner_d = reference.project_near(corners.reshape(-1, 2), np.repeat(stations, 4))
    if np.any(np.abs(corner_d) > cfg.road_half_width):
        candidate.reason = 'road_boundary'
        return False
    collision, _ = collision_and_clearance(xy, yaw, geometry, cfg)
    if collision:
        candidate.reason = 'collision'
        return False
    return True
