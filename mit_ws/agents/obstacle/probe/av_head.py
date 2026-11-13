"""Static-obstacle avoidance geometry, independent of ROS.

Candidates are quintic d(s) transitions beside a spline reference, followed by
a return after the observed obstacle. A configurable reference corridor is used
because the public course file does not contain surveyed road boundaries.
"""
from dataclasses import dataclass, field, replace
import math

import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline
from scipy.spatial import cKDTree


@dataclass
class Ego:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass
class AvoidanceConfig:
    horizon: float = 30.0
    step: float = 0.2
    road_half_width: float = 3.75
    vehicle_length: float = 4.635
    vehicle_width: float = 1.892
    rear_overhang: float = 0.79
    wheelbase: float = 3.0
    max_steer_deg: float = 35.4
    margin: float = 0.35
    surface_padding: float = 0.25
    max_lateral_accel: float = 1.8
    cruise_speed: float = 5.0
    avoidance_speed: float = 3.0
    offsets: tuple = (-2.2, -1.8, -1.2, 0.0, 1.2, 1.8, 2.2)
    transition_times: tuple = (2.0, 3.0, 4.0)
    return_time: float = 2.5    # 장애물을 지난 뒤 기준 경로로 돌아오는 시간 (s, avoidance_speed 기준 길이)

    def __post_init__(self):
        positive = ('horizon', 'step', 'road_half_width', 'vehicle_length',
                    'vehicle_width', 'wheelbase', 'max_lateral_accel',
                    'cruise_speed', 'avoidance_speed', 'return_time')
        if any(not math.isfinite(getattr(self, k)) or getattr(self, k) <= 0 for k in positive):
            raise ValueError('Avoidance dimensions, speeds and sampling must be positive and finite')
        if not math.isfinite(self.rear_overhang) or not 0 <= self.rear_overhang < self.vehicle_length:
            raise ValueError('Invalid rear overhang')
        if self.step > 0.25:
            raise ValueError('Collision sampling step must not exceed 0.25 m')
        if (not all(math.isfinite(v) for v in (self.margin, self.surface_padding, self.max_steer_deg))
                or self.margin < 0 or self.surface_padding < 0 or not 0 < self.max_steer_deg < 90):
            raise ValueError('Invalid margin or steering limit')
        if not self.offsets or not all(math.isfinite(v) for v in self.offsets):
            raise ValueError('Lateral offsets must be finite and nonempty')
        if not self.transition_times or not all(math.isfinite(v) and v > 0 for v in self.transition_times):
            raise ValueError('Transition times must be positive and finite')


@dataclass
class Candidate:
    s: np.ndarray
    d: np.ndarray
    xy: np.ndarray
    yaw: np.ndarray
    curvature: np.ndarray
    offset: float
    transition: float
    reason: str = 'ok'
    cost: float = math.inf
    d_prime: np.ndarray | None = None
    return_start: float = 0.0
    return_length: float = 0.0


@dataclass
class PlanResult:
    selected: Candidate | None
    candidates: list = field(default_factory=list)
    speed: float = 0.0
    reason: str = 'no_safe_path'


class ReferencePath:
    def __init__(self, points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError('Reference needs at least two finite XY points')
        keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-6]
        self.points = points[keep]
        if len(self.points) < 2:
            raise ValueError('Reference has zero length')
        self.stations = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.length = float(self.stations[-1])
        self.spline = CubicSpline(self.stations, self.points, bc_type='natural')

    def evaluate(self, s):
        s = np.clip(s, 0.0, self.length)
        xy, derivative, second = self.spline(s), self.spline(s, 1), self.spline(s, 2)
        speed = np.maximum(np.linalg.norm(derivative, axis=-1), 1e-6)
        yaw = np.arctan2(derivative[..., 1], derivative[..., 0])
        curvature = (derivative[..., 0] * second[..., 1] - derivative[..., 1] * second[..., 0]) / speed**3
        return xy, yaw, curvature, speed

    def project(self, points, low=0.0, high=None):
        """Local segment projection; bounds prevent jumping to another crossing."""
        points = np.atleast_2d(np.asarray(points, dtype=float))
        high = self.length if high is None else min(high, self.length)
        first = max(0, int(np.searchsorted(self.stations, low)) - 1)
        last = min(len(self.points) - 1, int(np.searchsorted(self.stations, high)) + 1)
        last = max(first + 1, last)
        a, vector = self.points[first:last], np.diff(self.points[first:last + 1], axis=0)
        length2 = np.sum(vector * vector, axis=1)
        delta = points[:, None, :] - a[None, :, :]
        fraction = np.clip(np.sum(delta * vector, axis=2) / length2, 0.0, 1.0)
        projected = a + fraction[..., None] * vector
        indices = np.argmin(np.sum((points[:, None] - projected)**2, axis=2), axis=1)
        s = self.stations[first + indices] + fraction[np.arange(len(points)), indices] * np.sqrt(length2[indices])
        # Refine on the smooth reference rather than retaining a polyline error.
        for _ in range(3):
            delta = self.spline(s) - points
            derivative, second = self.spline(s, 1), self.spline(s, 2)
            denominator = np.sum(derivative**2 + delta * second, axis=1)
            correction = np.sum(delta * derivative, axis=1) / np.maximum(denominator, 1e-6)
            s = np.clip(s - correction, max(0.0, low), high)
        xy, yaw, _, _ = self.evaluate(s)
        difference = points - xy
        d = -np.sin(yaw) * difference[:, 0] + np.cos(yaw) * difference[:, 1]
        return s, d

    def project_near(self, points, initial_s):
        """Footprint corners already have a local station; avoid a global search."""
        points = np.asarray(points, dtype=float)
        initial_s = np.asarray(initial_s, dtype=float)
        s = np.clip(initial_s, 0, self.length)
        for _ in range(3):
            delta = self.spline(s) - points
            derivative, second = self.spline(s, 1), self.spline(s, 2)
            denominator = np.sum(derivative**2 + delta * second, axis=-1)
            correction = np.sum(delta * derivative, axis=-1) / np.maximum(denominator, 0.1)
            s = np.clip(s - correction, np.maximum(0, initial_s - 6), np.minimum(self.length, initial_s + 6))
        xy, yaw, _, _ = self.evaluate(s)
        delta = points - xy
        return -np.sin(yaw) * delta[..., 0] + np.cos(yaw) * delta[..., 1]


def speed_profile(reference, max_speed, lateral_accel, accel, decel, step=0.5, smooth_m=2.0, min_speed=1.0,
                  zones=()):
    """Route speed limit v(s): sqrt(a_lat/|kappa|) capped by max_speed and by zones [(s0, s1, v_max), ...],
    then forward (accel) and backward (decel) passes so the vehicle can actually reach each limit.
    Returns (stations, speeds)."""
    stations = np.arange(0.0, reference.length + step, step)
    _, _, curvature, _ = reference.evaluate(stations)
    # Peak |kappa| in a short window: a spline wiggle between samples must not hide a corner.
    window = max(1, int(round(smooth_m / step)))
    padded = np.pad(np.abs(curvature), window, mode='edge')
    peak = np.max(np.lib.stride_tricks.sliding_window_view(padded, 2 * window + 1), axis=1)
    speed = np.minimum(max_speed, np.sqrt(lateral_accel / np.maximum(peak, 1e-6)))
    for s0, s1, cap in zones:
        speed[(stations >= s0) & (stations <= s1)] = np.minimum(speed[(stations >= s0) & (stations <= s1)], cap)
    speed = np.maximum(speed, min_speed)
    for i in range(1, len(speed)):
        speed[i] = min(speed[i], math.sqrt(speed[i - 1]**2 + 2 * accel * step))
    for i in range(len(speed) - 2, -1, -1):
        speed[i] = min(speed[i], math.sqrt(speed[i + 1]**2 + 2 * decel * step))
    return stations, speed


def jump_distance_clusters(ranges, angle_min, angle_increment, range_min,
                           range_max, jump_base=0.3, jump_scale=1.5,
                           max_range=45.0):
    """Keep beam order and gaps; keep walls and even singleton returns.

    -Inf denotes a too-close return in this simulator, so it becomes a hazard
    at range_min. +Inf denotes no return. Neither is treated as NaN dropout.
    """
    ranges = np.asarray(ranges, dtype=float)
    effective = np.where(np.isneginf(ranges), range_min, ranges)
    valid = np.isfinite(effective) & (effective >= range_min) & (effective <= min(range_max, max_range))
    ids = np.flatnonzero(valid)
    if not len(ids):
        return []
    angles = angle_min + ids * angle_increment
    points = effective[ids, None] * np.column_stack((np.cos(angles), np.sin(angles)))
    thresholds = jump_base + jump_scale * np.minimum(effective[ids[:-1]], effective[ids[1:]]) * abs(angle_increment)
    split = (np.diff(ids) != 1) | (np.linalg.norm(np.diff(points, axis=0), axis=1) > thresholds)
    clusters = list(np.split(points, np.flatnonzero(split) + 1))
    full_circle = abs(angle_increment) * len(ranges) >= 2 * math.pi - 2 * abs(angle_increment)
    if full_circle and len(clusters) > 1 and ids[0] == 0 and ids[-1] == len(ranges) - 1:
        threshold = jump_base + jump_scale * min(effective[0], effective[-1]) * abs(angle_increment)
        if np.linalg.norm(clusters[-1][-1] - clusters[0][0]) <= threshold:
            clusters = [np.vstack((clusters[-1], clusters[0]))] + clusters[1:-1]
    return clusters


def convex_hull(points):
    points = sorted(set(map(tuple, np.asarray(points, dtype=float))))
    if len(points) <= 2:
        return np.asarray(points).reshape(-1, 2)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1])


def obstacle_geometry(clusters):
    """Compact objects use filled hulls; long walls keep their observed contour.

    A hull across an entire curved wall could incorrectly fill the road.
    """
    starts, ends, hulls = [], [], []
    for cluster in clusters:
        points = np.asarray(cluster, dtype=float).reshape(-1, 2)
        if not len(points):
            continue
        if np.max(np.ptp(points, axis=0)) <= 8.0:
            points = convex_hull(points)
            if len(points) >= 3:
                hulls.append(points)
                starts.extend(points)
                ends.extend(np.roll(points, -1, axis=0))
                continue
        if len(points) == 1:
            starts.append(points[0])
            ends.append(points[0])
        else:
            starts.extend(points[:-1])
            ends.extend(points[1:])
    starts, ends = np.asarray(starts).reshape(-1, 2), np.asarray(ends).reshape(-1, 2)
    tree = cKDTree((starts + ends) / 2) if len(starts) else None
    half_lengths = np.linalg.norm(ends - starts, axis=1) / 2
    return starts, ends, hulls, tree, half_lengths


def footprint_corners(xy, yaw, cfg, padding=0.0):
    rear = -cfg.rear_overhang - padding
    front = cfg.vehicle_length - cfg.rear_overhang + padding
    half = cfg.vehicle_width / 2 + padding
    local = np.array([[rear, -half], [front, -half], [front, half], [rear, half]])
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotations = np.stack((np.stack((cosine, -sine), axis=-1), np.stack((sine, cosine), axis=-1)), axis=-2)
    return np.asarray(xy)[:, None, :] + np.einsum('nij,kj->nki', rotations, local)


def collision_and_clearance(xy, yaw, geometry, cfg):
    """Clip every observed contour edge against every oriented car rectangle."""
    starts, ends, hulls, tree, half_lengths = geometry
    if not len(starts):
        return False, 10.0
    # Half a sample covers translation between checks. Rotation is covered by
    # the corner travel between adjacent yaw samples, in addition to the margin.
    spacing = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    delta_yaw = np.abs(np.diff(np.unwrap(yaw)))
    radius = math.hypot(cfg.vehicle_length - cfg.rear_overhang, cfg.vehicle_width / 2)
    rotation = 0.5 * radius * np.max(delta_yaw, initial=0.0)
    swept = 0.5 * np.max(spacing, initial=0.0) + rotation
    padding = cfg.margin + cfg.surface_padding + swept
    # 표본 사이 이동은 차 진행 방향이라 옆면은 자기 자신을 따라 쓸고 지나갑니다. 옆 여유에는
    # 회전 항만 더합니다 (이동 항 0.1 m까지 더하면 좁은 틈에서 현재 자세부터 충돌로 판정되어 멈춤).
    side = cfg.margin + cfg.surface_padding + rotation
    lower = np.array([-cfg.rear_overhang - padding, -cfg.vehicle_width / 2 - side])
    upper = np.array([cfg.vehicle_length - cfg.rear_overhang + padding, cfg.vehicle_width / 2 + side])
    cosine, sine = np.cos(yaw), np.sin(yaw)
    body_center = xy + (cfg.vehicle_length / 2 - cfg.rear_overhang) * np.column_stack((cosine, sine))
    body_radius = math.hypot(cfg.vehicle_length / 2, cfg.vehicle_width / 2) + padding
    neighbors = tree.query_ball_point(body_center, body_radius + float(np.max(half_lengths)) + 1.0)
    counts = np.fromiter((len(v) for v in neighbors), dtype=int, count=len(xy))
    pose_ids = np.repeat(np.arange(len(xy)), counts)
    edge_ids = np.concatenate(neighbors).astype(int) if len(pose_ids) else np.empty(0, dtype=int)
    if len(pose_ids):
        midpoints = (starts[edge_ids] + ends[edge_ids]) / 2
        relevant = np.linalg.norm(midpoints - body_center[pose_ids], axis=1) <= body_radius + half_lengths[edge_ids] + 1.0
        pose_ids, edge_ids = pose_ids[relevant], edge_ids[relevant]
    cosine, sine = cosine[pose_ids], sine[pose_ids]

    def local(points):
        delta = points[edge_ids] - xy[pose_ids]
        return np.stack((cosine * delta[..., 0] + sine * delta[..., 1],
                         -sine * delta[..., 0] + cosine * delta[..., 1]), axis=-1)

    a, b = local(starts), local(ends)
    vector = b - a
    parallel = np.abs(vector) < 1e-9
    outside_parallel = np.any(parallel & ((a < lower) | (a > upper)), axis=-1)
    safe_vector = np.where(parallel, 1.0, vector)
    t1, t2 = (lower - a) / safe_vector, (upper - a) / safe_vector
    enter = np.max(np.where(parallel, -np.inf, np.minimum(t1, t2)), axis=-1)
    leave = np.min(np.where(parallel, np.inf, np.maximum(t1, t2)), axis=-1)
    if np.any(~outside_parallel & (np.maximum(enter, 0.0) <= np.minimum(leave, 1.0))):
        return True, 0.0
    # An entire vehicle can be inside a large compact object without touching
    # an observed edge, so also test containment in each convex hull.
    for hull in hulls:
        edge = np.roll(hull, -1, axis=0) - hull
        delta = xy[:, None, :] - hull
        cross = edge[None, :, 0] * delta[..., 1] - edge[None, :, 1] * delta[..., 0]
        if np.any(np.all(cross >= -1e-9, axis=1)):
            return True, 0.0
    distances = np.maximum(np.maximum(lower - a, a - upper), 0.0)
    return False, float(np.min(np.linalg.norm(distances, axis=-1), initial=10.0))


def _quintic(start, slope, end, length, station):
    """d, d' for zero endpoint d'' and zero final d'; station may be an array."""
    a0, a1 = start, slope * length
    remaining = end - a0 - a1
    matrix = np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]])
    a3, a4, a5 = np.linalg.solve(matrix, [remaining, -a1, 0.0])
    u = np.clip(station / length, 0.0, 1.0)
    d = a0 + a1 * u + a3 * u**3 + a4 * u**4 + a5 * u**5
    derivative = (a1 + 3 * a3 * u**2 + 4 * a4 * u**3 + 5 * a5 * u**4) / length
    return d, derivative


