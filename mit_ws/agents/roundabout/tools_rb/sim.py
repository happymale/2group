"""오프라인 폐루프 모의: 운동학 자전거 + control.py Stanley(지연 예측 포함) → lap_logger 차체 횡거리 지표."""
import csv, math, sys, argparse
from collections import deque
import numpy as np
sys.path.insert(0, '/mit_ws/agents/roundabout/src/hk_mit')
from hk_mit.roundabout import ring_reference
from hk_mit.avoidance import ReferencePath, speed_profile

ap = argparse.ArgumentParser()
ap.add_argument('--re', type=float, default=6.0); ap.add_argument('--rx', type=float, default=6.0)
ap.add_argument('--ring', type=float, default=11.0)
ap.add_argument('--lat', type=float, default=0.35); ap.add_argument('--delay', type=float, default=0.0)
ap.add_argument('--vmax', type=float, default=10.0); ap.add_argument('--alat', type=float, default=4.0)
ap.add_argument('--vcap', type=float, default=99.0); ap.add_argument('--k', type=float, default=1.0)
ap.add_argument("--wb", type=float, default=3.0); ap.add_argument("--sm", type=float, default=2.5); ap.add_argument("--rate", type=float, default=999); ap.add_argument('--orig', action='store_true'); ap.add_argument('--trace', action='store_true')
a = ap.parse_args()
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/roundabout/src/hk_mit/course/route.csv'))])
C = (108.8353, 118.7754)
if a.orig:
    ref, geo = route, None
else:
    ref, rear, geo = ring_reference(route, C, 11.0, a.ring, a.re, a.rx, wheelbase=a.wb, smooth_m=a.sm)
rp = ReferencePath(ref)
if a.orig: rear = ref
rr = ReferencePath(rear)
ps, pv = speed_profile(rr, a.vmax, a.alat, 2.0, 3.0)
# 뒤차축 경로 거리 → 제어 경로 거리 (가장 가까운 점 짝)
pair = np.array([rp.stations[int(np.argmin(np.linalg.norm(ref - q, axis=1)))] for q in rr.points])
ps = np.interp(ps, rr.stations, np.maximum.accumulate(pair))
if geo and a.vcap < 99:
    pv[(ps > 318) & (ps < 352)] = np.minimum(pv[(ps > 318) & (ps < 352)], a.vcap)
yawr = np.r_[np.arctan2(np.diff(ref[:, 1]), np.diff(ref[:, 0])), 0]; yawr[-1] = yawr[-2]
# lap_logger 지표
seg = np.diff(route, axis=0); sl = np.linalg.norm(seg, axis=1); sd = seg / sl[:, None]; st = np.r_[0, np.cumsum(sl)]
def off(p, near):
    lo, hi = max(0, near - 20), min(len(sl), near + 20)
    dlt = p - route[lo:hi]; al = np.clip(np.sum(dlt * sd[lo:hi], axis=1), 0, sl[lo:hi])
    return float(np.min(np.linalg.norm(p - (route[lo:hi] + al[:, None] * sd[lo:hi]), axis=1)))
def body(x, y, yaw):
    i = int(np.argmin(np.linalg.norm(route - (x, y), axis=1)))
    c, s = math.cos(yaw), math.sin(yaw)
    return max((off(np.array([x + lx * c - ly * s, y + lx * s + ly * c]), i), lx, ly) for lx in (-0.79, 3.845) for ly in (-0.946, 0.946)), st[i]
L = 3.0; MS = math.radians(35.4); dt = 0.02
i0 = int(np.argmin(np.abs(st - 300)))
x, y = route[i0]; yaw = math.atan2(*(route[i0 + 1] - route[i0])[::-1]); v = 4.3
hist = deque(maxlen=max(1, round(a.lat / dt))); last = 0.0
act = deque([0.0] * max(1, round(a.delay / dt) + 1), maxlen=max(1, round(a.delay / dt) + 1))
prev_applied = 0.0
ri = int(np.argmin(np.linalg.norm(ref - route[i0], axis=1))); worst = (0, 0); maxerr = 0; t = 0
def ang(a_): return (a_ + math.pi) % (2 * math.pi) - math.pi
while True:
    # planner: route.nearest 다음 path 창
    ri = min(range(ri, min(ri + 80, len(ref))), key=lambda k: math.hypot(ref[k][0] - x, ref[k][1] - y))
    path = [(ref[k][0], ref[k][1], yawr[k]) for k in range(max(0, ri - 4), min(len(ref), ri + 61))]
    sstat = rp.stations[ri]
    if sstat > 365: break
    lo_, hi_ = np.searchsorted(ps, [sstat, sstat + v * 0.6 + 1]); target = float(np.min(pv[lo_:max(hi_, lo_ + 1)]))
    # control
    px_, py_, yw = x, y, yaw
    h = list(hist) or [last]; d_ = a.lat / len(h)
    for pst in h:
        px_ += v * math.cos(yw) * d_; py_ += v * math.sin(yw) * d_; yw += v * math.tan(pst * MS) / L * d_
    fx, fy = px_ + L * math.cos(yw), py_ + L * math.sin(yw)
    k = min(range(len(path)), key=lambda n: math.hypot(path[n][0] - fx, path[n][1] - fy))
    qx, qy, pyaw = path[k]
    he = ang(pyaw - yw); ce = -math.sin(pyaw) * (qx - fx) + math.cos(pyaw) * (qy - fy)
    steer = max(-1, min(1, (he + math.atan2(a.k * ce, 1.0 + v)) / MS))
    last = steer; hist.append(steer); act.append(steer)
    applied = act[0] if a.rate > 900 else max(prev_applied - a.rate * dt, min(prev_applied + a.rate * dt, act[0])); prev_applied = applied
    v += max(-8.5, min(4.0, 2.0 * (target - v))) * dt
    x += v * math.cos(yaw) * dt; y += v * math.sin(yaw) * dt; yaw += v * math.tan(applied * MS) / L * dt; t += dt
    (b, blx, bly), s_ = body(x, y, yaw)
    if b > worst[0]: worst = (b, s_)
    if a.trace and round(t / dt) % 10 == 0 and 315 < s_ < 352: print(f'{t:5.2f} s={s_:6.1f} v={v:4.2f} body={b:4.2f}({blx:+.1f},{bly:+.1f}) xy=({x:.1f},{y:.1f}) hd={math.degrees(yaw):.0f} steer={applied:+.2f}')
print(f're={a.re} rx={a.rx} ring={a.ring} lat={a.lat} delay={a.delay} vcap={a.vcap}: worst body {worst[0]:.2f} at s={worst[1]:.1f} → margin {2.196 - worst[0]:+.2f}, time300-365 {t:.1f}s',
      f'geo={geo}' if a.trace else '')
