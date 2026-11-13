import pickle, sys, csv, time, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A
d = pickle.load(open(sys.argv[1], 'rb')); xs = float(sys.argv[2]); ce = float(sys.argv[3]); zone = (float(sys.argv[4]), float(sys.argv[5]))
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route)
cands = sorted(d["clouds"], key=lambda c: abs(c[1][0] - xs) if c[1] else 1e9)
best = cands[int(sys.argv[6]) if len(sys.argv) > 6 else 0]
st, od, pts, ids = best
clusters = [pts[ids == k].astype(float) for k in np.unique(ids)]
cfg = A.AvoidanceConfig(horizon=30, step=0.2, road_half_width=3.75, margin=float(__import__("os").environ.get("MARGIN", 0.2)), surface_padding=float(__import__("os").environ.get("PAD", 0.15)), max_lateral_accel=3.0,
                      cruise_speed=10, avoidance_speed=10, offsets=(-2.3, -2.0, -1.6, -1.0, 0.0, 1.0, 1.6, 2.0, 2.3),
                      transition_times=(1.2, 2.0, 3.0))
t = time.perf_counter()
r = A._plan_avoidance(ref, A.Ego(*od), clusters, cfg, None, None, ce, zone)
print('ego', np.round(od, 2), r.reason, None if r.selected is None else r.selected.offset, f'{1000*(time.perf_counter()-t):.0f} ms')
s0 = r.candidates[0].s[0]
# 근처 군집 기하
geo = A.obstacle_geometry(clusters)
for c in sorted(r.candidates, key=lambda c: c.cost)[:int(__import__("os").environ.get("N", 8))]:
    msg = ''
    if c.reason == 'collision' and np.isfinite(c.cost):
        for i in range(0, len(c.xy) - 1):
            if c.s[i] > ce: break
            col, _ = A.collision_and_clearance(c.xy[i:i+2], c.yaw[i:i+2], geo, cfg)
            if col:
                msg = f'first exact hit at pose {c.xy[i].round(2)} d={c.d[i]:.2f}'; break
    print(f'  off={c.offset:+.2f} T={c.transition} {c.reason} cost={c.cost:.2f} {msg} d={np.round(c.d[::10],2)}')
for k, cl in enumerate(clusters):
    if np.ptp(cl, axis=0).max() <= 8 and len(cl) > 2:
        cs, cd = ref.project(cl, s0 - 5, s0 + 40)
        if np.abs(cd).min() < 3.4 and cs.max() > s0:
            print('  hull cluster', k, 'x', cl[:, 0].min().round(1), cl[:, 0].max().round(1), 'y', cl[:, 1].min().round(2), cl[:, 1].max().round(2), len(cl))
s0 = r.candidates[0].s[0]; end = r.candidates[0].s[-1]; limit = min(end, ce)
P = pts.astype(float)
ss, dd = ref.project(P, max(0, s0 - 5), end + 5)
xy_, _, _, _ = ref.evaluate(ss)
near = (np.linalg.norm(P - xy_, axis=1) <= 6.75) & (ss >= s0 - 4.635) & (ss <= limit + 4.635)
ns, nd = ss[near], dd[near]
need = cfg.margin + cfg.surface_padding + 0.1
for c in [c for c in r.candidates if c.d_prime is not None][:int(__import__('os').environ.get('K',3))]:
    u = np.linspace(-0.79 - need, 3.845 + need, 11)
    ps = ns[:, None] - u[None]
    valid = (ps >= s0) & (ps <= limit)
    h = np.arctan(np.interp(ps, c.s, c.d_prime)); cc = np.interp(ps, c.s, c.d) + u * np.sin(h)
    gap = np.where(valid, np.abs(nd[:, None] - cc) - 0.946 * np.cos(h), 99)
    k = np.unravel_index(np.argmin(gap), gap.shape)
    print(f'cand {c.offset:+.2f}: min gap {gap[k]:.2f} at point s={ns[k[0]]:.2f} d={nd[k[0]]:.2f} u={u[k[1]]:.2f} pose_s={ps[k]:.2f}')
