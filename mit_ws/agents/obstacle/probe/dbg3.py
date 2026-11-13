import pickle, sys, csv, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A
d = pickle.load(open(sys.argv[1], 'rb')); xs = float(sys.argv[2]); ce = float(sys.argv[3])
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route)
best = min(d['clouds'], key=lambda c: abs(c[1][0] - xs) if c[1] else 1e9)
st, od, pts, ids = best
P = pts.astype(float)
s0 = float(ref.project([[od[0], od[1]]])[0][0]); end = s0 + 30; limit = min(end, ce)
s, dd = ref.project(P, max(0, s0 - 5), end + 5)
xy, _, _, _ = ref.evaluate(s)
near = (np.linalg.norm(P - xy, axis=1) <= 3.75 + 3) & (s >= s0 - 4.635) & (s <= limit + 4.635)
ns, nd = s[near], dd[near]
cfg = A.AvoidanceConfig(horizon=30, step=0.2, road_half_width=3.75, margin=0.2, surface_padding=0.15, max_lateral_accel=3.0,
                      cruise_speed=10, avoidance_speed=10, transition_times=(1.2, 2.0, 3.0))
st_ = np.linspace(s0, end, 151)
for off in (0.0, -2.2):
    dprof, dp = A._lateral_profile(st_, 0.0, 0.0, [(s0, 8.0, off), (440, 10, 0.0)])
    u = np.linspace(-0.79, 3.845, 9)
    ps = ns[:, None] - u[None]
    valid = (ps >= s0) & (ps <= limit)
    h = np.arctan(np.interp(ps, st_, dp)); c = np.interp(ps, st_, dprof) + u * np.sin(h)
    gap = np.where(valid, np.abs(nd[:, None] - c) - 0.946 * np.cos(h), 99)
    k = np.unravel_index(np.argmin(gap), gap.shape)
    print(off, 'min gap', gap[k].round(2), 'point s,d', ns[k[0]].round(2), nd[k[0]].round(2), 'u', u[k[1]].round(2))
    bad = np.flatnonzero(gap.min(axis=1) < 0.35)
    print('  colliding points', [(round(ns[i],1), round(nd[i],2)) for i in bad[:15]])
need = 0.35
bl = (ns > s0 - 0.79) & (ns < limit + 3.845) & (np.abs(nd) < 0.946 + need)
o = np.sort(ns[bl]); cuts = np.flatnonzero(np.diff(o) > 6.635)
groups = [(a[0], a[-1]) for a in np.split(o, cuts + 1)]
print('groups', groups)
for g0, g1 in groups:
    print('  gaps', A._gap_offsets(ns, nd, g0 - 1, g1 + 1, cfg, need))
r = A._plan_avoidance(ref, A.Ego(*od), [P[ids == k] for k in np.unique(ids)], cfg, None, None, ce)
for c in r.candidates[:12]:
    print(round(c.offset, 2), c.transition, c.reason, np.round(c.d[::10], 2))
