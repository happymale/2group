import pickle, sys, csv, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A
d = pickle.load(open(sys.argv[1], 'rb')); xs = float(sys.argv[2]); ce = float(sys.argv[3])
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route)
best = min(d['clouds'], key=lambda c: abs(c[1][0] - xs) if c[1] else 1e9)
st, od, pts, ids = best
P = pts.astype(float)
s, dd = ref.project(P, ref.project([[od[0], od[1]]])[0][0] - 5, ref.project([[od[0], od[1]]])[0][0] + 40)
m = (np.abs(dd) < 3.6) & (s > ref.project([[od[0], od[1]]])[0][0] - 2) & (s < ref.project([[od[0], od[1]]])[0][0] + 12)
order = np.argsort(s[m])
print('ego', od)
for a, b in zip(s[m][order], dd[m][order]):
    print(f'{a:6.2f} {b:+.2f}', end=' | ')
print()
cfg = A.AvoidanceConfig(horizon=30, step=0.2, road_half_width=3.75, margin=0.2, surface_padding=0.15, max_lateral_accel=3.0,
                      cruise_speed=10, avoidance_speed=10, offsets=(-2.3, -2.0, -1.6, -1.0, 0.0, 1.0, 1.6, 2.0, 2.3),
                      transition_times=(1.2, 2.0, 3.0))
clusters = [P[ids == k] for k in np.unique(ids)]
r = A._plan_avoidance(ref, A.Ego(*od), clusters, A.replace(cfg, margin=0.0), None, None, ce)
for c in r.candidates:
    fc = A._frenet_clearance(c.s, c.d, c.d_prime, s[np.abs(dd)<6.5], dd[np.abs(dd)<6.5], cfg, ce)
    print(f'{c.offset:+.2f} T={c.transition} {c.reason} approx_clear={fc:.2f} d={np.round(c.d[::20],2)}')
