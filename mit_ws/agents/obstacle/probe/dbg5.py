import pickle, sys, csv, time, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A
d = pickle.load(open(sys.argv[1], 'rb')); xs = [float(v) for v in sys.argv[2].split(',')]; ce = float(sys.argv[3]); zone = (float(sys.argv[4]), float(sys.argv[5]))
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route)
cfg = A.AvoidanceConfig(horizon=30, step=0.2, road_half_width=3.75, margin=0.2, surface_padding=0.15, max_lateral_accel=3.0,
                      cruise_speed=10, avoidance_speed=10, offsets=(-2.3, -2.0, -1.6, -1.0, 0.0, 1.0, 1.6, 2.0, 2.3),
                      transition_times=(1.2, 2.0, 3.0))
prev = None; lv = 0
for x in xs:
    st, od, pts, ids = min(d['clouds'], key=lambda c: abs(c[1][0] - x) if c[1] else 1e9)
    clusters = [pts[ids == k].astype(float) for k in np.unique(ids)]
    t = time.perf_counter()
    r = A.plan_avoidance(ref, A.Ego(*od), clusters, cfg, prev, None, ce, zone, start_level=lv if prev is not None else 0); lv = r.level
    from collections import Counter
    print(f'x={od[0]:.2f} d={153.7754-od[1]:+.2f} L={r.level} v={od[3]:.1f} {r.reason} sel={None if r.selected is None else round(r.selected.offset,2)} v={r.speed:.1f} {1000*(time.perf_counter()-t):.0f}ms', dict(Counter(c.reason for c in r.candidates)))
    if r.selected is None:
        for c in r.candidates[:12]: print('   ', round(c.offset,2), c.transition, c.reason, round(c.cost,2), np.round(c.d[::25],2))
    prev = r.selected or prev
