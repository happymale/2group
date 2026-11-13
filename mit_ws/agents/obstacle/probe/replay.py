"""probe 기록의 장애물 cloud로 plan_avoidance를 다시 돌려 후보별 결과·계산 시간을 봅니다."""
import pickle, sys, time, csv, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
from hk_mit.avoidance import *
pk, xsel = sys.argv[1], [float(v) for v in sys.argv[2].split(',')]
check_end = float(sys.argv[3]) if len(sys.argv) > 3 else None
d = pickle.load(open(pk, 'rb'))
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = ReferencePath(route)
cfg = AvoidanceConfig(horizon=30, step=0.2, road_half_width=3.75, margin=0.2, surface_padding=0.15, max_lateral_accel=3.0,
                      cruise_speed=10, avoidance_speed=10, offsets=(-2.3, -2.0, -1.6, -1.0, 0.0, 1.0, 1.6, 2.0, 2.3),
                      transition_times=(1.2, 2.0, 3.0))
for xs in xsel:
    best = min(d['clouds'], key=lambda c: abs(c[1][0] - xs) if c[1] else 1e9)
    st, od, pts, ids = best
    clusters = [pts[ids == k].astype(float) for k in np.unique(ids)]
    ego = Ego(*od)
    hint = float(ref.project([[od[0], od[1]]])[0][0])
    t = time.perf_counter()
    r = plan_avoidance(ref, ego, clusters, cfg, None, hint, check_end, (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else None)
    ms = 1000 * (time.perf_counter() - t)
    sel = r.selected
    print(f'x={od[0]:.2f} y={od[1]:.2f} v={od[3]:.1f} s={hint:.1f} -> {r.reason} sel={None if sel is None else (sel.offset, sel.transition, round(sel.return_start,1), round(sel.return_length,1))} speed={r.speed:.2f} {ms:.0f} ms')
    from collections import Counter
    print('   ', Counter(c.reason for c in r.candidates))
    if sel is not None:
        print('    d:', np.round(sel.d[::25], 2), 'maxk', np.abs(sel.curvature).max().round(3))
