import pickle, sys, csv, time, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A, hk_mit.detour as D
d = pickle.load(open(sys.argv[1], 'rb')); x = float(sys.argv[2]); zone = (410.4, 456.4)
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route); m = D.DetourMap()
clouds = [c for c in d['clouds'] if c[1] is not None]
k = min(range(len(clouds)), key=lambda i: abs(clouds[i][1][0] - x))
for st, od, pts, ids in clouds[:k + 1]:
    if len(pts):
        s, dd = ref.project(pts.astype(float), 390, 475); keep = (s > 395) & (s < 470) & (np.abs(dd) < 4.5)
        m.add(s[keep], dd[keep])
os_, od_ = m.points(); print('cells', len(os_))
cfg = A.AvoidanceConfig(margin=0.2, surface_padding=0.15, road_half_width=3.75, max_lateral_accel=2.0, cruise_speed=10, avoidance_speed=10)
od = clouds[k][1]; s0, d0 = ref.project([[od[0], od[1]]], 380, 470)
for mg, chk in (((0.2, 0.1, 0.0), True), ((0.0,), False)):
    t = time.perf_counter(); det, why = D.plan_detour(ref, float(s0[0]), float(d0[0]), 0.0, os_, od_, zone, 464.4, cfg, margins=mg, check=chk)
    print(mg, chk, why, None if det is None else (det.margin, round(det.clearance,2), [round(t[2],2) for t in det.targets]), f'{1000*(time.perf_counter()-t):.0f} ms', len(D.DEBUG))
