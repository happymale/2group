import pickle, sys, csv, numpy as np
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
os_, od_ = m.points()
cfg = A.AvoidanceConfig(margin=0.2, surface_padding=0.15, road_half_width=3.75, max_lateral_accel=3.0, cruise_speed=10, avoidance_speed=10)
od = clouds[k][1]; s0 = float(ref.project([[od[0], od[1]]], 380, 470)[0][0])
sc = D.read_obstacles(os_, od_, zone, s0, cfg)
print('groups', sc['groups'], sc['sides'])
det, why = D.plan_detour(ref, s0, 0.0, 0.0, os_, od_, zone, 464.4, A.replace(cfg, margin=0.0))
print(why)
if det is not None:
    cs, cd = sc['check_s'], sc['check_d']
    g = np.array([D._frenet_clearance(det.s, det.d, det.d_prime, cs[i:i+1], cd[i:i+1], cfg, 464.4, 0.45, wall_relief=0.15) for i in range(len(cs))])
    for i in np.argsort(g)[:8]: print(round(cs[i],2), round(cd[i],2), round(g[i]-0.35,2), 'plan d there', round(float(np.interp(cs[i], det.s, det.d)),2))
