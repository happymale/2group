import pickle, sys, csv, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
from hk_mit.avoidance import *
d = pickle.load(open(sys.argv[1], 'rb')); xs = float(sys.argv[2])
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = ReferencePath(route)
cfg = AvoidanceConfig(horizon=30, step=0.2, road_half_width=3.75, margin=0.2, surface_padding=0.15, max_lateral_accel=1.8,
                      cruise_speed=10, avoidance_speed=6, offsets=tuple(float(v) for v in sys.argv[3].split(",")), transition_times=(2.0,))
best = min(d['clouds'], key=lambda c: abs(c[1][0] - xs) if c[1] else 1e9)
st, od, pts, ids = best
clusters = [pts[ids == k].astype(float) for k in np.unique(ids)]
r = plan_avoidance(ref, Ego(*od), clusters, cfg, None, float(ref.project([[od[0], od[1]]])[0][0]))
geo = obstacle_geometry(clusters)
for c in r.candidates:
    print(c.offset, c.reason, 'return_start', c.return_start, c.return_length)
    for i in range(0, len(c.xy) - 1, 5):
        col, cl = collision_and_clearance(c.xy[i:i+2], c.yaw[i:i+2], geo, cfg)
        if col: print(i, c.s[i].round(1)) or print('  collide at', c.xy[i].round(2), 'd', c.d[i].round(2), 'yaw', c.yaw[i].round(3))
pw = pts[(pts[:,0]>25)&(pts[:,0]<50)&(pts[:,1]<-3)]
print('right wall y range', pw[:,1].min(), pw[:,1].max())
