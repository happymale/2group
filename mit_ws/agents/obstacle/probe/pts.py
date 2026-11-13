import pickle, sys, csv, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A
d = pickle.load(open(sys.argv[1], 'rb')); xs = float(sys.argv[2])
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route)
best = min(d['clouds'], key=lambda c: abs(c[1][0] - xs) if c[1] else 1e9)
st, od, pts, ids = best
s0 = float(ref.project([[od[0], od[1]]])[0][0])
s, dd = ref.project(pts.astype(float), s0 - 5, s0 + 35)
m = (np.abs(dd) < 3.4) & (s > s0 - 5.5) & (s < s0 + 34)
print('ego', np.round(od, 2), 's0', round(s0, 1))
o = np.argsort(s[m])
print(' '.join(f'({a:.1f},{b:+.2f})' for a, b in zip(s[m][o], dd[m][o])))
