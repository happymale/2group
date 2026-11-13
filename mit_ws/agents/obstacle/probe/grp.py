import pickle, sys, csv, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A, hk_mit.detour as D
d = pickle.load(open(sys.argv[1], 'rb')); x = float(sys.argv[2])
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route); m = D.DetourMap()
clouds = [c for c in d['clouds'] if c[1] is not None]
k = min(range(len(clouds)), key=lambda i: abs(clouds[i][1][0] - x))
for st, od, pts, ids in clouds[:k + 1]:
    if len(pts):
        s, dd = ref.project(pts.astype(float), 390, 475); keep = (s > 395) & (s < 470) & (np.abs(dd) < 4.5)
        m.add(s[keep], dd[keep])
os_, od_ = m.points()
inner = (os_ > 410.7) & (os_ < 456.1) & (np.abs(od_) < 3.45)
o = np.argsort(os_[inner]); ss, dd = os_[inner][o], od_[inner][o]
pieces = np.split(np.arange(len(ss)), np.flatnonzero(np.diff(ss) > 1.0) + 1)
for p in pieces:
    print(f's {ss[p[0]]:.1f}..{ss[p[-1]]:.1f} d {dd[p].min():+.2f}..{dd[p].max():+.2f} n={len(p)}')
cfg = A.AvoidanceConfig(margin=0.2, surface_padding=0.15, road_half_width=3.75)
inner = (os_ > 410.7) & (os_ < 456.1) & (np.abs(os_ - 0) >= 0) & (np.abs(od_) < 3.75 - 0.85) & (os_ > 400)
inner = D._drop_wall_runs(os_, od_, inner)
o = np.sort(os_[inner]); cuts = np.flatnonzero(np.diff(o) > 6.635)
groups = [(a[0], a[-1]) for a in np.split(o, cuts + 1)]
print('groups', groups)
s2, d2, grown = D.complete_boxes(os_, od_, groups, cfg, mask=inner)
print('grown', grown)
for a, b, sd in grown:
    sel = (os_ >= a - 1) & (os_ <= b + 1) & inner
    print(a, b, 'inner d', np.round(np.sort(od_[sel])[[0, -1]], 2) if sel.any() else None)
sel = inner & (os_ > 410) & (os_ < 421)
for a, b in sorted(zip(np.round(os_[sel], 1), np.round(od_[sel], 1))): print((a, b), end=' ')
