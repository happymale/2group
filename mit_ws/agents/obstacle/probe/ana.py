import pickle, json, sys, numpy as np
d = pickle.load(open(sys.argv[1], 'rb'))
st = d['status']
t0 = st[0][0]
prev = None
for t, od, s in st:
    j = json.loads(s)
    key = (j['state'], j['tight'], round(j['speed_limit_mps'], 1))
    if key != prev or j['plan_ms'] > 150:
        print(f"{t-t0:6.2f} x={od[0] if od else 0:6.2f} y={od[1] if od else 0:5.2f} v={od[3] if od else 0:4.1f} {j['state']:18s} L={j.get('level',0)} off={j['offset_m']:+.1f} v_lim={j['speed_limit_mps']:.1f} ms={j['plan_ms']} {j['candidate_results']}")
    prev = key
ms = np.array([json.loads(s)['plan_ms'] for _, _, s in st])
print('plan_ms mean %.0f p95 %.0f max %.0f n=%d' % (ms.mean(), np.percentile(ms, 95), ms.max(), len(ms)))
# 장애물: 존 안 (|y|<3.6, x 범위) 점들 합침
if len(sys.argv) > 2:
    x0, x1, y0, y1 = map(float, sys.argv[2].split(','))
    pts = np.vstack([c[2] for c in d['clouds'] if len(c[2])])
    m = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
    p = pts[m]
    # 0.5 m 격자로 묶어 출력
    g = np.unique(np.round(p / 0.5) * 0.5, axis=0)
    from scipy.cluster.hierarchy import fcluster, linkage
    if len(g) > 1:
        lab = fcluster(linkage(g, 'single'), 1.2, 'distance')
        for l in np.unique(lab):
            q = g[lab == l]
            print('obstacle cells x %.1f..%.1f y %.1f..%.1f n=%d' % (q[:, 0].min(), q[:, 0].max(), q[:, 1].min(), q[:, 1].max(), len(q)))
