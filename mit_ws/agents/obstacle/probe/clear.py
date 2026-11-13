"""ground truth 자세(CSV)의 차체와 probe의 장애물 점(구간 안, 벽 제외) 사이 최소 거리. 측정 전용."""
import pickle, sys, csv, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A
name, zs, ze = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
d = pickle.load(open(f'/mit_ws/agents/obstacle/runs/{name}.probe.pkl', 'rb'))
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route)
pts = np.vstack([c[2] for c in d['clouds'] if len(c[2])]).astype(float)
pts = np.unique(np.round(pts, 2), axis=0)
s, dd = ref.project(pts, zs - 2, ze + 2)
obs = pts[(s > zs + 0.5) & (s < ze - 0.5) & (np.abs(dd) < 3.45)]
rows = np.array([[float(v) for v in r.values()] for r in csv.DictReader(open(f'/mit_ws/agents/obstacle/runs/{name}.csv'))])
t, x, y, v, st = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3], rows[:, 4]
yaw = np.arctan2(np.gradient(y), np.gradient(x) + 1e-9)
moving = v > 0.3
best = (9, 0)
for i in np.flatnonzero(moving & (st > zs - 6) & (st < ze + 6)):
    c, sn = np.cos(yaw[i]), np.sin(yaw[i])
    loc = np.column_stack(((obs[:, 0] - x[i]) * c + (obs[:, 1] - y[i]) * sn, -(obs[:, 0] - x[i]) * sn + (obs[:, 1] - y[i]) * c))
    dx = np.maximum(np.maximum(-0.79 - loc[:, 0], loc[:, 0] - 3.845), 0); dy = np.maximum(np.abs(loc[:, 1]) - 0.946, 0)
    m = np.min(np.hypot(dx, dy)) if len(obs) else 9
    if m < best[0]: best = (m, i)
i = best[1]
t0 = t[np.argmax(v > 0.05)]
print(f'{name}: 장애물 최소 여유 {best[0]:.2f} m (x={x[i]:.1f}, v={v[i]:.1f}), 출발 후 시간 {t[-1]-t0:.1f}s (출발 대기 {t0:.1f}s), 최고 {v.max():.1f} m/s')
