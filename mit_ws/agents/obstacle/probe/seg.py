import csv, sys, numpy as np
rows = np.array([[float(v) for v in r.values()] for r in csv.DictReader(open(f'/mit_ws/agents/obstacle/runs/{sys.argv[1]}.csv'))])
a, b = float(sys.argv[2]), float(sys.argv[3])
t, s, v = rows[:, 0], rows[:, 4], rows[:, 3]
i0 = np.argmax(s >= a); i1 = np.argmax(s >= b) if (s >= b).any() else len(s) - 1
seg = slice(i0, i1 + 1)
print(f'{sys.argv[1]}: s {a:.0f}->{b:.0f}: {t[i1]-t[i0]:.1f}s, 평균 {v[seg].mean():.1f}, 최저 {v[seg].min():.1f}, 최고 {v[seg].max():.1f} m/s')
