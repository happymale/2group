"""실제 궤적(ground truth 뒤차축)과 설계 뒤차축 경로 사이 부호 있는 횡오차 (+ 왼쪽)."""
import csv, sys, math
import numpy as np
sys.path.insert(0, '/mit_ws/agents/roundabout/src/hk_mit')
from hk_mit.roundabout import ring_reference
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/roundabout/src/hk_mit/course/route.csv'))])
LO, HI = (float(v) for v in (sys.argv[2:4] or [316, 352])); args = [11.0, 7.25, 7.25]
_, rear, g = ring_reference(route, (108.8353, 118.7754), 11.0, args[0], args[1], args[2])
R = list(csv.DictReader(open(f'/mit_ws/agents/roundabout/runs/{sys.argv[1]}.csv')))
last = None
for r in R:
    s = float(r['s'])
    if not LO < s < HI or float(r['speed']) < 0.5: continue
    if last is not None and s - last < 0.9: continue
    last = s
    p = np.array([float(r['x']), float(r['y'])])
    i = int(np.argmin(np.linalg.norm(rear - p, axis=1)))
    a, b = rear[max(i - 1, 0)], rear[min(i + 1, len(rear) - 1)]
    t = (b - a) / np.linalg.norm(b - a)
    d = -t[1] * (p - rear[i])[0] + t[0] * (p - rear[i])[1]
    print(f"s={s:6.1f} v={float(r['speed']):4.2f} dev={d:+.2f} body={float(r['body']):.2f} steer={float(r['steer']):+.2f}")
