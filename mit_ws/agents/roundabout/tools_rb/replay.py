"""npc 기록(군집 중심)을 NpcTracker에 다시 넣어 추적 확인."""
import csv, sys, math
from collections import defaultdict
import numpy as np
sys.path.insert(0, '/mit_ws/agents/roundabout/src/hk_mit')
from hk_mit.roundabout import NpcTracker
tr = NpcTracker((108.8353, 118.7754), 11.0, 1, 0.33)
by = defaultdict(list)
for r in csv.DictReader(open(sys.argv[1])):
    if r['cx'] == '': by[float(r['t'])]; continue
    ex, ey = float(r['ext_x']), float(r['ext_y'])
    cx, cy = float(r['cx']), float(r['cy'])
    by[float(r['t'])].append(np.array([[cx - ex / 2, cy - ey / 2], [cx + ex / 2, cy + ey / 2]] * max(1, int(r['n']) // 2)))
t0 = min(by)
for t in sorted(by):
    tr.update(t, by[t])
    if round((t - t0) * 10) % 5 == 0:
        print(f'{t - t0:6.1f} ' + ' '.join(f'{math.degrees(k.angle):7.1f}/{k.rate:.2f}/h{k.hits}' for k in sorted(tr.tracks, key=lambda k: k.angle)))
