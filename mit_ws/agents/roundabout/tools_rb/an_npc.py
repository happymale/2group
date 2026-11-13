import csv, sys, math
import numpy as np
from collections import defaultdict
name = sys.argv[1]
t0 = None
by = defaultdict(list)
for r in csv.DictReader(open(f'/mit_ws/agents/roundabout/runs/{name}.npc.csv')):
    if r['cx'] == '': continue
    t = float(r['t']); t0 = t0 or t
    if 8 <= float(r['r']) <= 15 and float(r['ext_x']) < 6 and float(r['ext_y']) < 6:
        by[round(t - t0, 1)].append((float(r['ang']), float(r['r']), int(r['n']), float(r['ext_x']), float(r['ext_y']), float(r['ego_x']), float(r['ego_y'])))
for t in sorted(by)[:: int(sys.argv[2]) if len(sys.argv) > 2 else 1]:
    e = by[t][0][5:7]
    print(f'{t:6.1f} ego({e[0]:6.1f},{e[1]:6.1f}) ' + '  '.join(f'{a:6.1f}°/r{r:4.1f}/n{n}/{ex:.1f}x{ey:.1f}' for a, r, n, ex, ey, *_ in sorted(by[t])))
