import csv, sys, math
import numpy as np
name = sys.argv[1]
R = list(csv.DictReader(open(f'/mit_ws/agents/roundabout/runs/{name}.csv')))
a = np.array([[float(r[k]) for k in ['t','x','y','speed','s','lateral','body']] for r in R])
# 페널티 직전 구간
prev = None
for row in a[::5]:
    if 300 <= row[4] <= 350:
        print(' '.join(f'{v:7.2f}' for v in row))
