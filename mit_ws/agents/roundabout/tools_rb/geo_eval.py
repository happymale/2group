"""설계한 뒤차축 경로를 정확히 따른다고 할 때 차체 통로 여유 (기하만)."""
import csv, math, sys
import numpy as np
sys.path.insert(0, '/mit_ws/agents/roundabout/src/hk_mit')
from hk_mit.roundabout import ring_reference
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/roundabout/src/hk_mit/course/route.csv'))])
C = (108.8353, 118.7754)
seg = np.diff(route, axis=0); sl = np.linalg.norm(seg, axis=1); sd = seg / sl[:, None]; st = np.r_[0, np.cumsum(sl)]
def off(p, near):
    lo, hi = max(0, near - 20), min(len(sl), near + 20)
    dlt = p - route[lo:hi]; al = np.clip(np.sum(dlt * sd[lo:hi], axis=1), 0, sl[lo:hi])
    return float(np.min(np.linalg.norm(p - (route[lo:hi] + al[:, None] * sd[lo:hi]), axis=1)))
def margin(ref, lo=300, hi=365):
    worst = (0, 0, None)
    yaw = np.arctan2(np.gradient(ref[:, 1]), np.gradient(ref[:, 0]))
    for (x, y), h in zip(ref, yaw):
        i = int(np.argmin(np.linalg.norm(route - (x, y), axis=1)))
        if not lo < st[i] < hi: continue
        c, s = math.cos(h), math.sin(h)
        b = max((off(np.array([x + lx * c - ly * s, y + lx * s + ly * c]), i), lx, ly) for lx in (-0.79, 3.845) for ly in (-0.946, 0.946))
        if b[0] > worst[0]: worst = (b[0], st[i], b[1:])
    return 2.196 - worst[0], worst
if __name__ == '__main__':
    for rr in (10.0, 10.5, 11.0, 11.5):
        for re in (4.5, 5.0, 5.5, 6.0, 7.0):
            ref, g = ring_reference(route, C, rr, re, re)
            m, w = margin(ref)
            print(f'ring {rr} R {re}: margin {m:+.2f} at s={w[1]:.1f} corner {w[2]} entry_ang {math.degrees(g.entry_angle):.0f} exit {math.degrees(g.exit_angle):.0f}')
    m, w = margin(route); print('orig route exact', m, w)
