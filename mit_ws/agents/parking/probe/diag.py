import sys, math, json
sys.path.insert(0, '/mit_ws/agents/parking/src/hk_mit')
import numpy as np
from hk_mit.parking import *
exec(open('/mit_ws/agents/parking/probe/design_params.py').read())

def margin_report_old(segs, rects, cfg):
    for s in segs:
        x, y = outline_points(s.poses, body_outline(cfg, 0.0, 0.05))
        # 각 점의 자유공간 안쪽 거리 (직사각형 합집합: 각 직사각형 안쪽 거리의 최대)
        best = np.full(x.shape, -9.0)
        for x0, x1, y0, y1 in rects:
            d = np.minimum.reduce([x - x0, x1 - x, y - y0, y1 - y])
            best = np.maximum(best, d)
        per = best.min(axis=1)
        k = int(np.argmin(per))
        j = int(np.argmin(best[k]))
        print('   seg %-10s gear %+d len %.1f  min margin %.2f at pose %s body pt (%.2f, %.2f)' % (
            s.name, s.gear, len(s.poses) * cfg.step, per[k], np.round(s.poses[k], 2), x[k, j], y[k, j]))

for t, m in ms.items():
    y_road = -13.1647 - m.slot_center[0]
    frame = SlotFrame(m.slot_center, m.slot_yaw)
    if t == 'parallel_parking':
        cands = parallel_candidates(y_road, cfg, pp)
    else:
        cands = rear_candidates(y_road, float(frame.to_local(m.zone_center)[0]), cfg, rp)
    rects = [(-80, 80, y_road - 3.0, y_road + 3.0)] + free[t]
    print(t, len(cands))
    for cost, segs, info in cands[:int(sys.argv[1])]:
        final = segs[1].poses[-1]
        print('  cost %.1f slot_ok %s' % (cost, in_slot(final, m.slot_length, m.slot_width, cfg.slot_margin, cfg)), info)
        pass

def outside_pts(poses, rects, c):
    x, y = outline_points(poses, body_outline(cfg, c, 0.05))
    inside = np.zeros(x.shape, dtype=bool)
    for x0, x1, y0, y1 in rects:
        inside |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    return x, y, inside

print('---- clearance search')
for t, m in ms.items():
    y_road = -13.1647 - m.slot_center[0]
    frame = SlotFrame(m.slot_center, m.slot_yaw)
    cands = parallel_candidates(y_road, cfg, pp) if t == 'parallel_parking' else rear_candidates(y_road, float(frame.to_local(m.zone_center)[0]), cfg, rp)
    rects = [(-80, 80, y_road - 3.0, y_road + 3.0)] + free[t]
    for cost, segs, info in cands[:int(sys.argv[1])]:
        print(t, 'cost %.1f' % cost, {k: round(v, 2) for k, v in info.items() if isinstance(v, float)})
        for s in segs:
            best = -1
            for c in np.arange(0.0, 1.5, 0.05):
                x, y, inside = outside_pts(s.poses, rects, c)
                if inside.all():
                    best = c
                else:
                    bad = ~inside.all(axis=1)
                    k = int(np.argmax(bad)); j = int(np.argmax(~inside[k]))
                    print('   %-10s max clearance %.2f; at %.2f fails pose %s pt (%.2f,%.2f)' % (s.name, best, c, np.round(s.poses[k], 2), x[k, j], y[k, j]))
                    break
