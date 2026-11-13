import sys, math, json, itertools
sys.path.insert(0, '/mit_ws/agents/parking/src/hk_mit')
import numpy as np
from hk_mit.parking import *
cfg = ParkingConfig(radius_margin=1.05, clearance=0.0, slot_margin=0.15)
Rm = cfg.min_radius
course = json.load(open('/data/current/course.json'))
ms = {m['type']: Mission.from_json(m) for m in course['course']['missions'] if 'slot' in m}
free = {'parallel_parking': [(-4.0, 4.0, -2.05, 2.5)],
        'rear_parking': [(4.2, 9.2, -11.0, -2.0), (-2.8, 16.2, -2.475, 2.525)]}

def maxclear(poses, rects):
    best = -1
    for c in np.arange(0.0, 1.2, 0.05):
        x, y = outline_points(poses, body_outline(cfg, c, 0.05))
        inside = np.zeros(x.shape, dtype=bool)
        for x0, x1, y0, y1 in rects:
            inside |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        if inside.all():
            best = c
        else:
            break
    return best

which = sys.argv[1]
m = ms[which]
y_road = -13.1647 - m.slot_center[0]
frame = SlotFrame(m.slot_center, m.slot_yaw)
rects = [(-80, 80, y_road - 3.0, y_road + 3.0)] + free[which]
res = []
if which == 'parallel_parking':
    for R, yf, xb, ys, er in itertools.product([Rm, Rm*1.1], [0.5, 0.6, 0.65], [1.0, 1.1, 1.2], [0.0, -0.5, 0.5], [Rm, Rm*1.1, Rm*1.25]):
        pp = dict(radii=[R], final_lateral=[yf], final_back=[xb], start_lateral=[ys], shift_radius=12.0, straight_before_stop=0.0,
                  exit_radius=er, hold_s=3.3, cost_per_radius=0.0)
        for cost, segs, info in parallel_candidates(y_road, cfg, pp):
            if not in_slot(segs[1].poses[-1], m.slot_length, m.slot_width, 0.15, cfg):
                continue
            cl = [maxclear(s.poses, rects) for s in segs]
            res.append((min(cl), -cost, cl, (R, yf, xb, ys, er), [round(len(s.poses)*0.1, 1) for s in segs]))
else:
    xp0 = float(frame.to_local(m.zone_center)[0])
    for ra, rb, dx, sa, rc, rd, edx in itertools.product([Rm, 5.0, 5.5, 6.0], [Rm, 5.0], [-0.75, -0.25, 0.25, 0.75], [0.0, 1.0],
                                                     [Rm, 5.0], [Rm, 5.0, 5.5, 6.0], [-0.75, -0.25, 0.25, 0.75]):
        rp = dict(entry_radii=[ra], turn_radii=[rb], final_back=[0.0], passage_offsets=[dx], final_lateral=0.0, straight_before=0.0,
                  straight_after_turn=sa, exit_turn_radius=rc, exit_radius=rd, exit_passage_offset=edx, hold_s=3.3)
        for cost, segs, info in rear_candidates(y_road, xp0, cfg, rp):
            cl = [maxclear(s.poses, rects) for s in segs]
            res.append((min(cl), -cost, cl, (ra, rb, dx, sa, rc, rd, edx), [round(len(s.poses)*0.1, 1) for s in segs]))
res.sort(key=lambda r: (r[0], r[1]), reverse=True)
for r in res[:12]:
    print(round(r[0], 2), 'cost', round(-r[1], 1), 'clear', [round(c, 2) for c in r[2]], 'params', [round(v, 2) for v in r[3]], 'len', r[4])
