import sys, math, json
sys.path.insert(0, '/mit_ws/agents/parking/src/hk_mit')
import numpy as np
from hk_mit.parking import *

cfg = ParkingConfig(radius_margin=float(sys.argv[1]) if len(sys.argv) > 1 else 1.05, clearance=float(sys.argv[2]) if len(sys.argv) > 2 else 0.3)
Rm = cfg.min_radius
print('Rmin', Rm)
course = json.load(open('/data/current/course.json'))
ms = {m['type']: Mission.from_json(m) for m in course['course']['missions'] if 'slot' in m}
pp = dict(radii=[Rm, Rm * 1.1, Rm * 1.2], final_lateral=[0.2, 0.4, 0.6, 0.75], final_back=[0.6, 0.8, 1.0],
          start_lateral=[-1.5, -1.0, -0.5, 0.0, 0.5, 1.0], shift_radius=12.0, straight_before_stop=0.0,
          exit_radius=Rm * 1.1, hold_s=3.3, cost_per_radius=0.0)
rp = dict(entry_radii=[Rm, Rm * 1.2, 5.5], turn_radii=[Rm, Rm * 1.1], final_back=[0.0], passage_offsets=[-0.5, 0.0, 0.5],
          final_lateral=0.0, straight_before=0.0, straight_after_turn=0.0, exit_turn_radius=Rm, exit_radius=Rm * 1.2,
          exit_passage_offset=0.0, hold_s=3.3)
free = {'parallel_parking': [(-4.0, 4.0, -2.05, 2.5)],
        'rear_parking': [(4.2, 9.2, -11.0, -2.0), (-2.8, 16.2, -2.475, 2.525)]}
for t, m in ms.items():
    y_road = -13.1647 - m.slot_center[0]
    det = {'slide': 0.0, 'slot_center': m.slot_center, 'wall': 3.0}
    frame = SlotFrame(m.slot_center, m.slot_yaw)
    print(t, 'y_road', y_road)
    if t == 'parallel_parking':
        cands = parallel_candidates(y_road, cfg, pp)
    else:
        cands = rear_candidates(y_road, float(frame.to_local(m.zone_center)[0]), cfg, rp)
    rects = [(-80, 80, y_road - 3.0, y_road + 3.0)] + free[t]
    n = 0
    for cost, segs, info in cands:
        final = segs[1].poses[-1]
        okslot = in_slot(final, m.slot_length, m.slot_width, cfg.slot_margin, cfg)
        allp = np.vstack([s.poses for s in segs])
        ok, k = free_space_ok(allp, rects, cfg)
        # 최소 여유: clearance를 늘려가며
        if okslot and ok:
            lo = 0.0
            for c in np.arange(0.3, 1.2, 0.05):
                cfg2 = ParkingConfig(radius_margin=cfg.radius_margin, clearance=c)
                if free_space_ok(allp, rects, cfg2)[0]:
                    lo = c
            n += 1
            if n <= 6:
                print('  OK cost %.1f clear>=%.2f' % (cost, lo), {k: (round(v, 2) if isinstance(v, float) else v) for k, v in info.items()},
                      'seg len', [round(len(s.poses) * cfg.step, 1) for s in segs], 'final', np.round(final, 2))
    print('  feasible', n, 'of', len(cands))
