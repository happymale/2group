import sys, json, csv
sys.path.insert(0, '/mit_ws/agents/parking/src/hk_mit')
import numpy as np
from hk_mit.parking import *
cfg = ParkingConfig(radius_margin=1.05, clearance=0.3, slot_margin=0.15)
Rm = cfg.min_radius
course = json.load(open('/data/current/course.json'))
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/data/current/route.csv'))])
ms = {m['type']: Mission.from_json(m) for m in course['course']['missions'] if 'slot' in m}
params = {'parallel_parking': dict(radii=[Rm, Rm*1.1], final_lateral=[0.6, 0.5], final_back=[1.1, 1.0, 1.2], start_lateral=[0.0, -0.5, 0.5],
          shift_radius=12.0, straight_before_stop=0.0, exit_radius=Rm, hold_s=3.4, cost_per_radius=0.0),
          'rear_parking': dict(entry_radii=[Rm], turn_radii=[5.0, 4.7], final_back=[0.05], passage_offsets=[-0.75, -0.5], final_lateral=-0.2,
          straight_before=0.0, straight_after_turn=0.0, exit_turn_radius=Rm, exit_radius=Rm, exit_passage_offset=-0.25, hold_s=3.4)}
free = {'parallel_parking': [(-4.0, 4.0, -2.05, 2.5)], 'rear_parking': [(4.2, 9.2, -11.0, -2.0), (-2.8, 16.2, -2.475, 2.525)]}
for t, fn, gy in (('parallel_parking', sys.argv[1], -1e9), ('rear_parking', sys.argv[2], float(sys.argv[3]))):
    m = ms[t]
    y_road, spread = road_lateral(m, route)
    a = np.genfromtxt(fn, delimiter=',', names=True)
    sel = a['gy'] > gy   # 차가 이 y보다 북쪽일 때 본 점만 (계획 시점 흉내)
    pts = np.column_stack((a['x'][sel], a['y'][sel]))
    pts = np.unique(np.round(pts / 0.1), axis=0) * 0.1
    det = detect_slot(m, pts, y_road, cfg, 8.0 if t == "parallel_parking" else 5.0)
    print(t, 'y_road %.2f spread %.3f' % (y_road, spread), 'pts', len(pts), 'det', det)
    man = plan_maneuver(m, det, y_road, cfg, params[t], free[t], pts, log=print)
    if man is None:
        continue
    print('  info', man.info)
    sp = splice_route(route, man, m, 3.0)
    print('  first/last', sp.first, sp.last, 'len', len(sp.points))
    for s in sp.segments:
        print('   seg', s[:6], 'start', np.round(sp.points[s[0]], 2), 'end', np.round(sp.points[s[1]], 2), 'end rear', np.round(s[6], 2))
    d = np.linalg.norm(np.diff(sp.points, axis=0), axis=1)
    print('  max step', d.max().round(2), 'at', int(np.argmax(d)), 'min step', d.min().round(4))
