exec(open('/mit_ws/agents/parking/probe/search.py').read().split("which = sys.argv[1]")[0])
m = ms['rear_parking']; y_road = -13.1647 - m.slot_center[0]; frame = SlotFrame(m.slot_center, m.slot_yaw)
rects = [(-80, 80, y_road - 3.0, y_road + 3.0)] + free['rear_parking']
xp0 = float(frame.to_local(m.zone_center)[0])
res = []
for ra, rb, dx, yf, rc, rd, edx in itertools.product([Rm, 5.0], [Rm, 4.7, 5.0], [-1.0, -0.75, -0.5], [-0.6, -0.4, -0.2, 0.0],
                                                 [Rm, 4.7, 5.0], [Rm, 5.0], [-0.75, -0.5, -0.25]):
    rp = dict(entry_radii=[ra], turn_radii=[rb], final_back=[0.0], passage_offsets=[dx], final_lateral=yf, straight_before=0.0,
              straight_after_turn=0.0, exit_turn_radius=rc, exit_radius=rd, exit_passage_offset=edx, hold_s=3.3)
    for cost, segs, info in rear_candidates(y_road, xp0, cfg, rp):
        if not in_slot(segs[1].poses[-1], m.slot_length, m.slot_width, 0.15, cfg):
            continue
        cl = [maxclear(s.poses, rects) for s in segs]
        res.append((min(cl), -cost, cl, (ra, rb, dx, yf, rc, rd, edx), [round(len(s.poses)*0.1, 1) for s in segs], segs))
res.sort(key=lambda r: (r[0], r[1]), reverse=True)
for r in res[:8]:
    print(round(r[0], 2), 'cost', round(-r[1], 1), 'clear', [round(c, 2) for c in r[2]], 'params', [round(v, 2) for v in r[3]], 'len', r[4])
best = res[0]
c = best[0] + 0.05
for s in best[5]:
    x, y = outline_points(s.poses, body_outline(cfg, c, 0.05))
    inside = np.zeros(x.shape, dtype=bool)
    for x0, x1, y0, y1 in rects:
        inside |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    if not inside.all():
        bad = ~inside.all(axis=1); k = int(np.argmax(bad)); j = int(np.argmax(~inside[k]))
        print(s.name, 'fails at', c, 'pose', np.round(s.poses[k], 2), 'pt', round(x[k, j], 2), round(y[k, j], 2))
