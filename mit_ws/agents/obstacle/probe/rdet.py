"""probe 기록으로 DetourMap을 쌓고 지정 위치(x)에서 plan_detour 실행."""
import pickle, sys, csv, time, os, numpy as np
sys.path.insert(0, '/mit_ws/agents/obstacle/src/hk_mit')
import hk_mit.avoidance as A, hk_mit.detour as D
d = pickle.load(open(sys.argv[1], 'rb')); xs = [float(v) for v in sys.argv[2].split(',')]
zone = (float(sys.argv[3]), float(sys.argv[4])); s_exit = float(sys.argv[5])
route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/obstacle/src/hk_mit/course/route.csv'))])
ref = A.ReferencePath(route)
cfg = A.AvoidanceConfig(horizon=30, step=0.2, road_half_width=3.75, margin=float(os.environ.get('MARGIN', 0.2)), surface_padding=0.15, max_lateral_accel=3.0,
                        cruise_speed=10, avoidance_speed=10, transition_times=(1.2, 2.0, 3.0))
m = D.DetourMap()
clouds = [c for c in d['clouds'] if c[1] is not None]
for x in xs:
    # x 까지(진행 방향 무관: 시간 순으로 x에 가장 가까운 시점까지) 누적
    k = min(range(len(clouds)), key=lambda i: abs(clouds[i][1][0] - x))
    m.clear()
    for st, od, pts, ids in clouds[:k + 1]:
        if not len(pts): continue
        s, dd = ref.project(pts.astype(float), zone[0] - 20, s_exit + 10)
        keep = (s > zone[0] - 15) & (s < s_exit + 5) & (np.abs(dd) < 4.5)
        if keep.any(): m.add(s[keep], dd[keep])
    od = clouds[k][1]
    s0, d0 = ref.project([[od[0], od[1]]], zone[0] - 30, s_exit + 10)
    _, yaw, _, _ = ref.evaluate(s0)
    slope = np.tan(od[2] - yaw[0])
    os_, od_ = m.points()
    t = time.perf_counter()
    det, why = D.plan_detour(ref, float(s0[0]), float(d0[0]), float(slope), os_, od_, zone, s_exit, cfg)
    ms = 1000 * (time.perf_counter() - t)
    dbg0 = list(D.DEBUG)
    if det is None:
        det2, why2 = D.plan_detour(ref, float(s0[0]), float(d0[0]), float(slope), os_, od_, zone, s_exit, cfg, margin=0.0)
        dbg = list(D.DEBUG); det2, why2 = None, None
        print(f'x={od[0]:.1f} s={s0[0]:.1f} d={d0[0]:+.2f} -> {why} ({ms:.0f} ms); margin0: {why2}', '' if det2 is None else [tuple(round(v,2) for v in t) for t in det2.targets])
        print('   ', dbg0[:6]); continue
    print(f'x={od[0]:.1f} s={s0[0]:.1f} d={d0[0]:+.2f} -> ok {ms:.0f} ms targets={[tuple(round(v,2) for v in t) for t in det.targets]} '
          f'maxk={np.abs(det.curvature).max():.3f} vmin={det.speed.min():.1f} v0={det.speed[0]:.1f} clear={det.clearance:.2f} T={D._travel_time(det):.1f}s')
    print('    d:', np.round(det.d[::20], 2))
