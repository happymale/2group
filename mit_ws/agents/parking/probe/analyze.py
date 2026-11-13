"""gt.csv 분석: 정지 구간의 칸 안 여유·방향, 벽 모형까지 최소 여유. python3 analyze.py gt.csv ppark|rpark slot_y"""
import sys, math
sys.path.insert(0, '/mit_ws/agents/parking/src/hk_mit')
import numpy as np
from hk_mit.parking import ParkingConfig, SlotFrame, body_corners, outline_points, body_outline
cfg = ParkingConfig()
a = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
kind, sy = sys.argv[2], float(sys.argv[3])
if kind == 'ppark':
    sc, L, W = np.array([-18.1147, sy]), 7.4, 3.6
    rects = [(-80, 80, 1.95, 7.95), (-4.0, 4.0, -2.05, 7.95)]
else:
    sc, L, W = np.array([0.3353, sy]), 5.4, 3.6
    rects = [(-80, 80, -16.5, -10.5), (4.2, 9.2, -16.5, 2.525), (-2.8, 16.2, -2.475, 2.525)]
fr = SlotFrame(sc, -math.pi / 2)
t0 = a['t'][0]
loc = fr.to_local(np.column_stack((a['x'], a['y'])))
yaw = a['yaw'] + math.pi / 2
poses = np.column_stack((loc, yaw))
x, y = outline_points(poses, body_outline(cfg, 0.0, 0.1))
best = np.full(x.shape, -9.0)
for x0, x1, y0, y1 in rects:
    best = np.maximum(best, np.minimum.reduce([x - x0, x1 - x, y - y0, y1 - y]))
clear = best.min(axis=1)
k = int(np.argmin(clear))
print('min wall-model clearance %.2f at t=%.1f pose %s' % (clear[k], a['t'][k] - t0, np.round(poses[k], 2)))
for thr in (0.3, 0.15, 0.0):
    bad = np.flatnonzero(clear < thr)
    if len(bad):
        print('  clearance < %.2f: t %.1f~%.1f' % (thr, a['t'][bad[0]] - t0, a['t'][bad[-1]] - t0))
# 정지 구간
still = np.abs(a['v']) < 0.05
i = 0
while i < len(a):
    if still[i]:
        j = i
        while j + 1 < len(a) and still[j + 1]:
            j += 1
        if a['t'][j] - a['t'][i] > 0.8:
            c = body_corners(poses[j], cfg)
            mx = L / 2 - np.abs(c[:, 0]).max(); my = W / 2 - np.abs(c[:, 1]).max()
            print('still t=%.1f~%.1f (%.1fs) pose %s gear %d  slot margin long %.2f lat %.2f heading %.1f°  loc err %.2f' % (
                a['t'][i] - t0, a['t'][j] - t0, a['t'][j] - a['t'][i], np.round(poses[j], 2), a['gear'][j], mx, my,
                math.degrees(poses[j][2]), math.hypot(a['ex'][j] - a['x'][j], a['ey'][j] - a['y'][j])))
        i = j + 1
    else:
        i += 1
