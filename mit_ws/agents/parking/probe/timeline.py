import sys, numpy as np
a = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else None
# START 시각: 처음으로 위치가 바뀌기 전 마지막 정지 시작을 모름 → 첫 명령 accel>0 시각을 기준으로 출력
mv = np.flatnonzero(np.abs(a['v']) > 0.05)
t_first = a['t'][mv[0]]
prev = None
for k in range(len(a)):
    st = (int(a['gear'][k]), 'stop' if abs(a['v'][k]) < 0.05 else 'move')
    if st != prev:
        print('%6.2f  gear %d %s  pos (%.2f, %.2f) yaw %.1f v %.2f' % (a['t'][k] - t_first, st[0], st[1], a['x'][k], a['y'][k], np.degrees(a['yaw'][k]), a['v'][k]))
        prev = st
