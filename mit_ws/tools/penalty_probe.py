"""감점 직전 상태 분석 (오프라인, 주행 기록 CSV만 사용).

    python tools/penalty_probe.py <route.csv> <run.csv>...

lap_logger 기록에서 복귀(위치가 크게 튐) 직전 0.3초의 상태를 뽑아,
기준 경로 대비 차 중심 횡오차와 차체 네 모서리의 최대 횡거리를 계산합니다.
"""
import csv
import math
import sys

import numpy as np

L, W, RO = 4.635, 1.892, 0.79          # 차 길이·폭·뒤 오버행 (뒤차축 기준)


def load(path):
    with open(path, encoding='utf-8') as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


route = np.array([(r['x'], r['y']) for r in load(sys.argv[1])])
seg = np.diff(route, axis=0)
station = np.r_[0.0, np.cumsum(np.linalg.norm(seg, axis=1))]


def project(p):
    d = np.linalg.norm(route - p, axis=1)
    i = int(np.clip(np.argmin(d), 0, len(seg) - 1))
    t = seg[i] / np.linalg.norm(seg[i])
    n = np.array([-t[1], t[0]])
    return station[i], float(np.dot(p - route[i], n)), math.atan2(t[1], t[0])


def corners(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for lx in (-RO, L - RO):
        for ly in (-W / 2, W / 2):
            out.append(np.array([x + lx * c - ly * s, y + lx * s + ly * c]))
    return out


print(f'{"run":<14}{"t":>7}{"s":>7}{"v":>6}{"중심d":>7}{"방향차°":>8}{"모서리max|d|":>13}')
for path in sys.argv[2:]:
    rows = load(path)
    for k in range(5, len(rows)):
        jump = math.hypot(rows[k]['x'] - rows[k - 1]['x'], rows[k]['y'] - rows[k - 1]['y'])
        if jump < 2.0:
            continue
        r, q = rows[k - 1], rows[max(0, k - 6)]
        yaw = math.atan2(r['y'] - q['y'], r['x'] - q['x']) if math.hypot(r['x'] - q['x'], r['y'] - q['y']) > 0.05 else None
        s, d, ryaw = project(np.array([r['x'], r['y']]))
        if yaw is None:
            print(f'{path.split("/")[-1][:13]:<14}{r["t"]:7.2f}{s:7.1f}{r.get("speed", r.get("v", float("nan"))):6.2f}{d:7.2f}{"정지":>8}')
            continue
        dyaw = math.degrees((yaw - ryaw + math.pi) % (2 * math.pi) - math.pi)
        cd = max(abs(project(c)[1]) for c in corners(r['x'], r['y'], yaw))
        print(f'{path.split("/")[-1][:13]:<14}{r["t"]:7.2f}{s:7.1f}{r.get("speed", r.get("v", float("nan"))):6.2f}{d:7.2f}{dyaw:8.1f}{cd:13.2f}')
