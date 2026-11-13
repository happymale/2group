#!/usr/bin/env python3
"""곡률 제한 기준 경로 평활화 스케치 (표준 라이브러리만).

목적: route.csv(0.5 m 간격) 꺾임이 최소 회전반경(뒤차축 4.22 m)보다 급한 곳을 미리 펴서
      Stanley/퓨어퍼슈트가 조향 포화(±1)에 걸려 코너 안쪽·바깥으로 튀는 것을 막는다.
방법: Apollo FEM_POS_DEVIATION 평활기의 아이디어(2차 차분 smoothness + 원래 점 이탈 상한)를
      QP 대신 '곡률 초과 구간만 국소 라플라시안 반복 + 이탈 상한'으로 근사. 정식 QP(OSQP)로
      바꾸면 곡률 제약을 선형화해 한 번에 풀 수 있음. 이탈 상한 max_dev 로 통로(반폭 2.196 m)
      안에 남도록 보장 (차 반폭 0.946 + 여유 → max_dev 0.8~1.0 m).
사용: 오프라인으로 route.csv → route_smooth.csv 생성 후 planner의 route_csv로 지정.
"""
import csv
import math
import sys


def curvature(p, i, h=1):
    """i-h, i, i+h 세 점 외접원 부호 곡률 (h=2 → 0.5 m 간격에서 1 m 폭으로 잡음 감소)."""
    (x0, y0), (x1, y1), (x2, y2) = p[i - h], p[i], p[i + h]
    a, b, c = math.dist(p[i - h], p[i]), math.dist(p[i], p[i + h]), math.dist(p[i - h], p[i + h])
    cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
    return 2 * cross / max(a * b * c, 1e-9)


def smooth(points, kappa_max=1 / 4.8, max_dev=0.9, window_m=6.0, spacing=0.5, alpha=0.3, iters=4000):
    """곡률 초과점 주변 ±window_m 만 국소 라플라시안 평활. 원래 점에서 max_dev 넘게는 안 움직임.
    반환: (점 목록, 최대 곡률, 최대 이탈). max_dev로 못 펴는 꺾임은 그대로 남으므로 결과 곡률을 확인."""
    ref = [tuple(q) for q in points]
    p = [list(q) for q in points]
    n, w = len(p), int(window_m / spacing)
    h = 2
    for _ in range(iters):
        bad = [i for i in range(h, n - h) if abs(curvature(p, i, h)) > kappa_max]
        if not bad:
            break
        active = set()
        for i in bad:
            active.update(range(max(1, i - w), min(n - 1, i + w + 1)))
        moved = False
        for i in sorted(active):
            nx = p[i][0] + alpha * (0.5 * (p[i - 1][0] + p[i + 1][0]) - p[i][0])
            ny = p[i][1] + alpha * (0.5 * (p[i - 1][1] + p[i + 1][1]) - p[i][1])
            if math.hypot(nx - ref[i][0], ny - ref[i][1]) <= max_dev:
                p[i][0], p[i][1] = nx, ny
                moved = True
        if not moved:
            break
    worst = max(abs(curvature(p, i, h)) for i in range(h, n - h))
    dev = max(math.dist(a, b) for a, b in zip(ref, p))
    return [tuple(q) for q in p], worst, dev


def main():
    if len(sys.argv) < 3:
        # 자체 시험: 45° 꺾임 (직각은 이탈 0.9 m로는 R 4.8을 만들 수 없음: R≈dev/(1/sin(θ/2)-1))
        pts = [(x * 0.5, 0.0) for x in range(-20, 0)] + [(y * 0.5 * 0.7071, y * 0.5 * 0.7071) for y in range(0, 20)]
        out, worst, dev = smooth(pts)
        kin = max(abs(curvature(pts, i, 2)) for i in range(2, len(pts) - 2))
        print('원래 최대 곡률 %.2f → 평활 후 %.3f (R=%.2f m), 최대 이탈 %.2f m' % (kin, worst, 1 / worst, dev))
        return
    with open(sys.argv[1]) as f:
        rows = [r for r in csv.reader(f)]
    head = rows[0] if not rows[0][0].replace('.', '').replace('-', '').isdigit() else None
    data = rows[1:] if head else rows
    pts = [(float(r[0]), float(r[1])) for r in data]
    out, worst, dev = smooth(pts)
    with open(sys.argv[2], 'w', newline='') as f:
        w = csv.writer(f)
        if head:
            w.writerow(head[:2])
        for x, y in out:
            w.writerow(['%.4f' % x, '%.4f' % y])
    print('max curvature after smoothing: %.3f (R=%.2f m), max deviation %.2f m' % (worst, 1 / max(worst, 1e-9), dev))


if __name__ == '__main__':
    main()
