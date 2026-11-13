"""탐색용 경로: 후진주차 통로 안으로 전진해 들어가 멈춤."""
import csv, math, sys
r = [(float(a['x']), float(a['y'])) for a in csv.DictReader(open('/data/current/route.csv'))]
yc = float(sys.argv[2]) if len(sys.argv) > 2 else 74.25
R = 5.0
X = -13.1647
pts = [p for p in r if not (abs(p[0] - X) < 0.01 and p[1] < yc + R)]
pts = [p for p in pts if p[1] > yc + R or p[0] < -14 or True][:]
# 출발~원호 시작까지
out = []
for p in r:
    out.append(p)
    if abs(p[0] - X) < 0.05 and p[1] <= yc + R + 0.5:
        break
cx, cy = X + R, yc + R
for k in range(1, 16):
    a = math.pi + k * (math.pi / 2) / 16 * 1.0
    out.append((cx + R * math.cos(a), cy + R * math.sin(a)))
x = cx
while x < float(sys.argv[3]) if len(sys.argv) > 3 else -9.0:
    out.append((x, yc)); x += 0.5
with open(sys.argv[1], 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['x', 'y'])
    for p in out: w.writerow([f'{p[0]:.4f}', f'{p[1]:.4f}'])
print(len(out), out[-20:])
