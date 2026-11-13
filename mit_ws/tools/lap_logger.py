"""연습 주행 기록기 (튜닝 전용, 공식 주행에서는 /dev/ground_truth가 없습니다).

    python3 /mit_ws/tools/lap_logger.py [출력.csv] [--stop-s 거리]

--stop-s를 주면 경로 거리(m)가 그 값을 넘을 때 요약을 내고 끝냅니다(구간 시험용, 시도 중지는 run_test.sh가 함).

실패 감시 (빨리 끝내고 다시 시도하기 위해, 환경 변수로 조정):
  MAX_PENALTIES  감점이 이 횟수가 되면 바로 중단 (기본 1, 5면 시뮬레이터 판정까지 끝까지)
  STUCK_S        경로 거리가 이 시간(시뮬레이션 s) 동안 1 m도 늘지 않으면 '멈춤'으로 중단 (기본 15, 0 = 끔)
  WATCH_S        이 시간(s)마다 진행 상황 한 줄 출력 (기본 30)
중단하면 'ABORT: 이유'와 요약을 내고 종료 코드 3으로 끝납니다(run_test.sh가 시도를 중지).

시도 하나(START~END)를 기록하고 끝나면 요약을 출력합니다.
  - 경로 횡오차: 실제 위치(ground truth)와 route.csv 사이 거리
  - 차체 횡거리: 차체 네 모서리 중 경로에서 가장 먼 거리. 감점 기록을 보면 판정은 차 중심이 아니라
    차체가 통로(반폭 = corridor_width_m / 2) 밖으로 나가는지에 가깝습니다 → '통로 여유'가 음수면 위험
  - 위치 추정 오차: /hk/local/odom 과 ground truth 사이 거리
  - 구간(체크포인트 사이)별 시간과 평균·최고 속력
"""
import csv
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from hk_interfaces.msg import CtrlCmd, SimEvent

ROUTE = '/data/current/route.csv'
COURSE = '/data/current/course.json'


class LapLogger(Node):
    def __init__(self, out, stop_s=None):
        super().__init__('hk_lap_logger')
        self.stop_s = stop_s
        with open(ROUTE, encoding='utf-8') as f:
            self.route = np.array([(float(r['x']), float(r['y'])) for r in csv.DictReader(f)])
        seg = np.diff(self.route, axis=0)
        self.station = np.r_[0.0, np.cumsum(np.linalg.norm(seg, axis=1))]
        self.seg_start, self.seg_len = self.route[:-1], np.linalg.norm(seg, axis=1)
        self.seg_dir = seg / self.seg_len[:, None]
        with open(COURSE, encoding='utf-8') as f:
            course = json.load(f)
        vehicle = course['course']['vehicle']
        self.body = (vehicle['length'], vehicle['width'], vehicle['rear_overhang'])
        self.half_corridor = course.get('corridor_width_m', 4.392) / 2
        self.out = open(out, 'w', newline='') if out else None
        self.writer = csv.writer(self.out) if self.out else None
        if self.writer:
            self.writer.writerow(['t', 'x', 'y', 'speed', 's', 'lateral', 'loc_err', 'accel', 'brake', 'steer', 'body'])
        self.rows = []
        self.est = None
        self.cmd = (0.0, 0.0, 0.0)
        self.running = False
        self.t0 = None
        self.events = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SimEvent, '/sim/events', self.on_event, qos)
        self.create_subscription(Odometry, '/dev/ground_truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/hk/local/odom', self.on_est, 10)
        self.create_subscription(CtrlCmd, '/hk/control/command', self.on_cmd, 10)
        self.seen = set()
        self.history = set()
        self.started = time.monotonic()
        self.done = False
        self.aborted = None
        self.max_penalties = int(os.environ.get('MAX_PENALTIES', '1'))
        self.stuck_s = float(os.environ.get('STUCK_S', '15'))
        self.watch_s = float(os.environ.get('WATCH_S', '30'))
        self.penalties = 0
        self.best_s, self.best_t = -1.0, None     # 지금까지 가장 멀리 간 경로 거리와 그때 시각 (멈춤 감시)
        self.last_watch = None

    def on_event(self, ev):
        key = (ev.attempt_id, ev.event_id)
        if key in self.seen:
            return
        self.seen.add(key)
        if time.monotonic() - self.started < 2.0:      # 켤 때 다시 받는 지난 시도 이벤트(transient local)는 건너뜀
            self.history.add(ev.attempt_id)
            return
        if ev.attempt_id in self.history:
            return
        t = ev.stamp.sec + ev.stamp.nanosec * 1e-9
        if ev.kind == SimEvent.START:
            self.running, self.t0, self.rows, self.events = True, t, [], []
            self.penalties, self.best_s, self.best_t = 0, -1.0, None
            print(f'START {ev.detail}', flush=True)
        elif not self.running:
            return
        self.events.append((t - (self.t0 or t), ev.kind, ev.checkpoint, ev.detail))
        if ev.kind != SimEvent.START:
            print(f'{t - self.t0:7.2f}s kind={ev.kind} cp={ev.checkpoint} {ev.detail}', flush=True)
        if ev.kind == SimEvent.END:
            self.running = False
            self.summary()
            self.done = True
        elif ev.kind == SimEvent.RESPAWN:
            self.penalties += 1
            self.best_s, self.best_t = -1.0, None              # 복귀하면 멈춤 감시를 처음부터
            if self.max_penalties and self.penalties >= self.max_penalties:
                where = f'{self.rows[-1][4]:.0f}' if self.rows else '?'
                self.abort(f'감점 {self.penalties}회 (s={where} m 부근, cp{ev.checkpoint}로 복귀)')

    def abort(self, reason):
        if self.done:
            return
        print(f'ABORT: {reason}', flush=True)
        self.aborted = reason
        self.running = False
        self.summary()
        self.done = True

    def watch(self, t, s):
        """멈춤 감시와 주기적 진행 출력 (ground truth마다)."""
        if self.best_t is None or s > self.best_s + 1.0:
            self.best_s, self.best_t = max(s, self.best_s), t
        elif self.stuck_s and t - self.best_t > self.stuck_s:
            self.abort(f'멈춤: {self.stuck_s:.0f} s 동안 s={self.best_s:.0f} m에서 진행 없음')
            return
        if self.watch_s and (self.last_watch is None or t - self.last_watch >= self.watch_s):
            self.last_watch = t
            print(f'{t:7.2f}s 진행 s={s:.0f} m, 속력 {self.rows[-1][3]:.1f} m/s, 감점 {self.penalties}', flush=True)

    def on_est(self, m):
        self.est = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_cmd(self, m):
        self.cmd = (m.accel, m.brake, m.steering)

    def offset(self, p, near):
        """점 p의 경로까지 거리 (near 번째 점 주변 선분에서)."""
        lo, hi = max(0, near - 20), min(len(self.seg_len), near + 20)
        delta = p - self.seg_start[lo:hi]
        along = np.clip(np.sum(delta * self.seg_dir[lo:hi], axis=1), 0.0, self.seg_len[lo:hi])
        closest = self.seg_start[lo:hi] + along[:, None] * self.seg_dir[lo:hi]
        return float(np.min(np.linalg.norm(p - closest, axis=1)))

    def body_offset(self, x, y, yaw, near):
        length, width, rear = self.body
        c, s = math.cos(yaw), math.sin(yaw)
        return max(self.offset(np.array([x + lx * c - ly * s, y + lx * s + ly * c]), near)
                   for lx in (-rear, length - rear) for ly in (-width / 2, width / 2))

    def on_truth(self, m):
        if not self.running:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - self.t0
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        d = np.linalg.norm(self.route - (x, y), axis=1)
        i = int(np.argmin(d))
        lat = float(d[i])
        loc = math.hypot(self.est[0] - x, self.est[1] - y) if self.est else float('nan')
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        body = self.body_offset(x, y, yaw, i)
        row = (t, x, y, v, float(self.station[i]), lat, loc) + tuple(self.cmd) + (body,)
        self.rows.append(row)
        if self.writer:
            self.writer.writerow([f'{a:.3f}' for a in row])
        self.watch(t, row[4])
        if self.stop_s is not None and row[4] >= self.stop_s and not self.done:
            print(f'{t:7.2f}s 구간 끝 s={row[4]:.0f} m 도달', flush=True)
            self.running = False
            self.summary()
            self.done = True

    def summary(self):
        if not self.rows:
            print('기록 없음')
            return
        a = np.array(self.rows)
        print(f'시간 {a[-1, 0]:.1f}s, 진행 {a[:, 4].max():.0f} m, 평균 {a[:, 3].mean():.2f} m/s, 최고 {a[:, 3].max():.2f} m/s')
        print(f'경로 횡오차: 평균 {a[:, 5].mean():.2f} m, 95% {np.percentile(a[:, 5], 95):.2f} m, 최대 {a[:, 5].max():.2f} m '
              f'(s={a[np.argmax(a[:, 5]), 4]:.0f} m)')
        k = int(np.argmax(a[:, 10]))
        print(f'차체 횡거리: 최대 {a[k, 10]:.2f} m (s={a[k, 4]:.0f} m), 통로 반폭 {self.half_corridor:.2f} m → '
              f'최소 여유 {self.half_corridor - a[k, 10]:+.2f} m')
        loc = a[:, 6][np.isfinite(a[:, 6])]
        if len(loc):
            print(f'위치 추정 오차: 평균 {loc.mean():.2f} m, 최대 {loc.max():.2f} m')
        # 50 m 구간별 시간
        edges = np.arange(0, a[:, 4].max() + 50, 50)
        parts = []
        for lo, hi in zip(edges[:-1], edges[1:]):
            sel = (a[:, 4] >= lo) & (a[:, 4] < hi)
            if sel.any():
                parts.append(f'{lo:.0f}-{hi:.0f}:{a[sel, 0].max() - a[sel, 0].min():.1f}s/'
                             f'{a[sel, 3].mean():.1f}/{a[sel, 5].max():.2f}/{self.half_corridor - a[sel, 10].max():+.2f}')
        print('구간(s m: 시간/평균속력/최대횡오차/최소통로여유) ' + ' '.join(parts), flush=True)


def main():
    rclpy.init()
    args = sys.argv[1:]
    stop_s = None
    if '--stop-s' in args:
        k = args.index('--stop-s')
        stop_s = float(args[k + 1])
        del args[k:k + 2]
    node = LapLogger(args[0] if args else None, stop_s)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    aborted = node.aborted
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(3 if aborted else 0)


if __name__ == '__main__':
    main()
