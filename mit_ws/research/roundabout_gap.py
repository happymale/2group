#!/usr/bin/env python3
"""회전교차로 NPC 추적 + 진입 간격 판단 + IDM 추종 스케치 (표준 라이브러리만, ROS 불필요).

course.json: m_round center=(108.8353, 118.7754), npc_radius=11.0.
우리 차: 동쪽점(각 0)에서 합류 → 반시계(CCW) 1/4 바퀴 → 북쪽점(각 +90°)에서 북쪽으로 이탈.
NPC도 같은 원 위를 CCW로 돈다고 가정 → 상태를 원 위의 각도 phi, 각속도 w 로 두면
2D 위치 대신 1D 칼만 필터(등각속도 모델)로 충분하고 데이터 연관도 각도 최근접으로 단순해집니다.

통합 방법(planner.py에 추가할 위치):
  1) lidar_perception이 발행하는 /hk/perception/obstacles(map 좌표 군집)를 받아
     군집 중심 중 |dist(center) - 11| < 1.5 m 인 것만 링 후보로 사용.
  2) RingTracker.update(t, [phi...]) 를 10 Hz로 호출.
  3) 합류 전: merge_ok(...) 가 False면 정지선(동쪽점 전 ~3 m)에서 speed_limit을 0으로.
  4) 링 위: idm_speed(...) 로 앞 NPC를 따라가며 speed_limit 상한을 발행.
"""
import math

CENTER = (108.8353, 118.7754)
RING_R = 11.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Track:
    def __init__(self, t, phi):
        self.t, self.phi, self.w = t, phi, 0.0
        # 공분산 [[Ppp, Ppw], [Ppw, Pww]]
        self.P = [[0.05, 0.0], [0.0, 0.3]]
        self.hits, self.miss = 1, 0

    def predict(self, t, q=0.05):
        dt = t - self.t
        if dt <= 0:
            return
        self.phi = wrap(self.phi + self.w * dt)
        P = self.P
        p00 = P[0][0] + 2 * dt * P[0][1] + dt * dt * P[1][1] + q * dt ** 3 / 3
        p01 = P[0][1] + dt * P[1][1] + q * dt ** 2 / 2
        p11 = P[1][1] + q * dt
        self.P = [[p00, p01], [p01, p11]]
        self.t = t

    def correct(self, phi_meas, r=(0.15 / RING_R) ** 2):
        y = wrap(phi_meas - self.phi)
        S = self.P[0][0] + r
        k0, k1 = self.P[0][0] / S, self.P[0][1] / S
        self.phi = wrap(self.phi + k0 * y)
        self.w += k1 * y
        p00, p01, p11 = self.P[0][0], self.P[0][1], self.P[1][1]
        self.P = [[(1 - k0) * p00, (1 - k0) * p01], [(1 - k0) * p01, p11 - k1 * p01]]
        self.hits += 1
        self.miss = 0


class RingTracker:
    """최근접 이웃(각도 게이트) + 등각속도 KF. 군집 중심이 차 뒷면만 보여도 각도는 거의 정확."""

    def __init__(self, gate_rad=2.5 / RING_R, max_miss=5):
        self.tracks, self.gate, self.max_miss = [], gate_rad, max_miss

    @staticmethod
    def to_phi(x, y):
        return math.atan2(y - CENTER[1], x - CENTER[0])

    def update(self, t, phis):
        for tr in self.tracks:
            tr.predict(t)
        free = list(phis)
        # 탐욕적 최근접 연관 (NPC 수가 적어 헝가리안 불필요)
        pairs = sorted(((abs(wrap(p - tr.phi)), i, j) for i, tr in enumerate(self.tracks)
                        for j, p in enumerate(free)))
        used_t, used_m = set(), set()
        for d, i, j in pairs:
            if d > self.gate or i in used_t or j in used_m:
                continue
            self.tracks[i].correct(free[j])
            used_t.add(i)
            used_m.add(j)
        for i, tr in enumerate(self.tracks):
            if i not in used_t:
                tr.miss += 1
        self.tracks = [tr for tr in self.tracks if tr.miss <= self.max_miss]
        for j, p in enumerate(free):
            if j not in used_m:
                self.tracks.append(Track(t, p))

    def confirmed(self):
        return [tr for tr in self.tracks if tr.hits >= 3]


def merge_ok(tracks, ego_dist_to_merge, ego_speed, ego_ring_speed, t_crit=2.5, follow_T=1.0,
             b_npc=3.0, s0=3.0, car_len=4.7):
    """합류 판단. 합류점 = 각 0 (동쪽점). CCW 진행이므로 '상류'는 phi<0 쪽(남동쪽)에서 온다.
    조건 1 (뒤에서 오는 차): NPC가 합류점에 도착하는 시간 - 우리가 합류점을 지나 차 길이만큼
           벗어나는 시간 > t_crit (임계 간격, 문헌상 회전교차로 4~5 s, 시뮬은 반응이 확실하므로 2~3 s)
    조건 2 (뒤차 안전 거리): NPC가 우리 뒤에 붙을 때 v_npc*T + (v_npc^2 - v_ego^2)/(2b) 이상 떨어져 있을 것
    조건 3 (앞차): 합류점 앞(0 < phi < 90°+여유)에 있는 NPC와의 거리 > s0 + 차 길이.
    """
    t_ego = (ego_dist_to_merge + car_len) / max(ego_speed, 0.5)
    for tr in tracks:
        v = abs(tr.w) * RING_R
        arc_behind = -tr.phi * RING_R if tr.phi < 0 else (2 * math.pi - tr.phi) * RING_R
        arc_ahead = tr.phi * RING_R if tr.phi >= 0 else None
        # 뒤에서 오는 차 (합류점까지 남은 호 길이)
        if arc_behind < 40.0:
            t_npc = arc_behind / max(v, 0.1)
            if t_npc - t_ego < t_crit:
                return False
            gap_at_merge = arc_behind - v * t_ego
            need = v * follow_T + max(0.0, v * v - ego_ring_speed ** 2) / (2 * b_npc)
            if gap_at_merge < need + car_len:
                return False
        # 앞에 있는 차 (우리 1/4 바퀴 구간)
        if arc_ahead is not None and arc_ahead < (math.pi / 2) * RING_R + 8.0:
            if arc_ahead < s0 + car_len + ego_ring_speed * 0.5:
                return False
    return True


def idm_speed(v, v_lead, gap, v0=6.5, T=1.0, a=2.0, b=3.0, s0=2.5, dt=0.1):
    """IDM: 다음 제어 주기 목표 속력. gap = 앞차 뒷범퍼까지 우리 앞범퍼 거리(호 길이)."""
    if gap is None:
        s_term = 0.0
    else:
        s_star = s0 + max(0.0, v * T + v * (v - v_lead) / (2 * math.sqrt(a * b)))
        s_term = (s_star / max(gap, 0.1)) ** 2
    acc = a * (1 - (v / v0) ** 4 - s_term)
    return max(0.0, v + acc * dt)


if __name__ == '__main__':
    # 자체 시험: NPC 하나가 링에서 4 m/s로 돌 때 추적·합류 판단
    trk = RingTracker()
    w = 4.0 / RING_R
    for k in range(30):
        t = k * 0.1
        phi = wrap(-1.2 + w * t)
        trk.update(t, [phi + (0.01 if k % 2 else -0.01)])
    tr = trk.confirmed()[0]
    print('추정 phi=%.3f (참 %.3f), v=%.2f m/s' % (tr.phi, wrap(-1.2 + w * 2.9), tr.w * RING_R))
    for d in (2.0, 8.0):
        print('합류점까지 %.0f m, 4 m/s 접근 → merge_ok=%s' % (d, merge_ok(trk.confirmed(), d, 4.0, 5.0)))
    print('IDM: v=6, 앞차 4 m/s, 간격 8 m → 다음 목표 %.2f m/s' % idm_speed(6.0, 4.0, 8.0))
