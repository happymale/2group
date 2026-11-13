"""시뮬레이터 이벤트 받기 (세 노드 공통, 보통 고칠 필요가 없습니다).

/sim/events의 START·RESPAWN·END를 받아 노드에 알려 줍니다.
  on_reset(checkpoint, reload): 시작(출발 = 0, 연습 모드면 고른 체크포인트)·복귀. 시작이면 reload = True(파일을 다시 읽음).
  on_end(): 시도가 끝남.
시도 중에 노드를 다시 켜서 START를 못 받았으면(이벤트 기록에서 밀려남), 다음 RESPAWN부터 그 자세로 다시 시작합니다.
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from hk_interfaces.msg import SimEvent


class SimEvents:
    def __init__(self, node, on_reset, on_end):
        self.on_reset, self.on_end = on_reset, on_end
        self.active = False
        self.seen, self.ended = set(), set()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(SimEvent, '/sim/events', self._on_event, qos)

    def _on_event(self, ev):
        if (ev.attempt_id, ev.event_id) in self.seen:
            return
        self.seen.add((ev.attempt_id, ev.event_id))
        if ev.kind == SimEvent.START:
            self.active = True
            self.on_reset(ev.checkpoint, True)
        elif ev.kind == SimEvent.RESPAWN and ev.attempt_id not in self.ended:
            reload = not self.active
            self.active = True
            self.on_reset(ev.checkpoint, reload)
        elif ev.kind == SimEvent.END:
            self.ended.add(ev.attempt_id)
            self.active = False
            self.on_end()
