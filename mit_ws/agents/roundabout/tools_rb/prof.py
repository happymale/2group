import sys, numpy as np, rclpy
rclpy.init(args=['--ros-args', '--params-file', '/mit_ws/agents/roundabout/install/hk_mit/share/hk_mit/config/bringup.yaml',
                 '--params-file', '/mit_ws/agents/roundabout/install/hk_mit/share/hk_mit/config/all_options.yaml'])
from hk_mit.planner import Planner
n = Planner(); n.on_reset(5, True)
print(n.ring_s)
for s0 in range(300, 400, 2):
    i = np.searchsorted(n.profile_s, s0); print(s0, round(float(n.profile_v[i]), 2), end=' | ')
print()
rs = n.speed_reference
_, _, k, _ = rs.evaluate(np.arange(330, 380, 1.0)); print(np.round(k, 3))
