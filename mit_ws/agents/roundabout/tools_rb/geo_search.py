import sys
sys.path.insert(0, '/mit_ws/agents/roundabout/tools_rb')
from geo_eval import *
for SM in (1.5, 2.5, 3.5):
 for rr, re, rx in ((11.0, 7.25, 7.25),):
     _, ref, g = ring_reference(route, C, 11.0, rr, re, rx, smooth_m=SM)
     m1, w1 = margin(ref, 300, 336); m2, w2 = margin(ref, 336, 365)
     print(f'ring {rr} Re {re} Rx {rx}: entry {m1:+.2f} at {w1[1]:.1f} {w1[2]} | exit {m2:+.2f} at {w2[1]:.1f} {w2[2]}', flush=True)
print(g)
