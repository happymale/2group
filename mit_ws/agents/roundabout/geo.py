import csv,math
import numpy as np
P=np.array([(float(r['x']),float(r['y'])) for r in csv.DictReader(open('/mit_ws/agents/roundabout/src/hk_mit/course/route.csv'))])
s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]
c=np.array([108.8353,118.7754])
for i in range(len(P)):
    if 270<=s[i]<=410 and (i%6==0 or 318<s[i]<352):
        r=np.linalg.norm(P[i]-c); a=math.degrees(math.atan2(*(P[i]-c)[::-1]))
        h=math.degrees(math.atan2(*(P[min(i+1,len(P)-1)]-P[i])[::-1]))
        print(f"{i} {s[i]:6.1f} {P[i][0]:8.3f} {P[i][1]:8.3f} r={r:6.2f} ang={a:7.1f} hdg={h:7.1f}")
