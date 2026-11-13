import sys, numpy as np
a = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
on = np.abs(a['x'] + 13.16) < 1.0
def first(c):
    k = np.flatnonzero(c); return a['t'][k[0]] if len(k) else np.nan
t8 = first(on & (a['y'] < 134.78)); room = first(a['x'] > -5); t10 = first(on & (a['y'] < 57.78) & (a['t'] > room))
print('cp8->cp10 %.1f s' % (t10 - t8))
