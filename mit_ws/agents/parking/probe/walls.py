import sys
import numpy as np
for fn in sys.argv[1:]:
    a = np.genfromtxt(fn, delimiter=',', names=True)
    def st(name, s, k):
        v = a[k][s]
        print(f'  {name:18s} n={len(v):6d}', np.percentile(v, [1, 50, 99]).round(3) if len(v) else '')
    print(fn)
    st('pocket N y', (a['x'] > -20) & (a['x'] < -17) & (a['y'] > 122) & (a['y'] < 132), 'y')
    st('pocket S y', (a['x'] > -20) & (a['x'] < -17) & (a['y'] < 122) & (a['y'] > 110), 'y')
    st('pocket back x', (a['x'] < -17) & (a['y'] > 110) & (a['y'] < 132), 'x')
    st('room N y', (a['x'] > -1.5) & (a['x'] < 2) & (a['y'] > 78), 'y')
    st('room S y', (a['x'] > -1.5) & (a['x'] < 2) & (a['y'] < 71), 'y')
    st('room E x', (a['y'] > 66) & (a['y'] < 83) & (a['x'] > 1), 'x')
    st('room W x', (a['y'] > 63) & (a['y'] < 86) & (a['x'] > -5) & (a['x'] < -1) & ((a['y'] < 71.5) | (a['y'] > 77.5)), 'x')
    st('passage N y', (a['x'] > -9) & (a['x'] < -4) & (a['y'] > 75) & (a['y'] < 80), 'y')
    st('passage S y', (a['x'] > -9) & (a['x'] < -4) & (a['y'] < 74) & (a['y'] > 69), 'y')
    s = (a['x'] > -1.5) & (a['x'] < 2) & (a['y'] > 65.5) & (a['y'] < 83.5)
    print('  room interior pts', s.sum())
