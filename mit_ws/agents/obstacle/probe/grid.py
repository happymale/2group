import pickle, sys, numpy as np
d = pickle.load(open(sys.argv[1], 'rb'))
x0, x1, y0, y1 = map(float, sys.argv[2].split(','))
pts = np.vstack([c[2] for c in d['clouds'] if len(c[2])])
m = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
p = pts[m]
r = 0.5
xs = np.arange(x0, x1, r); ys = np.arange(y1, y0, -r)
H = np.zeros((len(ys), len(xs)), int)
ix = ((p[:, 0] - x0) / r).astype(int); iy = ((y1 - p[:, 1]) / r).astype(int)
np.add.at(H, (np.clip(iy, 0, len(ys)-1), np.clip(ix, 0, len(xs)-1)), 1)
print('      ' + ''.join(f'{x:<10.0f}' for x in xs[::20]))
for j, y in enumerate(ys):
    print(f'{y:5.1f} ' + ''.join('#' if h > 20 else ('+' if h > 3 else ('.' if h else ' ')) for h in H[j]))
