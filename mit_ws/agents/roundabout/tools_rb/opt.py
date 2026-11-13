"""뒤차축 경로 수치 최적화: 기본 원호 설계 + 법선 방향 오프셋 d(σ) (스플라인), 최소 차체 여유 최대화."""
import sys, math, time
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize
sys.path.insert(0, '/mit_ws/agents/roundabout/tools_rb')
from geo_eval import route, C, ring_reference, st as rst
seg = np.diff(route, axis=0); sl = np.linalg.norm(seg, axis=1); sd = seg / sl[:, None]
_, base, g = ring_reference(route, C, 11.0, 11.0, 7.25, 7.25)
bs = np.r_[0, np.cumsum(np.linalg.norm(np.diff(base, axis=0), axis=1))]
# 창: 기본 경로에서 원래 경로 s 312~356 부근
def base_s_of(xy): return bs[int(np.argmin(np.linalg.norm(base - xy, axis=1)))]
s0, s1 = base_s_of(route[np.searchsorted(rst, 314)]), base_s_of(route[np.searchsorted(rst, 356)])
fs = np.arange(s0 - 2, s1 + 2, 0.1)
B = CubicSpline(bs, base)
bx, by = B(fs)[:, 0], B(fs)[:, 1]
tg = B(fs, 1); tg /= np.linalg.norm(tg, axis=1, keepdims=True); nrm = np.column_stack((-tg[:, 1], tg[:, 0]))
K = int(sys.argv[1]) if len(sys.argv) > 1 else 14
knots = np.linspace(s0, s1, K + 2)
seg_lo = np.searchsorted(rst, 300); seg_hi = np.searchsorted(rst, 370)
A, D, Ls = route[seg_lo:seg_hi], sd[seg_lo:seg_hi], sl[seg_lo:seg_hi]
KMAX = 1 / 5.3
def path(dk):
    d = CubicSpline(knots, np.r_[0, dk, 0], bc_type='clamped')(np.clip(fs, s0, s1))
    return np.column_stack((bx, by)) + d[:, None] * nrm
def corners(p):
    h = np.gradient(p, axis=0); h /= np.linalg.norm(h, axis=1, keepdims=True)
    lat = np.column_stack((-h[:, 1], h[:, 0]))
    return np.concatenate([p + lx * h + ly * lat for lx in (-0.79, 3.845) for ly in (-0.946, 0.946)])
def dist(q):
    dl = q[:, None, :] - A[None]; al = np.clip(np.einsum('nmk,mk->nm', dl, D), 0, Ls)
    return np.min(np.linalg.norm(dl - al[..., None] * D[None], axis=2), axis=1)
def curv(p):
    d1 = np.gradient(p, 0.1, axis=0); d2 = np.gradient(d1, 0.1, axis=0)
    return (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.linalg.norm(d1, axis=1) ** 3
def f(dk, verbose=False):
    p = path(dk); m = 2.196 - dist(corners(p))
    soft = -np.log(np.sum(np.exp(-m / 0.03))) * 0.03
    k = curv(p); pen = np.sum(np.maximum(np.abs(k) - KMAX, 0) ** 2) * 50
    dk2 = np.gradient(k, 0.1); pen += np.sum(np.maximum(np.abs(dk2) - 0.12, 0) ** 2) * 5    # 곡률 변화율 (조향 속도)
    if verbose: print(f'min margin {m.min():+.3f} max|k| {np.abs(k).max():.3f} (R {1/np.abs(k).max():.2f}) max dk {np.abs(dk2).max():.3f}')
    return -soft + pen
x0 = np.zeros(K)
f(x0, True)
t = time.time()
r = minimize(f, x0, method='Powell', options={'maxiter': 20000, 'xtol': 1e-3, 'ftol': 1e-5})
print(time.time() - t, r.message)
f(r.x, True); print('dk', np.round(r.x, 3).tolist())
np.save('/mit_ws/agents/roundabout/tools_rb/opt_path.npy', path(r.x))
