"""UTM zone 52N <-> WGS84 (Krueger series, mm accuracy) without pyproj."""
import math

A = 6378137.0
F = 1 / 298.257223563
K0 = 0.9996
E0 = 500000.0
ZONE = 52
LON0 = math.radians(-183 + 6 * ZONE)
# MORAI K-City map origin (also origin_utm_52n in course/morai_course.json).
ORIGIN = (302595.0, 4124145.0)

_n = F / (2 - F)
_A = A / (1 + _n) * (1 + _n ** 2 / 4 + _n ** 4 / 64)
_alpha = (_n / 2 - 2 * _n ** 2 / 3 + 5 * _n ** 3 / 16, 13 * _n ** 2 / 48 - 3 * _n ** 3 / 5, 61 * _n ** 3 / 240)
_beta = (_n / 2 - 2 * _n ** 2 / 3 + 37 * _n ** 3 / 96, _n ** 2 / 48 + _n ** 3 / 15, 17 * _n ** 3 / 480)
_delta = (2 * _n - 2 * _n ** 2 / 3 - 2 * _n ** 3, 7 * _n ** 2 / 3 - 8 * _n ** 3 / 5, 56 * _n ** 3 / 15)
_e = math.sqrt(F * (2 - F))


def ll_to_utm(lat_deg: float, lon_deg: float):
    phi = math.radians(lat_deg)
    lam = math.radians(lon_deg) - LON0
    t = math.sinh(math.atanh(math.sin(phi)) - 2 * math.sqrt(_n) / (1 + _n) * math.atanh(2 * math.sqrt(_n) / (1 + _n) * math.sin(phi)))
    xi_p = math.atan2(t, math.cos(lam))
    eta_p = math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    xi = xi_p + sum(a * math.sin(2 * j * xi_p) * math.cosh(2 * j * eta_p) for j, a in enumerate(_alpha, 1))
    eta = eta_p + sum(a * math.cos(2 * j * xi_p) * math.sinh(2 * j * eta_p) for j, a in enumerate(_alpha, 1))
    return E0 + K0 * _A * eta, K0 * _A * xi


def utm_to_ll(e: float, n: float):
    xi = n / (K0 * _A)
    eta = (e - E0) / (K0 * _A)
    xi_p = xi - sum(b * math.sin(2 * j * xi) * math.cosh(2 * j * eta) for j, b in enumerate(_beta, 1))
    eta_p = eta - sum(b * math.cos(2 * j * xi) * math.sinh(2 * j * eta) for j, b in enumerate(_beta, 1))
    chi = math.asin(math.sin(xi_p) / math.cosh(eta_p))
    phi = chi + sum(d * math.sin(2 * j * chi) for j, d in enumerate(_delta, 1))
    lam = LON0 + math.atan2(math.sinh(eta_p), math.cos(xi_p))
    return math.degrees(phi), math.degrees(lam)


def map_to_ll(x: float, y: float, origin=ORIGIN):
    """Map point -> (lat, lon). `origin` is the course's origin_utm_52n."""
    return utm_to_ll(x + origin[0], y + origin[1])


def ll_to_map(lat: float, lon: float, origin=ORIGIN):
    e, n = ll_to_utm(lat, lon)
    return e - origin[0], n - origin[1]

