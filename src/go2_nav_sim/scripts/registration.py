"""Planar scan registration used by lidar_odom.py. Pure numpy/scipy, no ROS, so it can be unit-tested anywhere.

register() is Gauss-Newton on point-to-line residuals against a map with PCA normals, Geman-McClure robust weights, and
optionally yaw held at the prediction (gyro-anchored). Translation gets a prior toward the prediction only along directions the
scan barely constrains (eigenvalues of the data information below deg_eig), as in Zhang et al., "On degeneracy of
optimization-based state estimation problems", ICRA 2016. A uniform prior would bias every well-constrained solution toward the
prediction (2.5 cm for a 25 cm prediction error in the unit test); this one leaves them untouched and only stops sliding along a
corridor."""
import bisect, math, threading
import numpy as np
from scipy.spatial import cKDTree  # type: ignore[reportAttributeAccessIssue]  # scipy ships _ckdtree compiled, without a stub


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def voxel_downsample(p, voxel):
    _, i = np.unique(np.floor(p / voxel).astype(np.int64), axis=0, return_index=True)
    return p[i]


class LineMap:
    """Point map with per-point normals from a PCA over k neighbours; line-like neighbourhoods (walls) get weight ~1."""

    def __init__(self, pts, k=8):
        self.pts = pts; self.tree = cKDTree(pts)
        k = min(k, len(pts)); _, idx = self.tree.query(pts, k=k)
        nb = pts[idx] - pts[idx].mean(1, keepdims=True)
        c = np.einsum('mki,mkj->mij', nb, nb) / k
        ang = 0.5 * np.arctan2(2 * c[:, 0, 1], c[:, 0, 0] - c[:, 1, 1])          # principal (line) direction
        self.nrm = np.stack([-np.sin(ang), np.cos(ang)], 1)
        tr = c[:, 0, 0] + c[:, 1, 1]; det = c[:, 0, 0] * c[:, 1, 1] - c[:, 0, 1] ** 2
        lam_min = 0.5 * (tr - np.sqrt(np.maximum(tr * tr - 4 * det, 0)))
        self.wgt = np.clip(1.0 - lam_min / (0.25 * tr + 1e-9), 0.15, 1.0)


def register(src, lmap, guess, sigma=0.12, trans_prior=10.0, fix_yaw=True, iters=12, min_matches=25, deg_eig=0.02):
    """Align scan points src (Nx2, robot frame) to lmap. Returns (pose [x, y, yaw], matches, rms, ok, cov_xy).
    deg_eig is per matched point: the information grows with the number of matches, so the threshold is normalised by it.
    cov_xy = rms^2 * inverse of the 2x2 translation information, the Gauss-Newton approximation of the estimate's covariance."""
    x = np.array(guess, float); matches = 0; rms = float('nan'); H = np.eye(3)
    for it in range(iters):
        q = src @ rot(x[2]).T + x[:2]
        tau = max(0.25, 0.8 - 0.1 * it)                                         # shrinking association gate
        d, j = lmap.tree.query(q, distance_upper_bound=tau)
        ok = np.isfinite(d)
        if ok.sum() < min_matches:
            return x, int(ok.sum()), float('nan'), False, None
        n = lmap.nrm[j[ok]]; res = np.einsum('ij,ij->i', n, q[ok] - lmap.pts[j[ok]])
        w = lmap.wgt[j[ok]] * (sigma ** 2 / (sigma ** 2 + res ** 2)) ** 2          # Geman-McClure
        perp = np.stack([-(q[ok][:, 1] - x[1]), q[ok][:, 0] - x[0]], 1)
        J = np.column_stack([n[:, 0], n[:, 1], np.einsum('ij,ij->i', n, perp)])
        H = J.T @ (J * w[:, None]) + 1e-6 * np.eye(3); g = J.T @ (w * res)
        ev, V = np.linalg.eigh(H[:2, :2])                                        # translation information per direction
        Pr = sum(trans_prior * np.outer(V[:, k], V[:, k]) for k in range(2) if ev[k] / ok.sum() < deg_eig) if trans_prior else 0
        if np.ndim(Pr):                                                          # prior only where the scan is degenerate
            H[:2, :2] += Pr; g[:2] += Pr @ (x[:2] - np.asarray(guess[:2], float))
        if fix_yaw:
            dx = np.zeros(3); dx[:2] = -np.linalg.solve(H[:2, :2], g[:2])
        else:
            dx = -np.linalg.solve(H, g)
        x += dx; x[2] = wrap(x[2]); matches = int(ok.sum()); rms = float(np.sqrt(np.mean(res ** 2)))
        if np.linalg.norm(dx[:2]) < 1e-4 and abs(dx[2]) < 1e-4:
            break
    cov = (rms ** 2) * np.linalg.inv(H[:2, :2] + 1e-9 * np.eye(2))
    return x, matches, rms, True, cov


class GyroYaw:
    """Integrates the yaw rate into a timestamped history and answers yaw increments by time, so callback order and late samples
    do not matter. With roll and pitch available the Euler-rate relation is used, psi_dot = (sin(roll) w_y + cos(roll) w_z) / cos(pitch);
    with roll = pitch = 0 it reduces to w_z. Samples may arrive out of order; they are kept sorted. Thread-safe."""

    def __init__(self, maxlen=800):
        self.lock = threading.Lock(); self.maxlen = maxlen; self.samples = []        # sorted list of (t, yaw_rate)

    @staticmethod
    def yaw_rate(wy, wz, roll=0.0, pitch=0.0):
        return (math.sin(roll) * wy + math.cos(roll) * wz) / math.cos(pitch)

    def add(self, t, wz, wy=0.0, roll=0.0, pitch=0.0):
        with self.lock:
            bisect.insort(self.samples, (t, self.yaw_rate(wy, wz, roll, pitch)))
            if len(self.samples) > self.maxlen:
                del self.samples[0]

    def dyaw(self, t0, t1):
        with self.lock:
            s = np.array(self.samples)
        if len(s) < 10 or s[0, 0] > t0 or s[-1, 0] < t1 - 0.05:                   # IMU does not cover the interval
            return None
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (s[1:, 1] + s[:-1, 1]) * np.diff(s[:, 0]))])   # trapezoid
        return float(np.interp(t1, s[:, 0], cum) - np.interp(t0, s[:, 0], cum))
