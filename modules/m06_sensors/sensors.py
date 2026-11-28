"""Module 06: sensor models for the Go2 in MuJoCo, from first principles.

build_model()     derived MJCF (MjSpec): gyro, accelerometer and orientation at the Menagerie `imu` site and at a second IMU on
                  the lidar mount of the Gazebo URDF (module 02), joint encoders, foot touch sensors, and an optional mocap
                  "calibration rig" welded to the base (inactive unless switched on).
Inertial        noise parameters of one triaxial sensor; corrupt() applies scale/misalignment, bias random walk, white noise,
                  saturation and quantisation to a truth signal.
encoder()         joint angle quantisation; backward-difference velocity.
allan()           overlapping Allan variance; allan_fit() reads N (white), K (rate random walk), Q (quantisation) from it.
bias_sd()         predicted standard deviation of a static bias estimate averaged over T seconds.
calib_known()     accelerometer scale/misalignment and bias from static poses with known orientation (linear least squares).
calib_unknown()   the same without orientations: |U (f - b)| = g, solved with module 00's Levenberg-Marquardt.
xcorr_offset()    time offset between two sampled signals: cross-correlation peak with parabolic sub-sample interpolation.
detect_contact()  thresholded foot force, optionally with hysteresis.
MuJoCo is used here only to build and compile the model; nothing below computes a sensor quantity with a MuJoCo function."""
import pathlib
import sys
from dataclasses import dataclass, field

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path += [str(HERE.parent / 'm00_math'), str(HERE.parent / 'm02_description')]   # appended: their validate.py must not shadow
import sim  # noqa: E402  (sets MUJOCO_GL before mujoco is imported)
import mujoco  # noqa: E402
from common import JOINTS, LEGS, require_model  # noqa: E402
from optim import levenberg_marquardt  # noqa: E402  (module 00)

IMU_RATE_DIV = 2                    # IMU sampled every 2nd physics step: 250 Hz with the 2 ms step
TOUCH_RADIUS = 1.2 * sim.FOOT_RADIUS


def lidar_mount():
    """Position of `lidar_link` in the trunk frame of the Gazebo URDF (module 02 parser). The MJCF base frame is the trunk."""
    import description as D
    _, joints, _ = D.parse_urdf(D.GAZEBO_URDF)
    by_child = {j.child: j for j in joints.values()}

    def pose(link):                                     # T_base_link of `link`, chained through fixed joints
        if link == 'base_link':
            return np.eye(3), np.zeros(3)
        j = by_child[link]; R, p = pose(j.parent)
        return R @ j.R, R @ j.xyz + p
    Rt, pt = pose('trunk'); Rl, pl = pose('lidar_link')
    return Rt.T @ Rl, Rt.T @ (pl - pt)


def build_model(weld_timeconst=0.004):
    """Go2 scene plus sensors (names below) and an inactive weld from the base to a mocap body `rig`."""
    s = mujoco.MjSpec.from_file(require_model(scene=True))
    base = s.body('base')
    R_l, p_l = lidar_mount()
    assert np.allclose(R_l, np.eye(3))                  # sensors.xacro mounts the lidar with rpy 0 0 0
    base.add_site(name='imu_lidar', pos=p_l)
    S, O = mujoco.mjtSensor, mujoco.mjtObj
    for site, tag in (('imu', ''), ('imu_lidar', '_lidar')):
        s.add_sensor(name='gyro' + tag, type=S.mjSENS_GYRO, objtype=O.mjOBJ_SITE, objname=site)
        s.add_sensor(name='acc' + tag, type=S.mjSENS_ACCELEROMETER, objtype=O.mjOBJ_SITE, objname=site)
        s.add_sensor(name='quat' + tag, type=S.mjSENS_FRAMEQUAT, objtype=O.mjOBJ_SITE, objname=site)
    for leg in LEGS:
        for p in JOINTS:
            s.add_sensor(name=f'q_{leg}_{p}', type=S.mjSENS_JOINTPOS, objtype=O.mjOBJ_JOINT, objname=f'{leg}_{p}_joint')
            s.add_sensor(name=f'dq_{leg}_{p}', type=S.mjSENS_JOINTVEL, objtype=O.mjOBJ_JOINT, objname=f'{leg}_{p}_joint')
        s.body(f'{leg}_calf').add_site(name=f'touch_{leg}', pos=[-0.002, 0, -0.213], size=[TOUCH_RADIUS, 0, 0],
                                       type=mujoco.mjtGeom.mjGEOM_SPHERE, rgba=[0, 0, 0, 0])
        s.add_sensor(name=f'touch_{leg}', type=S.mjSENS_TOUCH, objtype=O.mjOBJ_SITE, objname=f'touch_{leg}')
    s.worldbody.add_body(name='rig', mocap=True, pos=[0, 0, 0.6])
    e = s.add_equality(type=mujoco.mjtEq.mjEQ_WELD, name1='base', name2='rig', objtype=O.mjOBJ_BODY)
    e.active = False
    e.solref = [weld_timeconst, 1.0]
    e.data = np.r_[np.zeros(6), 1, 0, 0, 0, 1, np.zeros(len(e.data) - 11)]   # anchor, relative pose = identity, torquescale
    m = s.compile()
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    return m, mujoco.MjData(m)


def sensor_slices(m):
    """{sensor name: slice into d.sensordata}."""
    return {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i): slice(m.sensor_adr[i], m.sensor_adr[i] + m.sensor_dim[i])
            for i in range(m.nsensor)}


def read(m, d, names, sl=None):
    """Concatenated sensordata of `names`."""
    sl = sl or sensor_slices(m)
    return np.concatenate([d.sensordata[sl[n]] for n in names])


# ---- noise models --------------------------------------------------------------------------------------------------------

@dataclass
class Inertial:
    """One triaxial inertial sensor. Units: gyro rad/s, accelerometer m/s^2 (written `u` below).
    N: white noise density [u/sqrt(Hz)]  (per-sample sd N/sqrt(dt));  K: bias random-walk density [u/s/sqrt(Hz)];
    q: quantisation step [u] (0: none);  limit: saturation [u];  b0: initial bias [u];  M: scale factor and misalignment."""
    N: float
    K: float
    q: float = 0.0
    limit: float = np.inf
    b0: np.ndarray = field(default_factory=lambda: np.zeros(3))
    M: np.ndarray = field(default_factory=lambda: np.eye(3))

    def corrupt(self, truth, dt, rng):
        """y_k = Q(sat(M x_k + b_k + n_k)),  b_{k+1} = b_k + K sqrt(dt) w_k,  n_k ~ N(0, N^2/dt). Returns (y, b)."""
        x = np.asarray(truth, float)
        b = self.b0 + np.cumsum(self.K * np.sqrt(dt) * rng.standard_normal(x.shape), axis=0)
        Mx = x if np.array_equal(self.M, np.eye(3)) else x @ self.M.T          # other widths: one axis per column
        y = Mx + b + self.N / np.sqrt(dt) * rng.standard_normal(x.shape)
        y = np.clip(y, -self.limit, self.limit)
        return (quantise(y, self.q) if self.q else y), b


def quantise(x, q):
    """Mid-tread uniform quantiser with step q."""
    return q * np.round(np.asarray(x) / q)


def encoder(q_true, res, dt):
    """Quantised joint angle and its backward-difference velocity (first sample: 0)."""
    qm = quantise(q_true, res)
    v = np.zeros_like(qm); v[1:] = np.diff(qm, axis=0) / dt
    return qm, v


# ---- Allan variance ------------------------------------------------------------------------------------------------------

def allan(y, dt, ms):
    """Overlapping Allan variance of a rate signal y (n, ...) for cluster sizes ms. theta = integral of y;
    sigma^2(tau) = sum_k (theta_{k+2m} - 2 theta_{k+m} + theta_k)^2 / (2 tau^2 (n + 1 - 2m)), tau = m dt."""
    y = y - y.mean(axis=0)              # sigma^2 is unchanged by a constant; removing it keeps the cumulative sum small (rounding)
    theta = np.concatenate([np.zeros((1,) + y.shape[1:]), np.cumsum(y, axis=0) * dt])
    out = []
    for m in ms:
        d2 = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        out.append((d2 ** 2).mean(axis=0) / (2 * (m * dt) ** 2))
    return np.array(ms) * dt, np.array(out)


def allan_nonoverlap(y, dt, m):
    """Non-overlapping Allan variance from cluster means: sigma^2 = 1/2 mean((ybar_{j+1} - ybar_j)^2). Independent code path."""
    n = (len(y) // m) * m
    yb = y[:n].reshape((n // m, m) + y.shape[1:]).mean(axis=1)
    return 0.5 * (np.diff(yb, axis=0) ** 2).mean(axis=0)


def log_taus(n, dt, per_decade=8, min_clusters=9):
    """Cluster sizes on a log grid, from one sample up to n / min_clusters samples."""
    ms = np.unique(np.round(np.logspace(0, np.log10(n / min_clusters), int(per_decade * np.log10(n / min_clusters)) + 1)))
    return ms.astype(int)


def allan_fit(tau, avar, n_samples=None, dt=None):
    """Weighted least squares of sigma^2(tau) = 3 Q^2 / tau^2 + N^2 / tau + K^2 tau / 3 (each term >= 0, active set: a
    negative coefficient is dropped and the fit repeated). Weight 1/sigma^2(tau): relative residuals, so every octave counts.
    Returns dict(Q, N, K)."""
    A = np.stack([1 / tau ** 2, 1 / tau, tau], axis=1) / avar[:, None]
    rhs = np.ones_like(avar)
    active = [0, 1, 2]
    while True:
        x = np.zeros(3)
        x[active] = np.linalg.lstsq(A[:, active], rhs, rcond=None)[0]
        if (x[active] >= 0).all():
            break
        active.remove(active[int(np.argmin(x[active]))])
    return dict(Q=float(np.sqrt(x[0] / 3)), N=float(np.sqrt(x[1])), K=float(np.sqrt(3 * x[2])))


def allan_model(tau, N, K, Q=0.0):
    return 3 * Q ** 2 / tau ** 2 + N ** 2 / tau + K ** 2 * tau / 3


# ---- static calibration --------------------------------------------------------------------------------------------------

def bias_sd(N, K, T):
    """sd of (mean of y over [0, T]) - b(T) for white density N and bias random walk K: sqrt(N^2/T + K^2 T/3)."""
    return np.sqrt(N ** 2 / T + K ** 2 * T / 3)


def calib_known(f_ref, f_meas):
    """f_meas = M f_ref + b from n >= 4 non-coplanar static poses: linear least squares [f_ref 1] X = f_meas. Returns (M, b)."""
    A = np.hstack([f_ref, np.ones((len(f_ref), 1))])
    X = np.linalg.lstsq(A, f_meas, rcond=None)[0]
    return X[:3].T, X[3]


IU = np.triu_indices(3)


def calib_unknown(f_meas, g, x0=None):
    """Orientation-free calibration: find upper-triangular U and b with |U (f_i - b)| = g for every static pose.
    The rotation part of M^-1 is not observable from magnitudes, so U is the triangular factor of M^-1 = Q U.
    Residual r_i = |w_i| - g, w_i = U z_i, z_i = f_i - b;  dr_i/dU_jk = w_ij z_ik / |w_i| (j <= k),  dr_i/db = -w_i^T U / |w_i|.
    Returns (U, b, LM result)."""
    def fun(x):
        U = np.zeros((3, 3)); U[IU] = x[:6]
        z = f_meas - x[6:]; w = z @ U.T; nw = np.linalg.norm(w, axis=1)
        u = w / nw[:, None]
        J = np.hstack([u[:, IU[0]] * z[:, IU[1]], -u @ U])
        return nw - g, J
    x0 = np.r_[np.eye(3)[IU], 0, 0, 0] if x0 is None else x0
    res = levenberg_marquardt(fun, x0)
    U = np.zeros((3, 3)); U[IU] = res['x'][:6]
    return U, res['x'][6:], res


def triangular_factor(Minv):
    """Upper-triangular U with positive diagonal and rotation Q such that Minv = Q U (QR decomposition, signs fixed)."""
    Q, U = np.linalg.qr(Minv)
    s = np.sign(np.diag(U))
    return (s[:, None] * U), Q * s


# ---- time offset ---------------------------------------------------------------------------------------------------------

def xcorr(a, b, max_lag):
    """Normalised cross-correlation c[k] = sum_n a_n b_{n+k} / (|a| |b|) over the overlap, k = -max_lag..max_lag."""
    a = a - a.mean(); b = b - b.mean(); n = len(a)
    lags = np.arange(-max_lag, max_lag + 1)
    c = np.empty(len(lags))
    for i, k in enumerate(lags):
        x, y = (a[:n - k], b[k:]) if k >= 0 else (a[-k:], b[:n + k])
        c[i] = x @ y / np.sqrt((x @ x) * (y @ y))
    return lags, c


def xcorr_offset(a, b, dt, max_lag):
    """Delay d of b relative to a (b(t) = a(t - d)): peak of xcorr plus a parabola through the peak and its neighbours,
    delta = (c_{-1} - c_{+1}) / (2 (c_{-1} - 2 c_0 + c_{+1})). Returns (d, lags, c)."""
    lags, c = xcorr(a, b, max_lag)
    i = int(np.argmax(c)); delta = 0.0
    if 0 < i < len(c) - 1:
        den = c[i - 1] - 2 * c[i] + c[i + 1]
        delta = 0.5 * (c[i - 1] - c[i + 1]) / den if den < 0 else 0.0
    return (lags[i] + delta) * dt, lags, c


# ---- contact -------------------------------------------------------------------------------------------------------------

def detect_contact(F, on, off=None):
    """Contact flags from normal force samples F (n, legs). Plain threshold when off is None, else hysteresis: switch on above
    `on`, off below `off` (off < on). Starts in contact if the first sample is above `on`."""
    if off is None:
        return F > on
    out = np.zeros(F.shape, bool); state = F[0] > on
    for k in range(len(F)):
        state = np.where(state, F[k] > off, F[k] > on); out[k] = state
    return out


def toggles(flags):
    """Number of state changes per column."""
    return np.abs(np.diff(flags.astype(int), axis=0)).sum(axis=0)
