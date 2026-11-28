"""Module 12: terrain perception for the Go2, from first principles.

terrain()          the test course as oriented boxes: flat start with an obstacle, a field of random blocks, three stairs, a
                   platform and a 12 degree down-slope. The floor plane z = 0 is part of the terrain.
raycast()          first hit of rays against those boxes and the floor: slab method in each box frame (README, Theory).
true_height()      ground-truth surface height, a vertical raycast from above.
region_label()     ground-truth semantic label of a point, from the course layout.
DepthCamera        pinhole depth camera: pixel rays, projection, the axial noise model sigma_z(z).
Lidar              spinning multi-beam LiDAR: beam directions from elevation and azimuth.
build_model()      Go2 scene with the course added (geom group 0), a `depth_cam` site on the base in the optical frame and a
                   `lidar` site on the base with the base orientation.
scan()             one depth image: the rays are cast by mj_multiRay (the simulated sensor), depth = range along the optical axis.
lidar_scan()       one LiDAR sweep cast by mj_multiRay: range per beam (nan outside [rmin, rmax]) and the world unit directions.
measure()          depth image -> world points, per-point height variance from the sensor and the Jacobian w.r.t. base pose.
ElevationMap       2.5-D grid: per-scan cell aggregation with a top-surface gate, scalar Kalman fusion of height and variance, information-form fusion.
features()         per-window plane fit: slope, roughness (rms residual) and step height (residual range).
traversability()   weighted, saturated sum of the three features against critical values.
semantic()         flat / slope / rough / stairs / obstacle from the same features.
MuJoCo is used here to build the scene and to cast the sensor's rays; the slab intersection, the map, the features and the labels
never call MuJoCo."""
import pathlib
import sys
from dataclasses import dataclass, field

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import sim  # noqa: E402  (sets MUJOCO_GL before mujoco is imported)
import mujoco  # noqa: E402
from common import require_model  # noqa: E402

RES = 0.05                                   # m, map cell size
MAP_X, MAP_Y = (-0.5, 6.5), (-1.2, 1.2)      # m, map extent (cell edges at multiples of RES)
LABELS = ('flat', 'slope', 'rough', 'stairs', 'obstacle')
FLAT, SLOPE, ROUGH, STAIRS, OBSTACLE = range(5)
SLOPE_DEG, RISE, TREAD = 12.0, 0.12, 0.30    # course design
X_ROUGH, X_STAIRS, X_PLATFORM, X_SLOPE = (1.4, 2.4), (2.7, 3.6), (3.6, 4.0), 4.0
TOP = 3 * RISE                               # platform height, m
X_SLOPE_END = X_SLOPE + TOP / np.tan(np.radians(SLOPE_DEG))
OBST_C, OBST_H = np.array([1.0, 0.65]), np.array([0.15, 0.15, 0.125])   # obstacle centre (x, y), half sizes, m
HALF_W = 1.0                                 # course half width in y, m
GROUP = 0                                    # geom group of the terrain; the robot's geoms are in groups 2 and 3
GATE = (0.02, 6.0)                           # top-surface gate of ElevationMap.aggregate: 2 cm + 6 sigma below the cell maximum


@dataclass
class Box:
    c: np.ndarray                            # centre, world
    h: np.ndarray                            # half sizes
    R: np.ndarray = field(default_factory=lambda: np.eye(3))   # box frame -> world


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def terrain(seed=0, block_h=(0.005, 0.06)):
    """List of Box. Blocks of 0.1 m with heights uniform in block_h (seeded); everything else fixed by the constants above."""
    rng = np.random.default_rng(seed)
    B = [Box(np.r_[OBST_C, OBST_H[2]], OBST_H.copy())]
    for i in range(round((X_ROUGH[1] - X_ROUGH[0]) / 0.1)):
        for j in range(round(2 * HALF_W / 0.1)):
            z = rng.uniform(*block_h)
            B.append(Box(np.array([X_ROUGH[0] + 0.05 + 0.1 * i, -HALF_W + 0.05 + 0.1 * j, z / 2]), np.array([0.05, 0.05, z / 2])))
    for k in range(3):
        top = RISE * (k + 1)
        B.append(Box(np.array([X_STAIRS[0] + TREAD * (k + 0.5), 0, top / 2]), np.array([TREAD / 2, HALF_W, top / 2])))
    B.append(Box(np.array([sum(X_PLATFORM) / 2, 0, TOP / 2]), np.array([(X_PLATFORM[1] - X_PLATFORM[0]) / 2, HALF_W, TOP / 2])))
    a = np.radians(SLOPE_DEG); R = rot_y(a); t = 0.05            # rot_y(+a) maps x to (cos a, 0, -sin a): descending in +x
    top_c = np.array([(X_SLOPE + X_SLOPE_END) / 2, 0, TOP / 2])
    B.append(Box(top_c - t * R[:, 2], np.array([TOP / np.sin(a) / 2, HALF_W, t]), R))
    return B


def region_label(x, y):
    """Ground-truth label from the layout (point-wise, arrays allowed). The platform is flat; the obstacle is its own region."""
    x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
    L = np.full(x.shape, FLAT)
    inside = np.abs(y) < HALF_W
    L[inside & (x >= X_ROUGH[0]) & (x < X_ROUGH[1])] = ROUGH
    L[inside & (x >= X_STAIRS[0]) & (x < X_STAIRS[1])] = STAIRS
    L[inside & (x >= X_SLOPE) & (x < X_SLOPE_END)] = SLOPE
    L[(np.abs(x - OBST_C[0]) < OBST_H[0]) & (np.abs(y - OBST_C[1]) < OBST_H[1])] = OBSTACLE
    return L


def raycast(boxes, o, u, floor=True, chunk=512):
    """First hit of rays o + t u (t > 0) on the boxes and the floor plane z = 0.
    Returns t (N,) (inf: no hit), which (N,) (box index, -1 floor, -2 none), normal (N, 3) world unit outward normal.
    Slab method: in each box frame the ray enters the box at max_i min(t1_i, t2_i) and leaves at min_i max(t1_i, t2_i)."""
    o = np.atleast_2d(np.asarray(o, float)); u = np.atleast_2d(np.asarray(u, float))
    o, u = np.broadcast_arrays(o, u)
    C = np.array([b.c for b in boxes]); Hh = np.array([b.h for b in boxes]); Rs = np.array([b.R for b in boxes])
    N = len(o)
    T = np.full(N, np.inf); W = np.full(N, -2); Nrm = np.zeros((N, 3))
    for s in range(0, N, chunk):
        oo, uu = o[s:s + chunk], u[s:s + chunk]
        ol = np.einsum('bji,nbj->nbi', Rs, oo[:, None, :] - C[None])          # R^T (o - c), box frame
        ul = np.einsum('bji,nj->nbi', Rs, uu)
        ul = np.where(np.abs(ul) < 1e-300, 1e-300, ul)                          # parallel to a slab: t = +-inf, sign kept
        t1, t2 = (-Hh - ol) / ul, (Hh - ol) / ul
        tn, tf = np.minimum(t1, t2), np.maximum(t1, t2)
        tin, tout, ax = tn.max(-1), tf.min(-1), tn.argmax(-1)
        tin = np.where((tin <= tout) & (tin > 0), tin, np.inf)                  # origin inside a box: ignored (never here)
        k = tin.argmin(1); n = np.arange(len(oo)); t = tin[n, k]
        nl = np.zeros((len(oo), 3)); nl[n, ax[n, k]] = -np.sign(ul[n, k, ax[n, k]])
        nrm = np.einsum('nij,nj->ni', Rs[k], nl)
        hit = np.isfinite(t)
        T[s:s + chunk] = t; W[s:s + chunk] = np.where(hit, k, -2); Nrm[s:s + chunk] = nrm
    if floor:
        with np.errstate(divide='ignore', invalid='ignore'):
            tp = np.where((u[:, 2] < 0) & (o[:, 2] > 0), -o[:, 2] / u[:, 2], np.inf)
        f = tp < T
        T[f] = tp[f]; W[f] = -1; Nrm[f] = (0, 0, 1)
    return T, W, Nrm


def true_height(boxes, xy, z0=2.0):
    """Surface height at points xy (N, 2): z0 minus the first hit of a ray cast straight down from z0."""
    xy = np.atleast_2d(xy)
    t, _, _ = raycast(boxes, np.c_[xy, np.full(len(xy), z0)], np.array([[0.0, 0.0, -1.0]]))
    return z0 - t


@dataclass
class DepthCamera:
    W: int = 80
    H: int = 60
    hfov_deg: float = 86.0
    pos_b: tuple = (0.34, 0.0, 0.02)         # optical centre in the base frame, m (in front of the head)
    pitch_deg: float = 40.0                  # optical axis below the base x axis
    zmin: float = 0.3
    zmax: float = 3.0                        # m, valid depth range

    @property
    def f(self):                             # focal length in pixels, square pixels: fx = fy
        return (self.W / 2) / np.tan(np.radians(self.hfov_deg) / 2)

    @property
    def K(self):
        return np.array([[self.f, 0, self.W / 2], [0, self.f, self.H / 2], [0, 0, 1.0]])

    def pixel_dirs(self):
        """(H, W, 3) d_c = K^-1 [u v 1] at pixel centres (z component 1), optical frame: x right, y down, z forward."""
        u, v = np.meshgrid(np.arange(self.W) + 0.5, np.arange(self.H) + 0.5)
        return np.stack([(u - self.W / 2) / self.f, (v - self.H / 2) / self.f, np.ones_like(u)], -1)

    def project(self, p_c):
        """(N, 3) optical-frame points -> (N, 2) pixel coordinates (u, v)."""
        q = p_c @ self.K.T
        return q[:, :2] / q[:, 2:3]

    @property
    def R_bc(self):
        """Optical frame -> base frame; columns are the optical x (right), y (down), z (forward, pitched down) axes."""
        a = np.radians(self.pitch_deg)
        z = np.array([np.cos(a), 0, -np.sin(a)]); x = np.array([0.0, -1.0, 0.0])
        return np.column_stack([x, np.cross(z, x), z])

    @staticmethod
    def sigma_z(z):
        """Axial depth noise sd, m: Nguyen, Izadi and Lovell (2012), Kinect: 0.0012 + 0.0019 (z - 0.4)^2."""
        return 0.0012 + 0.0019 * (np.asarray(z) - 0.4) ** 2


@dataclass
class Lidar:
    """16 beams from -15 to +15 deg elevation, n_az azimuth steps over 360 deg (a VLP-16-like pattern; values chosen here).
    Frame L is the base frame translated to pos_b: x forward, z up."""
    elev_deg: tuple = tuple(np.linspace(-15.0, 15.0, 16))
    n_az: int = 360
    pos_b: tuple = (0.0, 0.0, 0.12)          # on top of the base, m
    rmin: float = 0.1
    rmax: float = 30.0                       # m, valid range

    def dirs(self):
        """(n_elev * n_az, 3) unit beam directions in frame L: (cos e cos a, cos e sin a, sin e)."""
        e, a = np.meshgrid(np.radians(self.elev_deg), np.arange(self.n_az) * 2 * np.pi / self.n_az, indexing='ij')
        return np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1).reshape(-1, 3)


def build_model(boxes, cam, lidar=None):
    """(m, d): Go2 scene + terrain boxes (group 0, contype 0: the robot is posed kinematically) + sites `depth_cam`, `lidar`."""
    s = mujoco.MjSpec.from_file(require_model(scene=True))
    for i, b in enumerate(boxes):
        q = np.empty(4); mujoco.mju_mat2Quat(q, b.R.ravel())
        s.worldbody.add_geom(name=f'terrain{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=b.c, quat=q, size=b.h, group=GROUP,
                             contype=0, conaffinity=0, rgba=[0.72, 0.72, 0.70, 1])
    q = np.empty(4); mujoco.mju_mat2Quat(q, cam.R_bc.ravel())
    s.body('base').add_site(name='depth_cam', pos=cam.pos_b, quat=q, size=[0.01, 0, 0])
    s.body('base').add_site(name='lidar', pos=(lidar or Lidar()).pos_b, size=[0.01, 0, 0])
    m = s.compile()
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    return m, mujoco.MjData(m)


def scan(m, d, cam, site):
    """Noise-free depth image (H, W) (nan outside [zmin, zmax] or no hit); per ray (flat): geom id, range, unit direction, normal.
    Rays leave the `depth_cam` site along R_wc d_c / |d_c| and see geom group GROUP only."""
    dc = cam.pixel_dirs().reshape(-1, 3)
    R = d.site_xmat[site].reshape(3, 3)
    u = (dc / np.linalg.norm(dc, axis=1, keepdims=True)) @ R.T
    n = len(u); gid = np.zeros(n, np.int32); dist = np.zeros(n); nrm = np.zeros((n, 3))
    grp = np.zeros(6, np.uint8); grp[GROUP] = 1
    mujoco.mj_multiRay(m, d, d.site_xpos[site].copy(), u.ravel().copy(), grp, 1, -1, gid, dist, nrm.reshape(-1), n, 50.0)
    z = np.where(dist >= 0, dist / np.linalg.norm(dc, axis=1), np.nan)
    z[(z < cam.zmin) | (z > cam.zmax)] = np.nan
    return z.reshape(cam.H, cam.W), gid, dist, u, nrm


def lidar_scan(m, d, lidar, site):
    """One sweep: range (n,) along each beam (nan: no hit or outside [rmin, rmax]) and the world unit directions (n, 3).
    Range noise is not modelled. Rays see geom group GROUP only."""
    u = lidar.dirs() @ d.site_xmat[site].reshape(3, 3).T
    n = len(u); gid = np.zeros(n, np.int32); dist = np.zeros(n)
    grp = np.zeros(6, np.uint8); grp[GROUP] = 1
    mujoco.mj_multiRay(m, d, d.site_xpos[site].copy(), u.ravel().copy(), grp, 1, -1, gid, dist, None, n, lidar.rmax)
    r = np.where((dist >= lidar.rmin) & (dist <= lidar.rmax), dist, np.nan)
    return r, u


def add_noise(z, cam, rng):
    """Measured depth z + sigma_z(z) n, n ~ N(0, 1); invalid pixels stay nan and the valid range is applied again."""
    zm = z + cam.sigma_z(z) * rng.standard_normal(z.shape)
    zm[(zm < cam.zmin) | (zm > cam.zmax)] = np.nan
    return zm


def cam_pose(cam, R_wb, t_wb):
    """T_wc = T_wb T_bc: (R_wc, t_wc)."""
    return R_wb @ cam.R_bc, t_wb + R_wb @ np.asarray(cam.pos_b)


def measure(depth, cam, R_wb, t_wb):
    """World points of the valid pixels and their height statistics.
    Returns xy (N, 2), h (N,), var_s (N,) = (e_z^T R_wc d_c)^2 sigma_z(z)^2, and J (N, 3) = dh / d(dz, dth_x, dth_y) of the
    base pose (world-frame small rotation applied on the left of R_wb). sigma_z is evaluated at the measured depth."""
    ok = np.isfinite(depth).ravel()
    dc = cam.pixel_dirs().reshape(-1, 3)[ok]; z = depth.ravel()[ok]
    R_wc, t_wc = cam_pose(cam, R_wb, t_wb)
    p = t_wc + (z[:, None] * dc) @ R_wc.T
    uz = dc @ R_wc[2]                                             # e_z^T R_wc d_c
    v = p - t_wb                                                  # lever arm from the base origin
    J = np.c_[np.ones(len(p)), v[:, 1], -v[:, 0]]                 # d p_z: 1 for dz; (dth x v)_z = dth_x v_y - dth_y v_x
    return p[:, :2], p[:, 2], (uz * cam.sigma_z(z)) ** 2, J


class ElevationMap:
    """Height h and variance var per cell, var = inf where unobserved. Arrays are (nx, ny), x index first."""

    def __init__(self, res=RES, xr=MAP_X, yr=MAP_Y):
        self.res, self.x0, self.y0 = res, xr[0], yr[0]
        self.nx, self.ny = round((xr[1] - xr[0]) / res), round((yr[1] - yr[0]) / res)
        self.h = np.full((self.nx, self.ny), np.nan); self.var = np.full((self.nx, self.ny), np.inf)

    def centres(self):
        x = self.x0 + (np.arange(self.nx) + 0.5) * self.res; y = self.y0 + (np.arange(self.ny) + 0.5) * self.res
        return np.meshgrid(x, y, indexing='ij')

    def cells(self, xy):
        """Flat cell index of each point and a mask of points inside the map."""
        i = np.floor((xy[:, 0] - self.x0) / self.res).astype(int); j = np.floor((xy[:, 1] - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        return i * self.ny + j, ok

    def aggregate(self, xy, h, var_s, J, Sigma, gate=GATE):
        """One measurement per cell from one scan: the mean height of its n points, with variance
        sum(var_s)/n^2 + Jbar Sigma Jbar^T (the pose error is common to all points of a scan, so it does not average out).
        gate = (b, k): before averaging, drop points more than b + k sigma_i below the highest point of the cell in this scan,
        so that points on a riser below the top surface do not enter the mean. gate=None averages every point."""
        k, ok = self.cells(xy); k, h, var_s, J = k[ok], h[ok], var_s[ok], J[ok]
        N = self.nx * self.ny
        if gate is not None:
            hmax = np.full(N, -np.inf); np.maximum.at(hmax, k, h)
            keep = h >= hmax[k] - (gate[0] + gate[1] * np.sqrt(var_s))
            k, h, var_s, J = k[keep], h[keep], var_s[keep], J[keep]
        n = np.bincount(k, minlength=N); c = np.flatnonzero(n)
        hm = np.bincount(k, h, N)[c] / n[c]
        vs = np.bincount(k, var_s, N)[c] / n[c] ** 2
        Jm = np.stack([np.bincount(k, J[:, i], N)[c] for i in range(J.shape[1])], 1) / n[c, None]
        return c, hm, vs + np.einsum('ci,ij,cj->c', Jm, Sigma, Jm)

    def fuse(self, c, z, r):
        """Scalar Kalman update of cells c with measurements z of variance r (static state, no process noise)."""
        h, var = self.h.reshape(-1), self.var.reshape(-1)
        new = ~np.isfinite(var[c]); v = np.where(new, 0.0, var[c])
        K = np.where(new, 1.0, v / (v + r))
        h[c] = np.where(new, z, np.nan_to_num(h[c]) + K * (z - np.nan_to_num(h[c])))
        var[c] = np.where(new, r, (1 - K) * v)

    def fuse_information(self, c, z, r):
        """Same fusion in information form, cells may repeat: 1/var += sum 1/r, h/var += sum z/r."""
        h, var = self.h.reshape(-1), self.var.reshape(-1)
        N = self.nx * self.ny
        I0 = np.where(np.isfinite(var), 1 / var, 0.0); y0 = np.where(np.isfinite(var), np.nan_to_num(h) * I0, 0.0)
        I = I0 + np.bincount(c, 1 / r, N); y = y0 + np.bincount(c, z / r, N)
        seen = I > 0
        h[seen] = y[seen] / I[seen]; var[seen] = 1 / I[seen]


def plane_design(k, res):
    """Rows [dx, dy, 1] of a k x k window (x index first, row-major) and its pseudo-inverse."""
    o = (np.arange(k) - (k - 1) / 2) * res
    dx, dy = np.meshgrid(o, o, indexing='ij')
    A = np.c_[dx.ravel(), dy.ravel(), np.ones(k * k)]
    return A, np.linalg.solve(A.T @ A, A.T)


def features(Hgrid, k=7, res=RES):
    """Slope (rad), roughness (m) and step height (m) of the k x k window centred on each cell; nan if any cell is unobserved.
    Plane h = a dx + b dy + c by least squares: slope = atan(sqrt(a^2 + b^2)), roughness = sqrt(sum e^2 / (k^2 - 3)),
    step = max(e) - min(e), with e the plane residuals."""
    from numpy.lib.stride_tricks import sliding_window_view
    A, P = plane_design(k, res)
    Wn = sliding_window_view(Hgrid, (k, k)).reshape(Hgrid.shape[0] - k + 1, Hgrid.shape[1] - k + 1, k * k)
    coef = Wn @ P.T
    e = Wn - coef @ A.T
    out = [np.full(Hgrid.shape, np.nan) for _ in range(3)]
    r = k // 2
    for o, v in zip(out, (np.arctan(np.hypot(coef[..., 0], coef[..., 1])),
                          np.sqrt((e ** 2).sum(-1) / (k * k - 3)), e.max(-1) - e.min(-1))):
        o[r:Hgrid.shape[0] - r, r:Hgrid.shape[1] - r] = v
    return out


CRIT = dict(slope=np.radians(30.0), rough=0.06, step=0.20)   # critical values chosen for the Go2 in this module (README)


def traversability(slope, rough, step, crit=CRIT, w=(1 / 3, 1 / 3, 1 / 3)):
    """t = 1 - sum_i w_i min(1, x_i / x_crit,i), and t = 0 when any feature reaches its critical value; nan where unknown."""
    xs = [slope / crit['slope'], rough / crit['rough'], step / crit['step']]
    t = 1 - sum(wi * np.minimum(1, x) for wi, x in zip(w, xs))
    t[np.any([x >= 1 for x in xs], 0)] = 0
    return t


THR = dict(slope=np.radians(6.0), rough=0.01, step_rough=0.03, step_stairs=0.08, step_obstacle=0.18)


def semantic(slope, rough, step, thr=THR):
    """Label per cell, later rules override earlier ones: flat; slope >= 6 deg; rough if roughness >= 1 cm or step >= 3 cm;
    stairs if step >= 8 cm; obstacle if step >= 18 cm. -1 where the features are unknown."""
    L = np.full(slope.shape, FLAT)
    L[slope >= thr['slope']] = SLOPE
    L[(rough >= thr['rough']) | (step >= thr['step_rough'])] = ROUGH
    L[step >= thr['step_stairs']] = STAIRS
    L[step >= thr['step_obstacle']] = OBSTACLE
    L[~np.isfinite(slope)] = -1
    return L
