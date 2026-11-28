#!/usr/bin/env python3
"""Experiment for module 12, all three levels.
  fundamental: the terrain course against mj_ray; own slab ray casting against mj_multiRay on every depth pixel of every scan; the
      camera frame chain against MuJoCo's site pose; pinhole round trip; noise-free points on the true surface; depth noise model.
  intermediate: elevation map fused over 51 scans along the course with per-cell variance; sequential Kalman against batch
      information form; error, NEES and 2-sigma coverage against the true height on cells of constant true height; coverage.
  advanced: per-scan base pose errors with and without their Jacobian in the variance, and with point-wise fusion; slope,
      roughness, step height and traversability against the same quantities on the true height grid; semantic labels against the
      course layout.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--no-media]"""
import argparse, json, pathlib, sys, time
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
sys.path.append(str(HERE.parent / 'm04_kinematics'))          # appended: its validate.py must not shadow this one
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
import mujoco  # noqa: E402
from numpy.lib.stride_tricks import sliding_window_view  # noqa: E402
import perception as P  # noqa: E402
from common import LEGS, axis_angle_R  # noqa: E402
from kinematics import Leg  # noqa: E402  (module 04)

FIG, MEDIA = HERE / 'figures', HERE / 'media'
SEED_TERRAIN, SEED_DEPTH, SEED_POSE = 0, 1, 2
PATH_X = np.round(np.arange(0.0, 5.0 + 1e-9, 0.1), 10)       # base x of the 51 scan poses, y = 0, yaw = 0
SIGMA_POSE = np.diag([0.01, np.radians(0.5), np.radians(0.5)]) ** 2   # per-scan base error (z m, roll rad, pitch rad), advanced
WIN = 7                                                       # feature window, cells (0.35 m)
DT_SCAN, FPS = 0.3, 20                                        # video: one scan every 0.3 s of video time

# Acceptance thresholds, fixed before the first full run (MODULE_SPEC.md). (key, op, threshold).
ACCEPT = {
    # fundamental
    'true_height_vs_mj_ray_max_m': ('<', 1e-9), 'slab_vs_multiray_hit_mismatch': ('<=', 0),
    'slab_vs_multiray_dist_max_m': ('<', 1e-9), 'slab_vs_multiray_normal_agree': ('>=', 0.999),
    'camera_pose_vs_site_max': ('<', 1e-12), 'projection_roundtrip_max_px': ('<', 1e-9),
    'noisefree_on_surface_frac': ('>=', 0.999), 'depth_noise_norm_sd_err': ('<', 0.02),
    'lidar_vs_multiray_valid_mismatch': ('<=', 0), 'lidar_vs_multiray_range_max_m': ('<', 1e-9),   # added with the LiDAR model
    # intermediate
    'kalman_vs_batch_h_max_m': ('<', 1e-12), 'kalman_vs_batch_var_rel_max': ('<', 1e-10),
    'rmse_const_m': ('<', 0.005), 'nees_const_mean': ('in', (0.8, 1.25)), 'cover2s_const_dev': ('<', 0.02),
    'coverage_course': ('>=', 0.90),
    # advanced
    'nees_pose_aware_const_mean': ('in', (0.7, 1.4)), 'nees_unaware_over_aware': ('>=', 2.0),
    'slope_true_grid_err_max_deg': ('<', 1e-6),
    'trav_slope_rmse_deg': ('<', 2.0), 'trav_rough_rmse_m': ('<', 0.005), 'trav_step_rmse_m': ('<', 0.02),
    'trav_t_rmse': ('<', 0.05), 'trav_binary_agree': ('>=', 0.98),
    'label_acc_interior': ('>=', 0.90), 'label_recall_min_interior': ('>=', 0.80),
    'media_mp4_under_6MB': ('>=', 1.0),
}


def check(key, val):
    op, thr = ACCEPT[key]
    if op == 'in':
        return bool(thr[0] <= val <= thr[1])
    return bool({'<': val < thr, '<=': val <= thr, '>=': val >= thr}[op])


# ---- course, robot poses, scans -----------------------------------------------------------------------------------------

def setup():
    boxes = P.terrain(SEED_TERRAIN); cam = P.DepthCamera()
    m, d = P.build_model(boxes, cam)
    return boxes, cam, m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'depth_cam')


def pose_on_terrain(m, boxes, legs, nominal, x):
    """qpos for the base at (x, 0): feet at their nominal x y on the true surface, pitch from front and rear foot heights, joints
    from module 04's closed-form IK (solution nearest the stand pose). The robot is posed kinematically, not walked."""
    xy = np.array([x, 0.0]) + nominal[:, :2]
    hz = P.true_height(boxes, xy) + sim.FOOT_RADIUS
    dx = nominal[:2, 0].mean() - nominal[2:, 0].mean()
    pitch = -np.arctan((hz[:2].mean() - hz[2:].mean()) / dx)
    R = P.rot_y(pitch)
    t = np.array([x, 0.0, hz.mean() - nominal[:, 2].mean()])
    q = sim.Q_STAND.copy()
    for i, L in enumerate(legs):
        sols = L.ik_closed_form(R.T @ (np.r_[xy[i], hz[i]] - t))
        if len(sols):
            q[3 * i:3 * i + 3] = sols[np.argmin(np.abs(sols - sim.Q_STAND[:3]).sum(1))]
    quat = np.empty(4); mujoco.mju_mat2Quat(quat, R.ravel())
    return np.r_[t, quat, q]


def collect_scans(boxes, cam, m, d, site):
    legs = [Leg(m, l) for l in LEGS]
    nominal = np.array([L.fk(sim.Q_STAND[:3]) for L in legs])
    rng = np.random.default_rng(SEED_DEPTH)
    S = []
    for x in PATH_X:
        d.qpos[:] = pose_on_terrain(m, boxes, legs, nominal, x); mujoco.mj_forward(m, d)
        z0, gid, dist, u, nrm = P.scan(m, d, cam, site)
        t_s, w_s, n_s = P.raycast(boxes, d.site_xpos[site], u)               # own slab method on the same rays
        t_wb, _, R_wb = sim.base_pose(d)
        S.append(dict(x=x, qpos=d.qpos.copy(), z0=z0, z=P.add_noise(z0, cam, rng), dist=dist, nrm=nrm, u=u, t_slab=t_s,
                      n_slab=n_s, R=R_wb, t=t_wb, site_R=d.site_xmat[site].reshape(3, 3).copy(), site_p=d.site_xpos[site].copy()))
    return S


# ---- fundamental --------------------------------------------------------------------------------------------------------

def fundamental(boxes, cam, m, d, S, M):
    r = {}
    X, Y = M.centres(); xy = np.c_[X.ravel(), Y.ravel()]
    grp = np.zeros(6, np.uint8); grp[P.GROUP] = 1; g = np.zeros(1, np.int32)
    h_mj = np.array([2.0 - mujoco.mj_ray(m, d, np.r_[p, 2.0], np.array([0, 0, -1.0]), grp, 1, -1, g) for p in xy])
    r['true_height_vs_mj_ray_max_m'] = float(np.abs(P.true_height(boxes, xy) - h_mj).max())
    hit_mj = np.concatenate([s['dist'] >= 0 for s in S]); t_mj = np.concatenate([s['dist'] for s in S])
    t_sl = np.concatenate([s['t_slab'] for s in S])
    r['slab_vs_multiray_hit_mismatch'] = int((hit_mj != np.isfinite(t_sl)).sum())
    r['slab_vs_multiray_dist_max_m'] = float(np.abs(t_sl[hit_mj] - t_mj[hit_mj]).max())
    dn = np.abs(np.concatenate([s['n_slab'] for s in S]) - np.concatenate([s['nrm'] for s in S])).max(1)[hit_mj]
    r['slab_vs_multiray_normal_agree'] = float(np.mean(dn < 1e-9)); r['rays_total'] = int(hit_mj.size)
    err = []
    for s in S:
        R_wc, t_wc = P.cam_pose(cam, s['R'], s['t'])
        err.append(max(np.abs(R_wc - s['site_R']).max(), np.abs(t_wc - s['site_p']).max()))
    r['camera_pose_vs_site_max'] = float(max(err))
    dc = cam.pixel_dirs().reshape(-1, 3); z = S[10]['z0'].ravel(); ok = np.isfinite(z)
    uv = np.stack(np.meshgrid(np.arange(cam.W) + 0.5, np.arange(cam.H) + 0.5), -1).reshape(-1, 2)
    r['projection_roundtrip_max_px'] = float(np.abs(cam.project(z[ok, None] * dc[ok]) - uv[ok]).max())
    on, tot, nz = 0, 0, []
    for s in S:
        ok = np.isfinite(s['z0']).ravel()
        xy_p, h, _, _ = P.measure(s['z0'], cam, s['R'], s['t'])
        top = s['n_slab'][ok, 2] > 0.5                                       # hits on tops and on the slope, not on risers
        on += int((np.abs(h - P.true_height(boxes, xy_p))[top] < 1e-9).sum()); tot += int(top.sum())
        both = np.isfinite(s['z0']) & np.isfinite(s['z'])
        nz.append(((s['z'] - s['z0']) / cam.sigma_z(s['z0']))[both])
    r['noisefree_on_surface_frac'] = on / tot; r['noisefree_top_points'] = tot
    nz = np.concatenate(nz)
    r['depth_noise_norm_sd'] = float(nz.std()); r['depth_noise_norm_sd_err'] = float(abs(nz.std() - 1))
    r['valid_pixels_per_scan_mean'] = float(np.mean([np.isfinite(s['z']).sum() for s in S]))
    xy0 = P.measure(S[0]['z'], cam, S[0]['R'], S[0]['t'])[0]                 # first scan, base x = 0
    r['first_scan_camera_x_m'] = float(P.cam_pose(cam, S[0]['R'], S[0]['t'])[1][0])
    r['first_scan_nearest_point_x_m'] = float(xy0[:, 0].min())
    lid = P.Lidar(); ls = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'lidar'); mis, dmax, nv, nt = 0, 0.0, 0, 0
    for s in S[::5]:                                                         # LiDAR sweep at every 5th pose (11 poses)
        d.qpos[:] = s['qpos']; mujoco.mj_forward(m, d)
        rl, u = P.lidar_scan(m, d, lid, ls)
        t = P.raycast(boxes, d.site_xpos[ls], u)[0]
        t = np.where((t >= lid.rmin) & (t <= lid.rmax), t, np.nan)
        ok = np.isfinite(rl); mis += int((ok != np.isfinite(t)).sum()); nv += int(ok.sum()); nt += len(rl)
        dmax = max(dmax, float(np.abs(rl[ok] - t[ok]).max()))
    r.update(lidar_rays_total=nt, lidar_valid_frac=nv / nt, lidar_vs_multiray_valid_mismatch=mis, lidar_vs_multiray_range_max_m=dmax)
    return r, nz


# ---- intermediate -------------------------------------------------------------------------------------------------------

def cell_classes(boxes, M, n=8):
    """True height at the cell centres, and masks: 'const' (true height constant over the cell plus a half-cell border),
    'planar' (not constant, but one plane over that footprint: the slope), 'edge' (the rest: steps, block and obstacle edges)."""
    X, Y = M.centres()
    Hc = P.true_height(boxes, np.c_[X.ravel(), Y.ravel()]).reshape(X.shape)
    o = np.linspace(-0.999, 0.999, n) * M.res                              # footprint of width 2 res, samples straddle edges
    ox, oy = np.meshgrid(o, o, indexing='ij')
    pts = np.c_[(X.ravel()[:, None] + ox.ravel()).ravel(), (Y.ravel()[:, None] + oy.ravel()).ravel()]
    Hs = P.true_height(boxes, pts).reshape(-1, n * n)
    const = (Hs.max(1) - Hs.min(1)) < 1e-9
    A = np.c_[ox.ravel(), oy.ravel(), np.ones(n * n)]
    res = Hs - (Hs @ np.linalg.pinv(A).T) @ A.T
    planar = ~const & (np.abs(res).max(1) < 1e-9)
    return Hc, const.reshape(X.shape), planar.reshape(X.shape), (~const & ~planar).reshape(X.shape)


def build_map(cam, S, Sigma=None, pose_err=None, pointwise=False, snapshots=False, gate=P.GATE):
    """Fuse all scans. Sigma: pose covariance the mapper assumes (None = 0); pose_err: (n, 3) actual (dz, dth_x, dth_y) per scan."""
    M = P.ElevationMap(); Z = np.zeros((3, 3)) if Sigma is None else Sigma
    log, snaps = [], []
    for k, s in enumerate(S):
        R, t = s['R'], s['t']
        if pose_err is not None:
            e = pose_err[k]
            R = axis_angle_R([e[1], e[2], 0.0], np.hypot(e[1], e[2])) @ R if np.hypot(e[1], e[2]) > 0 else R
            t = t + np.array([0, 0, e[0]])
        xy, h, vs, J = P.measure(s['z'], cam, R, t)
        if pointwise:
            c, ok = M.cells(xy)
            M.fuse_information(c[ok], h[ok], vs[ok] + np.einsum('ni,ij,nj->n', J[ok], Z, J[ok]))
        else:
            c, hm, r = M.aggregate(xy, h, vs, J, Z, gate)
            M.fuse(c, hm, r); log.append((c, hm, r))
        if snapshots:
            snaps.append((M.h.copy(), M.var.copy(), np.c_[xy, h]))
    return M, log, snaps


def stats(M, Hc, mask):
    obs = np.isfinite(M.var) & mask
    e = (M.h - Hc)[obs]; v = M.var[obs]
    return dict(n=int(obs.sum()), rmse=float(np.sqrt(np.mean(e ** 2))), nees=float(np.mean(e ** 2 / v)),
                cover2s=float(np.mean(np.abs(e) < 2 * np.sqrt(v))), maxabs=float(np.abs(e).max()))


def intermediate(cam, S, M, log, Hc, masks):
    r = {}
    B = P.ElevationMap()
    B.fuse_information(*[np.concatenate(a) for a in zip(*log)])
    seen = np.isfinite(M.var)
    r['kalman_vs_batch_h_max_m'] = float(np.abs(M.h - B.h)[seen].max())
    r['kalman_vs_batch_var_rel_max'] = float(np.abs(M.var[seen] / B.var[seen] - 1).max())
    r['measurements_per_cell_median'] = float(np.median(np.bincount(np.concatenate([c for c, _, _ in log]), minlength=M.nx * M.ny)[seen.ravel()]))
    for name, mk in zip(('const', 'planar', 'edge'), masks):
        st = stats(M, Hc, mk)
        for k, v in st.items():
            r[f'{k}_{name}' if k != 'rmse' else f'rmse_{name}_m'] = v
    r['nees_const_mean'] = r.pop('nees_const'); r['cover2s_const_dev'] = abs(r['cover2s_const'] - 0.9545)
    X, Y = M.centres(); course = (np.abs(Y) < P.HALF_W) & (X >= 0) & (X <= 6.0)
    r['coverage_course'] = float(np.isfinite(M.var)[course].mean())
    r['coverage_course_x_ge_1p2'] = float(np.isfinite(M.var)[course & (X >= 1.2)].mean())   # first lateral view of |y| = 1
    Lt = P.region_label(X, Y)
    r['coverage_by_label'] = {P.LABELS[c]: float(np.isfinite(M.var)[course & (Lt == c)].mean()) for c in range(5)}
    r['valid_pixels_min_scan'] = [float(S[int(np.argmin([np.isfinite(s['z']).sum() for s in S]))]['x']),
                                  int(min(np.isfinite(s['z']).sum() for s in S))]
    r['sigma_const_median_m'] = float(np.median(np.sqrt(M.var[np.isfinite(M.var) & masks[0]])))
    return r


# ---- advanced -----------------------------------------------------------------------------------------------------------

def pose_experiment(cam, S, Hc, const):
    pe = np.random.default_rng(SEED_POSE).multivariate_normal(np.zeros(3), SIGMA_POSE, len(S))
    out, maps = {}, {}
    for name, kw in (('aware', dict(Sigma=SIGMA_POSE)), ('unaware', dict()), ('pointwise', dict(Sigma=SIGMA_POSE, pointwise=True))):
        Mx = build_map(cam, S, pose_err=pe, **kw)[0]
        st = stats(Mx, Hc, const); maps[name] = Mx
        out[f'nees_pose_{name}_const_mean'] = st['nees']; out[f'rmse_pose_{name}_const_m'] = st['rmse']
        out[f'cover2s_pose_{name}_const'] = st['cover2s']
    out['nees_unaware_over_aware'] = out['nees_pose_unaware_const_mean'] / out['nees_pose_aware_const_mean']
    out['pose_err_sd_drawn'] = pe.std(0).tolist()
    return out, maps


def interior(Ltrue, k=WIN):
    """Cells whose whole k x k window lies in one region of the layout."""
    W = sliding_window_view(Ltrue, (k, k)); m = np.zeros(Ltrue.shape, bool); r = k // 2
    m[r:-r, r:-r] = (W.min((-1, -2)) == W.max((-1, -2)))
    return m


def advanced(M, Hc, X, Y):
    r = {}
    Fe = P.features(M.h, WIN); Ft = P.features(Hc, WIN)
    Lt = P.region_label(X, Y); inn = interior(Lt)
    sl = inn & (Lt == P.SLOPE)
    r['slope_true_grid_err_max_deg'] = float(np.abs(np.degrees(Ft[0][sl]) - P.SLOPE_DEG).max())
    both = np.isfinite(Fe[0]) & np.isfinite(Ft[0])
    r['trav_cells'] = int(both.sum())
    r['trav_slope_rmse_deg'] = float(np.degrees(np.sqrt(np.mean((Fe[0] - Ft[0])[both] ** 2))))
    r['trav_rough_rmse_m'] = float(np.sqrt(np.mean((Fe[1] - Ft[1])[both] ** 2)))
    r['trav_step_rmse_m'] = float(np.sqrt(np.mean((Fe[2] - Ft[2])[both] ** 2)))
    Te, Tt = P.traversability(*Fe), P.traversability(*Ft)
    r['trav_t_rmse'] = float(np.sqrt(np.mean((Te - Tt)[both] ** 2)))
    r['trav_binary_agree'] = float(np.mean((Te[both] > 0) == (Tt[both] > 0)))
    r['trav_untraversable_true_cells'] = int((Tt[both] == 0).sum())
    big = both & (np.abs(Te - Tt) > 0.5)                                    # cells where one map says blocked, the other free
    r['trav_t_err_gt_0p5_cells'] = int(big.sum())
    r['trav_t_rmse_without_gt_0p5'] = float(np.sqrt(np.mean((Te - Tt)[both & ~big] ** 2)))
    st = both & inn & (Lt == P.STAIRS)
    r['stairs_inner_step_median_est_m'] = float(np.median(Fe[2][st])); r['stairs_inner_step_median_true_m'] = float(np.median(Ft[2][st]))
    r['stairs_inner_step_min_true_m'] = float(Ft[2][st].min()); r['stairs_inner_step_max_true_m'] = float(Ft[2][st].max())
    for name, msk in (('inner', both & inn), ('boundary', both & ~inn)):
        r[f'trav_t_rmse_{name}'] = float(np.sqrt(np.mean((Te - Tt)[msk] ** 2)))
        r[f'trav_step_rmse_{name}_m'] = float(np.sqrt(np.mean((Fe[2] - Ft[2])[msk] ** 2)))
    Le, Ltg = P.semantic(*Fe), P.semantic(*Ft)
    ev = inn & (Le >= 0) & (Lt != P.OBSTACLE)
    r['label_interior_cells'] = int((inn & (Lt != P.OBSTACLE)).sum())
    r['label_unknown_frac_interior'] = float(np.mean(Le[inn & (Lt != P.OBSTACLE)] < 0))
    r['label_acc_interior'] = float(np.mean(Le[ev] == Lt[ev]))
    rec = {P.LABELS[c]: float(np.mean(Le[ev & (Lt == c)] == c)) for c in (P.FLAT, P.SLOPE, P.ROUGH, P.STAIRS)}
    r['label_recall_interior'] = rec; r['label_recall_min_interior'] = min(rec.values())
    r['label_acc_true_grid_interior'] = float(np.mean(Ltg[ev] == Lt[ev]))
    conf = np.zeros((4, 5), int)
    for i, c in enumerate((P.FLAT, P.SLOPE, P.ROUGH, P.STAIRS)):
        for j in range(5):
            conf[i, j] = int(((Lt == c) & ev & (Le == j)).sum())
    r['label_confusion_rows_true_cols_pred'] = conf.tolist()
    return r, dict(Fe=Fe, Ft=Ft, Te=Te, Tt=Tt, Le=Le, Lt=Lt, inn=inn, conf=conf)


# ---- video --------------------------------------------------------------------------------------------------------------

class MapRecorder(sim.Recorder):
    """sim.Recorder that draws extra geoms (map cells, scan points) into the scene before each frame."""
    overlay = None

    def render(self, d):
        if not self.ok:
            return None
        self.renderer.update_scene(d, camera=self.cam, scene_option=self.opt)
        if self.overlay is not None:
            self.overlay(self.renderer.scene)
        return self.renderer.render()


def _add(scn, gtype, size, pos, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(scn.geoms[scn.ngeom], gtype, size, pos, np.eye(3).ravel(), rgba); scn.ngeom += 1


def video(m, d, S, snaps, M):
    X, Y = M.centres(); cmap = plt.get_cmap('viridis')
    n = len(S); T = (n - 1) * DT_SCAN + 1.0
    rec = MapRecorder(m, MEDIA / 'map_building.mp4', width=800, height=450, fps=FPS, distance=2.6, azimuth=25, elevation=-32)
    if not rec.ok:
        return None, rec
    state = {}

    def overlay(scn):
        h, var, pts = state['snap']
        ok = np.isfinite(var)
        lv = np.clip((np.log10(np.sqrt(var[ok]) * 1000) + 1.5) / 2.0, 0, 1)   # sigma 0.03 mm (dark) .. 3 mm (yellow)
        for x, y, z, c in zip(X[ok], Y[ok], h[ok], cmap(lv)):
            _add(scn, mujoco.mjtGeom.mjGEOM_BOX, [0.46 * M.res, 0.46 * M.res, 0.002], [x, y, z + 0.008], c)
        sel = pts[:: max(1, len(pts) // 350)]
        for p in sel:
            _add(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.008, 0, 0], p + [0, 0, 0.006], [0.48, 0.27, 0.84, 1])
    rec.overlay = overlay
    for f in range(int(T * FPS)):
        t = f / FPS; i = min(int(t / DT_SCAN), n - 1); a = min(t / DT_SCAN - i, 1.0) if i < n - 1 else 0.0
        q = S[i]['qpos'] if i == n - 1 else (1 - a) * S[i]['qpos'] + a * S[i + 1]['qpos']
        d.qpos[:] = q; d.qpos[3:7] /= np.linalg.norm(q[3:7]); mujoco.mj_forward(m, d)
        state['snap'] = snaps[i]; d.time = t
        rec(m, d)
    path = rec.close()
    rec.cam.distance, rec.cam.azimuth, rec.cam.elevation = 3.4, 215, -38
    rec.still(d, MEDIA / 'map_building.png')
    return path, rec


# ---- figures ------------------------------------------------------------------------------------------------------------

# flat, slope, rough, stairs, obstacle: own categorical palette, kept apart from figstyle's reserved meaning colours
LCOL = ['#e3dcc8', '#6fb3a8', '#a0703f', '#d47fb0', '#3a3a3a']
EXT = [P.MAP_X[0], P.MAP_X[1], P.MAP_Y[0], P.MAP_Y[1]]


def topview(ax, A, title, **kw):
    im = ax.imshow(A.T, origin='lower', extent=EXT, aspect='equal', interpolation='nearest', **kw)
    ax.set(title=title, xlabel='x [m]', ylabel='y [m]'); ax.grid(False)
    return im


def fig_diagram(cam):
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.8), gridspec_kw=dict(width_ratios=[1.15, 1]))
    for ax in (a, b):
        ax.axis('off'); ax.set_aspect('equal')
    a.set_xlim(-0.12, 1.25); a.set_ylim(-0.1, 0.5)
    xe = 0.75                                                            # riser position in the sketch
    a.plot([-0.1, xe, xe, 1.2], [0, 0, 0.12, 0.12], color=fs.C['gt'], lw=2)
    for xc in np.arange(0.15, 1.2, 0.05 * 2):
        a.plot([xc, xc], [-0.025, -0.005], color=fs.C['muted'], lw=0.6)
    a.text(0.15, -0.065, 'cell edges (drawn every 2nd)', color=fs.C['muted'], fontsize=7.5)
    c = np.array([0.0, 0.36]); al = np.radians(cam.pitch_deg)
    zax = np.array([np.cos(al), -np.sin(al)]); yax = np.array([-np.sin(al), -np.cos(al)])
    a.plot(*c, 'o', color=fs.C['ink'], ms=4)
    a.plot(*np.c_[c, c + 0.62 * zax], '--', color=fs.C['muted'], lw=0.7)
    fs.frame_axes(a, c, R2=((zax[0], yax[0]), (zax[1], yax[1])), length=0.13, names=('z_c', 'y_c'))
    p = np.array([0.30, 0.0]); ray = (p - c) / np.linalg.norm(p - c)
    a.plot(*np.c_[c, p], color=fs.C['est2'], lw=1.1); a.plot(*p, 'o', color=fs.C['est2'], ms=4)
    zdep = (p - c) @ zax; q = c + zdep * zax
    a.plot(*np.c_[p, q], ':', color=fs.C['muted'], lw=0.9)
    a.text(*(c + 0.62 * zdep * zax + 0.03 * np.array([np.sin(al), np.cos(al)])), 'depth $z$', color=fs.C['muted'], fontsize=8,
           rotation=-cam.pitch_deg, rotation_mode='anchor')
    for s_ in (-1, 1):
        a.plot(*(p + s_ * 0.035 * ray), marker=(2, 0, 90 - np.degrees(np.arctan2(ray[1], ray[0]))), color=fs.C['est2'], ms=7, mew=1.2)
    fs.label(a, p - 0.035 * ray, r'$\sigma_z(z)$ along the ray', xytext=(-95, 18), color=fs.C['est2'], fontsize=8)
    a.annotate('', xy=(p[0] + 0.06, 0.025), xytext=(p[0] + 0.06, -0.025), arrowprops=dict(arrowstyle='<->', color=fs.C['est'], lw=0.9))
    a.text(p[0] + 0.08, 0.035, r'$\sigma_h = |e_z^\top R_{wc} d_c|\,\sigma_z$', color=fs.C['est'], fontsize=8.5)
    fs.label(a, (xe, 0.07), 'riser: points between two\nheights land in one cell', xytext=(15, 55), color=fs.C['bad'], fontsize=8)
    a.set_title('Depth pixel to cell height (side view)')
    b.set_xlim(0, 1); b.set_ylim(0, 1.05)
    steps = ['depth image $z(u, v)$', 'points $p_w = t_{wc} + z R_{wc} d_c$', 'per scan, per cell: $\\bar h$, $\\bar\\sigma^2$',
             'Kalman fusion: $h$, $\\sigma^2$', 'window plane fit', 'slope, roughness, step', 'traversability $t$, label']
    for i, s_ in enumerate(steps):
        y0 = 0.93 - i * 0.145
        fs.box(b, (0.12, y0 - 0.05), 0.76, 0.095, s_, fontsize=8.5)
        if i:
            fs.arrow(b, (0.5, y0 + 0.095 + 0.0), (0.5, y0 + 0.048), color=fs.C['muted'], lw=0.8)
    b.text(0.9, 0.93 - 2 * 0.145, ' + pose\n   Jacobian', fontsize=7.5, color=fs.C['muted'], va='center')
    b.set_title('Pipeline')
    fs.save(fig, FIG / 'method_diagram')


def fig_course(M, Hc, X, Y):
    fig, (a, b) = plt.subplots(2, 1, figsize=(10, 5.4), gridspec_kw=dict(height_ratios=[1.3, 1]))
    im = topview(a, Hc, 'True height of the course and the 51 scan positions', cmap='Greys', vmin=-0.05, vmax=0.45)
    a.plot(PATH_X, np.zeros_like(PATH_X), '.', color=fs.C['plan'], ms=3, label='base position at each scan')
    fig.colorbar(im, ax=a, label='height [m]', shrink=0.8); a.legend(loc='upper right')
    j = M.ny // 2
    L = P.region_label(X[:, j], Y[:, j])
    b.plot(X[:, j], Hc[:, j], color=fs.C['gt'], lw=1.2)
    for c in range(5):
        s_ = L == c
        if s_.any():
            b.scatter(X[s_, j], Hc[s_, j], s=8, color=LCOL[c], label=P.LABELS[c], zorder=3)
    b.set(xlabel='x [m]', ylabel='height [m]', title='Profile at y = 0.025 m with the layout labels')
    b.legend(ncol=5, loc='upper left'); b.set_xlim(*P.MAP_X)
    fs.save(fig, FIG / 'course')


def fig_scan(cam, S, boxes, nz, k=18):
    s = S[k]
    fig, (a, b, c) = plt.subplots(1, 3, figsize=(11, 3.4), gridspec_kw=dict(width_ratios=[1, 1.6, 1]))
    im = a.imshow(s['z'], cmap='viridis'); a.grid(False)
    a.set(title=f'Depth image (x = {s["x"]:.1f} m)', xlabel='u [px]', ylabel='v [px]'); fig.colorbar(im, ax=a, label='z [m]', shrink=0.6)
    xy, h, vs, _ = P.measure(s['z'], cam, s['R'], s['t'])
    xs = np.linspace(s['x'], s['x'] + 3.2, 600)
    b.plot(xs, P.true_height(boxes, np.c_[xs, np.zeros_like(xs)]), color=fs.C['gt'], lw=1.2, label='true height at y = 0')
    sel = np.abs(xy[:, 1]) < 0.3
    sc = b.scatter(xy[sel, 0], h[sel], c=np.sqrt(vs[sel]) * 1000, s=4, cmap='viridis', label='points, |y| < 0.3 m')
    fig.colorbar(sc, ax=b, label=r'$\sigma_h$ [mm]', shrink=0.8)
    b.set(xlabel='x [m]', ylabel='height [m]', title='Point cloud, side view'); b.legend(loc='upper left')
    c.hist(nz, bins=80, density=True, color=fs.C['est'], alpha=0.7, label='all scans')
    g = np.linspace(-4, 4, 200); c.plot(g, np.exp(-g ** 2 / 2) / np.sqrt(2 * np.pi), color=fs.C['gt'], label='N(0, 1)')
    c.set(xlabel=r'$(z_{meas} - z)/\sigma_z(z)$', ylabel='density', title='Depth noise against its model'); c.legend()
    fs.save(fig, FIG / 'single_scan')


def fig_map(M, Hc, masks):
    fig, ax = plt.subplots(3, 1, figsize=(10, 8))
    im = topview(ax[0], M.h, 'Fused elevation map h (51 scans, true poses)', cmap='Greys', vmin=-0.05, vmax=0.45)
    fig.colorbar(im, ax=ax[0], label='m', shrink=0.8)
    im = topview(ax[1], np.log10(np.sqrt(M.var)) + 3, r'Cell standard deviation $\log_{10}(\sigma / \mathrm{mm})$', cmap='viridis')
    fig.colorbar(im, ax=ax[1], shrink=0.8)
    E = (M.h - Hc) * 1000
    im = topview(ax[2], E, 'h minus true height at the cell centre [mm] (edge cells dominate)', cmap='RdBu_r', vmin=-20, vmax=20)
    fig.colorbar(im, ax=ax[2], label='mm', shrink=0.8)
    fs.save(fig, FIG / 'elevation_map')


def fig_consistency(M, Hc, masks, res, maps):
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.6))
    obs = np.isfinite(M.var) & masks[0]
    e = ((M.h - Hc) / np.sqrt(M.var))[obs]
    a.hist(np.clip(e, -5, 5), bins=60, density=True, color=fs.C['est'], alpha=0.7, label=f'constant-height cells ({obs.sum()})')
    g = np.linspace(-5, 5, 200); a.plot(g, np.exp(-g ** 2 / 2) / np.sqrt(2 * np.pi), color=fs.C['gt'], label='N(0, 1)')
    a.set(xlabel=r'$(h - h_{true}) / \sigma$', ylabel='density', title='Normalised map error, true poses'); a.legend()
    names = ['true pose\nconst', 'true pose\nslope', 'true pose\nedge', 'pose error\naware', 'pose error\nunaware', 'pose error\npoint-wise']
    vals = [res['nees_const_mean'], res['nees_planar'], res['nees_edge'], res['nees_pose_aware_const_mean'],
            res['nees_pose_unaware_const_mean'], res['nees_pose_pointwise_const_mean']]
    cols = [fs.C['good'] if 0.7 <= v <= 1.4 else fs.C['bad'] for v in vals]
    b.bar(range(6), vals, color=cols); b.set_yscale('log'); b.axhline(1, color=fs.C['gt'], lw=0.8)
    for i, v in enumerate(vals):
        b.text(i, v * 1.15, f'{v:.2f}', ha='center', fontsize=8)
    b.set_xticks(range(6)); b.set_xticklabels(names, fontsize=7.5)
    b.set(ylabel='mean NEES', title='Mean NEES (1 = consistent)')
    fs.save(fig, FIG / 'consistency')


def fig_trav(A, X):
    fig, ax = plt.subplots(3, 1, figsize=(10, 7.5))
    for a, T, name in ((ax[0], A['Te'], 'from the fused map'), (ax[1], A['Tt'], 'from the true height grid')):
        im = topview(a, T, f'Traversability {name} (white: unknown)', cmap='RdYlGn', vmin=0, vmax=1)
        fig.colorbar(im, ax=a, shrink=0.8, label='t')
    j = A['Te'].shape[1] // 2 + 6
    xs = X[:, j]
    for k, (name, sc, unit) in enumerate((('slope', np.degrees(1) / 10, 'deg/10'), ('roughness', 100, 'cm'), ('step', 100, 'cm'))):
        col = [fs.C['plan'], fs.C['est2'], fs.C['est']][k]
        ax[2].plot(xs, A['Ft'][k][:, j] * sc, color=fs.C['gt'], lw=2.2, alpha=0.6)
        ax[2].plot(xs, A['Fe'][k][:, j] * sc, color=col, lw=1.1, label=f'{name} [{unit}]')
    ax[2].plot([], [], color=fs.C['gt'], lw=2.2, alpha=0.6, label='same on the true grid')
    ax[2].set(xlabel='x [m]', title=f'Features along y = {(j + 0.5) * P.RES + P.MAP_Y[0]:.3f} m', xlim=P.MAP_X)
    ax[2].legend(ncol=4, loc='upper left')
    fs.save(fig, FIG / 'traversability')


def fig_semantic(A):
    fig = plt.figure(figsize=(10, 5.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[3, 1])
    cm = ListedColormap(['white'] + LCOL)
    for i, (L, name) in enumerate(((A['Le'], 'Labels from the fused map (white: unknown)'), (A['Lt'], 'Layout labels'))):
        a = fig.add_subplot(gs[i, 0]); topview(a, L.astype(float), name, cmap=cm, vmin=-1.5, vmax=4.5)
        if i == 1:
            a.contour(*np.meshgrid(np.linspace(*P.MAP_X, A['inn'].shape[0]), np.linspace(*P.MAP_Y, A['inn'].shape[1]), indexing='ij'),
                      A['inn'].astype(float), levels=[0.5], colors=fs.C['ink'], linewidths=0.6)
    c = fig.add_subplot(gs[:, 1]); conf = A['conf']
    rown = conf / np.maximum(conf.sum(1, keepdims=True), 1)
    c.imshow(rown, cmap='Blues', vmin=0, vmax=1); c.grid(False)
    for i in range(4):
        for j in range(5):
            c.text(j, i, f'{conf[i, j]}', ha='center', va='center', fontsize=7.5, color='white' if rown[i, j] > 0.5 else fs.C['ink'])
    c.set_xticks(range(5)); c.set_xticklabels(P.LABELS, rotation=60, fontsize=7.5)
    c.set_yticks(range(4)); c.set_yticklabels(P.LABELS[:4], fontsize=7.5)
    c.set(xlabel='predicted', ylabel='layout', title='Interior cells')
    fs.save(fig, FIG / 'semantic')


def fig_failure(cam, S, M, Mn, Hc, masks, boxes):
    """Left: one cell row across the stairs, every point and both maps. Right: slope cells, error against the predicted 2 sigma."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw=dict(width_ratios=[1.5, 1]))
    X, Y = M.centres(); j = M.ny // 2; x0, x1 = 2.55, 3.75
    xs = np.linspace(x0, x1, 800); a = ax[0]
    a.plot(xs, P.true_height(boxes, np.c_[xs, np.full_like(xs, Y[0, j])]), color=fs.C['gt'], lw=1.5, label='true surface')
    pts = []
    for s in S:
        xy, h, _, _ = P.measure(s['z'], cam, s['R'], s['t'])
        sel = (np.floor((xy[:, 1] - M.y0) / M.res) == j) & (xy[:, 0] > x0) & (xy[:, 0] < x1)
        pts.append(np.c_[xy[sel, 0], h[sel]])
    pts = np.concatenate(pts)
    a.plot(pts[:, 0], pts[:, 1], '.', color=fs.C['muted'], ms=1.5, alpha=0.35, label='points, all scans')
    sel = (X[:, j] > x0) & (X[:, j] < x1)
    a.plot(X[sel, j], Mn.h[sel, j], 'o', ms=4, mfc='none', color=fs.C['est2'], label='map, every point averaged')
    a.plot(X[sel, j], M.h[sel, j], 's', ms=3, color=fs.C['est'], label='map, top-surface gate')
    a.set(xlabel='x [m]', ylabel='height [m]', title=f'Stair edges, cell row y = {Y[0, j]:.3f} m', xlim=(x0, x1))
    a.legend(loc='upper left', fontsize=7.5)
    b = ax[1]; pl = np.isfinite(M.var) & masks[1]
    e = (M.h - Hc)[pl] * 1000; s2 = 2 * np.sqrt(M.var[pl]) * 1000
    b.scatter(s2, np.abs(e), s=4, color=fs.C['est'], alpha=0.5, label=f'slope cells ({pl.sum()})')
    g = np.array([s2.min(), s2.max()]); b.plot(g, g, color=fs.C['gt'], lw=1, label=r'$|e| = 2\sigma$')
    b.set(xscale='log', yscale='log', xlabel=r'predicted $2\sigma$ [mm]', ylabel=r'$|h - h_{true}|$ at the cell centre [mm]',
          title=f'Slope cells: {np.mean(np.abs(e) < s2):.0%} within 2 sigma')
    b.legend(loc='upper left', fontsize=7.5)
    fs.save(fig, FIG / 'failure_cases')


# ---- main ---------------------------------------------------------------------------------------------------------------

def main(media=True):
    t0 = time.time()
    boxes, cam, m, d, site = setup()
    S = collect_scans(boxes, cam, m, d, site)
    M, log, snaps = build_map(cam, S, snapshots=media)
    res = dict(n_scans=len(S), n_boxes=len(boxes), map_cells=M.nx * M.ny, camera_pixels=cam.W * cam.H,
               camera_f_px=cam.f, camera_vfov_deg=float(np.degrees(2 * np.arctan(cam.H / 2 / cam.f))))
    r, nz = fundamental(boxes, cam, m, d, S, M); res.update(r)
    Hc, *masks = cell_classes(boxes, M)
    res.update(intermediate(cam, S, M, log, Hc, masks))
    r, maps = pose_experiment(cam, S, Hc, masks[0]); res.update(r)
    X, Y = M.centres()
    r, A = advanced(M, Hc, X, Y); res.update(r)
    rp, _ = advanced(maps['aware'], Hc, X, Y)
    res.update({f'posenoise_{k}': rp[k] for k in ('trav_slope_rmse_deg', 'trav_rough_rmse_m', 'trav_step_rmse_m', 'trav_t_rmse',
                                                  'label_acc_interior')})
    Mn = build_map(cam, S, gate=None)[0]                                     # first design: every point averaged
    rn, An = advanced(Mn, Hc, X, Y); sn = stats(Mn, Hc, masks[0])
    res.update({f'nogate_{k}': rn[k] for k in ('trav_rough_rmse_m', 'trav_step_rmse_m', 'trav_t_rmse', 'label_acc_interior',
                                               'label_recall_interior', 'label_recall_min_interior')})
    res['nogate_nees_const_mean'] = sn['nees']; res['nogate_rmse_edge_m'] = stats(Mn, Hc, masks[2])['rmse']
    j = M.ny // 2; ie = np.round((np.array([2.7, 3.0, 3.3]) - M.x0) / M.res).astype(int)   # cell row y = 0.025 m, the three risers
    res['stairs_row_y_m'] = float(Y[0, j]); res['stairs_row_upper_tread_m'] = [float(Hc[i, j]) for i in ie]
    for pre, Mx in (('', M), ('nogate_', Mn)):
        res[f'{pre}stairs_row_upper_cell_h_m'] = [float(Mx.h[i, j]) for i in ie]          # cell centre x = edge + 0.025 m
        res[f'{pre}stairs_row_lower_cell_h_m'] = [float(Mx.h[i - 1, j]) for i in ie]      # cell centre x = edge - 0.025 m
    res['media_mp4_under_6MB'] = float('nan')
    if media:
        path, rec = video(m, d, S, snaps, M)
        if path is not None:
            res['media_bytes'] = path.stat().st_size; res['media_mp4_under_6MB'] = float(path.stat().st_size < 6e6)
        rec.__exit__()
        fig_diagram(cam); fig_course(M, Hc, X, Y); fig_scan(cam, S, boxes, nz); fig_map(M, Hc, masks)
        fig_consistency(M, Hc, masks, res, maps); fig_trav(A, X); fig_semantic(A); fig_failure(cam, S, M, Mn, Hc, masks, boxes)
    res['wall_time_s'] = time.time() - t0
    res['checks'] = {k: check(k, res[k]) for k in ACCEPT if k in res and not (isinstance(res[k], float) and np.isnan(res[k]))}
    res['passed'] = int(sum(res['checks'].values())); res['total'] = len(res['checks'])
    (HERE / 'results.json').write_text(json.dumps(res, indent=1, default=float))
    for k, ok in res['checks'].items():
        print(f"{'PASS' if ok else 'MISS'}  {k:36s} {res[k]:.4g}   ({ACCEPT[k][0]} {ACCEPT[k][1]})")
    print(f"{res['passed']} of {res['total']} checks pass; wall {res['wall_time_s']:.1f} s")
    return res


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--no-media', action='store_true')
    main(media=not ap.parse_args().no_media)
