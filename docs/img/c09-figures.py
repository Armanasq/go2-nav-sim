"""Figures for tutorial chapters 9 to 12 (docs/img/c09-*.png). Every number drawn comes from the repository: the configs in
src/go2_nav_sim/config, the registration code and its unit-test geometry, site/data/runs/w2 and results/run_w*.json.

    MUJOCO_GL=glfw DISPLAY=:1 modules/../.venv-modules/bin/python docs/img/c09-figures.py [name ...]   # no name: all
"""
import importlib.util
import json
import math
import os
import pathlib
import sys
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / 'docs/img'
SCRIPTS = ROOT / 'src/go2_nav_sim/scripts'
sys.path[:0] = [str(ROOT / 'modules'), str(ROOT / 'modules/m04_kinematics'), str(SCRIPTS)]
os.environ.setdefault('MUJOCO_GL', 'glfw')

import numpy as np  # noqa: E402
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle, Polygon, Ellipse  # noqa: E402

C = fs.C
MONO = 'DejaVu Sans Mono'
RUN = json.load(open(ROOT / 'site/data/runs/w2/run.json'))
FIGS = {}


def fig(f):
    FIGS[f.__name__] = f
    return f


def save(f, name):
    """PNG only (150 dpi), the chapters embed PNG; prints the size so the 400 KB budget is visible."""
    p = OUT / f'c09-{name}.png'
    f.savefig(p, dpi=150, bbox_inches='tight', pad_inches=0.06)
    plt.close(f)
    print(f'{p.name}: {p.stat().st_size // 1024} KB')


# ---- drawing helpers ---------------------------------------------------------------------------------------------------------
def node(ax, x, y, w, h, title, sub=None, ec=None, fill=None, dashed=False, fs_title=8.5):
    """ROS node / block: rounded box, title on top, optional smaller detail lines below. (x, y) is the centre."""
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle='round,pad=0,rounding_size=0.06',
                                fc=fill or C['fill'], ec=ec or C['ink'], lw=0.9, ls='--' if dashed else '-'))
    if sub:
        ax.text(x, y + h / 2 - 0.08, title, ha='center', va='top', fontsize=fs_title, weight='semibold', color=C['ink'])
        ax.text(x, y + h / 2 - 0.30, sub, ha='center', va='top', fontsize=7.2, color=C['muted'], linespacing=1.25)
    else:
        ax.text(x, y, title, ha='center', va='center', fontsize=fs_title, weight='semibold', color=C['ink'], linespacing=1.2)


def link(ax, a, b, text=None, color=None, ls='-', lw=1.0, rad=0.0, tpos=0.5, toff=(0, 4), ha='center', mono=True, tsize=7.2):
    """Arrow a -> b (data coords) with an optional label (a topic name, monospace by default) at fraction tpos."""
    color = color or C['ink']
    ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='-|>', color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0,
                                                    mutation_scale=9, connectionstyle=f'arc3,rad={rad}'))
    if text:
        p = (a[0] + tpos * (b[0] - a[0]), a[1] + tpos * (b[1] - a[1]))
        ax.annotate(text, p, xytext=toff, textcoords='offset points', ha=ha, va='bottom', fontsize=tsize, color=color,
                    family=MONO if mono else None)


def canvas(w, h):
    f, ax = fs.diagram(w, h, (0, w), (0, h), equal=True)
    return f, ax


def arena_outline(ax, color=None, lw=0.0, fill='#c9ced6'):
    """The arena of run w2 (walls, boxes, cylinders from run.json) drawn as filled shapes."""
    a = RUN['arena']
    for cx, cy, sx, sy, *_ in a['walls'] + a['boxes']:
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, fc=fill, ec=color or fill, lw=lw))
    for cx, cy, r, *_ in a['cylinders']:
        ax.add_patch(Circle((cx, cy), r, fc=fill, ec=color or fill, lw=lw))


def run_arrays():
    s = RUN['samples']
    t = np.array([x['t'] for x in s]); gt = np.array([x['gt'] for x in s])
    od = np.array([x['odom'] for x in s]); am = np.array([x['amcl'] for x in s])
    return t, gt, od, am


def test_geometry():
    """The synthetic arena scan generator of src/go2_nav_sim/test/test_registration.py (same geometry as make_arena.py)."""
    spec = importlib.util.spec_from_file_location('tr', ROOT / 'src/go2_nav_sim/test/test_registration.py')
    assert spec is not None and spec.loader is not None
    tr = importlib.util.module_from_spec(spec); spec.loader.exec_module(tr)
    return tr


# ---- MuJoCo renders of the Go2 -----------------------------------------------------------------------------------------------
def go2_render(qpos, azimuth, elevation, lookat, extent_m, size=(1200, 900), bg=(255, 255, 255)):
    """Orthographic render of the Menagerie Go2 with the background (and floor) replaced by `bg`.
    Returns (rgba, extent) where extent = (left, right, bottom, top) in the camera's right/up axes, metres, so the image can be
    placed with imshow(extent=...) and annotated in world units. For azimuth 90, elevation 0: right = +x, up = +z.
    For elevation -90 (top view), azimuth 90: right = +x, up = +y."""
    import mujoco
    from common import require_model
    m = mujoco.MjModel.from_xml_path(require_model(True)); d = mujoco.MjData(m)
    W, H = size
    m.vis.global_.offwidth, m.vis.global_.offheight = max(W, 640), max(H, 480)
    m.vis.global_.orthographic = 1; m.vis.global_.fovy = extent_m
    d.qpos[:] = qpos; mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat; cam.distance = 3.0; cam.azimuth = azimuth; cam.elevation = elevation
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    r.update_scene(d, camera=cam)
    c = mujoco.mjv_averageCamera(r.scene.camera[0], r.scene.camera[1])
    rgb = r.render().copy()
    r.enable_segmentation_rendering(); r.update_scene(d, camera=cam); seg = r.render(); r.disable_segmentation_rendering()
    objid, objtype = seg[:, :, 0], seg[:, :, 1]                                 # Renderer segmentation: (object id, object type)
    rgba = np.dstack([rgb, np.full(rgb.shape[:2], 255, np.uint8)])
    rgba[(objid < 0) | ((objtype == int(mujoco.mjtObj.mjOBJ_GEOM)) & (objid == floor))] = (*bg, 0)   # transparent background
    pos, fwd, up = np.array(c.pos), np.array(c.forward), np.array(c.up)
    right = np.cross(fwd, up)
    cx, cy = np.array(lookat) @ right, np.array(lookat) @ up                    # image centre, world point projected on right/up
    half_h = (c.frustum_top - c.frustum_bottom) / 2; half_w = half_h * W / H
    return rgba, (cx - half_w, cx + half_w, cy - half_h, cy + half_h)


def go2_pose(trunk_h, yaw=0.0):
    """qpos with the trunk at height trunk_h and each foot under its hip (closed-form IK of module 04)."""
    import mujoco
    from common import require_model, LEGS
    from kinematics import Leg
    m = mujoco.MjModel.from_xml_path(require_model(True))
    r_foot = m.geom_size[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')][0]
    q = []
    for leg in LEGS:
        L = Leg(m, leg)
        q += list(L.ik_closed_form(L.hip + np.array([0, L.d, -(trunk_h - r_foot)]))[0])
    return np.r_[0, 0, trunk_h, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2), q]


# ==== chapter 9: state estimation ==============================================================================================
@fig
def ch09_dataflow():
    f, ax = canvas(10.4, 5.4)
    # inputs
    node(ax, 1.0, 4.3, 1.6, 0.75, '3-D lidar', '/lidar/points\n16 beams, 10 Hz', ec=C['gt'])
    node(ax, 1.0, 2.6, 1.6, 0.75, 'IMU', '/imu/data', ec=C['gt'])
    node(ax, 1.0, 0.5, 1.6, 0.7, 'Gazebo ground truth', '/odom_gt', ec=C['gt'], dashed=True)
    # lidar_odom.py container
    ax.add_patch(FancyBboxPatch((2.25, 1.05), 4.65, 4.05, boxstyle='round,pad=0,rounding_size=0.08', fc='#fbfcfd', ec=C['est'], lw=1.1))
    ax.text(2.38, 4.98, 'lidar_odom.py', va='top', fontsize=9, weight='semibold', color=C['est'], family=MONO)
    steps = ['1  transform into base_footprint (TF)',
             '2  crop 0.30–1.0 m high, 0.5–12 m range,\n    voxel 15 cm',
             '3  predict: constant velocity + gyro Δyaw',
             '4  register: Gauss-Newton, point-to-line,\n    Geman-McClure, translation only',
             '5  gate: > 0.6 m/s or > 0.8 rad/s (+ slack)\n    → keep prediction; 8 in a row → new map',
             '6  local map: add after 15 cm or 0.1 rad,\n    keep points within 12 m']
    ys = [4.42, 3.86, 3.30, 2.70, 2.07, 1.45]
    x0, x1 = 2.62, 5.55
    for y, s in zip(ys, steps):
        ax.add_patch(FancyBboxPatch((x0, y - 0.24), x1 - x0, 0.48, boxstyle='round,pad=0,rounding_size=0.04', fc=C['paper'], ec=C['grid'], lw=0.8))
        ax.text(x0 + 0.1, y, s, va='center', fontsize=7.0, color=C['ink'], linespacing=1.15)
    for a, b in zip(ys[:-1], ys[1:]):
        link(ax, (4.1, a - 0.24), (4.1, b + 0.24), lw=0.8)
    ax.annotate('', xy=(x0, 2.70), xytext=(x0, 1.45), arrowprops=dict(arrowstyle='-|>', color=C['est'], lw=0.9,
                connectionstyle='arc3,rad=-0.55', mutation_scale=9))
    ax.text(2.30, 2.05, 'map', fontsize=6.6, color=C['est'], va='center', rotation=90)
    node(ax, 6.25, 2.0, 1.1, 0.7, 'gyro history', 'own callback\ngroup', fs_title=7.5, ec=C['est'])
    ax.plot([6.25, 6.25], [2.35, 3.30], color=C['est'], lw=0.9); link(ax, (6.25, 3.30), (x1, 3.30), color=C['est'], lw=0.9)
    ax.text(6.3, 2.9, 'Δyaw by\ntimestamp', fontsize=6.4, color=C['est'], va='center')
    link(ax, (1.8, 4.3), (x0, 4.42), lw=0.9)
    # IMU: one line under the container, into the gyro history and into the EKF
    ax.plot([1.0, 1.0, 7.6, 7.6], [2.22, 0.85, 0.85, 1.9], color=C['ink'], lw=0.9)
    link(ax, (6.25, 0.85), (6.25, 1.65), lw=0.9); link(ax, (7.6, 1.9), (7.6, 2.225), lw=0.9)
    ax.plot([6.25, 7.6], [0.85, 0.85], 'o', ms=2.5, color=C['ink'])
    ax.text(3.6, 0.80, '/imu/data', fontsize=7.2, family=MONO, va='top')
    # EKF
    node(ax, 8.75, 2.75, 2.75, 1.05, 'odom_ekf  (robot_localization)',
         '50 Hz, two_d_mode\nx, y  ←  /lidar_odom with its covariance\nyaw rate  ←  /imu/data', ec=C['est'])
    link(ax, (6.9, 3.75), (7.55, 3.28), color=C['est'])
    ax.text(7.0, 3.84, '/lidar_odom', fontsize=7.2, family=MONO, color=C['est'])
    node(ax, 8.75, 4.6, 2.75, 0.75, 'Nav2, AMCL, slam_toolbox', 'chapter 10')
    link(ax, (8.75, 3.28), (8.75, 4.22), 'TF odom → base_footprint\n/odom', color=C['est'], toff=(6, -8), ha='left')
    node(ax, 8.75, 0.5, 2.75, 0.7, 'odom_eval.py, bag_metrics.py', 'scoring only, never fed back', dashed=True, ec=C['gt'])
    ax.plot([1.8, 7.0], [0.5, 0.5], color=C['gt'], lw=0.9, ls='--')
    link(ax, (7.0, 0.5), (7.375, 0.5), color=C['gt'], ls='--')
    ax.plot([10.125, 10.3, 10.3, 10.125], [2.75, 2.75, 0.5, 0.5], color=C['gt'], lw=0.9, ls='--')
    save(f, 'ch09-dataflow')


@fig
def ch09_heights():
    h = 0.217; champ = 0.426; off = champ - h                                   # 0.209 m: base_footprint below the floor
    rgb, ext = go2_render(go2_pose(h), 90, 0, (0.02, 0, 0.19), 0.62)
    f, ax = plt.subplots(figsize=(8.6, 4.8))
    ax.imshow(rgb, extent=ext, zorder=1)
    ax.set_xlim(-0.62, 1.55); ax.set_ylim(-0.29, 0.86); ax.set_aspect('equal'); ax.grid(False)
    ax.axhspan(-0.29, 0, color='#eceef1', zorder=0, lw=0)
    ax.axhline(0, color=C['ink'], lw=1.0, zorder=2)
    ax.text(-0.36, -0.02, 'real ground', va='top', fontsize=8, color=C['ink'])
    band = (0.30 - off, 1.0 - off)
    ax.axhspan(*band, xmin=0.0, xmax=1, color=C['est'], alpha=0.07, zorder=0, lw=0)
    for z in band:
        ax.axhline(z, color=C['est'], lw=0.8, ls='--', zorder=2)
    ax.text(1.53, band[1] - 0.012, 'lidar_odom keeps points between\n0.30 and 1.0 m in base_footprint\n= 0.09 to 0.79 m above the ground',
            ha='right', va='top', fontsize=7.6, color=C['est'])
    # frames
    fs.frame_axes(ax, (0, h), length=0.09, color=C['ink']); ax.plot(0, h, 'o', ms=3.5, color=C['ink'], zorder=4)
    ax.annotate('base_link', (0, h), xytext=(-125, 22), textcoords='offset points', fontsize=8, family=MONO,
                arrowprops=dict(arrowstyle='-', color=C['muted'], lw=0.6))
    ax.plot(0, h + 0.10, 's', ms=5, color=C['plan'], zorder=4)
    ax.annotate('lidar_link of this stack\n(base_link + 0.10 m)', (0, h + 0.10), xytext=(-150, 40), textcoords='offset points',
                fontsize=7.6, color=C['plan'], arrowprops=dict(arrowstyle='-', color=C['plan'], lw=0.6))
    fs.frame_axes(ax, (0, -off), length=0.07, color=C['bad']); ax.plot(0, -off, 'o', ms=3.5, color=C['bad'], zorder=4)
    ax.text(0.10, -off - 0.04, 'base_footprint as CHAMP publishes it:\na constant 0.426 m under base_link', fontsize=7.6,
            color=C['bad'], va='top', family=None)
    # dimensions
    def dim(x, z0, z1, txt, color, side='right', ty=None):
        ax.annotate('', xy=(x, z1), xytext=(x, z0), arrowprops=dict(arrowstyle='<|-|>', color=color, lw=0.8, mutation_scale=7,
                                                                      shrinkA=0, shrinkB=0))
        ax.text(x + (0.015 if side == 'right' else -0.015), (z0 + z1) / 2 if ty is None else ty, txt, color=color, fontsize=7.8, va='center',
                ha='left' if side == 'right' else 'right')
    dim(0.62, 0, h, '0.217 m real trunk height\n(Gazebo, while walking)', C['ink'], ty=0.165)
    dim(-0.40, -off, h, '0.426 m', C['bad'], side='left', ty=-0.1)
    dim(0.48, -off, 0, '0.209 m', C['bad'])
    ax.set_xticks([]); ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8]); ax.set_ylabel('height above the real ground [m]')
    ax.spines['bottom'].set_visible(False)
    save(f, 'ch09-heights')


@fig
def ch09_point_to_line():
    from registration import register, rot
    tr = test_geometry()
    true = np.array([-4.4, -3.5, 0.0]); src = tr.scan_from(true)
    guess = true + np.array([0.20, -0.15, 0.0])                                 # the offset used in the unit test
    lm = tr.LMAP
    sol = register(src, lm, guess)[0]
    f, (ax, ax2) = plt.subplots(1, 2, figsize=(9.2, 4.2), gridspec_kw=dict(width_ratios=[1.35, 1]))
    win = (-6.15, -4.55, -5.15, -3.75)
    inwin = lambda p: (p[:, 0] > win[0]) & (p[:, 0] < win[1]) & (p[:, 1] > win[2]) & (p[:, 1] < win[3])
    mp = lm.pts; k = inwin(mp)
    ax.plot(mp[k, 0], mp[k, 1], 'o', color=C['gt'], ms=3.5, label='map points, short line = PCA direction', zorder=2)
    for p, n in zip(mp[k], lm.nrm[k]):
        t = np.array([-n[1], n[0]])
        ax.plot(*np.c_[p - 0.07 * t, p + 0.07 * t], color=C['gt'], lw=0.6, zorder=1)
    q = src @ rot(guess[2]).T + guess[:2]; kq = inwin(q)
    d, j = lm.tree.query(q[kq], distance_upper_bound=0.8)
    ax.plot(q[kq, 0], q[kq, 1], 'o', color=C['est'], ms=3.2, label='scan at the prediction (0.25 m off)', zorder=3)
    for qi, di, ji in zip(q[kq], d, j):
        if np.isfinite(di):
            n = lm.nrm[ji]; r = n @ (qi - lm.pts[ji])
            ax.plot(*np.c_[qi, qi - r * n], color=C['plan'], lw=0.9, zorder=2)
    qs = src @ rot(sol[2]).T + sol[:2]; ks = inwin(qs)
    ax.plot(qs[ks, 0], qs[ks, 1], 'x', color=C['good'], ms=4, mew=0.9, label='scan after registration', zorder=4)
    ax.plot([], [], color=C['plan'], label='residual $r_i$ along the normal')
    ax.set_xlim(win[0], win[1]); ax.set_ylim(win[2], win[3]); ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='upper right', fontsize=7.4)
    err0 = np.hypot(*(guess[:2] - true[:2])) * 100; err1 = np.hypot(*(sol[:2] - true[:2])) * 100
    ax.set_title(f'Corner seen from A: error {err0:.1f} cm → {err1:.1f} cm')
    # robust weight and gate
    r = np.linspace(-0.6, 0.6, 400); s = 0.12
    ax2.plot(r, (s ** 2 / (s ** 2 + r ** 2)) ** 2, color=C['est'], label=r'Geman-McClure weight, $\sigma$ = 0.12 m')
    ax2.plot(r, np.ones_like(r), color=C['gt'], ls='--', lw=1, label='least squares (weight 1)')
    for it in range(12):
        tau = max(0.25, 0.8 - 0.1 * it)
        ax2.axvline(tau, color=C['plan'], lw=0.5, alpha=0.6); ax2.axvline(-tau, color=C['plan'], lw=0.5, alpha=0.6)
    ax2.text(0.80, 0.62, 'association gate\n0.8 m → 0.25 m\nover the iterations', color=C['plan'], fontsize=7.4, ha='right')
    ax2.plot([s], [0.25], 'o', color=C['est']); ax2.annotate('$r=\\sigma$: weight 0.25', (s, 0.25), xytext=(12, 8),
                                                             textcoords='offset points', fontsize=7.6, color=C['est'])
    ax2.set_xlim(-0.85, 0.85); ax2.set_ylim(0, 1.12); ax2.set_xlabel('residual $r$ [m]'); ax2.set_ylabel('weight $w(r)$')
    ax2.legend(loc='upper left', fontsize=7.2); ax2.set_title('How much one residual counts')
    save(f, 'ch09-point-to-line')
    return err0, err1


def _info(src, lmap, pose):
    """2x2 translation information of register()'s Gauss-Newton system at `pose` (same weights, gate 0.25 m)."""
    from registration import rot
    q = src @ rot(pose[2]).T + pose[:2]; d, j = lmap.tree.query(q, distance_upper_bound=0.25); ok = np.isfinite(d)
    n = lmap.nrm[j[ok]]; res = np.einsum('ij,ij->i', n, q[ok] - lmap.pts[j[ok]])
    w = lmap.wgt[j[ok]] * (0.12 ** 2 / (0.12 ** 2 + res ** 2)) ** 2
    ev, V = np.linalg.eigh((n * w[:, None]).T @ n)
    return ev, V, int(ok.sum())


@fig
def ch09_degeneracy():
    from registration import LineMap, voxel_downsample
    tr = test_geometry()
    pose = np.array([0.0, 0.0, 0.0]); src = tr.scan_from(pose)
    ev_a, V_a, n_a = _info(src, tr.LMAP, pose)
    xs = np.arange(-15, 15, 0.03)                                                # the corridor of the unit test
    walls = voxel_downsample(np.vstack([np.c_[xs, np.full_like(xs, -1.0)], np.c_[xs, np.full_like(xs, 1.0)]]), 0.15)
    near = walls[np.hypot(*walls.T) < 12]
    ev_c, V_c, n_c = _info(near, LineMap(walls), np.zeros(3))
    f, axs = plt.subplots(1, 3, figsize=(10.0, 3.6), gridspec_kw=dict(width_ratios=[1.25, 1.6, 0.9]))
    for ax, pts, ev, V, n, title, lim in [(axs[0], src, ev_a, V_a, n_a, f'Arena, {n_a} matched points', (-6.4, 6.4, -5.3, 5.3)),
                                          (axs[1], near, ev_c, V_c, n_c, f'Corridor, {n_c} matched points', (-12.5, 12.5, -3, 3))]:
        ax.plot(pts[:, 0], pts[:, 1], '.', color=C['gt'], ms=2)
        ax.plot(0, 0, 'o', color=C['ink'], ms=4)
        for k in range(2):
            col = C['bad'] if ev[k] / n < 0.02 else C['est']
            v = V[:, k] * np.sign(V[np.argmax(np.abs(V[:, k])), k])
            if ev[k] / n >= 0.02:                                               # fixed arrow length, value written at the tip
                ax.annotate('', xy=tuple(2.0 * v), xytext=(0, 0), arrowprops=dict(arrowstyle='-|>', color=col, lw=1.5, mutation_scale=10))
                ax.text(*(2.0 * v + np.array([0.15, 0.15])), f'{ev[k] / n:.3f}', color=col, fontsize=7.6, weight='semibold')
            else:
                ax.annotate('', xy=tuple(1.6 * v), xytext=tuple(-1.6 * v), arrowprops=dict(arrowstyle='<|-|>', color=col, lw=1.2,
                                                                                         ls='--', mutation_scale=9))
                ax.text(*(1.7 * v + np.array([0, 0.25])), 'no information:\nprior holds the prediction', color=col, fontsize=7.2)
        ax.set_xlim(*lim[:2]); ax.set_ylim(*lim[2:]); ax.set_aspect('equal'); ax.set_title(title)
        ax.set_xlabel('x [m]')
    axs[0].set_ylabel('y [m]')
    b = axs[2]
    vals = [ev_a[0] / n_a, ev_a[1] / n_a, ev_c[0] / n_c, ev_c[1] / n_c]
    cols = [C['bad'] if v < 0.02 else C['est'] for v in vals]
    b.bar(range(4), vals, color=cols, width=0.6)
    for i, v in enumerate(vals):
        b.text(i, v + 0.02, f'{v:.3f}', ha='center', fontsize=7.4)
    b.axhline(0.02, color=C['bad'], lw=0.9, ls='--'); b.text(0.5, 0.25, 'threshold 0.02:\nbelow it the prior\nis switched on', color=C['bad'], fontsize=7.2, ha='center')
    b.set_xticks(range(4), ['arena\nλ₁', 'arena\nλ₂', 'corridor\nλ₁', 'corridor\nλ₂'], fontsize=7.4)
    b.set_ylim(0, 1.1); b.set_title('Eigenvalue / matched points')
    save(f, 'ch09-degeneracy')
    return vals, ev_a, n_a


@fig
def ch09_yaw_flip():
    from registration import LineMap, rot, voxel_downsample
    tr = test_geometry()
    pose = np.array([0.0, 0.0, 0.0]); src = tr.scan_from(pose)

    def cost(s, lm, yaw, sig=0.12):
        q = s @ rot(yaw).T; d, j = lm.tree.query(q, distance_upper_bound=0.8); ok = np.isfinite(d)
        n = lm.nrm[j[ok]]; res = np.einsum('ij,ij->i', n, q[ok] - lm.pts[j[ok]])
        return ((res ** 2 / (sig ** 2 + res ** 2)).sum() + (~ok).sum()) / len(s)    # robust loss, unmatched points count 1
    pts = tr.outline_points()
    room = voxel_downsample(pts[(np.abs(pts[:, 0]) > 5.8) | (np.abs(pts[:, 1]) > 4.8)], 0.15); lroom = LineMap(room)
    yaws = np.radians(np.arange(-180, 181, 2))
    c_arena = [cost(src, tr.LMAP, y) for y in yaws]; c_room = [cost(room, lroom, y) for y in yaws]
    f, (ax, ax2) = plt.subplots(1, 2, figsize=(9.6, 3.7), gridspec_kw=dict(width_ratios=[1.1, 1]))
    arena_outline(ax)
    flip = src @ rot(math.pi).T
    ax.plot(src[:, 0], src[:, 1], '.', color=C['est'], ms=2.2, label='scan at the true yaw')
    ax.plot(flip[:, 0], flip[:, 1], '.', color=C['est2'], ms=2.2, label='same scan turned 180°')
    ax.plot(0, 0, 'o', color=C['ink'], ms=4)
    ax.set_xlim(-6.4, 6.4); ax.set_ylim(-5.4, 5.4); ax.set_aspect('equal')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.16), fontsize=7.2, markerscale=4, ncol=2)
    ax.set_title('Robot at (0, 0) in the arena'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    deg = np.degrees(yaws)
    ax2.plot(deg, c_arena, color=C['est'], label='arena scan')
    ax2.plot(deg, c_room, color=C['gt'], ls='--', label='outer walls only')
    i0 = np.argmin(np.abs(deg)); i1 = np.argmin(np.abs(deg - 180))
    ax2.annotate(f'{c_arena[i1]:.2f}', (180, c_arena[i1]), xytext=(-30, 18), textcoords='offset points', fontsize=7.6, color=C['est'],
                 arrowprops=dict(arrowstyle='-', color=C['est'], lw=0.6))
    ax2.annotate(f'{c_arena[i0]:.3f}', (0, c_arena[i0]), xytext=(14, 30), textcoords='offset points', fontsize=7.6, color=C['est'],
                 arrowprops=dict(arrowstyle='-', color=C['est'], lw=0.6))
    ax2.set_xticks([-180, -90, 0, 90, 180]); ax2.set_xlabel('yaw offset from the truth [deg]')
    ax2.set_ylabel('mean robust cost per point'); ax2.legend(loc='upper center', fontsize=7.4, ncol=2)
    ax2.set_ylim(0, 1.15); ax2.set_title('A second minimum at 180°')
    save(f, 'ch09-yaw-flip')
    return c_arena[i0], c_arena[i1], c_room[i1]


@fig
def ch09_callbacks():
    f, ax = plt.subplots(figsize=(9.0, 3.7))
    ax.set_xlim(-0.06, 0.32); ax.set_ylim(-0.8, 4.6); ax.axis('off')
    rows = {'before: one thread': 3.6, 'after: two callback groups,\nmulti-threaded executor': 1.2}
    imu_t = np.arange(0.0, 0.3, 0.01)
    for name, y in rows.items():
        ax.text(-0.058, y + 0.45, name, fontsize=8.5, weight='semibold', va='center')
        ax.plot([0, 0.3], [y, y], color=C['grid'], lw=0.8, zorder=0); ax.plot([0, 0.3], [y - 0.75, y - 0.75], color=C['grid'], lw=0.8)
        ax.text(-0.004, y, 'scan', ha='right', va='center', fontsize=7.6, color=C['muted'])
        ax.text(-0.004, y - 0.75, 'IMU', ha='right', va='center', fontsize=7.6, color=C['muted'])
        for ts in (0.0, 0.1, 0.2):
            ax.add_patch(Rectangle((ts, y - 0.17), 0.055, 0.34, fc=C['est'], ec='none', alpha=0.85, zorder=2))
            ax.text(ts + 0.0275, y, 'register', ha='center', va='center', fontsize=6.6, color='white', zorder=3)
        for ti in imu_t:
            busy = any(ts <= ti < ts + 0.055 for ts in (0.0, 0.1, 0.2))
            if name.startswith('before') and busy:
                ax.plot(ti, y - 0.75, 'o', mfc='white', mec=C['bad'], ms=4.2, mew=0.9)                 # waits in the queue
            else:
                ax.plot(ti, y - 0.75, 'o', color=C['good'], ms=4.2)
    ax.annotate('IMU samples wait while registration runs\nand miss the next scan\'s integration window', (0.035, 3.6 - 0.75),
                xytext=(0.12, 2.3), fontsize=7.6, color=C['bad'], va='center', arrowprops=dict(arrowstyle='-', color=C['bad'], lw=0.6))
    ax.annotate('every sample goes into the cumulative yaw history;\nthe scan looks up Δyaw by timestamp', (0.235, 1.2 - 0.75),
                xytext=(0.12, -0.5), fontsize=7.6, color=C['good'], va='center', arrowprops=dict(arrowstyle='-', color=C['good'], lw=0.6))
    ax.text(0.30, 4.45, 'schematic, durations not to scale', ha='right', fontsize=7, color=C['muted'])
    ax.annotate('', xy=(0.3, -0.05), xytext=(0, -0.05), arrowprops=dict(arrowstyle='-|>', color=C['muted'], lw=0.7))
    ax.text(0.3, 0.02, 'time', ha='right', fontsize=7.2, color=C['muted'])
    save(f, 'ch09-callbacks')


@fig
def ch09_run_w2():
    t, gt, od, am = run_arrays()
    upd = np.r_[True, np.any(np.diff(am, axis=0) != 0, axis=1)]
    e_od = np.hypot(*(od[:, :2] - gt[:, :2]).T) * 100; e_am = np.hypot(*(am[:, :2] - gt[:, :2]).T) * 100
    f, (ax, ax2) = plt.subplots(1, 2, figsize=(10.0, 4.0), gridspec_kw=dict(width_ratios=[1.05, 1]))
    arena_outline(ax)
    ax.plot(gt[:, 0], gt[:, 1], color=C['gt'], lw=3.2, alpha=0.6, label='ground truth')
    ax.plot(od[:, 0], od[:, 1], color=C['est'], lw=1.1, label='lidar-inertial odometry')
    ax.plot(am[upd, 0], am[upd, 1], '.', color=C['est2'], ms=2.5, label='AMCL updates')
    for k, (x, y) in RUN['arena'].items() if False else [('A', RUN['arena']['A']), ('B', RUN['arena']['B'])]:
        ax.plot(x, y, 'o', mfc='white', mec=C['ink'], ms=7); ax.text(x + 0.25, y + 0.2, k, fontsize=9, weight='semibold')
    ax.set_xlim(-6.3, 6.3); ax.set_ylim(-5.3, 5.3); ax.set_aspect('equal'); ax.legend(loc='lower right', fontsize=7.2)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title('Run w2, A to B')
    ax2.plot(t, e_od, color=C['est'], label='odometry')
    ax2.plot(t[upd], e_am[upd], '.', color=C['est2'], ms=3, label='AMCL at its updates')
    ax2.set_xlabel('time since the recording started [s] (wall clock)'); ax2.set_ylabel('distance to ground truth [cm]'); ax2.set_title('Position error')
    ax2.legend(loc='upper left', fontsize=7.4); ax2.set_ylim(0, None)
    save(f, 'ch09-run-w2')
    rms = lambda e: float(np.sqrt(np.mean(e ** 2)))
    return rms(e_od), float(e_od.max()), rms(e_am[upd]), int(upd.sum())



# ==== chapter 10: mapping and navigation =======================================================================================
def nav2_cost(d, r_ins=0.33, r_inf=0.75, k=4.0):
    """Nav2 inflation layer cost at distance d [m] from the nearest lethal cell (nav2_params.yaml: robot_radius 0.33,
    inflation_radius 0.75, cost_scaling_factor 4.0): 254 on the obstacle, 253 within the inscribed radius,
    252 exp(-k (d - r_ins)) out to the inflation radius, 0 beyond."""
    d = np.asarray(d, float)
    c = np.where(d <= r_inf, 252.0 * np.exp(-k * (d - r_ins)), 0.0)
    c = np.where(d <= r_ins, 253.0, c)
    return np.where(d <= 0, 254.0, c)


@fig
def ch10_nav_graph():
    f, ax = canvas(10.9, 6.3)
    c1, c2, c3, c4 = 1.1, 4.0, 6.8, 9.75
    r1, r2, r3, r4, r5 = 5.75, 4.35, 2.95, 1.55, 0.4
    W = 1.9; H = 0.72; w2 = W / 2; h2 = H / 2
    node(ax, c1, r1, W, H, 'Gazebo', 'robot, arena, sensors', ec=C['gt'])
    node(ax, c2, r1, W, H, 'pointcloud_to_laserscan', 'band 0.30–0.90 m\n(scan.yaml)')
    node(ax, c3, r1, W, H, 'slam_toolbox', 'passive, own\nslam_map frame', dashed=True)
    node(ax, c4, r1, W, H, 'rosbag, dashboard', 'records the topics\n(chapter 11)', dashed=True, ec=C['gt'])
    node(ax, c1, r2, W, H, 'lidar_odom, odom_ekf', 'chapter 9', ec=C['est'])
    node(ax, c2, r2, W, H, 'amcl', 'scan against the known map', ec=C['est2'])
    node(ax, c3, r2, W, H, 'map_server', 'maps/arena.yaml')
    node(ax, c1, r3, W, H, 'go_to_goal.py', 'true start pose, goal B')
    node(ax, c2, r3, W, H, 'bt_navigator', 'behaviour tree')
    node(ax, c3, r3, W, H, 'planner_server', 'NavFn A*, global costmap', ec=C['plan'])
    node(ax, c3, r4, W, H, 'controller_server', 'Pure Pursuit 0.25 m/s,\nlocal costmap', ec=C['plan'])
    node(ax, c4, r4, W, H, 'velocity_smoother', '≤ 0.3 m/s, ≤ 0.5 rad/s')
    node(ax, c4, r5, W, H, 'CHAMP', 'gait + leg controllers', ec=C['gt'])
    L = lambda a, b, txt=None, **k: link(ax, a, b, txt, **k)
    L((c1 + w2, r1), (c2 - w2, r1), '/lidar/points', toff=(0, 3))
    L((c1, r1 - h2), (c1, r2 + h2), '/lidar/points\n/imu/data', toff=(4, -10), ha='left')
    L((c2, r1 - h2), (c2, r2 + h2), '/scan', toff=(4, -4), ha='left', color=C['est2'])
    L((c2 + w2, r1), (c3 - w2, r1), '/scan', toff=(0, 3))
    xm = (c2 + w2 + c3 - w2) / 2
    ax.plot([c2 + w2, xm, xm], [r1 - 0.2, r1 - 0.2, r3 + h2 + 0.25], color=C['ink'], lw=1.0)
    L((xm, r3 + h2 + 0.25), (c3 - 0.5, r3 + h2))
    ax.text(xm + 0.05, (r1 + r2) / 2 + 0.15, '/scan\n(costmaps)', fontsize=7.2, family=MONO, va='center')
    L((c3 + w2, r1), (c4 - w2, r1), '/slam/map', toff=(0, 3))
    L((c1 + w2, r2), (c2 - w2, r2), 'TF odom→\nbase_footprint', toff=(0, 3), color=C['est'], tsize=6.6)
    L((c3 - w2, r2), (c2 + w2, r2), '/map', toff=(0, 3))
    L((c3, r2 - h2), (c3, r3 + h2), '/map', toff=(4, -4), ha='left')
    L((c2 + 0.5, r2 - h2), (c3 - w2, r3 + 0.12), 'TF map→odom', tpos=0.45, toff=(-6, -12), ha='right', color=C['est2'], tsize=6.6)
    L((c1 + 0.6, r3 + h2), (c2 - 0.6, r2 - h2), '/initialpose', tpos=0.5, toff=(-4, 0), ha='right', color=C['est2'])
    L((c1 + w2, r3), (c2 - w2, r3), 'navigate_to_pose', toff=(0, 3), tsize=6.4)
    ax.text((c1 + c2) / 2, r3 - 0.06, 'action', fontsize=6.3, color=C['muted'], ha='center', va='top')
    L((c2 + w2, r3 - 0.1), (c3 - w2, r3 - 0.1), 'compute_path_\nto_pose', toff=(0, 3), tsize=6.4)
    L((c2 + 0.6, r3 - h2), (c3 - w2, r4), 'follow_path', tpos=0.55, toff=(-6, -8), ha='right', tsize=6.4)
    L((c3, r3 - h2), (c3, r4 + h2), '/plan', toff=(4, -4), ha='left', color=C['plan'])
    L((c3 + w2, r4), (c4 - w2, r4), '/cmd_vel_nav', toff=(0, 3))
    L((c4, r4 - h2), (c4, r5 + h2), '/cmd_vel', toff=(4, -4), ha='left')
    ax.plot([c4 - w2, 0.1, 0.1], [r5, r5, r1], color=C['gt'], lw=0.9)
    L((0.1, r1), (c1 - w2, r1), color=C['gt'])
    ax.text(0.18, r5 + 0.06, 'joint effort commands', fontsize=7.0, color=C['gt'], va='bottom')
    save(f, 'ch10-nav-graph')


@fig
def ch10_tf_tree():
    f, ax = canvas(9.6, 4.9)
    def fr(x, y, name, color=None, dashed=False, w=1.75):
        node(ax, x, y, w, 0.46, name, ec=color or C['ink'], dashed=dashed, fs_title=8.2)
    def edge(a, b, who, color=None, ha='left', off=(5, 0)):
        link(ax, (a[0], a[1] - 0.23), (b[0], b[1] + 0.23), color=color or C['ink'])
        ax.annotate(who, ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), xytext=off, textcoords='offset points', fontsize=7.0,
                    color=color or C['muted'], ha=ha, va='center', linespacing=1.15)
    P = dict(map=(2.4, 4.5), odom=(2.4, 3.45), bf=(2.4, 2.4), bl=(2.4, 1.35), lidar=(0.9, 0.3), cam=(3.15, 0.3),
             opt=(5.4, 0.3), legs=(7.6, 0.3), slam=(7.2, 4.5))
    fr(*P['map'], 'map', C['est2']); fr(*P['odom'], 'odom', C['est']); fr(*P['bf'], 'base_footprint', C['bad'])
    fr(*P['bl'], 'base_link'); fr(*P['lidar'], 'lidar_link', w=1.5); fr(*P['cam'], 'camera_link', w=1.6)
    fr(*P['opt'], 'camera_optical', w=1.75); fr(*P['legs'], 'hip, thigh, calf, foot\n× 4 legs', w=1.9)
    fr(*P['slam'], 'slam_map', dashed=True)
    edge(P['map'], P['odom'], 'amcl, at each update\n(corrects drift; may jump)', C['est2'])
    edge(P['odom'], P['bf'], 'odom_ekf, 50 Hz\n(smooth, never jumps)', C['est'])
    edge(P['bf'], P['bl'], "CHAMP base_to_footprint:\nconstant (0, 0, 0.426), no tilt", C['bad'])
    link(ax, (2.1, 1.12), (P['lidar'][0], 0.53)); link(ax, (2.4, 1.12), (P['cam'][0] - 0.2, 0.53))
    link(ax, (2.7, 1.12), (P['legs'][0] - 0.4, 0.53))
    link(ax, (P['cam'][0] + 0.8, 0.3), (P['opt'][0] - 0.875, 0.3))
    ax.text(0.9, 0.85, '(0, 0, 0.10)', fontsize=6.8, color=C['muted'], ha='center')
    ax.text(3.0, 0.9, '(0.28, 0, 0.03)', fontsize=6.8, color=C['muted'], ha='left')
    ax.text(4.45, 0.42, 'rpy −90°, 0, −90°', fontsize=6.6, color=C['muted'], ha='center', va='bottom')
    ax.text(5.2, 0.92, 'robot_state_publisher, from the URDF\n(static or from joint states)', fontsize=6.8, color=C['muted'])
    ax.text(P['slam'][0], P['slam'][1] - 0.38, 'slam_toolbox publishes no transform\n(transform_publish_period: 0.0);\n'
            'only /slam/map is expressed in it', fontsize=7.0, color=C['muted'], ha='center', va='top')
    ax.text(P['map'][0] - 1.0, P['map'][1], 'root', fontsize=7.0, color=C['muted'], ha='right', va='center')
    save(f, 'ch10-tf-tree')


@fig
def ch10_height_band():
    off = 0.209; h_lidar = 0.217 + 0.10                                        # real heights: trunk 0.217 m, lidar mount +0.10 m
    f, ax = plt.subplots(figsize=(9.4, 4.2))
    ax.axhspan(-0.35, 0, color='#eceef1', lw=0, zorder=0); ax.axhline(0, color=C['ink'], lw=1.0)
    band = (0.30 - off, 0.90 - off)
    ax.axhspan(*band, color=C['est'], alpha=0.08, lw=0, zorder=0)
    for z in band:
        ax.axhline(z, color=C['est'], ls='--', lw=0.8)
    ax.axhline(0.12 - off, color=C['bad'], ls='--', lw=0.9)
    ax.text(7.95, 0.12 - off - 0.03, 'first version: min_height 0.12 m = 0.09 m below the floor', color=C['bad'], fontsize=7.6,
            ha='right', va='top')
    ax.text(7.95, band[1] - 0.02, 'scan.yaml keeps 0.30–0.90 m in base_footprint\n= 0.09–0.69 m above the real ground',
            color=C['est'], fontsize=7.6, ha='right', va='top')
    angs = np.linspace(-0.261799, 0.261799, 16)                                  # sensors.xacro: 16 beams, ±15°
    hits, obst = [], []
    for a in angs:
        z_box = h_lidar + 5.6 * math.tan(a)                                      # front face of a 0.45 m obstacle at 5.6 m
        x_floor = h_lidar / math.tan(-a) if a < 0 else np.inf
        if x_floor < 5.6:
            x_end = x_floor; hits.append(x_floor)
        elif 0 <= z_box <= 0.45:
            x_end = 5.6; obst.append(z_box)
        else:
            x_end = 8.0
        ax.plot([0, x_end], [h_lidar, h_lidar + x_end * math.tan(a)], color=C['gt'], lw=0.5, alpha=0.8)
    ax.plot(hits, np.zeros(len(hits)), 'o', color=C['bad'], ms=4, zorder=4, label='floor returns')
    ax.plot(np.full(len(obst), 5.6), obst, 'o', color=C['est'], ms=4, zorder=4, label='obstacle returns')
    ax.add_patch(Rectangle((5.6, 0), 0.5, 0.45, fc='#c9ced6', ec=C['muted'], lw=0.6, zorder=3))
    ax.text(5.85, 0.47, 'lowest obstacle\n0.45 m', ha='center', va='bottom', fontsize=7.4)
    ax.plot(0, h_lidar, 's', color=C['plan'], ms=6, zorder=5)
    ax.annotate('lidar, 0.317 m above the real ground\n(0.526 m in base_footprint)', (0, h_lidar), xytext=(10, 32),
                textcoords='offset points', fontsize=7.6, color=C['plan'], arrowprops=dict(arrowstyle='-', color=C['plan'], lw=0.6))
    ax.annotate('', xy=(3.0, 0.19), xytext=(3.0, 0.0), arrowprops=dict(arrowstyle='<|-|>', color=C['ink'], lw=0.8, mutation_scale=7))
    ax.text(3.06, 0.06, '3.6° of tilt moves a point 3 m away\nby 0.19 m, twice the 0.09 m margin', fontsize=7.4, va='bottom')
    ax.text(min(hits) - 0.05, -0.05, f'the lowest beam reaches the floor {min(hits):.2f} m out', fontsize=7.4, color=C['bad'],
            va='top')
    ax.set_xlim(-0.2, 8.0); ax.set_ylim(-0.35, 1.05)
    ax.set_xlabel('horizontal distance from the lidar [m]'); ax.set_ylabel('height above the real ground [m]')
    sec = ax.secondary_yaxis('right', functions=(lambda z: z + off, lambda z: z - off))
    sec.set_ylabel('height in base_footprint [m]', color=C['bad']); sec.tick_params(colors=C['bad'])
    ax.legend(loc='upper left', fontsize=7.4)
    save(f, 'ch10-height-band')
    return min(hits)


@fig
def ch10_inflation():
    d = np.linspace(0, 1.0, 1001)
    f, (ax, ax2) = plt.subplots(1, 2, figsize=(9.8, 3.7), gridspec_kw=dict(width_ratios=[1.1, 1]))
    ax.plot(d, nav2_cost(d), color=C['ink'])
    ax.axvline(0.33, color=C['muted'], lw=0.7, ls=':'); ax.axvline(0.75, color=C['muted'], lw=0.7, ls=':')
    ax.text(0.335, 40, 'robot_radius\n0.33 m', fontsize=7.2, color=C['muted']); ax.text(0.755, 120, 'inflation_radius\n0.75 m', fontsize=7.2, color=C['muted'])
    ax.axhline(253, color=C['bad'], lw=0.6, ls='--'); ax.text(0.98, 243, '253 inscribed: the robot centre may not enter', color=C['bad'],
                                                             fontsize=7.2, ha='right', va='top')
    c0 = float(nav2_cost(0.425)); ax.plot(0.425, c0, 'o', color=C['plan'])
    ax.annotate(f'centre of a 0.85 m gap: {c0:.0f}', (0.425, c0), xytext=(18, 10), textcoords='offset points', fontsize=7.6,
                color=C['plan'], arrowprops=dict(arrowstyle='-', color=C['plan'], lw=0.6))
    ax.text(0.5, 60, r'$252\,e^{-4.0\,(d-0.33)}$', fontsize=9, color=C['ink'])
    ax.set_xlabel('distance from the nearest obstacle cell $d$ [m]'); ax.set_ylabel('cost'); ax.set_ylim(0, 270)
    ax.set_title('Inflation cost with this stack\'s parameters')
    # across the gap
    x = np.linspace(-0.6, 0.6, 1201)
    for gap, col, ls in [(0.85, C['plan'], '-'), (2.0, C['good'], '--')]:
        dd = np.minimum(np.abs(x + gap / 2), np.abs(x - gap / 2)); c = nav2_cost(dd)
        c[np.abs(x) >= gap / 2] = 254
        ax2.plot(x, c, color=col, ls=ls, label=f'{gap:.2f} m gap')
    ax2.axhspan(254, 270, color=C['ink'], alpha=0.08, lw=0)
    ax2.add_patch(Rectangle((-0.6, 0), 0.6 - 0.425, 254, fc='#c9ced6', ec='none', zorder=0))
    ax2.add_patch(Rectangle((0.425, 0), 0.6 - 0.425, 254, fc='#c9ced6', ec='none', zorder=0))
    ax2.annotate('', xy=(0.33, 30), xytext=(-0.33, 30), arrowprops=dict(arrowstyle='<|-|>', color=C['est'], lw=0.9, mutation_scale=8))
    ax2.text(0, 36, 'footprint circle, 0.66 m: fits', ha='center', fontsize=7.4, color=C['est'])
    ax2.set_xlabel('position across the gap [m]'); ax2.set_ylabel('cost'); ax2.set_ylim(0, 270); ax2.set_xlim(-0.6, 0.6)
    ax2.legend(loc='lower left', fontsize=7.4, bbox_to_anchor=(0.0, 0.25)); ax2.set_title('Cost across a gap between two obstacles')
    save(f, 'ch10-inflation')
    return c0, float(nav2_cost(0.43))


@fig
def ch10_costmap():
    from scipy import ndimage
    import matplotlib.colors as mcolors
    yml: dict[str, Any] = dict(resolution=0.05, origin=(-6.5, -5.5))                               # src/go2_nav_sim/maps/arena.yaml
    raw = plt.imread(ROOT / 'src/go2_nav_sim/maps/arena.pgm')
    raw = raw if raw.dtype == np.uint8 else (raw * 255).astype(np.uint8)
    occ = (255 - raw.astype(float)) / 255 > 0.65                                 # trinary: occupied above occupied_thresh
    d = np.asarray(ndimage.distance_transform_edt(~occ)) * yml['resolution']
    cost = nav2_cost(d)
    H, Wd = occ.shape; ext = (yml['origin'][0], yml['origin'][0] + Wd * 0.05, yml['origin'][1], yml['origin'][1] + H * 0.05)
    cmap = mcolors.LinearSegmentedColormap.from_list('cost', ['#ffffff', '#cfd5dd', '#7a8392', '#1f2328'])
    f, ax = plt.subplots(figsize=(7.4, 6.0))
    ax.imshow(cost, origin='upper', extent=ext, cmap=cmap, vmin=0, vmax=254, interpolation='nearest')
    ax.contour(np.flipud(cost >= 253), levels=[0.5], extent=ext, colors=[C['bad']], linewidths=0.6)
    t, gt, od, am = run_arrays()
    p0 = np.array(RUN['plans'][0]['pts'])
    ax.plot(p0[:, 0], p0[:, 1], color=C['plan'], lw=1.6, label='first plan from planner_server')
    ax.plot(gt[:, 0], gt[:, 1], color=C['gt'], lw=1.0, ls='--', label='path walked (ground truth)')
    for k in ('A', 'B'):
        x, y = RUN['arena'][k]; ax.plot(x, y, 'o', mfc='white', mec=C['ink'], ms=7); ax.text(x + 0.25, y + 0.2, k, weight='semibold')
    a, b = np.array([1.4, -1.2]), np.array([1.9, -1.9])                         # box corners: (1.0, 0.0, 0.8, 2.4) and (2.5, -2.5, 1.2, 1.2)
    ax.plot(*np.c_[a, b], color=C['est'], lw=1.4); ax.annotate(f'{np.linalg.norm(b - a):.2f} m', (b + a) / 2, xytext=(8, 2),
                                                               textcoords='offset points', color=C['est'], fontsize=8)
    ax.plot([], [], color=C['bad'], lw=0.8, label='cost 253: centre may not cross')
    ax.set_xlim(-6.4, 6.4); ax.set_ylim(-5.4, 5.4); ax.set_aspect('equal'); ax.grid(False)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='upper left', fontsize=7.4, framealpha=0.9, frameon=True)
    ax.set_title('Global costmap rebuilt from arena.pgm with the Nav2 parameters')
    save(f, 'ch10-costmap')


@fig
def ch10_footprint():
    rgba, ext = go2_render(go2_pose(0.27), 90, -90, (0.0, 0, 0.2), 0.9, size=(1200, 1200))
    f, ax = plt.subplots(figsize=(6.2, 5.2))
    ax.imshow(rgba, extent=ext, zorder=1)
    ax.add_patch(Rectangle((-0.325, -0.15), 0.65, 0.30, fill=False, ec=C['ink'], lw=1.2, zorder=3))
    ax.add_patch(Circle((0, 0), 0.358, fill=False, ec=C['gt'], lw=1.1, ls='--', zorder=3))
    ax.add_patch(Circle((0, 0), 0.33, fill=False, ec=C['plan'], lw=1.6, zorder=3))
    ax.text(0.0, 0.17, '0.65 × 0.30 m body', ha='center', va='bottom', fontsize=8, zorder=4)
    ax.annotate('circumscribed circle, r = 0.358 m', (0.358 * math.cos(0.6), 0.358 * math.sin(0.6)), xytext=(30, 40),
                textcoords='offset points', fontsize=8, color=C['gt'], arrowprops=dict(arrowstyle='-', color=C['gt'], lw=0.6))
    ax.annotate('Nav2 footprint: robot_radius 0.33 m', (0.33 * math.cos(-0.7), 0.33 * math.sin(-0.7)), xytext=(25, -40),
                textcoords='offset points', fontsize=8, color=C['plan'], arrowprops=dict(arrowstyle='-', color=C['plan'], lw=0.6))
    fs.frame_axes(ax, (0, 0), length=0.12, names=('x', 'y'))
    ax.set_xlim(-0.48, 0.75); ax.set_ylim(-0.5, 0.5); ax.set_aspect('equal'); ax.grid(False)
    ax.set_xlabel('x [m] (forward)'); ax.set_ylabel('y [m]')
    save(f, 'ch10-footprint')


@fig
def ch10_initialpose():
    f, ax = plt.subplots(figsize=(9.0, 4.0)); ax.set_xlim(0, 9); ax.set_ylim(0, 4.3); ax.axis('off')
    lanes = {'go_to_goal.py': 1.2, 'amcl': 4.5, 'TF buffer': 7.6}
    for n, x in lanes.items():
        node(ax, x, 4.0, 1.7, 0.42, n, fs_title=8.2)
        ax.plot([x, x], [0.15, 3.78], color=C['grid'], lw=1.0, zorder=0)
    def msg(y, a, b, txt, color=C['ink'], ls='-'):
        link(ax, (lanes[a], y), (lanes[b], y), color=color, ls=ls)
        ax.text((lanes[a] + lanes[b]) / 2, y + 0.06, txt, ha='center', va='bottom', fontsize=7.4, color=color)
    msg(3.35, 'go_to_goal.py', 'amcl', '/initialpose, stamp = sim time now', C['bad'])
    msg(2.95, 'amcl', 'TF buffer', 'lookup at that stamp', C['bad'])
    ax.text(lanes['TF buffer'] + 0.1, 2.75, 'newest transform is a few ms older', fontsize=7.2, color=C['bad'], ha='center', va='top')
    ax.text(lanes['amcl'], 2.55, '"Failed to transform initial pose in time":\npose dropped, map → odom stays wrong', fontsize=7.4,
            color=C['bad'], ha='center', va='top', bbox=dict(fc='white', ec=C['bad'], lw=0.6, boxstyle='round,pad=0.25'))
    msg(1.6, 'go_to_goal.py', 'amcl', '/initialpose, stamp = 0', C['good'])
    msg(1.25, 'amcl', 'TF buffer', 'stamp 0 = use the latest', C['good'])
    msg(0.8, 'amcl', 'go_to_goal.py', '/amcl_pose', C['est2'])
    ax.text(lanes['go_to_goal.py'], 0.55, 'repeat until /amcl_pose\nagrees with the true pose', fontsize=7.4, color=C['good'],
            ha='center', va='top')
    ax.text(0.05, 3.15, 'first try', fontsize=7.6, weight='semibold', color=C['bad'], rotation=90, va='center')
    ax.text(0.05, 1.2, 'fix', fontsize=7.6, weight='semibold', color=C['good'], rotation=90, va='center')
    ax.plot([0.2, 8.9], [2.0, 2.0], color=C['grid'], lw=0.8, ls='--')
    save(f, 'ch10-initialpose')


@fig
def ch10_slam_map():
    meta = json.load(open(ROOT / 'site/data/runs/w2/slam_map.json'))
    img = plt.imread(ROOT / 'site/data/runs/w2/slam_map.png')
    ox, oy = meta['origin']; r = meta['resolution']
    ext = (ox, ox + meta['width'] * r, oy, oy + meta['height'] * r)
    f, ax = plt.subplots(figsize=(7.0, 5.6))
    arena_outline(ax, fill='none', color=C['bad'], lw=0.9)
    ax.imshow(img, extent=ext, origin='upper', interpolation='nearest', zorder=0)
    t, gt, od, am = run_arrays()
    ax.plot(gt[:, 0], gt[:, 1], color=C['gt'], lw=1.0, ls='--')
    for k in ('A', 'B'):
        x, y = RUN['arena'][k]; ax.plot(x, y, 'o', mfc='white', mec=C['ink'], ms=6); ax.text(x + 0.25, y + 0.2, k, weight='semibold')
    ax.plot([], [], color=C['bad'], lw=0.9, label='true obstacle outlines (run.json)')
    ax.plot([], [], color=C['gt'], ls='--', label='path walked')
    ax.set_xlim(-6.5, 6.5); ax.set_ylim(-5.5, 5.5); ax.set_aspect('equal'); ax.grid(False)
    ax.legend(loc='upper left', fontsize=7.4, frameon=True, framealpha=0.9); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('/slam/map at the end of run w2, placed in the world frame')
    print('rigid fit to truth: %.3f m, %.2f deg' % slam_map_fit())
    save(f, 'ch10-slam-map')



def slam_map_fit():
    """Rigid SE(2) fit of the occupied /slam/map cells to the true outlines with registration.register (full solve)."""
    from registration import register, voxel_downsample
    tr = test_geometry()
    meta = json.load(open(ROOT / 'site/data/runs/w2/slam_map.json')); img = plt.imread(ROOT / 'site/data/runs/w2/slam_map.png')
    occ = (img[:, :, 3] > 0.5) & (img[:, :, 2] > 0.7) & (img[:, :, 0] < 0.3)       # occupied cells are opaque blue in the export
    rows, cols = np.nonzero(occ); r = meta['resolution']; ox, oy = meta['origin']
    pts = voxel_downsample(np.c_[ox + (cols + 0.5) * r, oy + (img.shape[0] - rows - 0.5) * r], 0.1)
    x = register(pts, tr.LMAP, np.zeros(3), fix_yaw=False, trans_prior=0.0, iters=50)[0]
    return float(np.hypot(x[0], x[1])), float(math.degrees(x[2]))


# ==== chapter 11: record and visualise =========================================================================================
@fig
def ch11_pipeline():
    f, ax = canvas(10.4, 4.3)
    node(ax, 1.25, 3.3, 2.1, 1.3, 'scripts/run_mission.sh', '1  up.sh: fresh stack\n2  record_bag.sh in the background\n3  go_to_goal.py: A to B\n4  stop the recording')
    node(ax, 4.15, 3.3, 2.0, 1.3, 'bags/run1', 'MCAP, zstd chunks\nabout 8 MB/s\n18 topics, sim-time stamps', fill='#fffaf0', ec=C['plan'])
    node(ax, 7.0, 3.95, 2.3, 0.62, 'bag_metrics.py', None)
    node(ax, 7.0, 2.65, 2.3, 0.62, 'render_dashboard.py', None)
    node(ax, 7.0, 1.0, 2.3, 0.62, 'export_run.py', None)
    node(ax, 9.45, 3.95, 1.7, 0.62, 'results/run1.json', None, fs_title=7.6)
    node(ax, 9.45, 2.65, 1.7, 0.62, 'videos/run1.mp4\n+ still .png', None, fs_title=7.6)
    node(ax, 9.45, 1.0, 1.7, 0.62, 'site/data/runs/', None, fs_title=7.6)
    link(ax, (2.3, 3.3), (3.15, 3.3))
    for y in (3.95, 2.65):
        link(ax, (5.15, 3.3), (5.85, y))
    ax.plot([4.15, 4.15], [2.65, 1.0], color=C['ink'], lw=1.0); link(ax, (4.15, 1.0), (5.85, 1.0))
    for y in (3.95, 2.65, 1.0):
        link(ax, (8.15, y), (8.6, y))
    ax.text(7.0, 3.55, 'ATE, RPE, AMCL error, time to goal;\nGT interpolated at each estimate\'s stamp', ha='center', va='top',
            fontsize=7.0, color=C['muted'])
    ax.text(7.0, 2.25, '9 panels, --cards, --still;\n76 s run: about 3 min to render', ha='center', va='top', fontsize=7.0, color=C['muted'])
    ax.text(7.0, 0.6, 'run.json (10 Hz), imu.json, slam_map,\nsnapshots for the website', ha='center', va='top', fontsize=7.0,
            color=C['muted'])
    ax.text(2.3, 2.0, 'the simulator is not needed\nfrom here on: re-render and\nre-analyse any time', ha='center', va='top',
            fontsize=7.2, color=C['plan'])
    save(f, 'ch11-pipeline')


@fig
def ch11_dashboard():
    from PIL import Image
    im = Image.open(ROOT / 'docs/img/dashboard.png').convert('RGB')
    W0, H0 = im.size
    panels = [((24, 84), '1', '/spectator/cam/image_raw'), ((653, 84), '2', '/camera/image_raw'),
              ((1282, 84), '3', '/camera/depth/image_raw'), ((24, 476), '4', '/lidar/points'),
              ((653, 768), '5', '/slam/map  /plan  /scan  /odom_gt'), ((1282, 768), '6', '/odom  /amcl_pose  /odom_gt'),
              ((24, 868), '7', '/imu/data'), ((653, 868), '7', '/imu/data'), ((1282, 868), '7', '/imu/data')]
    f = plt.figure(figsize=(9.6, 9.6 * H0 / W0)); ax = f.add_axes((0, 0, 1, 1)); ax.imshow(im); ax.axis('off')
    for (x, y), n, topic in panels:
        ax.text(x + 8, y + 52, f' {n} ', fontsize=10, weight='bold', color='#1f2328', va='top',
                bbox=dict(fc='#ffd27a', ec='none', boxstyle='round,pad=0.25'))
        ax.text(x + 70, y + 52, topic, fontsize=7.6, color='#1f2328', va='top', family=MONO,
                bbox=dict(fc='white', ec='none', alpha=0.88, boxstyle='round,pad=0.25'))
    p = OUT / 'c09-ch11-dashboard.png'
    f.savefig(p, dpi=125); plt.close(f)
    Image.open(p).convert('RGB').quantize(colors=200, method=Image.Quantize.MEDIANCUT).save(p, optimize=True)
    print(f'{p.name}: {p.stat().st_size // 1024} KB')


@fig
def ch11_runs():
    runs = {k: json.load(open(ROOT / f'results/run_{k}.json')) for k in ('w1', 'w2', 'w3')}
    f, axs = plt.subplots(1, 3, figsize=(10.0, 3.3), gridspec_kw=dict(width_ratios=[1.1, 1.1, 0.9]))
    x = np.arange(3); names = list(runs)
    def bars(ax, keys, labels, cols, title, unit):
        w = 0.8 / len(keys)
        for i, (k, lab, c) in enumerate(zip(keys, labels, cols)):
            v = [runs[n][k] for n in names]
            ax.bar(x + (i - (len(keys) - 1) / 2) * w, v, w * 0.92, color=c, label=lab)
            for xi, vi in zip(x + (i - (len(keys) - 1) / 2) * w, v):
                ax.text(xi, vi, f'{vi:g}', ha='center', va='bottom', fontsize=6.8)
        ax.set_xticks(x, [f'run {n}' for n in names]); ax.set_title(title); ax.set_ylabel(unit); ax.legend(fontsize=7, loc='upper left')
    bars(axs[0], ['odom_ate_aligned_cm', 'odom_ate_anchored_cm', 'odom_final_err_cm'], ['ATE, SE(2)-aligned', 'ATE, anchored', 'final error'],
         [C['est'], '#5aa3c9', '#a9cfe3'], 'Odometry', 'cm')
    bars(axs[1], ['amcl_rmse_cm', 'amcl_max_cm'], ['RMSE at updates', 'maximum'], [C['est2'], '#b79cf0'], 'AMCL', 'cm')
    axs[1].set_ylim(0, 32); axs[0].set_ylim(0, 4.2)
    v = [runs[n]['goal_error_m'] for n in names]
    axs[2].bar(x, v, 0.55, color=[C['good'] if e <= 0.25 + 1e-9 else C['bad'] for e in v])
    for xi, vi in zip(x, v):
        axs[2].text(xi, vi, f'{vi:.2f}', ha='center', va='bottom', fontsize=7)
    axs[2].axhline(0.25, color=C['ink'], ls='--', lw=0.8)
    axs[2].text(-0.35, 0.40, 'dashed: xy_goal_tolerance 0.25 m', ha='left', va='bottom', fontsize=7)
    axs[2].set_xticks(x, [f'run {n}' for n in names]); axs[2].set_title('Distance to B at the end'); axs[2].set_ylabel('m')
    axs[2].set_ylim(0, 0.42)
    save(f, 'ch11-runs')


@fig
def ch11_pairing():
    t, gt, od, am = run_arrays()
    upd = np.r_[True, np.any(np.diff(am, axis=0) != 0, axis=1)]
    e_held = np.hypot(*(am[:, :2] - gt[:, :2]).T) * 100
    rms = lambda e: float(np.sqrt(np.mean(e ** 2)))
    w = (t >= 44) & (t <= 50)
    f, (ax, ax2) = plt.subplots(2, 1, figsize=(8.6, 4.6), sharex=True, gridspec_kw=dict(height_ratios=[1.2, 1]))
    ax.plot(t[w], gt[w, 0], color=C['gt'], lw=2.4, alpha=0.7, label='ground truth x')
    ax.step(t[w], am[w, 0], where='post', color=C['est2'], lw=0.9, ls='--', label='last AMCL pose, held')
    ax.plot(t[w & upd], am[w & upd, 0], 'o', color=C['est2'], ms=4, label='AMCL update')
    ax.set_ylabel('x [m]'); ax.legend(loc='upper left', fontsize=7.4, ncol=3); ax.set_title('Run w2, six seconds of the timeline')
    ax2.plot(t[w], e_held[w], color=C['est2'], lw=0.9, ls='--', label='held: compared at every sample')
    ax2.plot(t[w & upd], e_held[w & upd], 'o', color=C['est2'], ms=4, label='compared only at updates')
    ax2.set_xlabel('time since the recording started [s] (wall clock)'); ax2.set_ylabel('AMCL error [cm]'); ax2.legend(loc='upper left', fontsize=7.4, ncol=2)
    ax2.set_ylim(0, None)
    save(f, 'ch11-pairing')
    return rms(e_held), rms(e_held[upd])


# ==== chapter 12: troubleshooting ==============================================================================================
@fig
def ch12_decision_tree():
    tree = [
        ('Topics or sensors missing', [
            ('no /scan, /imu/data or camera topics,\nno error', 'install ros-humble-gazebo-plugins\n(or setup.sh unpacks it to overlay/)')]),
        ('Robot does not move', [
            ('stands limp, ignores /cmd_vel', 'patch spawns controllers with the\ncontroller_manager spawner'),
            ('keeps walking after you stop', 'publish a zero Twist (CHAMP holds\nthe last cmd_vel)')]),
        ('Build or launch fails', [
            ("colcon: No module named 'em'", 'source env.sh, rebuild with\n--cmake-clean-cache'),
            ('gzserver exits with 255', 'stop.sh; wait until port 11345 is free'),
            ("launch: name 'true' is not defined", 'rename your launch argument (slam)'),
            ('lifecycle_manager: failed to bring up', 'Cyclone DDS (env.sh); up.sh retries')]),
        ('Planner finds no path', [
            ('goal is off the global costmap', 'navigate on a complete map'),
            ('blob in the middle of the room', 'min_height 0.30 (floor returns)'),
            ('long diagonal streaks in the costmap', '/initialpose with zero stamp, repeat'),
            ('robot sits inside the inflated zone', 'widen the gap or tune inflation')]),
        ('Waits forever', [
            ('waitUntilNav2Active never returns', 'publish an initial pose (go_to_goal.py)'),
            ('spin_once loop never sees TF', 'drain in a loop or TF listener thread')]),
        ('Slow', [
            ('camera 5–6 Hz in a Python script', 'measure with ros2 bag record / info'),
            ('Gazebo cameras about 5 fps', 'NVIDIA PRIME offload (env.sh)')]),
        ('Processes die', [
            ('pkill -f kills your terminal (144)', 'stop.sh skips its own ancestors'),
            ('stopping one stack kills another', 'own ROS_DOMAIN_ID and Gazebo port')]),
        ('Wrong numbers', [
            ('base_footprint→base_link constant 0.426', 'open: publish it from IMU tilt\nand a kinematic height')]),
    ]
    n_leaf = sum(len(b) for _, b in tree) + 0.6 * len(tree)
    Hh = n_leaf * 0.36 + 0.7
    f, ax = canvas(11.0, Hh)
    root = (0.75, Hh / 2 + 0.15)
    node(ax, *root, 1.3, 0.9, 'What do\nyou see?', fs_title=9)
    y = Hh - 0.35
    for g, (group, leaves) in enumerate(tree):
        ys = []
        for sym, fix in leaves:
            ys.append(y); y -= 0.36 * (1.35 if '\n' in sym or '\n' in fix else 1.0)
        gy = (ys[0] + ys[-1]) / 2
        col = [C['est'], C['gt'], C['muted'], C['plan'], C['est2'], C['good'], C['bad'], C['bad']][g]
        ax.plot([root[0] + 0.65, 1.75, 1.75, 2.0], [root[1], root[1], gy, gy], color=C['grid'], lw=1.0)
        node(ax, 2.95, gy, 1.9, 0.34, group, ec=col, fs_title=7.8)
        for yy, (sym, fix) in zip(ys, leaves):
            ax.plot([3.9, 4.1, 4.1, 4.25], [gy, gy, yy, yy], color=C['grid'], lw=0.9)
            ax.text(4.3, yy, sym, va='center', fontsize=7.2, color=C['ink'], linespacing=1.1)
            link(ax, (7.05, yy), (7.35, yy), color=col, lw=0.9)
            ax.text(7.42, yy, fix, va='center', fontsize=7.2, color=col if col not in (C['gt'], C['muted']) else C['ink'], linespacing=1.1)
        y -= 0.36 * 0.6
    ax.text(4.3, Hh - 0.05, 'symptom', fontsize=7.6, weight='semibold', color=C['muted'], va='top')
    ax.text(7.42, Hh - 0.05, 'fix (details in the table)', fontsize=7.6, weight='semibold', color=C['muted'], va='top')
    save(f, 'ch12-decision-tree')


def _go2_sim(limp_after, T, fps=0, cb=None):
    """Go2 on the floor, PD stand at the 'home' keyframe; torques drop to zero after `limp_after` s. cb(m, d) per video frame."""
    import mujoco
    from common import require_model
    m = mujoco.MjModel.from_xml_path(require_model(True)); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0); q0 = m.key_qpos[0][7:].copy()
    kp, kd = 60.0, 2.0; n = int(T / m.opt.timestep); every = int(round(1 / (fps * m.opt.timestep))) if fps else 0
    for i in range(n):
        tau = kp * (q0 - d.qpos[7:]) - kd * d.qvel[6:]
        d.ctrl[:] = 0.0 if d.time >= limp_after else np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
        mujoco.mj_step(m, d)
        if every and i % every == 0 and cb:
            cb(m, d)
    return m, d


@fig
def ch12_limp():
    _, d_stand = _go2_sim(limp_after=99, T=1.5)
    _, d_limp = _go2_sim(limp_after=0.0, T=1.5)
    f, axs = plt.subplots(1, 2, figsize=(8.8, 3.2))
    for ax, d, title in [(axs[0], d_stand, 'controller running'), (axs[1], d_limp, 'no joint torques')]:
        x0 = float(d.qpos[0])
        rgba, ext = go2_render(d.qpos.copy(), 90, 0, (x0, 0, 0.2), 0.5, size=(1100, 550))
        ax.axhspan(-0.05, 0, color='#eceef1', lw=0); ax.axhline(0, color=C['ink'], lw=0.8)
        ax.imshow(rgba, extent=ext); ax.set_xlim(x0 - 0.45, x0 + 0.45); ax.set_ylim(-0.04, 0.42); ax.set_aspect('equal'); ax.grid(False)
        ax.set_title(f'{title}: trunk at {d.qpos[2]:.3f} m'); ax.set_xticks([]); ax.set_ylabel('height [m]')
    save(f, 'ch12-limp')
    return float(d_stand.qpos[2]), float(d_limp.qpos[2])


if __name__ == '__main__':
    for name in sys.argv[1:] or list(FIGS):
        print(name, FIGS[name]() or '')


def video_limp(path=ROOT / 'media/c09-ch12-limp.mp4', T=5.0, fps=25, limp_after=2.0, size=(960, 540)):
    """MuJoCo clip for chapter 12: the Go2 stands under a PD controller, then all joint torques go to zero at `limp_after` s."""
    import subprocess
    import mujoco
    W, H = size; frames = []; r = cam = None
    def grab(m, d):
        nonlocal r, cam
        if r is None or cam is None:
            m.vis.global_.offwidth, m.vis.global_.offheight = W, H
            r = mujoco.Renderer(m, H, W); cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.distance = 1.5; cam.azimuth = 135; cam.elevation = -18
        cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.15]
        r.update_scene(d, camera=cam); frames.append(r.render().copy())
    _go2_sim(limp_after, T, fps, grab)
    ff = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(fps), '-i', '-',
                           '-c:v', 'libx264', '-preset', 'slow', '-crf', '26', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)],
                          stdin=subprocess.PIPE)
    ff.communicate(b''.join(f.tobytes() for f in frames))
    print(f'{path.name}: {len(frames)} frames, {path.stat().st_size // 1024} KB')
