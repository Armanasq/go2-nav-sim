"""Figures and clips for tutorial chapters 0-3 (docs/img/c00-*.png, media/c00-*.mp4). Every number drawn comes from a file in
this repository; the source is named next to it. Run from the repository root:

    MUJOCO_GL=glfw .venv-modules/bin/python docs/img/c00-make_figures.py            # figures
    MUJOCO_GL=glfw .venv-modules/bin/python docs/img/c00-make_figures.py --videos   # also the clips (ffmpeg)
"""
import json
import pathlib
import subprocess
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'modules'), str(ROOT / 'modules/m04_kinematics')]
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle, Wedge  # noqa: E402

C, LEGS = fs.C, fs.LEGS
IMG, MEDIA = ROOT / 'docs/img', ROOT / 'media'
MONO = dict(family='DejaVu Sans Mono', fontsize=7.6)
# Fills that say where a block comes from (legend in each figure that uses them).
ORIGIN = dict(sim=dict(fill='#eceff3', color=C['gt']), upstream=dict(fill='#ffffff', color=C['ink']),
              ours=dict(fill='#e3f0f7', color=C['est']), eval=dict(fill='#ffffff', color=C['gt']))


def save(fig, name, dpi=150):
    p = IMG / f'c00-{name}.png'
    fig.savefig(p, dpi=dpi, bbox_inches='tight', pad_inches=0.06)
    plt.close(fig)
    kb = p.stat().st_size / 1024
    assert kb < 400, f'{p.name} is {kb:.0f} KB'
    print(f'{p.relative_to(ROOT)}  {kb:.0f} KB')


def box(ax, x, y, w, h, text, kind='upstream', dashed=False, fs_=8.2, **kw):
    s = ORIGIN[kind]
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0,rounding_size=0.015', fc=s['fill'], ec=s['color'],
                                lw=0.9, ls='--' if dashed else '-'))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fs_, color=C['ink'], linespacing=1.25, **kw)


def topic(ax, x, y, text, color=None):
    ax.text(x, y, text, ha='center', va='center', color=color or C['ink'], **MONO,
            bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=C['grid'], lw=0.6))


def line(ax, pts, color=None, lw=1.0, ls='-', arrow=True):
    color = color or C['ink']
    pts = np.asarray(pts, float)
    ax.plot(pts[:-1, 0], pts[:-1, 1], color=color, lw=lw, ls=ls, solid_capstyle='butt')
    if arrow:
        ax.annotate('', xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle='-|>', color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0, mutation_scale=9))
    else:
        ax.plot(pts[-2:, 0], pts[-2:, 1], color=color, lw=lw, ls=ls)


def legend_origin(ax, x, y, kinds, dx=0.2):
    names = dict(sim='simulated (Gazebo)', upstream='upstream, unmodified', ours='written for this project',
                 patched='upstream, patched', eval='evaluation only')
    for i, k in enumerate(kinds):
        s = ORIGIN['upstream' if k == 'patched' else k]
        ax.add_patch(FancyBboxPatch((x + i * dx, y), 0.025, 0.018, boxstyle='round,pad=0,rounding_size=0.004',
                                    fc=s['fill'], ec=s['color'], lw=0.9, ls='--' if k in ('patched', 'eval') else '-',
                                    transform=ax.transAxes, clip_on=False))
        ax.text(x + i * dx + 0.033, y + 0.009, names[k], transform=ax.transAxes, fontsize=7.6, va='center', color=C['muted'])


# ----------------------------------------------------------------------------------------------------------------- MuJoCo
def mj_setup(w=1280, h=800):
    import mujoco
    from common import require_model
    m = mujoco.MjModel.from_xml_path(require_model(scene=True))
    m.vis.global_.offwidth, m.vis.global_.offheight = w, h
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return mujoco, m, d


def free_cam(mujoco, lookat, dist, az, el):
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, dist, az, el
    return cam


def projector(m, cam, w, h):
    """World point -> pixel for MuJoCo's free camera (pinhole with fovy = model.vis.global_.fovy)."""
    az, el = np.radians(cam.azimuth), np.radians(cam.elevation)
    f = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    up = np.array([-np.sin(el) * np.cos(az), -np.sin(el) * np.sin(az), np.cos(el)])
    right = np.cross(f, up)
    pos = np.array(cam.lookat) - cam.distance * f
    fy = (h / 2) / np.tan(np.radians(m.vis.global_.fovy) / 2)

    def proj(p):
        v = np.asarray(p) - pos
        return w / 2 + fy * v @ right / (v @ f), h / 2 - fy * v @ up / (v @ f)
    return proj


def colour_legs(mujoco, m, pairs=False):
    """Recolour each leg's visual geoms (fixed per-leg colours, or one colour per trot pair)."""
    for g in range(m.ngeom):
        b = m.body(m.geom_bodyid[g]).name
        if b[:2] in LEGS and m.geom_contype[g] == 0:
            col = (C['est'] if b[:2] in ('FL', 'RR') else C['plan']) if pairs else LEGS[b[:2]]
            m.geom_matid[g] = -1
            m.geom_rgba[g] = [*(int(col[i:i + 2], 16) / 255 for i in (1, 3, 5)), 1]


def fig_go2_render():
    mujoco, m, d = mj_setup(960, 720)
    r = mujoco.Renderer(m, 720, 960)
    base = d.xpos[1].copy()
    shots = []
    for az, el, dist in ((135, -18, 1.15), (90, -2, 1.05)):
        r.update_scene(d, free_cam(mujoco, base + [0, 0, -0.08], dist, az, el))
        shots.append(r.render())
    fig, axs = plt.subplots(1, 2, figsize=(7.6, 2.9))
    for ax, im, t in zip(axs, shots, ('three-quarter view', 'side view, nose to the right')):
        ax.imshow(im[60:-60]); ax.axis('off'); ax.set_title(t, fontsize=8.5, color=C['muted'], fontweight='normal')
    save(fig, '00-go2-render')


def fig_go2_joints():
    mujoco, m, d = mj_setup(1280, 860)
    colour_legs(mujoco, m)
    W, H = 1280, 860
    r = mujoco.Renderer(m, H, W)
    cam = free_cam(mujoco, d.xpos[1] + [0.02, 0, -0.1], 1.0, 238, -16)
    r.update_scene(d, cam)
    im = r.render()
    P = projector(m, cam, W, H)
    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    ax.imshow(im); ax.axis('off')
    bb = dict(boxstyle='round,pad=0.25', fc='white', ec='none', alpha=0.85)

    def tag(p3, text, xy, color=C['ink'], **kw):
        ax.annotate(text, P(p3), xytext=xy, textcoords='data', color=color, fontsize=8.3, va='center', bbox=bb,
                    arrowprops=dict(arrowstyle='-', color=color, lw=0.7, shrinkA=0, shrinkB=2), **kw)
    urdf = dict(FL='lf', FR='rf', RL='lh', RR='rh')
    where = dict(FL=(330, 800), FR=(190, 690), RL=(1010, 640), RR=(1080, 760))
    for leg in LEGS:
        foot = d.geom_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, leg)]
        tag(foot, f'{leg} foot  ({urdf[leg]} in the URDF)', where[leg], LEGS[leg], fontweight='semibold', ha='center')
    # joint axes of the FL leg, drawn from the model (jnt_axis in the body frame, rotated to world)
    names = dict(hip='hip abduction, axis x', thigh='hip flexion (thigh), axis y', calf='knee (calf), axis y')
    spots = dict(hip=(90, 130), thigh=(90, 215), calf=(90, 300))
    for part in ('hip', 'thigh', 'calf'):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'FL_{part}_joint')
        b = m.jnt_bodyid[j]
        o = d.xpos[b]; axw = d.xmat[b].reshape(3, 3) @ m.jnt_axis[j]
        a0, a1 = P(o - 0.07 * axw), P(o + 0.07 * axw)
        ax.annotate('', xy=a1, xytext=a0, arrowprops=dict(arrowstyle='-|>', color=C['ink'], lw=1.8, mutation_scale=12))
        ax.plot(*P(o), 'o', color='white', mec=C['ink'], ms=5)
        tag(o, names[part], spots[part], ha='left')
    # base frame glyph on top of the trunk
    o = d.xpos[1] + [0, 0, 0.075]
    for k, (n, col) in enumerate((('x', C['bad']), ('y', C['good']), ('z', C['est']))):
        e = np.zeros(3); e[k] = 0.11
        a, b = P(o), P(o + e)
        ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='-|>', color=col, lw=1.8, mutation_scale=12))
        ax.text(b[0] + 6, b[1] - 6, n, color=col, fontsize=9.5, fontweight='semibold', bbox=bb)
    tag(o, 'base frame: x forward, y left, z up', (820, 90), ha='left')
    tag(d.site_xpos[0], 'IMU site', (870, 330), C['muted'], ha='left')
    ax.set_xlim(40, W - 20); ax.set_ylim(H - 10, 40)
    save(fig, '01-go2-joints', dpi=120)


def video_trot(seconds=8.0, fps=25, slow=4.0):
    """Kinematic playback of CHAMP's trot on the MuJoCo Go2: base moved at v, feet from CHAMP's Bezier swing and cosine stance
    (trajectory_planner.h, go2 gait.yaml), joints from the closed-form IK in modules/m04_kinematics. No dynamics."""
    from kinematics import Leg
    mujoco, m, d = mj_setup(1280, 720)
    colour_legs(mujoco, m, pairs=True)
    W, H = 1280, 720
    r = mujoco.Renderer(m, H, W)
    legs = {k: Leg(m, k) for k in LEGS}
    v, Ts, h0 = 0.2, 0.25, 0.225
    L = Ts * v                                            # stroke = stance duration x velocity (leg_controller.h)
    r_foot = m.geom_size[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')][0]
    offset = dict(FL=0.0, RR=0.0, FR=0.5, RL=0.5)         # trot: diagonal pairs half a stride apart
    tmp = MEDIA / 'c00-tmp-frames'; tmp.mkdir(exist_ok=True)
    n = int(seconds * fps)
    for i in range(n):
        t = i / fps / slow
        d.qpos[:3] = [v * t, 0, h0 + r_foot]
        d.qpos[3:7] = [1, 0, 0, 0]
        for k, leg in legs.items():
            ph = (t / (2 * Ts) + offset[k]) % 1.0
            x, y = foot_xy(ph, L)
            q = leg.ik_closed_form(leg.hip + [x, leg.d, -h0 + y])[0]
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{k}_hip_joint')
            d.qpos[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 3] = q
        mujoco.mj_kinematics(m, d)
        r.update_scene(d, free_cam(mujoco, [v * t, 0, 0.18], 1.15, 120, -14))
        from PIL import Image
        Image.fromarray(r.render()).save(tmp / f'{i:04d}.png')
    out = MEDIA / 'c00-01-trot-kinematic.mp4'
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-framerate', str(fps), '-i', str(tmp / '%04d.png'),
                    '-vf', 'scale=960:540', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '26', str(out)], check=True)
    for p in tmp.iterdir():
        p.unlink()
    tmp.rmdir()
    print(out.relative_to(ROOT), f'{out.stat().st_size / 1024:.0f} KB')


# ----------------------------------------------------------------------------------------------------------------- gait
SW_H, ST_D, T_ST = 0.04, 0.01, 0.25                    # go2_config gait.yaml: swing_height, stance_depth, stance_duration
REF_X = [-0.15, -0.2805, -0.3, -0.3, -0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15]   # trajectory_planner.h
REF_Y = [-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5]


def bezier_swing(s, L):
    from math import comb
    hr, lr = SW_H / 0.15, L / 0.4
    cx = [-L / 2] + [x * lr for x in REF_X[1:-1]] + [L / 2]
    cy = [-(y * hr + 0.5 * hr) for y in REF_Y]
    B = np.array([comb(11, i) * s ** i * (1 - s) ** (11 - i) for i in range(12)])
    return float(B @ cx), float(-(B @ cy))


def foot_xy(phase, L):
    """Foot offset from its nominal point, phase in [0, 1): first half stance, second half swing (equal durations)."""
    if phase < 0.5:
        s = phase / 0.5
        x = (L / 2) * (1 - 2 * s)
        return x, -ST_D * np.cos(np.pi * x / L)
    return bezier_swing((phase - 0.5) / 0.5, L)


def fig_trot():
    fig, ax = plt.subplots(figsize=(7.2, 3.0))
    rows = [('lf', 'FL', 0.0), ('rh', 'RR', 0.0), ('rf', 'FR', 0.25), ('lh', 'RL', 0.25)]
    T = 1.0
    for i, (name, k, off) in enumerate(rows):
        yv = 3 - i
        col = C['est'] if k in ('FL', 'RR') else C['plan']
        t0 = -off
        while t0 < T:
            ax.add_patch(Rectangle((max(t0, 0), yv - 0.32), min(t0 + T_ST, T) - max(t0, 0), 0.64, fc=col, ec='none', alpha=0.85))
            t0 += 2 * T_ST
        ax.text(-0.02, yv, name, ha='right', va='center', fontsize=9, fontweight='semibold', color=col, family='DejaVu Sans Mono')
    for t in np.arange(0, T + 1e-9, 0.25):
        ax.axvline(t, color=C['grid'], lw=0.8, zorder=0)
    tt = np.linspace(0, T, 400)
    ax.plot(tt, -0.75 + 0.3 * np.cos(2 * np.pi * 4.0 * tt), color=C['gt'], lw=1.2)
    ax.text(-0.02, -0.75, 'trunk z', ha='right', va='center', fontsize=8, color=C['gt'])
    ax.text(T, -1.25, 'trunk height: schematic, two cycles per stride = 4.0 Hz (measured, chapter 7)', ha='right', fontsize=7.6,
            color=C['gt'])
    ax.set_xlim(-0.08, T); ax.set_ylim(-1.4, 4.3)
    ax.set_yticks([]); ax.spines['left'].set_visible(False); ax.grid(False)
    ax.set_xlabel('time [s]')
    ax.annotate('', xy=(0.5, 3.95), xytext=(0.0, 3.95), arrowprops=dict(arrowstyle='<|-|>', color=C['ink'], lw=0.8, mutation_scale=8))
    ax.text(0.25, 4.1, 'one stride = 0.25 s stance + 0.25 s swing', ha='center', fontsize=8.2)
    ax.text(T, 4.1, 'filled = stance (foot on the ground), gap = swing', ha='right', fontsize=7.6, color=C['muted'])
    save(fig, '01-trot-gait')


def fig_foot_trajectory():
    v = 0.2
    L = T_ST * v
    s = np.linspace(0, 1, 200)
    sw = np.array([bezier_swing(u, L) for u in s])
    xs = np.linspace(L / 2, -L / 2, 50)
    st = -ST_D * np.cos(np.pi * xs / L)
    fig, ax = plt.subplots(figsize=(6.8, 3.4))
    ax.axhline(0, color=C['gt'], lw=0.8)
    ax.text(-5.9, -0.15, 'nominal foot\npoint, 0.225 m\nbelow the hip', fontsize=7.4, color=C['muted'], ha='left', va='top')
    ax.plot(sw[:, 0] * 100, sw[:, 1] * 100, color=C['est'], label='swing: 12-point Bezier, scaled to swing_height')
    ax.plot(xs * 100, st * 100, color=C['plan'], label='stance: pressed down by stance_depth')
    for frac in (0.25, 0.5, 0.75):
        i = int(frac * 199)
        ax.annotate('', xy=sw[i + 3] * 100, xytext=sw[i] * 100, arrowprops=dict(arrowstyle='-|>', color=C['est'], lw=1, mutation_scale=9))
    ax.annotate('', xy=(xs[30] * 100, st[30] * 100), xytext=(xs[24] * 100, st[24] * 100),
                arrowprops=dict(arrowstyle='-|>', color=C['plan'], lw=1, mutation_scale=9))
    ax.annotate('', xy=(L / 2 * 100, -1.6), xytext=(-L / 2 * 100, -1.6), arrowprops=dict(arrowstyle='<|-|>', color=C['ink'], lw=0.7, mutation_scale=7))
    ax.text(0, -1.9, f'stroke = stance_duration x v = {T_ST} s x {v} m/s = {L * 100:.0f} cm', ha='center', va='top', fontsize=7.6)
    peak = sw[:, 1].max()
    ax.annotate(f'peak {peak * 100:.1f} cm\n(swing_height {SW_H * 100:.0f} cm)', (0.7, peak * 100), xytext=(75, 6),
                textcoords='offset points', fontsize=8, arrowprops=dict(arrowstyle='-', color=C['muted'], lw=0.6))
    ax.set(xlabel='foot x relative to its nominal point [cm]  (forward = right)', ylabel='foot z [cm]', xlim=(-6, 6),
           ylim=(-3.0, 7.0))
    ax.set_aspect('equal'); ax.legend(loc='upper left', fontsize=7.4)
    save(fig, '01-foot-trajectory')
    return peak


# ----------------------------------------------------------------------------------------------------------------- diagrams
def fig_leg_geometry():
    """Side and front sketch of the left front leg, URDF values from const.xacro / leg.xacro (chapter 1 table). Angles drawn as
    allowed ranges; the pose itself (thigh 0.8 rad, knee -1.6 rad) is only for illustration."""
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.6, 3.8), gridspec_kw=dict(width_ratios=[1.3, 1]))
    l1 = l2 = 0.213
    for a in (a1, a2):
        a.set_aspect('equal'); a.axis('off')
    q2, q3 = 0.8, -1.6
    hip = np.array([0.0, 0.0])
    dirn = lambda q: np.array([-np.sin(q), -np.cos(q)])   # noqa: E731  leg direction for a pitch angle (0 = straight down)
    knee = hip + l1 * dirn(q2)
    foot = knee + l2 * dirn(q2 + q3)
    ang = lambda q: 270 - np.degrees(q)                    # noqa: E731  matplotlib angle of that direction
    a1.add_patch(FancyBboxPatch((-0.40, -0.03), 0.45, 0.06, boxstyle='round,pad=0,rounding_size=0.01', fc=C['fill'], ec=C['muted'], lw=0.8))
    a1.text(-0.175, 0.0, 'trunk', fontsize=7.8, color=C['muted'], ha='center', va='center')
    a1.annotate('', xy=(0.1, 0.06), xytext=(0.0, 0.06), arrowprops=dict(arrowstyle='-|>', color=C['muted'], lw=0.8, mutation_scale=8))
    a1.text(0.105, 0.06, 'forward', fontsize=7.4, color=C['muted'], va='center')
    a1.add_patch(Wedge(hip, 0.07, ang(3.491), ang(-1.571), width=0.01, fc=LEGS['FL'], alpha=0.3, ec='none'))  # type: ignore[arg-type]  # matplotlib stubs type this point as tuple[float, float]; a length-2 ndarray works at runtime
    a1.add_patch(Wedge(knee, 0.06, ang(q2 - 0.838), ang(q2 - 2.723),  # type: ignore[arg-type]  # matplotlib stubs: center as tuple; ndarray works
                       width=0.01, fc=C['plan'], alpha=0.45, ec='none'))
    a1.plot(*np.c_[hip, knee], color=LEGS['FL'], lw=5, solid_capstyle='round')
    a1.plot(*np.c_[knee, foot], color=LEGS['FL'], lw=3.5, solid_capstyle='round', alpha=0.75)
    for p in (hip, knee):
        a1.add_patch(Circle(p, 0.012, fc='white', ec=C['ink'], lw=1, zorder=5))  # type: ignore[arg-type]  # matplotlib stubs type this point as tuple[float, float]; a length-2 ndarray works at runtime
    a1.add_patch(Circle(foot, 0.012, fc=C['ink'], ec=C['ink'], zorder=5))  # type: ignore[arg-type]  # matplotlib stubs type this point as tuple[float, float]; a length-2 ndarray works at runtime
    a1.axhline(foot[1] - 0.012, color=C['gt'], lw=0.8)
    a1.text(0.08, -0.05, 'hip flexion (thigh)\n-1.571 to 3.491 rad, 23.7 N·m', fontsize=7.6, color=LEGS['FL'], va='top')
    a1.text(knee[0] - 0.07, knee[1], 'knee\n-2.723 to -0.838 rad\n35.55 N·m', fontsize=7.6, ha='right', va='center', color=C['plan'])
    a1.text(*((hip + knee) / 2 + [-0.035, 0.0]), 'thigh\n0.213 m', fontsize=7.6, ha='right')
    a1.text(*((knee + foot) / 2 + [0.025, 0.0]), 'calf 0.213 m', fontsize=7.6)
    a1.text(foot[0] + 0.022, foot[1] + 0.01, 'foot: sphere, r = 2 cm', fontsize=7.6, va='bottom')
    a1.set_title('side view: two pitch joints, axis y', fontsize=9)
    a1.set_xlim(-0.42, 0.3); a1.set_ylim(foot[1] - 0.03, 0.1)
    a2.add_patch(FancyBboxPatch((-0.1, -0.03), 0.2, 0.06, boxstyle='round,pad=0,rounding_size=0.01', fc=C['fill'], ec=C['muted'], lw=0.8))
    a2.text(-0.03, 0.0, 'trunk', fontsize=7.8, color=C['muted'], ha='center', va='center')
    hp = np.array([0.0465, 0.0])                         # hip_offset_y (const.xacro), seen from the front: robot's left = right
    a2.add_patch(Wedge(hp, 0.16, 270 - 60, 270 + 60, width=0.01, fc=LEGS['FL'], alpha=0.3, ec='none'))  # type: ignore[arg-type]  # matplotlib stubs type this point as tuple[float, float]; a length-2 ndarray works at runtime
    for th, alpha in ((1.047, 0.3), (-1.047, 0.3), (0.0, 1.0)):                # the two limits and zero (leg.xacro)
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        lat = hp + R @ [0.0955, 0]
        ft = lat + R @ [0, -0.3]
        a2.plot(*np.c_[hp, lat], color=C['muted'], lw=2.2, alpha=alpha)
        a2.plot(*np.c_[lat, ft], color=LEGS['FL'], lw=4, alpha=alpha, solid_capstyle='round')
    a2.add_patch(Circle(hp, 0.012, fc='white', ec=C['ink'], lw=1, zorder=5))  # type: ignore[arg-type]  # matplotlib stubs type this point as tuple[float, float]; a length-2 ndarray works at runtime
    a2.text(0.17, -0.2, 'hip abduction, axis x\n-1.047 to 1.047 rad\n23.7 N·m', fontsize=7.6, color=LEGS['FL'], va='center')
    a2.text(hp[0] + 0.105, 0.0, 'offset\n0.0955 m', fontsize=7.0, color=C['muted'], ha='left', va='center')
    a2.set_title("front view: robot's left leg (on the right)", fontsize=9)
    a2.set_xlim(-0.26, 0.42); a2.set_ylim(foot[1] - 0.03 + 0.0, 0.1)
    save(fig, '01-leg-geometry')


def fig_layers():
    fig, ax = fs.diagram(7.4, 4.3, (0, 1), (0, 1), equal=False)
    rows = [('robot description', 'URDF from xacro: links, joints, inertias, meshes, <ros2_control> tags, Gazebo plugins', 'description'),
            ('ros2_control', 'GazeboSystem (effort command, position + velocity state), controller_manager 250 Hz,\n'
             'joint_state_broadcaster, joint_trajectory_controller (PID per joint)', 'upstream'),
            ('gait controller', 'CHAMP quadruped_controller_node: /cmd_vel in, 12 joint targets out, 200 Hz', 'upstream'),
            ('state estimation', 'odom -> base_footprint: lidar odometry + IMU EKF, or simulator ground truth', 'ours'),
            ('navigation', 'AMCL (map -> odom) and Nav2, which publishes /cmd_vel', 'upstream')]
    h, gap, y0 = 0.13, 0.045, 0.86
    for i, (name, txt, kind) in enumerate(rows):
        y = y0 - i * (h + gap)
        box(ax, 0.02, y, 0.2, h, name, kind='ours' if kind == 'ours' else 'upstream', fs_=8.6, fontweight='semibold')
        ax.text(0.25, y + h / 2, txt, fontsize=7.9, va='center', color=C['ink'])
    # arrows: commands go down, measurements go up
    yb = y0 - 4 * (h + gap)
    fs.arrow(ax, (0.965, yb + h), (0.965, y0), color=C['plan'], lw=1.2)
    ax.text(0.955, (y0 + yb + h) / 2, 'commands', rotation=90, color=C['plan'], ha='right', va='center', fontsize=8)
    fs.arrow(ax, (0.99, y0), (0.99, yb + h), color=C['est'], lw=1.2)
    ax.text(1.0, (y0 + yb + h) / 2, 'measurements', rotation=90, color=C['est'], ha='left', va='center', fontsize=8)
    ax.text(0.02, 0.0, 'Blue block: the layer with code written for this project (lidar_odom.py). A physical Go2 would replace '
            'GazeboSystem\nwith a driver plugin for its motors; the higher layers stay the same.', fontsize=7.6, color=C['muted'])
    save(fig, '01-layers')


def fig_champ_loop():
    fig, ax = fs.diagram(7.6, 3.5, (0, 1.7), (0, 0.72), equal=False)
    y, h = 0.42, 0.14
    blocks = [(0.02, 0.2, 'phase clock\nper leg'), (0.27, 0.22, 'Raibert foot\nplacement'), (0.53, 0.24, 'swing: Bezier\nstance: press down'),
              (0.81, 0.17, 'leg IK\n3 joints'), (1.07, 0.24, 'joint_trajectory_\ncontroller (PID)'), (1.41, 0.27, 'Gazebo physics\n(GazeboSystem)')]
    for i, (x, w, t) in enumerate(blocks):
        box(ax, x, y, w, h, t, kind='sim' if i == 5 else 'upstream', fs_=7.9)
    ax.add_patch(FancyBboxPatch((0.0, y - 0.035), 0.995, h + 0.11, boxstyle='round,pad=0,rounding_size=0.02', fc='none',
                                ec=C['muted'], lw=0.7, ls='--'))
    ax.text(0.01, y + h + 0.05, 'CHAMP quadruped_controller_node, 200 Hz', fontsize=8, color=C['muted'])
    ax.text(1.07, y + h + 0.05, 'ros2_control, 250 Hz', fontsize=8, color=C['muted'])
    for (x0, w0, _), (x1, _, _) in zip(blocks[:-1], blocks[1:]):
        fs.arrow(ax, (x0 + w0, y + h / 2), (x1, y + h / 2), lw=1)
    ax.text(1.03, y + h / 2 - 0.015, '12\npositions', fontsize=7, color=C['muted'], ha='center', va='top')
    ax.text(1.36, y + h / 2 - 0.015, '12\nefforts', fontsize=7, color=C['muted'], ha='center', va='top')
    topic(ax, 0.1, 0.68, '/cmd_vel')
    line(ax, [(0.1, 0.655), (0.1, y + h)])
    # feedback only into the PID
    line(ax, [(1.545, y), (1.545, 0.3), (1.19, 0.3), (1.19, y)], color=C['est'])
    ax.text(1.37, 0.27, 'joint position + velocity', fontsize=7.4, color=C['est'], ha='center', va='top')
    # inputs CHAMP does not read
    for i, t in enumerate(('IMU', 'foot contacts', 'joint feedback')):
        x = 0.18 + i * 0.27
        ax.text(x, 0.14, t, ha='center', va='center', fontsize=7.8, color=C['bad'],
                bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=C['bad'], lw=0.6, ls='--'))
        line(ax, [(x, 0.17), (x, y - 0.04)], color=C['bad'], ls=':', arrow=False)
        ax.plot([x - 0.015, x + 0.015], [0.27, 0.30], color=C['bad'], lw=1.1)
        ax.plot([x - 0.015, x + 0.015], [0.30, 0.27], color=C['bad'], lw=1.1)
    ax.text(0.01, 0.04, 'Red: not read by CHAMP. The body is open loop: a push, a slope or a slip is not corrected. '
            'Only the joint PID closes a loop.', fontsize=7.6, color=C['muted'])
    save(fig, '01-champ-loop')


def fig_tf_tree():
    fig, ax = fs.diagram(7.4, 5.4, (0, 1), (-0.38, 1), equal=False)
    N = {}

    def node(name, x, y, kind='upstream'):
        N[name] = (x, y)
        s = ORIGIN[kind]
        ax.text(x, y, name, ha='center', va='center', **MONO, bbox=dict(boxstyle='round,pad=0.3', fc=s['fill'], ec=s['color'], lw=0.8))

    def edge(a, b, label=None, color=None, dashed=False):
        (x0, y0), (x1, y1) = N[a], N[b]
        ax.annotate('', xy=(x1, y1 + 0.035), xytext=(x0, y0 - 0.035),
                    arrowprops=dict(arrowstyle='-|>', color=color or C['ink'], lw=0.9, mutation_scale=8, ls='--' if dashed else '-'))
        if label:
            ax.text((x0 + x1) / 2 + 0.012, (y0 + y1) / 2, label, fontsize=7.4, color=color or C['ink'], va='center')

    node('map', 0.3, 0.94); node('odom', 0.3, 0.78); node('base_footprint', 0.3, 0.62); node('base_link', 0.3, 0.46)
    edge('map', 'odom', 'AMCL', C['est2'])
    edge('odom', 'base_footprint', 'EKF (lidar odometry + IMU)\nor gt_odom.py with odom:=gt', C['est'])
    edge('base_footprint', 'base_link', "CHAMP base_to_footprint_ekf\n(constant here: z = 0.426 m)")
    node('lidar_link', 0.06, 0.3); node('camera_link', 0.24, 0.3); node('chase_cam', 0.4, 0.3); node('trunk', 0.62, 0.3)
    node('camera_optical', 0.24, 0.14); node('imu_link', 0.52, 0.14)
    for c in ('trunk', 'lidar_link', 'camera_link', 'chase_cam'):
        edge('base_link', c)
    edge('camera_link', 'camera_optical'); edge('trunk', 'imu_link')
    node('lf_hip_link', 0.76, 0.14); node('lf_upper_leg_link', 0.76, -0.02); node('lf_lower_leg_link', 0.76, -0.18)
    node('lf_foot_link', 0.76, -0.34)
    edge('trunk', 'lf_hip_link', 'joint', C['muted'])
    ax.text(0.86, 0.14, 'x4 legs:\nlf, rf, lh, rh', fontsize=7.4, color=C['muted'], va='center')
    edge('lf_hip_link', 'lf_upper_leg_link', 'joint', C['muted']); edge('lf_upper_leg_link', 'lf_lower_leg_link', 'joint', C['muted'])
    edge('lf_lower_leg_link', 'lf_foot_link', 'fixed', C['muted'])
    ax.text(0.98, 0.94, 'dynamic, from the stack', ha='right', fontsize=7.6, color=C['est'])
    ax.text(0.98, 0.89, 'fixed: robot_state_publisher, /tf_static', ha='right', fontsize=7.6, color=C['ink'])
    ax.text(0.98, 0.84, 'legs: robot_state_publisher from /joint_states', ha='right', fontsize=7.6, color=C['muted'])
    save(fig, '01-tf-tree')


def fig_footprint_offset():
    fig, ax = fs.diagram(6.6, 3.2, (-0.55, 0.75), (-0.27, 0.36))
    ax.axhline(0, color=C['ink'], lw=1)
    ax.add_patch(Rectangle((-0.55, -0.27), 1.3, 0.27, fc=C['fill'], ec='none'))
    ax.text(0.73, 0.01, 'floor', ha='right', va='bottom', fontsize=8, color=C['muted'])
    zt = 0.217
    ax.add_patch(FancyBboxPatch((-0.19, zt - 0.045), 0.38, 0.09, boxstyle='round,pad=0,rounding_size=0.02', fc='white', ec=C['ink'], lw=1))
    for xh in (-0.19, 0.19):
        ax.plot([xh, xh - 0.06, xh], [zt, zt / 2, 0.012], color=LEGS['FL'], lw=2.4)
        ax.add_patch(Circle((xh, 0.012), 0.012, fc=C['ink']))
    ax.plot(0, zt, 'o', color=C['ink'], ms=4)
    ax.text(0.03, zt + 0.012, 'base_link', fontsize=8, family='DejaVu Sans Mono')
    zf = zt - 0.426
    ax.plot([-0.3, 0.3], [zf, zf], color=C['bad'], lw=1.2, ls='--')
    ax.plot(0, zf, 'o', color=C['bad'], ms=4)
    ax.text(0.03, zf + 0.012, 'base_footprint', fontsize=8, color=C['bad'], family='DejaVu Sans Mono')
    def dim(x, a, b, t, col=C['ink']):
        ax.annotate('', xy=(x, b), xytext=(x, a), arrowprops=dict(arrowstyle='<|-|>', color=col, lw=0.8, mutation_scale=7))
        ax.text(x + 0.015, (a + b) / 2, t, fontsize=7.8, va='center', color=col)
    dim(0.36, 0, zt, 'trunk about 0.217 m\nabove the floor')
    dim(-0.36, zf, zt, '0.426 m in TF\n(0.213 + 0.213,\nlegs straight)', C['bad'])
    ax.texts[-1].set_ha('right'); ax.texts[-1].set_position((-0.375, -0.12))
    dim(0.36, zf, 0, '0.209 m below\nthe floor', C['bad'])
    save(fig, '01-footprint-offset')


def fig_ros_graph():
    fig, ax = fs.diagram(7.8, 5.0, (0, 1.56), (0, 1.06), equal=False)
    box(ax, 0.01, 0.6, 0.25, 0.3, 'Gazebo Classic 11\narena, Go2 physics,\nlidar, RGB-D and\nIMU plugins', 'sim', fs_=7.9)
    box(ax, 0.01, 0.12, 0.25, 0.16, 'ros2_control\njoint_trajectory_\ncontroller', 'sim', fs_=7.8)
    box(ax, 0.42, 0.12, 0.24, 0.16, 'CHAMP\nquadruped_\ncontroller', 'upstream', dashed=True, fs_=7.8)
    box(ax, 0.42, 0.6, 0.24, 0.09, 'pointcloud_to_\nlaserscan', 'upstream', fs_=7.4)
    box(ax, 0.42, 0.8, 0.24, 0.09, 'lidar_odom.py', 'ours', fs_=7.9)
    box(ax, 0.8, 0.8, 0.2, 0.09, 'odom_ekf\n(robot_localization)', 'upstream', fs_=7.0)
    box(ax, 0.8, 0.5, 0.2, 0.09, 'AMCL +\nmap_server', 'upstream', fs_=7.6)
    box(ax, 0.8, 0.3, 0.2, 0.09, 'slam_toolbox\n(passive mapper)', 'upstream', fs_=7.2)
    box(ax, 1.14, 0.45, 0.18, 0.3, 'Nav2\nplanner,\ncontroller,\ncostmaps,\nbehaviour tree', 'upstream', fs_=7.6)
    box(ax, 1.36, 0.02, 0.19, 0.16, 'rosbag,\nmetrics and\ndashboard', 'ours', fs_=7.6)
    E = lambda pts, col=None, ls='-': line(ax, pts, color=col, ls=ls)  # noqa: E731
    E([(0.26, 0.85), (0.42, 0.85)]); E([(0.395, 0.85), (0.395, 0.645), (0.42, 0.645)]); ax.text(0.27, 0.865, '/lidar/points', **MONO)
    E([(0.2, 0.9), (0.2, 0.96), (0.54, 0.96), (0.54, 0.89)], C['est']); E([(0.54, 0.96), (0.9, 0.96), (0.9, 0.89)], C['est'])
    topic(ax, 0.38, 0.96, '/imu/data', C['est'])
    E([(0.66, 0.845), (0.8, 0.845)]); ax.text(0.733, 0.86, '/lidar_odom', ha='center', **MONO)
    E([(1.0, 0.845), (1.23, 0.845), (1.23, 0.75)], C['est'])
    ax.text(1.02, 0.875, '/odom + TF odom->base_footprint', color=C['est'], **MONO)
    E([(0.66, 0.645), (1.14, 0.645)]); topic(ax, 0.705, 0.645, '/scan')
    E([(0.76, 0.645), (0.76, 0.545), (0.8, 0.545)]); E([(0.76, 0.545), (0.76, 0.345), (0.8, 0.345)])
    E([(1.0, 0.545), (1.14, 0.545)], C['est2']); ax.text(1.07, 0.56, 'TF\nmap->odom', color=C['est2'], ha='center', va='bottom', family='DejaVu Sans Mono', fontsize=7.2)
    E([(1.0, 0.345), (1.455, 0.345), (1.455, 0.18)], C['gt']); topic(ax, 1.2, 0.345, '/slam/map', C['gt'])
    E([(1.23, 0.45), (1.23, 0.2), (0.66, 0.2)], C['plan']); topic(ax, 0.95, 0.2, '/cmd_vel', C['plan'])
    E([(0.42, 0.2), (0.26, 0.2)]); ax.text(0.34, 0.225, 'joint_\ntrajectory', ha='center', va='bottom', family='DejaVu Sans Mono', fontsize=7.2)
    E([(0.135, 0.28), (0.135, 0.6)]); ax.text(0.145, 0.44, 'efforts', fontsize=7.4, color=C['muted'])
    E([(0.04, 0.6), (0.04, 0.06), (1.36, 0.06)], C['gt'], ls='--'); topic(ax, 0.7, 0.06, '/odom_gt  (scoring only)', C['gt'])
    legend_origin(ax, 0.0, 0.985, ['sim', 'upstream', 'patched', 'ours'], dx=0.22)
    save(fig, '00-ros-graph')


def arena_axes(ax, run):
    a = run['arena']
    for cx, cy, w, h in a['walls']:
        ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fc=C['ink'], ec='none'))
    for b in a['boxes']:
        cx, cy, w, h = b[:4]
        ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fc=C['fill'], ec=C['muted'], lw=0.8))
    for cx, cy, r, _ in a['cylinders']:
        ax.add_patch(Circle((cx, cy), r, fc=C['fill'], ec=C['muted'], lw=0.8))
    for k, col in (('A', C['good']), ('B', C['bad'])):
        ax.add_patch(Circle(a[k], 0.3, fc=col, alpha=0.25, ec=col))
        ax.text(a[k][0], a[k][1] + 0.45, k, ha='center', fontsize=10, fontweight='semibold', color=col)
    ax.set_aspect('equal'); ax.set_xlim(-6.4, 6.4); ax.set_ylim(-5.4, 5.4)


def fig_arena(run):
    fig, ax = plt.subplots(figsize=(5.8, 5.0))
    arena_axes(ax, run)
    gt = np.array([p['gt'][:2] for p in run['samples']])
    ax.plot(*gt.T, color=C['gt'], lw=1.6, label='ground-truth path of run w2')
    for b in run['arena']['boxes']:
        ax.text(b[0], b[1], f"{b[4]:.2f} m", ha='center', va='center', fontsize=6.8, color=C['muted'])
    for cx, cy, r, h in run['arena']['cylinders']:
        ax.text(cx, cy, f"{h:.2f} m", ha='center', va='center', fontsize=6.8, color=C['muted'])
    A = run['arena']['A']
    ax.annotate('', xy=(A[0] + 0.8, A[1]), xytext=A, arrowprops=dict(arrowstyle='-|>', color=C['good'], lw=1.2, mutation_scale=9))
    ax.text(A[0] + 0.85, A[1], 'start heading +x', fontsize=7.2, color=C['good'], va='center')
    ax.set(xlabel='x [m]', ylabel='y [m]')
    m = run['metrics']
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), fontsize=7.6, ncol=1,
              title=f"path {m['path_m']} m, final error {m['goal_error_m']} m; numbers in obstacles = height", title_fontsize=7.6)
    save(fig, '00-arena')


def fig_goal_run(run):
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.8, 3.9), gridspec_kw=dict(width_ratios=[1.35, 1]))
    arena_axes(ax, run)
    s = run['samples']
    gt = np.array([p['gt'][:2] for p in s]); od = np.array([p['odom'][:2] for p in s])
    am = np.array([p['amcl'][:2] for p in s if p.get('amcl')])
    for i, pl in enumerate(run['plans'][::12]):
        pts = np.array(pl['pts'])
        ax.plot(*pts.T, color=C['plan'], lw=0.8, alpha=0.5, label='Nav2 plan (every 12th)' if i == 0 else None)
    ax.plot(*gt.T, color=C['gt'], lw=1.8, label='ground truth')
    ax.plot(*od.T, color=C['est'], lw=1.0, label='/odom (EKF), anchored at A')
    ax.plot(*am.T, color=C['est2'], lw=0.8, ls='--', label='AMCL')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.14), fontsize=7.2, ncol=2)
    ax.set(xlabel='x [m]', ylabel='y [m]')
    B = np.array(run['arena']['B']); m = run['metrics']
    ax2.add_patch(Circle(B, m['goal_tolerance_m'], fc='none', ec=C['bad'], ls='--', lw=1))  # type: ignore[arg-type]  # matplotlib stubs type this point as tuple[float, float]; a length-2 ndarray works at runtime
    ax2.text(B[0], B[1] + m['goal_tolerance_m'] + 0.03, f"goal tolerance {m['goal_tolerance_m']} m", ha='center', fontsize=7.6, color=C['bad'])
    ax2.plot(*gt.T, color=C['gt'], lw=1.8); ax2.plot(*od.T, color=C['est'], lw=1.0); ax2.plot(*am.T, color=C['est2'], lw=0.8, ls='--')
    ax2.plot(*B, '+', color=C['bad'], ms=10)
    ax2.plot(*gt[-1], 'o', color=C['gt'], ms=5)
    ax2.annotate(f"final ground-truth error {m['goal_error_m']} m", gt[-1], xytext=(20, -70), textcoords='offset points', fontsize=7.6,
                 ha='center', arrowprops=dict(arrowstyle='-', color=C['muted'], lw=0.6))
    ax2.set_xlim(B[0] - 0.8, B[0] + 0.45); ax2.set_ylim(B[1] - 0.6, B[1] + 0.45); ax2.set_aspect('equal')
    ax2.set_title('at B', fontsize=9); ax2.set(xlabel='x [m]')
    ax2.text(0.03, 0.97, f"odom ATE {m['odom_ate_aligned_cm']} cm (aligned)\nAMCL RMSE {m['amcl_rmse_cm']} cm\n"
             f"{m['path_m']} m in {m['duration_s']} s", transform=ax2.transAxes, fontsize=7.4, color=C['muted'], va='top')
    save(fig, '03-goal-run')


def fig_software_stack():
    fig, ax = fs.diagram(7.6, 4.2, (0, 1.5), (0, 0.84), equal=False)
    box(ax, 0.02, 0.02, 0.86, 0.1, 'Ubuntu 22.04', 'upstream', fs_=8.6)
    box(ax, 0.02, 0.15, 0.86, 0.1, 'ROS 2 Humble  +  Cyclone DDS (rmw_cyclonedds_cpp, set by env.sh)', 'upstream', fs_=8.2)
    box(ax, 0.02, 0.28, 0.3, 0.12, 'Gazebo Classic 11\nend of life 31 Jan 2025', 'sim', fs_=7.8)
    box(ax, 0.35, 0.28, 0.53, 0.12, 'gazebo_ros_pkgs (sensor plugins),\ngazebo_ros2_control, ros2_control,\nros2_controllers', 'sim', fs_=7.0)
    cols = [('navigation', 'navigation2\nnav2_bringup'), ('mapping', 'slam_toolbox'), ('estimation', 'robot_localization\n'
            'pointcloud_to_\nlaserscan\ntf2_sensor_msgs'), ('recording', 'rosbag2_storage_\nmcap\nrosbag2_compression_\nzstd')]
    for i, (role, pk) in enumerate(cols):
        x = 0.02 + i * 0.218
        box(ax, x, 0.43, 0.205, 0.2, pk, 'upstream', fs_=6.4)
        ax.text(x + 0.1025, 0.655, role, ha='center', fontsize=7.8, color=C['muted'])
    box(ax, 0.02, 0.69, 0.86, 0.1, 'this repository: launch files, lidar_odom.py, evaluation, dashboard (colcon build)', 'ours', fs_=7.4)
    # right: current pairing
    ax.text(1.18, 0.8, 'current pairing (not used here)', ha='center', fontsize=8, color=C['muted'])
    box(ax, 1.0, 0.56, 0.36, 0.1, 'Gazebo Harmonic', 'upstream', dashed=True, fs_=8)
    box(ax, 1.0, 0.42, 0.36, 0.1, 'ROS 2 Jazzy', 'upstream', dashed=True, fs_=8)
    ax.text(1.18, 0.34, 'port planned (module 03)', ha='center', fontsize=7.6, color=C['plan'])
    fs.arrow(ax, (0.9, 0.53), (0.99, 0.53), color=C['plan'], lw=1)
    ax.text(0.02, -0.04, 'Python: numpy, scipy, opencv, PIL; tools: colcon, ffmpeg, git (apt list in this chapter)', fontsize=7.4,
            color=C['muted'])
    save(fig, '02-software-stack')


def fig_env_traps():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.8, 3.3))
    for a in (a1, a2):
        a.set_xlim(0, 1); a.set_ylim(0, 1); a.axis('off')
    a1.set_title('trap 1: a virtualenv on PATH', fontsize=9)
    rows = [('PATH=.venv/bin:/usr/bin:...', C['bad']), ('python3 -> .venv/bin/python3', C['bad']), ('rosidl: import em', C['ink']),
            ("No module named 'em'  (colcon build fails)", C['bad'])]
    for i, (t, col) in enumerate(rows):
        y = 0.86 - i * 0.17
        a1.text(0.04, y, t, color=col, **MONO, va='center', bbox=dict(boxstyle='round,pad=0.3', fc='white', ec=C['grid'], lw=0.6))
        if i < 3:
            fs.arrow(a1, (0.1, y - 0.045), (0.1, y - 0.125), lw=0.8)
    a1.text(0.04, 0.17, 'Fix: env.sh drops every /.venv/ entry from PATH\nand unsets VIRTUAL_ENV before sourcing ROS.',
            fontsize=7.8, color=C['good'], va='center')
    a2.set_title('trap 2: lost service replies (Fast DDS)', fontsize=9)
    xs = (0.15, 0.75)
    for x, t in zip(xs, ('lifecycle_manager', 'amcl, planner, ...')):
        a2.text(x, 0.9, t, ha='center', **MONO, bbox=dict(boxstyle='round,pad=0.3', fc=C['fill'], ec=C['ink'], lw=0.6))
        a2.plot([x, x], [0.85, 0.3], color=C['grid'], lw=1)
    fs.arrow(a2, (xs[0], 0.75), (xs[1], 0.7), 'change_state(configure)', lw=0.9, offset=(0, 2), fontsize=7.4)
    fs.arrow(a2, (xs[1], 0.6), (0.42, 0.55), color=C['bad'], lw=0.9, ls='--')
    a2.text(0.42, 0.55, '  X', color=C['bad'], fontsize=11, va='center', ha='right', fontweight='bold')
    a2.text(0.6, 0.53, 'reply dropped\non a busy machine', fontsize=7.4, color=C['bad'], ha='left', va='top')
    a2.text(0.17, 0.4, '"client will not receive response"\n-> "Failed to bring up all\n    requested nodes"', fontsize=7.2, color=C['bad'], ha='left', va='center', family='DejaVu Sans Mono')
    a2.text(0.04, 0.17, 'Fix: env.sh sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nwith CYCLONEDDS_URI on the loopback interface.',
            fontsize=7.8, color=C['good'], va='center')
    save(fig, '02-env-traps')


def fig_setup_flow():
    fig, ax = fs.diagram(7.8, 2.9, (0, 1.6), (0, 0.6), equal=False)
    steps = [('git fetch upstream\nat fdac259', 'upstream'), ('git apply\npatches/upstream.patch', 'ours'),
             ('libgazebo_ros_camera.so\nin /opt/ros/humble?', 'upstream'), ('colcon build\n--symlink-install', 'upstream')]
    xs = [0.02, 0.42, 0.82, 1.28]
    for (t, k), x in zip(steps, xs):
        box(ax, x, 0.36, 0.3, 0.16, t, k, fs_=7.8)
    for x0, x1 in zip(xs[:-1], xs[1:]):
        fs.arrow(ax, (x0 + 0.3, 0.44), (x1, 0.44), lw=1)
    ax.text(1.2, 0.47, 'yes', fontsize=7.4, color=C['good'], ha='center')
    box(ax, 0.72, 0.02, 0.56, 0.2, 'apt-get download ros-humble-gazebo-plugins,\nros-humble-camera-info-manager,\nros-humble-camera-calibration-parsers;\ndpkg -x into overlay/', 'ours', fs_=7.0)
    line(ax, [(0.97, 0.36), (0.97, 0.22)], C['bad']); ax.text(0.985, 0.29, 'no', fontsize=7.4, color=C['bad'])
    line(ax, [(1.28, 0.12), (1.43, 0.12), (1.43, 0.36)])
    ax.text(0.17, 0.3, 'src/third_party/unitree-go2-ros2', ha='center', fontsize=7, color=C['muted'], family='DejaVu Sans Mono')
    ax.text(0.57, 0.3, 'skipped if already applied', ha='center', fontsize=7.2, color=C['muted'])
    ax.text(1.43, 0.55, 'build: 2 min 51 s here (32 threads, shared)', ha='center', fontsize=7.4, color=C['muted'])
    save(fig, '03-setup-flow')


def fig_up_sequence():
    fig, ax = fs.diagram(7.8, 4.2, (0, 1.6), (0, 0.97), equal=False)
    cols = [('stop.sh\nfree Gazebo port', 'sim', []),
            ('sim.launch.py', 'sim', ['gz stats reports a\nreal-time factor\n(75 tries, 2 s apart)', 'one /imu/data\nmessage arrives\n(40 tries)']),
            ('estimation.launch.py', 'ours', ['log shows " pose="\nfrom lidar_odom\n(30 tries)']),
            ('nav.launch.py', 'upstream', ['2 x "Managed nodes\nare active"\n(45 tries)'])]
    w, gap, yb = 0.3, 0.1, 0.62
    for i, (launch, k, checks) in enumerate(cols):
        x = 0.02 + i * (w + gap)
        box(ax, x, yb, w, 0.12, launch, k, fs_=7.8)
        if i < 3:
            fs.arrow(ax, (x + w, yb + 0.06), (x + w + gap, yb + 0.06), lw=0.9)
        y = yb
        for c in checks:
            fs.arrow(ax, (x + w / 2, y), (x + w / 2, y - 0.07), lw=0.8, color=C['good'])
            ax.text(x + w / 2, y - 0.14, c, ha='center', va='center', fontsize=7.2, bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=C['good'], lw=0.7))
            y -= 0.21
    xn = 0.02 + 3 * (w + gap)
    fs.arrow(ax, (xn + w / 2, yb - 0.21), (xn + w / 2, 0.3), lw=0.8, color=C['good'])
    ax.text(xn + w / 2, 0.26, 'stack up', ha='center', va='top', **MONO, color=C['good'])
    ax.annotate('', xy=(xn + 0.04, yb + 0.12), xytext=(xn + w - 0.04, yb + 0.12),
                arrowprops=dict(arrowstyle='-|>', color=C['bad'], lw=0.8, connectionstyle='arc3,rad=0.7', mutation_scale=8))
    ax.text(xn + w / 2, 0.96, 'on "Aborting bringup":\nstop nav, retry (3 attempts)', ha='center', va='top', fontsize=7.2, color=C['bad'])
    ax.text(0.02, 0.9, 'scripts/up.sh: the next launch file starts only after the checks under the previous one pass', fontsize=8, va='top')
    ax.text(0.02, 0.06, 'A check that never passes prints "TIMEOUT waiting for: ..." and up.sh exits. Logs: logs/up_sim.log, '
            'logs/up_est.log, logs/up_nav.log.', fontsize=7.4, color=C['muted'])
    save(fig, '03-up-sequence')


def fig_patch():
    fig, ax = fs.diagram(7.8, 3.0, (0, 1.6), (-0.12, 0.64), equal=False)
    ax.text(0.25, 0.59, 'upstream (teleoperation demo)', ha='center', fontsize=8.5, fontweight='semibold')
    ax.text(1.12, 0.59, 'after patches/upstream.patch', ha='center', fontsize=8.5, fontweight='semibold')
    rows = [('ros2 control load_controller\n(needs ros2controlcli; often missing,\nthen the legs get no commands)',
             'controller_manager spawner'),
            ('contact_sensor node\n(over 200% of a core, feature switched off)', 'removed'),
            ("CHAMP's leg-odometry EKF always\nowns the odom transform", 'gt_odom argument disables it, so another\nsource can own odom')]
    for i, (a, b) in enumerate(rows):
        y = 0.4 - i * 0.19
        ax.text(0.02, y, a, fontsize=7.6, va='center', color=C['bad'],
                bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=C['bad'], lw=0.6, ls='--'))
        ax.text(0.84, y, b, fontsize=7.6, va='center', color=C['good'],
                bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=C['good'], lw=0.6))
        fs.arrow(ax, (0.68, y), (0.8, y), lw=0.9)
    ax.text(0.02, -0.1, 'gazebo.launch.py (first two), bringup.launch.py (third), in champ_gazebo and champ_bringup', fontsize=7.2,
            color=C['muted'])
    save(fig, '03-patch')


# ----------------------------------------------------------------------------------------------------------------- added
def fig_scoring_pipeline():
    """Chapter 0, task 1.1: what happens to a run after the robot stops. Topics from scripts/record_bag.sh, outputs from the
    usage lines of bag_metrics.py, render_dashboard.py and export_run.py."""
    fig, ax = fs.diagram(7.8, 3.5, (0, 1.66), (0, 0.72), equal=False)
    box(ax, 0.01, 0.42, 0.25, 0.2, 'running stack\n(scripts/up.sh)', 'sim', fs_=8)
    box(ax, 0.36, 0.42, 0.25, 0.2, 'scripts/record_bag.sh\nros2 bag record\nMCAP, zstd, 10.5 MiB\nper simulated second', 'ours', fs_=7.4)
    box(ax, 0.71, 0.42, 0.2, 0.2, 'bags/<name>/\none MCAP bag', 'upstream', fs_=7.6)
    outs = [(0.56, 'bag_metrics.py', 'results/run_wN.json\nATE, RPE, AMCL RMSE, goal error'),
            (0.32, 'render_dashboard.py', 'dashboard video (1080p)\nmedia/go2_a2b_dashboard.mp4'),
            (0.08, 'export_run.py', 'site/data/runs/wN/\nrun.json (10 Hz), imu.json, map')]
    for y, script, out in outs:
        box(ax, 1.0, y, 0.28, 0.12, script, 'ours', fs_=7.4, family='DejaVu Sans Mono')
        ax.text(1.31, y + 0.06, out, fontsize=7.0, va='center', color=C['ink'], linespacing=1.3)
        line(ax, [(0.91, 0.52), (0.95, 0.52), (0.95, y + 0.06), (1.0, y + 0.06)])
    line(ax, [(0.26, 0.52), (0.36, 0.52)]); line(ax, [(0.61, 0.52), (0.71, 0.52)])
    groups = ['sensors  /lidar/points /scan /imu/data /camera/*', 'estimates  /odom /lidar_odom /amcl_pose',
              'navigation  /plan /cmd_vel /slam/map', 'frames  /clock /tf /tf_static']
    for i, g in enumerate(groups):
        ax.text(0.33, 0.34 - i * 0.05, g, fontsize=6.8, color=C['muted'], family='DejaVu Sans Mono')
    ax.text(0.33, 0.34 - 4 * 0.05, 'truth  /odom_gt', fontsize=6.8, color=C['bad'], family='DejaVu Sans Mono', fontweight='bold')
    ax.text(0.33, 0.39, 'recorded topics', fontsize=7.4, color=C['ink'])
    ax.text(0.01, 0.06, '/odom_gt is recorded for scoring.\nThe robot never reads it, except once:\ngo_to_goal.py initialises AMCL from it.',
            fontsize=7.0, color=C['bad'], va='bottom', linespacing=1.3)
    legend_origin(ax, 0.0, 0.97, ['sim', 'upstream', 'ours'], dx=0.25)
    save(fig, '00-scoring-pipeline')


def fig_final_approach():
    """Chapter 0 exercise and chapter 3 exercise: where each recorded run really stopped and where its last AMCL pose put it.
    Trajectories from site/data/runs/wN/run.json, goal errors from results/run_wN.json."""
    fig, axs = plt.subplots(1, 3, figsize=(7.8, 3.0), sharey=True)
    B = tol = None
    for ax, w in zip(axs, ('w1', 'w2', 'w3')):
        run = json.loads((ROOT / f'site/data/runs/{w}/run.json').read_text())
        res = json.loads((ROOT / f'results/run_{w}.json').read_text())
        B, tol = np.array(run['arena']['B']), res['goal_tolerance_m']
        gt = np.array([p['gt'][:2] for p in run['samples']])
        am = np.array([p['amcl'][:2] for p in run['samples'] if p.get('amcl')])
        d_gt, d_am = np.linalg.norm(gt[-1] - B), np.linalg.norm(am[-1] - B)
        assert abs(d_gt - res['goal_error_m']) < 0.01, (w, d_gt)
        ax.add_patch(Circle(B, tol, fc=C['fill'], ec=C['bad'], ls='--', lw=1))  # type: ignore[arg-type]  # matplotlib stubs type this point as tuple[float, float]; a length-2 ndarray works at runtime
        ax.plot(*B, '+', color=C['bad'], ms=9)
        ax.plot(*gt.T, color=C['gt'], lw=1.8, label='ground truth')
        ax.plot(*am.T, color=C['est2'], lw=0.9, ls='--', label='/amcl_pose')
        ax.plot(*gt[-1], 'o', color=C['gt'], ms=6); ax.plot(*am[-1], 'o', color=C['est2'], ms=5, mfc='white')
        out = d_gt > tol
        ax.set_title(f'run {w}', fontsize=9)
        ax.text(0.03, 0.97, f'truth {d_gt:.2f} m from B ({"outside" if out else "inside"})\nlast AMCL pose {d_am:.2f} m',
                transform=ax.transAxes, fontsize=7.2, color=C['bad'] if out else C['ink'], va='top')
        ax.set_xlim(B[0] - 0.75, B[0] + 0.4); ax.set_ylim(B[1] - 0.75, B[1] + 0.4); ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
    axs[0].set_ylabel('y [m]')
    axs[0].legend(loc='lower right', fontsize=7.0)
    assert B is not None and tol is not None                # set by the last run of the loop
    axs[1].text(B[0], B[1] - tol - 0.06, f'tolerance {tol} m', ha='center', fontsize=7.0, color=C['bad'])
    save(fig, '00-final-approach')


def fig_route():
    """Chapter 0, task 2.1: the route as cumulative hours (the table in 00-overview.md; * = estimate not in gap-plan;
    ~ = not yet published, no README.md in its folder or no chapter file)."""
    steps = [('0  What you will build', 0.5), ('1  Legged robot foundations', 1.5), ('2, 3  Prerequisites, install', 2),
             ('4  ROS 2 from scratch', 4), ('p01  First steps', 2), ('5  Frames and TF2', 2.5), ('6  Import the Go2', 2),
             ('7  Make it walk', 2), ('p02  Teleop and safety', 3), ('p03  Move in shapes', 2), ('p04  Go to pose *', 2),
             ('8  Robot and sensors', 2), ('p05  Sense and stop ~', 2), ('9, 11  Estimation, record', 4),
             ('10  Mapping and navigation', 4.5), ('p11  Command arbitration *', 2), ('p12  Tune Nav2 ~', 2.5),
             ('p13  Behaviour trees', 1.5), ('p14  Test the stack', 1.5), ('13  Toward the real Go2', 2.5),
             ('p  Capstone ~', 3)]
    t0 = np.cumsum([0] + [h for _, h in steps])
    assert t0[-1] == 49 and t0[5] == 10
    fig, ax = plt.subplots(figsize=(7.6, 4.8))
    for i, ((name, h), s) in enumerate(zip(steps, t0)):
        proj = name.startswith('p')
        ax.barh(i, h, left=s, height=0.7, color=C['plan'] if proj else C['est'], alpha=0.3 if '~' in name else 0.85,
                hatch='///' if '*' in name else None, edgecolor='white', lw=0.5)
        ax.text(s - 0.4, i, name.replace(' ~', '').replace('p  ', ''), ha='right', va='center', fontsize=7.2,
                color=C['muted'] if '~' in name else C['ink'])
    ax.axvspan(0, 10, color=C['good'], alpha=0.07, lw=0)
    ax.text(5, -1.3, 'first 10 h: a running stack you can inspect and drive', ha='center', fontsize=7.4, color=C['good'])
    ax.set_ylim(len(steps) - 0.4, -1.8); ax.set_yticks([]); ax.set_xlim(-19, 50)
    ax.set_xticks(range(0, 50, 5)); ax.set_xlabel('cumulative hours (49 h; the gap plan\'s 45 h without the two * steps)')
    ax.spines['left'].set_visible(False); ax.grid(axis='y', visible=False)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=C['est'], label='chapter'), Patch(color=C['plan'], label='project'),
                       Patch(fc=C['plan'], ec='white', hatch='///', label='* hours estimated by comparison'),
                       Patch(color=C['gt'], alpha=0.3, label='not yet published')],
              loc='upper right', fontsize=7.2)
    save(fig, '00-route')


def fig_scan_band():
    """Chapter 1 exercise: the /scan height band of scan.yaml in base_footprint, next to the floor that sits 0.209 m above it."""
    floor, base = 0.426 - 0.217, 0.426                    # tf2_echo 0.426 m; trunk 0.217 m above the floor (chapter 1, task 4.2)
    fig, ax = plt.subplots(figsize=(6.6, 3.8))
    ax.axhspan(-0.1, floor, color=C['fill'], lw=0); ax.axhline(floor, color=C['ink'], lw=1.2)
    ax.axhline(0, color=C['bad'], lw=1, ls='--')
    ax.text(3.95, 0.0 - 0.025, 'base_footprint (z = 0)', color=C['bad'], fontsize=7.4, ha='right', va='top', family='DejaVu Sans Mono')
    ax.text(3.95, floor + 0.012, f'real floor, z = {floor:.3f} m', fontsize=7.4, ha='right', va='bottom')
    # robot sketch: trunk at base_link, two legs to the floor
    ax.add_patch(FancyBboxPatch((0.15, base - 0.04), 0.9, 0.08, boxstyle='round,pad=0,rounding_size=0.02', fc='white', ec=C['ink'], lw=0.9))
    ax.text(0.6, base, 'base_link 0.426', fontsize=7, ha='center', va='center', family='DejaVu Sans Mono')
    for x in (0.25, 0.95):
        ax.plot([x, x - 0.1, x], [base - 0.04, (base + floor) / 2, floor + 0.01], color=LEGS['FL'], lw=2.5)
    cols = [(1.5, 0.30, 0.90, 'scan.yaml\nmin_height 0.30', C['est']), (2.8, 0.05, 0.90, 'min_height 0.05\n(wheeled default)', C['bad'])]
    for x, lo, hi, name, col in cols:
        ax.add_patch(Rectangle((x, lo), 0.9, hi - lo, fc=col, alpha=0.15, ec=col, lw=1))
        ax.text(x + 0.45, hi + 0.02, name, ha='center', va='bottom', fontsize=7.4, color=col)
        span = f'{lo - floor:.2f} to {hi - floor:.2f} m' if lo > floor else f'{floor - lo:.2f} m below to\n{hi - floor:.2f} m'
        ax.text(x + 0.45, (max(lo, floor) + hi) / 2, f'{span}\nabove the floor',
                ha='center', va='center', fontsize=7.2, color=C['ink'])
    ax.plot([2.8, 3.7], [floor, floor], color=C['bad'], lw=3)
    ax.text(3.25, (0.05 + floor) / 2, 'floor inside the band:\nevery floor return is\nan obstacle in /scan', ha='center', va='center',
            fontsize=7.0, color=C['bad'])
    ax.set_xlim(0, 4.0); ax.set_ylim(-0.1, 1.05); ax.set_xticks([])
    ax.set_ylabel('height in base_footprint [m]'); ax.spines['bottom'].set_visible(False); ax.grid(axis='x', visible=False)
    save(fig, '01-scan-band')


def fig_sensor_rates():
    """Chapter 3, task 2.2: measured rates (ros2 topic hz -s, outputs in the chapter) against the configured ones
    (sensors.xacro, upstream gazebo.xacro, ekf.yaml), and the same lidar rate measured in wall-clock time."""
    rows = [('/imu/data', 100, 97.925, 'gazebo.xacro', '97.925'), ('/odom (EKF)', 50, 50.000, 'ekf.yaml', '50.000'),
            ('/lidar/points', 10, 10.027, 'sensors.xacro', '10.027'), ('/scan', 10, 9.984, 'follows the lidar', '9.984'),
            ('/camera/image_raw', 10, 1 / 0.29, 'sensors.xacro', '3.4 (one image per 0.29 s, busy GPU)')]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.8, 2.9), gridspec_kw=dict(width_ratios=[1.7, 1]))
    y = np.arange(len(rows))
    a1.barh(y + 0.18, [r[1] for r in rows], height=0.34, color=C['gt'], alpha=0.5, label='configured')
    a1.barh(y - 0.18, [r[2] for r in rows], height=0.34, color=C['est'], label='measured, simulated time')
    for i, (_, cfg, meas, src, txt) in enumerate(rows):
        a1.text(meas * 1.08, i - 0.18, f'{txt} Hz' if i < 4 else f'{txt}', va='center', fontsize=7, color=C['est'] if i < 4 else C['bad'])
        a1.text(cfg * 1.08, i + 0.18, f'{cfg} Hz  ({src})', va='center', fontsize=6.6, color=C['muted'])
    a1.set_xscale('log'); a1.set_xlim(1, 2000); a1.set_yticks(y, [r[0] for r in rows], fontsize=7.6, family='DejaVu Sans Mono')
    a1.invert_yaxis(); a1.set_xlabel('rate [Hz], log scale')
    a1.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=2, fontsize=7)
    a2.bar([0, 1], [10.03, 5.99], color=[C['est'], C['bad']], width=0.6)
    for x, v in ((0, 10.03), (1, 5.99)):
        a2.text(x, v + 0.2, f'{v} Hz', ha='center', fontsize=7.4)
    a2.set_xticks([0, 1], ['hz -s\n(sim time)', 'hz\n(wall clock)'], fontsize=7.4)
    a2.set_ylim(0, 12); a2.set_ylabel('/lidar/points [Hz]'); a2.set_title('same lidar, real-time factor ~0.58', fontsize=8.5)
    save(fig, '03-sensor-rates')


# ----------------------------------------------------------------------------------------------------------------- clips
def clip(name, start, dur, vf):
    src, out = MEDIA / 'go2_a2b_dashboard.mp4', MEDIA / f'c00-{name}.mp4'
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', str(start), '-t', str(dur), '-i', str(src), '-an', '-vf', vf,
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '28', '-preset', 'slow', '-movflags', '+faststart', str(out)],
                   check=True)
    kb = out.stat().st_size / 1024
    assert kb < 4096, f'{out.name} is {kb:.0f} KB'
    print(out.relative_to(ROOT), f'{kb:.0f} KB')


if __name__ == '__main__':
    run = json.loads((ROOT / 'site/data/runs/w2/run.json').read_text())
    figs = [fig_go2_render, fig_go2_joints, fig_ros_graph, lambda: fig_arena(run), fig_leg_geometry, fig_layers, fig_trot,
            lambda: print('swing peak [m]', round(fig_foot_trajectory(), 4)), fig_champ_loop, fig_tf_tree, fig_footprint_offset,
            fig_software_stack, fig_env_traps, fig_setup_flow, fig_up_sequence, fig_patch, lambda: fig_goal_run(run),
            fig_scoring_pipeline, fig_final_approach, fig_route, fig_scan_band, fig_sensor_rates]
    only = [a for a in sys.argv[1:] if not a.startswith('--')]          # e.g. "trot" runs only fig_trot
    for f in figs:
        if not only or any(o in (f.__name__ + ' '.join(f.__code__.co_names)) for o in only):
            f()
    if '--videos' in sys.argv:
        clip('00-a2b-start', 3, 12, 'scale=1280:720')
        clip('01-trunk-motion', 40, 10, 'crop=1920:600:0:470,scale=1280:400')
        clip('03-goal-reached', 88, 15, 'scale=1280:720')
        video_trot()
