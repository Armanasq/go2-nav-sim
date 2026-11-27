"""Annotated MuJoCo renders of the Go2 for chapters 6 and 8 (docs/img/c06-*.png).
Run from the repository root:  DISPLAY=:1 MUJOCO_GL=glfw .venv-modules/bin/python docs/img/c06-render.py
The model is the MuJoCo Menagerie Go2 (modules/assets/unitree_go2). Its hip, thigh and calf offsets (0.1934, 0.0465, 0.0955,
0.213 m) are the ones in upstream const.xacro, so the joint and sensor positions drawn here are the URDF's, mapped by name:
FL->lf, FR->rf, RL->lh, RR->rh."""
import os
import pathlib
import sys

import numpy as np
os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'modules'))
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from common import require_model  # noqa: E402

OUT = ROOT / 'docs/img'
URDF = dict(FL='lf', FR='rf', RL='lh', RR='rh')


def setup(h=720, w=1280, wash=(0.06, 0.18)):
    m = mujoco.MjModel.from_xml_path(require_model(scene=True))
    m.vis.global_.offwidth, m.vis.global_.offheight = max(w, 1280), max(h, 720)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, h, w)
    # paper-like background: no reflections, skybox and floor checker washed out to light grey, so annotations stay readable
    m.mat_reflectance[:] = 0
    for t in range(m.ntex):
        a, n = m.tex_adr[t], m.tex_width[t] * m.tex_height[t] * m.tex_nchannel[t]
        x = m.tex_data[a:a + n].astype(float)
        k = wash[0] if m.tex_type[t] == mujoco.mjtTexture.mjTEXTURE_SKYBOX else wash[1]
        m.vis.rgba.haze[:] = (1, 1, 1, 1)
        if m.tex_type[t] == mujoco.mjtTexture.mjTEXTURE_SKYBOX or 'groundplane' in m.texture(t).name:
            m.tex_data[a:a + n] = (255 - (255 - x) * k).astype(np.uint8)
            mujoco.mjr_uploadTexture(m, r._mjr_context, t)
    return m, d, r


def ghost(m, alpha=0.45):
    """Make the robot see-through so joint origins inside the shells stay visible."""
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    for g in range(m.ngeom):
        if g != floor:
            m.geom_rgba[g, 3] = alpha
    for k in range(m.nmat):
        if 'groundplane' not in m.material(k).name:
            m.mat_rgba[k, 3] = alpha


def render(m, d, r, lookat, dist, az, el, fovy=None, far=4.0):
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, dist, az, el
    if fovy:
        m.vis.global_.fovy = fovy
    opt = mujoco.MjvOption()
    r.update_scene(d, cam, opt)
    img = r.render().copy()
    r.enable_depth_rendering(); depth = r.render(); r.disable_depth_rendering()
    img[depth > far] = 255                       # blank the far floor and horizon: a plain background behind the annotations
    gl = r.scene.camera[0]
    pos, fwd, up = np.array(gl.pos), np.array(gl.forward), np.array(gl.up)
    right = np.cross(fwd, up)
    H, W = img.shape[:2]
    f = H / 2 / np.tan(np.radians(m.vis.global_.fovy) / 2)              # pinhole focal length in pixels

    def proj(p):
        v = np.asarray(p, float) - pos
        z = v @ fwd
        return np.array([W / 2 + f * (v @ right) / z, H / 2 - f * (v @ up) / z])
    return img, proj


def body(m, d, name):
    return d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)].copy()


def foot(m, d, leg):
    return d.geom_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, leg)].copy()


def stand(m, d, h):
    """Kinematic pose: base_link at height h, each foot sphere under its thigh joint and touching the floor (closed-form IK)."""
    sys.path.insert(0, str(ROOT / 'modules/m04_kinematics'))
    from kinematics import Leg
    d.qpos[:7] = [0, 0, h, 1, 0, 0, 0]
    for i, leg in enumerate(('FL', 'FR', 'RL', 'RR')):
        L = Leg(m, leg)
        sols = L.ik_closed_form([L.hip[0], L.hip[1] + L.d, -(h - 0.022)])
        q = sols[np.argmin(np.linalg.norm(sols - [0, 0.9, -1.8], axis=1))]      # the branch nearest the Menagerie home pose
        d.qpos[7 + 3 * i: 10 + 3 * i] = q
    mujoco.mj_forward(m, d)


def show(img, w=8.0):
    H, W = img.shape[:2]
    fig, ax = plt.subplots(figsize=(w, w * H / W))
    ax.imshow(img); ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis('off')
    return fig, ax


def arrow3(ax, proj, p, v, color, text=None, lw=2.0, fontsize=9, off=(4, 0)):
    a, b = proj(p), proj(np.asarray(p) + v)
    ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='-|>', color=color, lw=lw, shrinkA=0, shrinkB=0, mutation_scale=12))
    if text:
        ax.annotate(text, b, xytext=off, textcoords='offset points', color=color, fontsize=fontsize, fontweight='semibold',
                    va='center')


def tag(ax, xy, text, dx, dy, color=None, fontsize=8.5, ha='left'):
    ax.annotate(text, xy, xytext=(dx, dy), textcoords='offset points', fontsize=fontsize, ha=ha, va='center',
                color=color or fs.C['ink'], bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=color or fs.C['muted'], lw=0.6, alpha=0.92),
                arrowprops=dict(arrowstyle='-', color=color or fs.C['muted'], lw=0.8, shrinkA=0, shrinkB=2))


def save(fig, name):
    p = OUT / f'{name}.png'
    fig.savefig(p, dpi=110, bbox_inches='tight', pad_inches=0.02)
    plt.close(fig)
    print(p, p.stat().st_size // 1024, 'KB')


def note(ax, xy, text, at, color=None, fontsize=9, ha='left'):
    """Label at pixel position `at`, leader line to `xy`."""
    ax.annotate(text, xy, xytext=at, textcoords='data', fontsize=fontsize, ha=ha, va='center', color=color or fs.C['ink'],
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec=color or fs.C['muted'], lw=0.7),
                arrowprops=dict(arrowstyle='-', color=color or fs.C['muted'], lw=0.9, shrinkA=0, shrinkB=3))


def joints_figure():
    """Front-left three-quarter view; joint origins drawn on top of the opaque model (the hip joints sit inside the trunk)."""
    m, d, r = setup(720, 1280)
    base = body(m, d, 'base')
    img, P = render(m, d, r, base + [0.02, 0, -0.12], 1.4, -125, -24, fovy=35)
    fig, ax = show(img, 10)
    col = dict(hip=fs.C['bad'], thigh=fs.C['est'], calf=fs.C['good'])
    for leg in URDF:
        for part in ('hip', 'thigh', 'calf'):
            ax.plot(*P(body(m, d, f'{leg}_{part}')), 'o', ms=7, mfc=col[part], mec='white', mew=1.2)
        ax.plot(*P(foot(m, d, leg)), 's', ms=6, mfc=fs.C['ink'], mec='white', mew=1.0)
    W, H = img.shape[1], img.shape[0]
    for leg, u in URDF.items():
        f = P(foot(m, d, leg))
        dx, dy = dict(FL=(-14, 34), FR=(14, 34), RL=(14, 34), RR=(45, -10))[leg]
        ax.annotate(u, f, xytext=(f[0] + dx, f[1] + dy), ha='center', fontsize=12,
                    fontweight='semibold', color=fs.C['ink'], arrowprops=dict(arrowstyle='-', color=fs.C['muted'], lw=0.8))
    for part, axis, at in (('hip', [1, 0, 0], (40, 130)), ('thigh', [0, 1, 0], (40, 260)), ('calf', [0, 1, 0], (40, 420))):
        p = body(m, d, f'FL_{part}')
        arrow3(ax, P, p, 0.1 * np.array(axis), col[part], lw=2.2)
        txt = dict(hip='lf_hip_joint, axis x\nabduction, ±1.0472 rad',
                   thigh='lf_upper_leg_joint, axis y\nhip pitch, -1.5708 to 3.4907 rad',
                   calf='lf_lower_leg_joint, axis y\nknee, -2.7227 to -0.83776 rad')[part]
        note(ax, P(p), txt, at, color=col[part])
    note(ax, P(foot(m, d, 'FL')), 'lf_foot_joint, fixed\nfoot contact point', (40, 600))
    for v, n, c in (([0.15, 0, 0], 'x', fs.C['bad']), ([0, 0.15, 0], 'y', fs.C['good']), ([0, 0, 0.15], 'z', fs.C['est'])):
        arrow3(ax, P, base, np.array(v), c, n, fontsize=11)
    ax.plot(*P(base), 'P', ms=9, mfc=fs.C['ink'], mec='white')
    note(ax, P(base), 'base_link (trunk origin)\nx forward, y left, z up', (W - 330, 90))
    save(fig, 'c06-go2-joints')


def dim(ax, a, b, text, color, side=(0, 9), ha='center'):
    ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='<|-|>', color=color, lw=1.3, shrinkA=0, shrinkB=0, mutation_scale=9))
    ax.annotate(text, (np.asarray(a) + b) / 2, xytext=side, textcoords='offset points', ha=ha, va='center', color=color, fontsize=9.5,
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.85))


def dimensions_figure():
    """Left side and front views, all joint angles zero, with the URDF offsets drawn between joint origins."""
    m, d, r = setup(600, 760)
    d.qpos[:] = 0; d.qpos[2] = 0.426 + 0.022; d.qpos[3] = 1          # all joint angles zero: legs straight down
    mujoco.mj_forward(m, d)
    base = body(m, d, 'base')
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.6), gridspec_kw=dict(width_ratios=[1.3, 1]))
    J = lambda n: body(m, d, n)  # noqa: E731
    # side view from +y: x points left in the image
    ax = axs[0]
    img, P = render(m, d, r, base + [0, 0, -0.2], 2.3, -90, -3, fovy=22)
    ax.imshow(img); ax.set_xlim(0, img.shape[1]); ax.set_ylim(img.shape[0], 0); ax.axis('off')
    ax.set_title('left side (from +y), all joint angles zero')
    for n in ('FL_hip', 'RL_hip', 'FL_thigh', 'FL_calf', 'RL_calf'):
        ax.plot(*P(J(n)), 'o', ms=6, mfc=fs.C['plan'], mec='white')
    ax.plot(*P(base), 'P', ms=9, mfc=fs.C['ink'], mec='white')
    up = np.array([0, -60])
    dim(ax, P(base) + up, P(J('FL_hip')) + up, '0.1934 m', fs.C['bad'])
    dim(ax, P(base) + up, P(J('RL_hip')) + up, '0.1934 m', fs.C['bad'])
    for n in ('FL_hip', 'RL_hip'):
        ax.plot(*np.array([P(J(n)), P(J(n)) + up]).T, ':', color=fs.C['bad'], lw=0.9)
    ax.plot(*np.array([P(base), P(base) + up]).T, ':', color=fs.C['bad'], lw=0.9)
    ax.annotate('base_link', P(base), xytext=(0, 16), textcoords='offset points', ha='center', fontsize=9)
    lft = np.array([-62, 0])
    dim(ax, P(J('FL_thigh')) + lft, P(J('FL_calf')) + lft, 'thigh\n0.213 m', fs.C['est'], side=(-8, 0), ha='right')
    dim(ax, P(J('FL_calf')) + lft, P(J('FL_calf') + [0, 0, -0.213]) + lft, 'calf\n0.213 m', fs.C['est'], side=(-8, 0), ha='right')
    rgt = np.array([55, 0])
    dim(ax, P(J('RL_hip')) + rgt, P(J('RL_calf') + [0, 0, -0.213]) + rgt, '0.426 m\nbase_link to\nfoot point', fs.C['gt'],
        side=(8, 0), ha='left')
    # front view from +x: +y points right in the image
    ax = axs[1]
    img, P = render(m, d, r, base + [0, 0, -0.14], 2.0, 180, -3, fovy=22)
    ax.imshow(img); ax.set_xlim(0, img.shape[1]); ax.set_ylim(img.shape[0], 0); ax.axis('off')
    ax.set_title('front (from +x)')
    for n in ('FL_hip', 'FR_hip', 'FL_thigh', 'FR_thigh'):
        ax.plot(*P(J(n)), 'o', ms=6, mfc=fs.C['plan'], mec='white')
    ax.plot(*P(base), 'P', ms=9, mfc=fs.C['ink'], mec='white')
    for side, hip, th in ((1, 'FL_hip', 'FL_thigh'), (-1, 'FR_hip', 'FR_thigh')):
        y1, y2 = np.array([0, -95]), np.array([0, -130])
        dim(ax, P(base) + y1, P(J(hip)) + y1, '0.0465', fs.C['bad'], side=(side * 42, 0))
        dim(ax, P(J(hip)) + y2, P(J(th)) + y2, '0.0955', fs.C['est2'], side=(0, 11))
        for n in (hip, th):
            ax.plot(*np.array([P(J(n)), P(J(n)) + y2]).T, ':', color=fs.C['muted'], lw=0.8)
    ax.plot(*np.array([P(base), P(base) + [0, -95]]).T, ':', color=fs.C['muted'], lw=0.8)
    ft = P(foot(m, d, 'FL'))[1] + 50
    ax.annotate('lf, lh  (+y)', (P(J('FL_thigh'))[0], ft), ha='center', fontsize=9.5, color=fs.C['ink'])
    ax.annotate('rf, rh  (−y)', (P(J('FR_thigh'))[0], ft), ha='center', fontsize=9.5, color=fs.C['ink'])
    save(fig, 'c06-go2-dimensions')


def sensors_figure():
    """Side view of the robot with the sensor frames of go2_nav.xacro, the lowest lidar beam and the camera's vertical field of view.
    Heights: base_link at 0.216 m above the floor while standing (chapter 7), base_footprint 0.426 m below base_link (chapter 5)."""
    m, d, r = setup(720, 1280)
    hb = 0.216
    stand(m, d, hb)
    base = body(m, d, 'base')
    img, P = render(m, d, r, [0.42, base[1], 0.1], 3.2, 90, -2, fovy=30, far=3.6)
    img[int(P([base[0], base[1], 0])[1]):] = 236                           # section view: everything below the floor is ground
    fig, ax = show(img, 10)
    lidar = base + [0, 0, 0.10]; cam = base + [0.28, 0, 0.03]
    # lidar beams: 16 rows from -15 to +15 degrees (2 degree spacing)
    for el in np.radians(np.arange(-15, 16, 2)):
        L = 1.75
        e = lidar + L * np.array([np.cos(el), 0, np.sin(el)])
        if e[2] < 0:
            e = lidar + (lidar[2] / -np.sin(el)) * np.array([np.cos(el), 0, np.sin(el)])
        ax.plot(*np.array([P(lidar), P(e)]).T, '-', color=fs.C['est'], lw=0.6, alpha=0.7)
    hit = lidar + (lidar[2] / np.sin(np.radians(15))) * np.array([np.cos(np.radians(15)), 0, -np.sin(np.radians(15))])
    ax.plot(*P(hit), 'o', ms=6, color=fs.C['bad'])
    tag(ax, P(hit), f'lowest beam (-15°) meets the floor\n{hit[0] - lidar[0]:.2f} m ahead of the lidar', 10, 40, color=fs.C['bad'])
    # camera vertical field of view: 1.2 rad horizontal over 640 x 360 px
    vf = 2 * np.arctan(np.tan(0.6) * 360 / 640)
    for s in (-1, 1):
        e = cam + 1.55 * np.array([np.cos(vf / 2), 0, s * np.sin(vf / 2)])
        if e[2] < 0:
            t = cam[2] / np.sin(vf / 2); e = cam + t * np.array([np.cos(vf / 2), 0, -np.sin(vf / 2)])
        ax.plot(*np.array([P(cam), P(e)]).T, '-', color=fs.C['plan'], lw=1.3)
    tag(ax, P(cam + [0.95, 0, 0.42]), f'camera vertical FOV {np.degrees(vf):.1f}°\n(1.2 rad horizontal, 640 × 360)', 20, 18, color=fs.C['plan'])
    for p, txt, dx, dy in ((lidar, 'lidar_link  (0, 0, 0.10) from base_link', -60, 45),
                           (cam, 'camera_link  (0.28, 0, 0.03)', 30, 70),
                           (base, 'base_link', -120, 10)):
        ax.plot(*P(p), 'o', ms=6, mfc=fs.C['ink'], mec='white')
        tag(ax, P(p), txt, dx, dy)
    # heights
    x0 = base[0] - 0.98
    fp = np.array([x0, base[1], base[2] - 0.426])
    for z0, z1, txt, c in ((0, base[2], f'{hb:.3f} m\nmeasured\nstanding height', fs.C['gt']),
                           (fp[2], base[2], '0.426 m\nbase_footprint\nto base_link', fs.C['bad'])):
        xx = x0 if c == fs.C['bad'] else base[0] - 0.56
        a, b = P([xx, base[1], z0]), P([xx, base[1], z1])
        ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='<->', color=c, lw=1.4, shrinkA=0, shrinkB=0))
        ax.annotate(txt, (a + b) / 2, xytext=(-5, 0), textcoords='offset points', ha='right', va='center', color=c, fontsize=8.5)
    ax.plot(*np.array([P([x0 - 0.1, base[1], fp[2]]), P([base[0] + 0.3, base[1], fp[2]])]).T, '--', color=fs.C['bad'], lw=1.0)
    ax.annotate('base_footprint (0.209 m below the floor)', P([base[0] + 0.3, base[1], fp[2]]), xytext=(6, 0),
                textcoords='offset points', va='center', fontsize=8.5, color=fs.C['bad'])
    ax.plot(*np.array([P([x0 - 0.1, base[1], 0]), P([base[0] + 2.4, base[1], 0])]).T, '-', color=fs.C['ink'], lw=0.8)
    ax.annotate('floor', P([base[0] + 2.4, base[1], 0]), xytext=(4, 0), textcoords='offset points', ha='left', va='center', fontsize=8.5)
    ax.set_ylim(P([0, base[1], -0.3])[1], P([0, base[1], 0.75])[1])
    save(fig, 'c06-sensor-mounts')
    return base[2]


if __name__ == '__main__':
    which = sys.argv[1:] or ['joints', 'dimensions', 'sensors']
    for w in which:
        print(w, globals()[f'{w}_figure']())
