"""MuJoCo renders of the Go2 for chapter 5 (frames and TF2): annotated frame figure, to-scale side view of the base_footprint
error, and a short clip of the leg frames moving. Run from the repository root:
    MUJOCO_GL=glfw .venv-modules/bin/python docs/img/c04-render_go2.py
Writes docs/img/c04-go2-frames.png, docs/img/c04-footprint-error.png and media/c04-go2-frames.mp4.
The model is the MuJoCo Menagerie Go2, not the Gazebo URDF; sensor frames are placed at their URDF offsets from base_link
(go2_nav.xacro), taken here as the Menagerie `base` body origin. The pose is kinematic (mj_forward, no simulation)."""
import io
import pathlib
import subprocess
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'modules'))
import common  # noqa: E402
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402

IMG, MEDIA = ROOT / 'docs/img', ROOT / 'media'
BASE_Z = 0.217            # base_link height above the floor, Gazebo ground truth (chapter 5)
L, FOOT_R = 0.213, 0.022  # thigh and calf length, foot sphere radius (go2.xml)
LIDAR = np.array([0, 0, 0.10]); CAMERA = np.array([0.28, 0, 0.03])   # go2_nav.xacro, in base_link
URDF_LEG = dict(FL='lf', FR='rf', RL='lh', RR='rh')                    # Menagerie leg name -> URDF prefix (links.yaml)
R_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])                  # camera_link <- camera_optical (chapter 5)

model = mujoco.MjModel.from_xml_path(common.require_model(scene=True))
model.vis.global_.offwidth, model.vis.global_.offheight = 1920, 1080
model.mat_reflectance[:] = 0                                            # no floor mirror image under the robot


def see_through(alpha):
    """Robot materials semi-transparent so frames inside the links stay visible (floor material untouched)."""
    floor = model.material('groundplane').id
    for i in range(model.nmat):
        if i != floor:
            model.mat_rgba[i, 3] = alpha
data = mujoco.MjData(model)


def stance(base_z=BASE_Z):
    """Feet straight below the thigh joints with the foot sphere touching the floor: thigh a, calf -2a."""
    a = np.arccos((base_z - FOOT_R) / (2 * L))
    q = np.zeros(model.nq); q[2] = base_z; q[3] = 1
    q[7:] = np.tile([0, a, -2 * a], 4)
    return q


def body(name):
    i = model.body(name).id
    return data.xpos[i].copy(), data.xmat[i].reshape(3, 3).copy()


def add_axes(scn, p, R, length=0.08, width=0.006, alpha=1.0):
    for k, rgba in enumerate(([0.85, 0.15, 0.15, alpha], [0.1, 0.65, 0.2, alpha], [0.1, 0.35, 0.9, alpha])):
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(rgba, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, p, p + length * R[:, k])
        scn.ngeom += 1


def add_sphere(scn, p, r, rgba):
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([r, 0, 0]), p, np.eye(3).ravel(), np.array(rgba, np.float32))
    scn.ngeom += 1


def frames_to_draw():
    pb, Rb = body('base')
    out = [('base_link', pb, Rb), ('lidar_link', pb + Rb @ LIDAR, Rb), ('camera_link', pb + Rb @ CAMERA, Rb)]
    for leg in ('FL',):
        for mj, urdf in (('hip', 'hip_link'), ('thigh', 'upper_leg_link'), ('calf', 'lower_leg_link')):
            p, R = body(f'{leg}_{mj}'); out.append((f'{URDF_LEG[leg]}_{urdf}', p, R))
        p, R = body(f'{leg}_calf'); out.append((f'{URDF_LEG[leg]}_foot_link', p + R @ np.array([0, 0, -L]), R))
    for leg in ('FR', 'RL', 'RR'):
        p, R = body(f'{leg}_hip'); out.append((f'{URDF_LEG[leg]}_hip_link', p, R))
    return out


class Cam:
    def __init__(self, lookat, distance, azimuth, elevation, ortho=False):
        self.c = mujoco.MjvCamera(); self.c.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.c.lookat[:] = lookat; self.c.distance = distance; self.c.azimuth = azimuth; self.c.elevation = elevation
        self.c.orthographic = int(ortho)

    def project(self, gl, P, W, H):
        """World points -> pixel (u, v) with MuJoCo's own glcamera frustum (top-left origin). The scene holds two eye
        cameras offset by +-ipd/2; the mono image is rendered from their mean."""
        glcam = gl[0]
        pos = (np.array(gl[0].pos) + np.array(gl[1].pos)) / 2
        fwd, up = np.array(glcam.forward), np.array(glcam.up)
        right = np.cross(fwd, up)
        top, bot, near = glcam.frustum_top, glcam.frustum_bottom, glcam.frustum_near
        hw = glcam.frustum_width or (top - bot) / 2 * W / H
        P = np.atleast_2d(P) - pos
        x, y, z = P @ right, P @ up, P @ fwd
        if not glcam.orthographic:
            x, y = x / z * near, y / z * near
        u = (x - (glcam.frustum_center - hw)) / (2 * hw) * W
        v = (top - y) / (top - bot) * H
        return np.c_[u, v]


def render(cam, W, H, extra=None, opt=None):
    r = mujoco.Renderer(model, H, W)
    r.update_scene(data, camera=cam.c, scene_option=opt)
    if extra:
        extra(r.scene)
    img = r.render().copy()
    gl = [r.scene.camera[0], r.scene.camera[1]]
    proj = lambda P: cam.project(gl, P, W, H)  # noqa: E731
    r.close()
    return img, proj


def save_png(fig, path, colors=None):
    """PNG through PIL; `colors` quantises to a palette to stay under the 400 KB budget for photographic renders."""
    buf = io.BytesIO(); fig.savefig(buf, format='png', dpi=150, bbox_inches='tight', pad_inches=0.04); plt.close(fig)
    im = Image.open(buf).convert('RGB')
    if colors:
        im = im.quantize(colors=colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    im.save(path, optimize=True)
    print(path, path.stat().st_size // 1024, 'KB', im.size)


def go2_frames():
    data.qpos[:] = stance(); mujoco.mj_forward(model, data)
    fr = frames_to_draw()
    cam = Cam(lookat=[0.05, 0.0, 0.17], distance=1.05, azimuth=-125, elevation=-18)

    def extra(scn):
        for name, p, R in fr:
            add_axes(scn, p, R, 0.13 if name == 'base_link' else 0.085, 0.008 if name == 'base_link' else 0.006)
        add_sphere(scn, fr[1][1], 0.018, [0.79, 0.52, 0.06, 0.9]); add_sphere(scn, fr[2][1], 0.014, [0.79, 0.52, 0.06, 0.9])

    W, H = 1500, 900
    see_through(0.3)
    img, proj = render(cam, W, H, extra)
    see_through(1.0)
    fig, ax = plt.subplots(figsize=(7.6, 4.56)); ax.imshow(img); ax.axis('off'); ax.grid(False)
    offs = {'base_link': (-55, 55), 'lidar_link': (0, 45), 'camera_link': (-50, 45), 'lf_hip_link': (-80, -25),
            'lf_upper_leg_link': (-75, -60), 'lf_lower_leg_link': (85, -10), 'lf_foot_link': (70, -25),
            'rf_hip_link': (-85, 5), 'lh_hip_link': (75, 25), 'rh_hip_link': (40, 50)}
    for name, p, R in fr:
        u, v = proj(p)[0]
        ax.annotate(name, (u, v), xytext=offs[name], textcoords='offset points', fontsize=8, color=fs.C['ink'],
                    ha='center', va='center', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=fs.C['muted'], lw=0.5),
                    arrowprops=dict(arrowstyle='-', color=fs.C['ink'], lw=0.7, shrinkA=0, shrinkB=1))
    ax.text(0.01, 0.02, 'axes: x red, y green, z blue', transform=ax.transAxes, fontsize=8, color='white')
    save_png(fig, IMG / 'c04-go2-frames.png', colors=200)


def footprint_error():
    data.qpos[:] = stance(); mujoco.mj_forward(model, data)
    pb, _ = body('base')
    # side view from -y looking along +y, so x points right in the image
    cam = Cam(lookat=[0.45, 0.0, 0.07], distance=4.0, azimuth=90, elevation=0, ortho=True)
    model.vis.global_.fovy, model.vis.global_.orthographic = 0.80, 1     # orthographic: vertical extent in metres
    W, H = 1600, 640
    img, proj = render(cam, W, H)
    model.vis.global_.fovy, model.vis.global_.orthographic = 45, 0
    # pixel <-> metres along x (world) and z, from two projected points
    (u0, v0), (u1, v1) = proj(np.array([[0, 0, 0], [1, 0, 1]]))
    sx, sz = u1 - u0, v1 - v0
    x_left, x_right = -u0 / sx, (W - u0) / sx
    z_top, z_bot = -v0 / sz, (H - v0) / sz
    bg = img[2, 2].astype(int)                       # haze colour behind the robot (the floor is edge-on at elevation 0)
    rgba = np.dstack([img, np.where(np.abs(img.astype(int) - bg).sum(2) < 12, 0, 255).astype(np.uint8)])
    fig, ax = plt.subplots(figsize=(8.2, 4.2))
    ax.imshow(rgba, extent=(x_left, x_right, z_bot, z_top), zorder=0)
    ax.set_xlim(-0.45, 1.45); ax.set_ylim(-0.27, 0.40); ax.set_aspect('equal'); ax.grid(False)
    ax.axhline(0, color=fs.C['ink'], lw=1.0)
    bz = pb[2]; lz = bz + LIDAR[2]; fpz = bz - 0.426
    ax.axhline(fpz, color=fs.C['bad'], lw=1.0, ls='--')
    ax.text(1.43, fpz - 0.012, 'base_footprint as TF has it: 0.426 m below base_link', ha='right', va='top',
            color=fs.C['bad'], fontsize=8.5)
    ax.text(1.43, -0.012, 'floor (where base_footprint should be)', ha='right', va='top', color=fs.C['ink'], fontsize=8.5)
    ax.plot([pb[0], 0.80], [bz, bz], ':', color=fs.C['muted'], lw=0.8)
    # lowest lidar beam, 15 deg down
    d = lz / np.tan(np.radians(15))
    ax.plot([pb[0], pb[0] + d], [lz, 0], color=fs.C['plan'], lw=1.2)
    ax.plot([pb[0] + d], [0], 'o', color=fs.C['plan'], ms=5)
    ax.text(pb[0] + 0.62 * d, 0.62 * lz * 0.38 + 0.19, f'lowest beam, 15° down,\nreaches the floor {d:.2f} m ahead',
            fontsize=8.5, color=fs.C['plan'], ha='left')
    for z, name in ((bz, 'base_link'), (lz, 'lidar_link')):
        ax.plot([pb[0]], [z], 'o', color=fs.C['est'], ms=4, zorder=3)
        ax.text(pb[0] - 0.03, z + 0.02, name, ha='right', va='bottom', fontsize=8.5, color=fs.C['est'],
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.85))

    def dim(x, z0, z1, text, color, zt=None):
        ax.annotate('', (x, z1), (x, z0), arrowprops=dict(arrowstyle='<->', color=color, lw=0.9, shrinkA=0, shrinkB=0))
        ax.text(x + 0.012, (z0 + z1) / 2 if zt is None else zt, text, ha='left', va='center', fontsize=8.5, color=color)
    dim(0.45, 0, bz, f'{bz:.3f} m,\nground truth', fs.C['gt'], zt=0.05)
    dim(0.78, fpz, bz, "0.426 m: CHAMP's height\nwith no foot contacts", fs.C['bad'], zt=-0.11)
    dim(1.22, fpz, 0, '0.209 m', fs.C['bad'], zt=-0.11)
    ax.set_xlabel('x in base_link [m]'); ax.set_ylabel('height above floor [m]')
    ax.set_title('Side view to scale: the floor sits 0.21 m above base_footprint')
    save_png(fig, IMG / 'c04-footprint-error.png', colors=128)


def frames_clip(seconds=10.0, fps=25, W=1280, H=720):
    """Legs sweep through a kinematic trot cycle (diagonal pairs in phase) while the trunk stays put: the leg frames move
    with the joints (dynamic, from /joint_states in the ROS stack), the sensor frames stay fixed to base_link (static)."""
    cam = Cam(lookat=[0.05, 0.0, 0.17], distance=1.25, azimuth=-125, elevation=-16)
    q0 = stance(); r = mujoco.Renderer(model, H, W); see_through(0.3)
    out = MEDIA / 'c04-go2-frames.mp4'
    ff = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(fps),
                           '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '28', '-preset', 'slow',
                           '-movflags', '+faststart', str(out)], stdin=subprocess.PIPE)
    f_gait = 1.0                          # Hz, 0.5 s per half cycle
    for k in range(int(seconds * fps)):
        t = k / fps
        q = q0.copy()
        for i, ph in enumerate((0, np.pi, np.pi, 0)):      # FL, FR, RL, RR: diagonal pairs share a phase
            s = np.sin(2 * np.pi * f_gait * t + ph)
            lift = max(s, 0.0)
            q[7 + 3 * i + 1] += 0.25 * np.cos(2 * np.pi * f_gait * t + ph) - 0.35 * lift
            q[7 + 3 * i + 2] += 0.7 * lift
        cam.c.azimuth = -125 + 40 * np.sin(2 * np.pi * t / seconds)
        data.qpos[:] = q; mujoco.mj_forward(model, data)
        fr = frames_to_draw()
        r.update_scene(data, camera=cam.c)
        for name, p, R in fr:
            add_axes(r.scene, p, R, 0.14 if name == 'base_link' else 0.09, 0.008 if name == 'base_link' else 0.006)
        add_sphere(r.scene, fr[1][1], 0.018, [0.79, 0.52, 0.06, 0.9]); add_sphere(r.scene, fr[2][1], 0.014, [0.79, 0.52, 0.06, 0.9])
        for leg in ('FR', 'RL', 'RR'):                     # the other legs' full chains too
            for mj in ('thigh', 'calf'):
                p, R = body(f'{leg}_{mj}'); add_axes(r.scene, p, R, 0.08, 0.005)
            p, R = body(f'{leg}_calf'); add_axes(r.scene, p + R @ np.array([0, 0, -L]), R, 0.08, 0.005)
        ff.stdin.write(r.render().tobytes())
    ff.stdin.close(); ff.wait(); r.close(); see_through(1.0)
    print(out, out.stat().st_size // 1024, 'KB')


if __name__ == '__main__':
    what = sys.argv[1:] or ['frames', 'footprint', 'clip']
    if 'frames' in what: go2_frames()
    if 'footprint' in what: footprint_error()
    if 'clip' in what: frames_clip()
