"""Shared MuJoCo harness for the modules: load the Go2 on a floor, stand it, read ground truth, step a controller, record video.

Conventions (MuJoCo's, stated once so modules do not re-derive them):
  qpos = [p_base (3, world), quat_base (4, w x y z), q_joints (12)];  qvel = [v_base (3, world), w_base (3, body), dq (12)]
  Joint and actuator order is LEGS x JOINTS = FL hip/thigh/calf, FR ..., RL ..., RR ...
  Foot positions are the centres of the collision spheres (radius FOOT_RADIUS); contact forces are those the ground applies
  to each foot, in the world frame, extracted with mj_contactForce.
Rendering uses MUJOCO_GL=glfw (offscreen on an X display). Without a display the Recorder disables itself and says so.
usage: python modules/sim.py      # demo: rise from a crouch, hold, write modules/assets/demo/stand.mp4 and stand.png"""
import os
import pathlib
import shutil
import subprocess
import sys
import warnings
from dataclasses import dataclass

os.environ.setdefault('MUJOCO_GL', 'glfw')            # must precede `import mujoco`; egl/osmesa do not work on this machine
import numpy as np                                     # noqa: E402
import mujoco                                          # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from common import JOINTS, LEGS, require_model  # noqa: E402

Q_STAND = np.tile([0.0, 0.9, -1.8], 4)                # Menagerie 'home' keyframe joint angles, rad
FOOT_RADIUS = 0.022                                    # m, go2.xml default class "foot"
MAX_W, MAX_H, MAX_SECONDS = 1280, 720, 25.0            # video limits from MODULE_SPEC.md


def load(scene=True):
    """MjModel and MjData for the Go2 (with floor when scene=True); offscreen buffer enlarged to 1280x720."""
    m = mujoco.MjModel.from_xml_path(require_model(scene=scene))
    m.vis.global_.offwidth, m.vis.global_.offheight = max(m.vis.global_.offwidth, MAX_W), max(m.vis.global_.offheight, MAX_H)
    return m, mujoco.MjData(m)


@dataclass
class Index:
    legs: dict          # leg -> {'qpos': (3,), 'qvel': (3,), 'act': (3,), 'foot': geom id, 'calf': body id}
    qpos: np.ndarray    # (12,) joint qpos addresses, LEGS x JOINTS order
    qvel: np.ndarray    # (12,) joint dof addresses
    act: np.ndarray     # (12,) actuator ids
    feet: np.ndarray    # (4,) foot geom ids
    base: int           # base body id


def index(m):
    """Name-resolved index maps; asserts that each actuator drives the joint of the same name."""
    legs = {}
    for leg in LEGS:
        j = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{p}_joint') for p in JOINTS]
        a = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{leg}_{p}') for p in JOINTS]
        assert min(j + a) >= 0 and all(m.actuator_trnid[ai, 0] == ji for ai, ji in zip(a, j))
        legs[leg] = dict(qpos=m.jnt_qposadr[j].copy(), qvel=m.jnt_dofadr[j].copy(), act=np.array(a),
                         foot=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, leg),
                         calf=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{leg}_calf'))
    cat = lambda k: np.concatenate([legs[l][k] for l in LEGS])  # noqa: E731
    return Index(legs, cat('qpos'), cat('qvel'), cat('act'), np.array([legs[l]['foot'] for l in LEGS]),
                 mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base'))


def place_on_ground(m, d, q=Q_STAND, xy=(0.0, 0.0), clearance=0.0):
    """Upright base at xy, joints at q, base height chosen so the lowest foot sphere touches z = clearance. Zero velocity."""
    idx = index(m)
    mujoco.mj_resetData(m, d)
    d.qpos[:2], d.qpos[2], d.qpos[3:7] = xy, 0.5, (1, 0, 0, 0)
    d.qpos[idx.qpos] = q
    mujoco.mj_kinematics(m, d)
    d.qpos[2] -= d.geom_xpos[idx.feet, 2].min() - FOOT_RADIUS - clearance
    mujoco.mj_forward(m, d)


def joint_pd(m, q_des, kp=60.0, kd=2.0, tau_ff=None):
    """Controller callback tau = kp (q_des - q) - kd dq (+ tau_ff); q_des may be an array or a function of time."""
    idx = index(m)

    def ctrl(m, d):
        qd = q_des(d.time) if callable(q_des) else q_des
        tau = kp * (qd - d.qpos[idx.qpos]) - kd * d.qvel[idx.qvel]
        return tau if tau_ff is None else tau + tau_ff
    return ctrl


def stand(m, d, settle=1.0, kp=60.0, kd=2.0):
    """Nominal standing pose on the floor, then `settle` seconds under joint PD to Q_STAND. Returns the base height."""
    place_on_ground(m, d)
    if settle > 0:
        run(m, d, joint_pd(m, Q_STAND, kp, kd), settle)
    return float(d.qpos[2])


def run(m, d, controller=None, duration=1.0, ctrl_dt=None, on_step=None):
    """Fixed-timestep loop of mj_step. controller(m, d) -> 12 torques (actuator order), called every ctrl_dt (default: every
    physics step, zero-order hold in between) and clipped to the actuator ctrlrange. on_step(m, d) runs after every step; its
    non-None return values are collected and returned as a list."""
    dt = m.opt.timestep
    dec = max(1, round((ctrl_dt or dt) / dt))
    lo, hi = m.actuator_ctrlrange.T
    idx = index(m) if controller else None
    out = []
    for k in range(round(duration / dt)):
        if controller and k % dec == 0:
            tau = controller(m, d)
            if tau is not None:
                d.ctrl[idx.act] = np.clip(tau, lo[idx.act], hi[idx.act])
        mujoco.mj_step(m, d)
        if on_step:
            r = on_step(m, d)
            if r is not None:
                out.append(r)
    return out


# ---- ground truth --------------------------------------------------------------------------------------------------------

def base_pose(d):
    """(p (3,) world, quat (4,) w x y z, R (3, 3) body -> world) of the base."""
    R = np.empty(9); mujoco.mju_quat2Mat(R, d.qpos[3:7])
    return d.qpos[:3].copy(), d.qpos[3:7].copy(), R.reshape(3, 3)


def base_velocity(d):
    """(v (3,) world frame, w (3,) body frame) of the base, read from the free joint's qvel."""
    return d.qvel[:3].copy(), d.qvel[3:6].copy()


def foot_positions(m, d):
    """(4, 3) world positions of the foot sphere centres, LEGS order (subtract FOOT_RADIUS in z for the sole)."""
    return d.geom_xpos[index(m).feet].copy()


def foot_forces(m, d):
    """(4, 3) world-frame force the environment applies to each foot, summed over its contacts, LEGS order. Zero in swing."""
    feet = list(index(m).feet)
    F, f6 = np.zeros((4, 3)), np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        for g, sign in ((c.geom2, 1.0), (c.geom1, -1.0)):      # contact normal points from geom1 to geom2
            if g in feet:
                mujoco.mj_contactForce(m, d, i, f6)
                F[feet.index(g)] += sign * c.frame.reshape(3, 3).T @ f6[:3]
    return F


def contacts(m, d, threshold=1.0):
    """(4,) bool, foot normal force above threshold (N)."""
    return foot_forces(m, d)[:, 2] > threshold


# ---- rendering -----------------------------------------------------------------------------------------------------------

class Recorder:
    """Offscreen frames from a camera tracking the base. Use as run(..., on_step=rec) to capture at `fps` of sim time, streamed
    to ffmpeg (H.264, yuv420p). still(d, path) writes a PNG. rec.ok is False (and every call a no-op) when rendering or ffmpeg
    is unavailable. rec.opt is the MjvOption: e.g. rec.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 1."""

    def __init__(self, m, mp4=None, width=960, height=540, fps=30, distance=1.3, azimuth=135.0, elevation=-18.0,
                 track='base', contacts=False):
        assert width <= MAX_W and height <= MAX_H and width % 2 == 0 and height % 2 == 0
        self.m, self.w, self.h, self.fps, self.mp4 = m, width, height, fps, mp4 and pathlib.Path(mp4)
        self.n, self.proc, self.renderer = 0, None, None
        try:
            self.renderer = mujoco.Renderer(m, height, width)
        except Exception as e:                                   # no display, no GL: skip, do not crash the experiment
            warnings.warn(f'Recorder disabled, offscreen rendering unavailable ({type(e).__name__}: {e})')
        if mp4 and not shutil.which('ffmpeg'):
            warnings.warn('ffmpeg not found: MP4 output disabled'); self.mp4 = None
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.cam.trackbodyid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, track)
        self.cam.distance, self.cam.azimuth, self.cam.elevation = distance, azimuth, elevation
        self.opt = mujoco.MjvOption()
        if contacts:
            self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 1

    @property
    def ok(self):
        return self.renderer is not None

    def render(self, d):
        """One RGB frame (h, w, 3) uint8, or None without a renderer."""
        if not self.ok:
            return None
        self.renderer.update_scene(d, camera=self.cam, scene_option=self.opt)
        return self.renderer.render()

    def __call__(self, m, d):
        if not (self.ok and self.mp4) or d.time + 1e-9 < self.n / self.fps:
            return None
        if self.n >= MAX_SECONDS * self.fps:
            if self.n == MAX_SECONDS * self.fps:
                warnings.warn(f'video capped at {MAX_SECONDS} s'); self.n += 1
            return None
        if self.proc is None:
            self.mp4.parent.mkdir(parents=True, exist_ok=True)
            self.proc = subprocess.Popen(
                ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{self.w}x{self.h}',
                 '-r', str(self.fps), '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '23', '-pix_fmt', 'yuv420p',
                 '-movflags', '+faststart', str(self.mp4)], stdin=subprocess.PIPE)
        self.proc.stdin.write(self.render(d).tobytes()); self.n += 1
        return None

    def still(self, d, path):
        """Write the current view as PNG; returns the path, or None without a renderer."""
        img = self.render(d)
        if img is None:
            return None
        import matplotlib.image
        path = pathlib.Path(path); path.parent.mkdir(parents=True, exist_ok=True)
        matplotlib.image.imsave(path, img)
        return path

    def close(self):
        """Finish the MP4 (the renderer stays usable for still()); returns its path, or None if nothing was written."""
        if self.proc is None:
            return None
        self.proc.stdin.close()
        ok = self.proc.wait() == 0
        self.proc = None
        if not ok:
            warnings.warn('ffmpeg failed')
        return self.mp4 if ok else None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        if self.renderer is not None:
            self.renderer.close(); self.renderer = None


def min_jerk(q0, q1, T):
    """q(t) on the minimum-jerk path from q0 to q1 over T seconds, then held at q1."""
    def q(t):
        s = np.clip(t / T, 0, 1)
        return q0 + (q1 - q0) * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
    return q


if __name__ == '__main__':
    out = pathlib.Path(__file__).resolve().parent / 'assets/demo'
    m, d = load()
    crouch = np.tile([0.0, 1.25, -2.6], 4)
    place_on_ground(m, d, crouch)
    mass = m.body_subtreemass[index(m).base]
    print(f'start: base z {d.qpos[2]:.4f} m (crouch), mass {mass:.4f} kg')
    with Recorder(m, out / 'stand.mp4', contacts=True) as rec:
        run(m, d, joint_pd(m, min_jerk(crouch, Q_STAND, 1.5)), 2.0, on_step=rec)   # rise over 1.5 s, settle to t = 2 s
        z2 = d.qpos[2]
        run(m, d, joint_pd(m, Q_STAND), 1.0, on_step=rec)                          # hold to t = 3 s
        video = rec.close()
        png = rec.still(d, out / 'stand.png')
    F = foot_forces(m, d)
    v, w = base_velocity(d)
    print(f'base z at t=2.0 s: {z2:.4f} m; final base z at t={d.time:.2f} s: {d.qpos[2]:.4f} m')
    print(f'sum foot Fz / (m g) = {F[:, 2].sum() / (mass * -m.opt.gravity[2]):.4f}; per foot Fz [N] = {np.round(F[:, 2], 2)}')
    print(f'|v_base| = {np.linalg.norm(v):.2e} m/s, |w_base| = {np.linalg.norm(w):.2e} rad/s, video {video}, still {png}')
    # self-check: static equilibrium means the ground carries the whole weight, vertically, with the base at rest
    assert abs(F[:, 2].sum() / (mass * -m.opt.gravity[2]) - 1) < 0.01 and np.abs(F[:, :2].sum(0)).max() < 0.5
    assert 0.22 < d.qpos[2] < 0.30 and np.linalg.norm(v) < 1e-2 and contacts(m, d).all()
