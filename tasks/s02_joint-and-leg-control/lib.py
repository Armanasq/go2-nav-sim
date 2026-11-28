"""Stage 2 shared code: the supported Go2, one joint-space torque controller, leg gravity torque and joint inertia from first
principles, step-response metrics, foot paths, and an IK that says why it rejects a target.

All twelve MuJoCo actuators are <motor> elements (torque, gear 1). Position and velocity control are loops written here that
compute a torque every 2 ms physics step; sim.run clips it to the ctrlrange (+-23.7 N m hip and thigh, +-45.43 N m knee)."""
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
MODULES = HERE.parents[1] / 'modules'
sys.path[:0] = [str(MODULES), str(MODULES / 'm04_kinematics')]
import mujoco  # noqa: E402
import sim  # noqa: E402  (sets MUJOCO_GL before mujoco is used for rendering)
from common import JOINTS, LEGS, axis_angle_R, quat_to_R, require_model  # noqa: E402
from kinematics import Leg  # noqa: E402

BASE_Z = 0.70                     # m, trunk height on the support; the lowest reachable foot centre is then about 0.27 m up
DT = 0.002                        # s, MuJoCo default timestep; the controller runs every step (500 Hz)
Q_HOLD = sim.Q_STAND.copy()       # (0, 0.9, -1.8) rad per leg: the standing angles, here with the feet in the air
KP, KD = 150.0, 2.0               # N m/rad, N m s/rad: default joint gains for the Cartesian tasks (2.08 onwards)


# ---- the supported robot -------------------------------------------------------------------------------------------------

def load_supported(z=BASE_Z):
    """Go2 with the base free joint deleted at compile time: the trunk becomes part of the world body at height z, so it
    cannot move or fall. A visual-only grey post (no collision) from the floor to the trunk shows the support in renders.
    Returns (m, d) with the legs at Q_HOLD and zero velocity."""
    s = mujoco.MjSpec.from_file(require_model(scene=True))
    base = s.body('base')
    s.delete(base.joints[0])                               # the <freejoint/>
    base.pos = [0, 0, z]
    for k in list(s.keys):                                 # the 'home' keyframe has 19 qpos values; the fixed model has 12
        s.delete(k)
    h = (z - 0.057) / 2                                    # 0.057 m: half-height of the trunk collision box
    s.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.02, h, 0], pos=[0, 0, h],
                         contype=0, conaffinity=0, rgba=[0.55, 0.57, 0.6, 1])
    m = s.compile()
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    d = mujoco.MjData(m)
    d.qpos[:] = Q_HOLD
    mujoco.mj_forward(m, d)
    return m, d


def base_origin(m):
    """World position of the base frame (fixed). Base orientation is the identity, so base-frame vectors are world vectors."""
    return m.body_pos[sim.index(m).base].copy()


# ---- leg dynamics terms from first principles ------------------------------------------------------------------------------

class LegModel(Leg):
    """m04 Leg plus the mass properties of its three bodies (read from the model, not computed by MuJoCo)."""

    def __init__(self, m, leg):
        super().__init__(m, leg)
        bid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{leg}_{p}') for p in JOINTS]
        dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{p}_joint')] for p in JOINTS]
        self.mass, self.com = m.body_mass[bid].copy(), m.body_ipos[bid].copy()
        self.I = [quat_to_R(m.body_iquat[b]) @ np.diag(m.body_inertia[b]) @ quat_to_R(m.body_iquat[b]).T for b in bid]
        self.armature, self.damping, self.frictionloss = m.dof_armature[dof], m.dof_damping[dof], m.dof_frictionloss[dof]
        self.g = m.opt.gravity.copy()                      # base frame = world frame on the support
        # sagittal reach rho = |thigh joint -> foot| as a function of the knee angle only (README, task 2.13)
        rho = [np.linalg.norm(self.fk(np.array([0, 0, q3])) - self.frames(np.array([0, 0, q3]))[0][1])
               for q3 in np.linspace(self.lo[2], self.hi[2], 2001)]
        self.rho_min, self.rho_max = float(min(rho)), float(max(rho))

    def bodies(self, q):
        """Per body after its hinge: joint origin o_i, axis z_i and rotation R_i, all in the base frame."""
        R, p, out = np.eye(3), np.zeros(3), []
        for (pos, Rb, axis), qi in zip(self.links, q):
            p = p + R @ pos; R = R @ Rb
            z = R @ axis
            R = R @ axis_angle_R(axis, qi)
            out.append((p.copy(), z, R.copy()))
        return out

    def gravity(self, q):
        """Joint torques that hold the leg still against gravity: tau_i = -z_i . sum_{b>=i} (c_b - o_i) x m_b g."""
        B = self.bodies(q)
        c = [p + R @ com for (p, _, R), com in zip(B, self.com)]
        return np.array([-B[i][1] @ sum(np.cross(c[b] - B[i][0], self.mass[b] * self.g) for b in range(i, 3))
                         for i in range(3)])

    def inertia(self, q, i):
        """Inertia about joint i with the later joints locked (MuJoCo's M_ii):
        armature_i + sum_{b>=i} z_i^T R_b I_b R_b^T z_i + m_b |z_i x (c_b - o_i)|^2."""
        B = self.bodies(q)
        o, z = B[i][0], B[i][1]
        J = self.armature[i]
        for b in range(i, 3):
            R = B[b][2]
            r = B[b][0] + R @ self.com[b] - o
            J += z @ R @ self.I[b] @ R.T @ z + self.mass[b] * np.cross(z, r) @ np.cross(z, r)
        return float(J)


def legs(m):
    return [LegModel(m, leg) for leg in LEGS]


# ---- joint-space torque controller -----------------------------------------------------------------------------------------

def hold_ref(q=Q_HOLD):
    q = np.asarray(q, float)
    return lambda t: (q, np.zeros(12))


def array_ref(Q, dt=DT):
    """Reference from joint angles sampled every dt (rows = samples, 12 columns); dq* by central differences."""
    Q = np.asarray(Q, float)
    dQ = np.gradient(Q, dt, axis=0)
    return lambda t: (Q[min(int(round(t / dt)), len(Q) - 1)], dQ[min(int(round(t / dt)), len(Q) - 1)])


def joint_controller(m, ref, kp=KP, kd=KD, gravity=True, damping_ff=True, override=None):
    """tau = kp (q* - q) + kd (dq* - dq) + b dq* + g(q).   kp, kd: scalars or (12,) arrays; ref(t) -> (q*, dq*).
    b dq* cancels the joint's passive damping along the reference; g(q) is LegModel.gravity. override(m, d) -> (mask (12,)
    bool, tau (12,)) replaces the torque of the masked joints (open-loop torque in 2.04, a velocity loop in 2.03)."""
    idx, L = sim.index(m), legs(m)
    b = m.dof_damping[idx.qvel]

    def ctrl(m, d):
        qd, dqd = ref(d.time)
        q, dq = d.qpos[idx.qpos], d.qvel[idx.qvel]
        tau = kp * (qd - q) + kd * (dqd - dq)
        if damping_ff:
            tau = tau + b * dqd
        if gravity:
            tau = tau + np.concatenate([Lg.gravity(q[3 * i:3 * i + 3]) for i, Lg in enumerate(L)])
        if override is not None:
            mask, tau_o = override(m, d)
            tau = np.where(mask, tau_o, tau)
        return tau
    return ctrl


def simulate(m, d, ctrl, T, rec=None):
    """Run T seconds; log time, joint angles, velocities, applied torques and the four foot-sphere centres (world, MuJoCo)."""
    idx = sim.index(m)

    def step(m, d):
        if rec is not None:
            rec(m, d)
        return d.time, d.qpos[idx.qpos].copy(), d.qvel[idx.qvel].copy(), d.ctrl[idx.act].copy(), d.geom_xpos[idx.feet].copy()
    t, q, dq, tau, feet = map(np.array, zip(*sim.run(m, d, ctrl, T, on_step=step)))
    return dict(t=t, q=q, dq=dq, tau=tau, feet=feet)


# ---- step response ---------------------------------------------------------------------------------------------------------

def step_metrics(t, y, y0, y1, band=0.05):
    """10-90 % rise time, percent overshoot, settling time into +-band of the step, final error. t starts at the step."""
    s = (np.asarray(y) - y0) / (y1 - y0)
    rise = t[np.argmax(s >= 0.9)] - t[np.argmax(s >= 0.1)] if s.max() >= 0.9 else float('nan')
    out = np.nonzero(np.abs(s - 1) > band)[0]
    settle = float('nan') if len(out) and out[-1] == len(s) - 1 else (t[out[-1] + 1] - t[0] if len(out) else 0.0)
    return dict(rise_s=float(rise), overshoot_pct=float(100 * max(0.0, s.max() - 1)), settling_s=float(settle),
                final_error_rad=float(abs(y[-1] - y1)))


def second_order_step(J, c, k, t):
    """Unit step of J y'' + c y' + k y = k u, closed form; also returns (zeta, wn)."""
    wn, zeta = np.sqrt(k / J), c / (2 * np.sqrt(k * J))
    t = np.asarray(t)
    if zeta < 1:
        wd = wn * np.sqrt(1 - zeta ** 2)
        y = 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta / np.sqrt(1 - zeta ** 2) * np.sin(wd * t))
    else:                                                   # ponytail: critically/over-damped by ODE, not a third formula
        from scipy.integrate import solve_ivp
        y = solve_ivp(lambda _, x: [x[1], (k - c * x[1] - k * x[0]) / J], (0, t[-1]), [0, 0], t_eval=t, rtol=1e-9).y[0]
    return y, float(zeta), float(wn)


def zeta_from_overshoot(os_pct):
    if os_pct <= 0:
        return None                                         # no overshoot: zeta >= 1 or friction ate it; not identifiable
    L = np.log(os_pct / 100)
    return float(-L / np.sqrt(np.pi ** 2 + L ** 2))


# ---- inverse kinematics with a reason ----------------------------------------------------------------------------------------

def ik_checked(L, target):
    """(q, None) for a reachable target (base frame), else (None, reason). Reasons, checked in this order:
    'inside_offset'  the target is closer to the hip axis than the thigh's lateral offset |d|
    'too_far'        sagittal distance rho above rho_max (knee at its straightest limit)
    'too_close'      rho below rho_min (knee folded to its limit)
    'joint_limit'    the distance is fine but every closed-form solution has the hip or thigh outside its range."""
    x, y, z = np.asarray(target, float) - L.hip
    r2 = y * y + z * z - L.d ** 2
    if r2 < 0:
        return None, 'inside_offset'
    rho = np.sqrt(x * x + r2)
    if rho > L.rho_max + 1e-9:
        return None, 'too_far'
    if rho < L.rho_min - 1e-9:
        return None, 'too_close'
    sols = L.ik_closed_form(target)
    return (sols[0], None) if len(sols) else (None, 'joint_limit')


def ik_closest(L, target, q_prev):
    """Closed-form solution nearest to q_prev (keeps the knee branch and avoids jumps), or None."""
    sols = L.ik_closed_form(target)
    return sols[np.argmin(np.linalg.norm(sols - q_prev, axis=1))] if len(sols) else None


# ---- foot paths (base frame, sampled every DT) -----------------------------------------------------------------------------

def segment(a, b, T, smooth=True):
    """Samples from a to b over T s: minimum-jerk timing (zero velocity at both ends) or constant speed."""
    s = np.arange(1, int(round(T / DT)) + 1) / round(T / DT)
    if smooth:
        s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return np.asarray(a) + np.outer(s, np.asarray(b) - np.asarray(a))


def polyline(points, times, smooth=True, cycles=1):
    """Closed loop through the points (the last returns to the first), repeated; times per segment."""
    P = [np.asarray(points[0], float)[None]]
    for _ in range(cycles):
        for i, T in enumerate(times):
            P.append(segment(points[i], points[(i + 1) % len(points)], T, smooth))
    return np.vstack(P)


def arc_cycle(p0, r, T_swing, T_stance, cycles=1):
    """Swing forward along the upper semicircle of radius r about p0 (x-z plane, minimum-jerk angle), then stance back
    along the chord at constant speed."""
    n = int(round(T_swing / DT))
    s = np.arange(1, n + 1) / n
    phi = np.pi * (1 - (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5))
    swing = p0 + r * np.stack([np.cos(phi), 0 * phi, np.sin(phi)], 1)
    stance = segment(p0 + [r, 0, 0], p0 - [r, 0, 0], T_stance, smooth=False)
    return np.vstack([(p0 - [r, 0, 0])[None]] + [np.vstack([swing, stance])] * cycles)


def lead_in(p0, P, T=0.5):
    """Prepend a minimum-jerk move from p0 to the path start; returns (path, number of lead-in samples)."""
    a = segment(p0, P[0], T)
    return np.vstack([p0[None], a, P[1:]]), len(a) + 1


def foot_ref(m, paths, q_start=Q_HOLD):
    """Joint reference for foot paths {leg: (N, 3) base-frame targets every DT}; other legs hold q_start. Raises if a sample
    has no IK solution."""
    L = {Lg.name: Lg for Lg in legs(m)}
    N = max(len(P) for P in paths.values())
    Q = np.tile(np.asarray(q_start, float), (N, 1))
    for leg, P in paths.items():
        i = LEGS.index(leg)
        q = Q[0, 3 * i:3 * i + 3].copy()
        for k in range(N):
            q = ik_closest(L[leg], P[min(k, len(P) - 1)], q)
            if q is None:
                raise ValueError(f'{leg}: no IK solution for path sample {k}')
            Q[k, 3 * i:3 * i + 3] = q
    return Q


def foot_errors(log, m, leg, P, start=0):
    """Tracking error |p_mujoco(t) - p*(t)| and path error (distance to the nearest reference sample), from sample `start`."""
    from scipy.spatial import cKDTree
    i = LEGS.index(leg)
    k = np.minimum(np.round(log['t'] / DT).astype(int), len(P) - 1)
    keep = k >= start
    p = log['feet'][keep, i] - base_origin(m)
    e_t = np.linalg.norm(p - P[k[keep]], axis=1)
    e_p = cKDTree(P[start:]).query(p)[0]
    return p, e_t, e_p


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


# ---- rendering with the reference path drawn -----------------------------------------------------------------------------

class PathRecorder(sim.Recorder):
    """sim.Recorder that also draws reference paths (amber dots) and the measured foot trail (blue dots) in world frame."""

    def __init__(self, m, mp4, paths, feet, lookat=None, **kw):
        super().__init__(m, mp4, **kw)
        if lookat is not None:                                     # fixed camera aimed at a point instead of tracking the base
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.cam.lookat[:] = lookat
        self.paths = [P[::max(1, len(P) // 120)] for P in paths]   # about 120 dots per path
        self.feet, self.trail, self.idx = feet, [], sim.index(m)

    def render(self, d):
        if not self.ok:
            return None
        self.renderer.update_scene(d, camera=self.cam, scene_option=self.opt)
        for f in self.feet:
            self.trail.append(d.geom_xpos[self.idx.feet[f]].copy())
        self.trail = self.trail[-12:]                          # the last 0.4 s at 30 frames/s
        scn = self.renderer.scene
        for pts, rgba, r in [(P, (0.79, 0.52, 0.06, 1), 0.0035) for P in self.paths] + [(self.trail, (0.04, 0.45, 0.66, 1), 0.006)]:
            for p in pts:
                if scn.ngeom >= scn.maxgeom:
                    break
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, [r, 0, 0], p, np.eye(3).ravel(), rgba)
                scn.ngeom += 1
        return self.renderer.render()
