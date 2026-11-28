"""Stage 3 shared code: the Go2 standing on the floor with no support, a posture controller, and the posture API that stage 5
uses (set body pose, shift the centre of mass, lift and place a foot).

Interface: all twelve MuJoCo actuators are <motor> elements (torque, gear 1, ctrlrange +-23.7 N m hip and thigh, +-45.43 N m
knee). Every 2 ms the controller computes
    tau = kp (q* - q) + kd (dq* - dq) + b dq*  +  g_leg(q)  -  J(q)^T R^T f
q* comes from the closed-form IK of the foot targets expressed in the body frame (stage 2), g_leg is the leg's own gravity
torque (stage 2 LegModel, with gravity rotated into the body frame) and f is the vertical foot force that statics asks of each
stance foot (force_distribution). The controller reads the joint encoders (q, dq) and the base orientation R (an IMU stands
in for it; here it is read from d.qpos). The base position is read once, in anchor(), to express the foot targets in world
coordinates; it is never fed back."""
import importlib.util
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('s02_lib', HERE.parent / 's02_joint-and-leg-control' / 'lib.py')
s02 = importlib.util.module_from_spec(_spec)
sys.modules['s02_lib'] = s02
_spec.loader.exec_module(s02)                       # also puts modules/ and modules/m04_kinematics on sys.path
import mujoco  # noqa: E402
import sim  # noqa: E402
from common import LEGS  # noqa: E402

DT = s02.DT
Q_STAND = sim.Q_STAND.copy()                         # (0, 0.9, -1.8) rad per leg, the MJCF 'home' keyframe
Q_REST = np.tile([0.0, 1.36, -2.65], 4)             # computed resting pose (README 3.01)
KP, KD = 150.0, 3.0                                  # N m/rad, N m s/rad
MIN_CLEARANCE = 0.020                                # m, trunk above the floor, guard in Posture.move
MIN_MARGIN = 0.020                                   # m, CoM projection inside the support polygon, guard in shift_com/lift_foot
BOX = np.array([0.1881, 0.04675, 0.057])            # trunk collision box half-sizes (go2.xml)
CHIN = (np.array([0.293, 0.0, -0.06]), 0.047)       # trunk collision sphere under the head: centre (base frame), radius


class Rejected(ValueError):
    """A posture request refused before any motion: reason in .reason ('too_far', 'joint_limit', 'clearance', 'margin', ...)."""

    def __init__(self, reason, detail=''):
        super().__init__(f'{reason}: {detail}')
        self.reason = reason


# ---- rotations, statics, support polygon --------------------------------------------------------------------------------------

def rpy_to_R(rpy):
    """R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    return np.array([np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(np.clip(R[2, 0], -1, 1)), np.arctan2(R[1, 0], R[0, 0])])


def min_jerk(N):
    s = np.arange(1, N + 1) / N
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def force_distribution(dxy, w, weight):
    """Vertical foot forces f (n,) with sum f = weight and sum f (x_i - c) = 0 (no moment about the CoM), the least-squares
    solution weighted by w: minimise sum f_i^2 / w_i, so f = W A^T (A W A^T)^-1 b. A foot with w = 0 gets no force; with three
    feet of w > 0 the solution is the unique statically determinate one. dxy (n, 2) is foot minus CoM, horizontal."""
    A = np.vstack([np.ones(len(dxy)), dxy.T])
    W = np.diag(w)
    f = W @ A.T @ np.linalg.solve(A @ W @ A.T, [weight, 0.0, 0.0])
    return np.maximum(f, 0.0)                         # ponytail: clip a pull to zero; only happens with the CoM outside


def support_margin(pts, c):
    """Signed distance (m) from point c to the edges of the convex hull of pts (n >= 3, horizontal); positive inside."""
    from scipy.spatial import ConvexHull
    pts = np.asarray(pts, float)[:, :2]
    if len(pts) < 3:
        return float('-inf')
    v = pts[ConvexHull(pts).vertices]                 # counter-clockwise
    e = np.roll(v, -1, 0) - v
    rel = np.asarray(c, float)[:2] - v
    return float(np.min((e[:, 0] * rel[:, 1] - e[:, 1] * rel[:, 0]) / np.linalg.norm(e, axis=1)))


def trunk_clearance(p, R):
    """Predicted height of the lowest trunk collision point above z = 0: the eight box corners and the chin sphere."""
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * BOX
    return float(min((p + corners @ R.T)[:, 2].min(), (p + R @ CHIN[0])[2] - CHIN[1]))


# ---- MuJoCo ground truth ---------------------------------------------------------------------------------------------------

def contact_points(m, d):
    """(4, 3) mean MuJoCo contact position of each foot (NaN when the foot has no contact)."""
    feet = list(sim.index(m).feet)
    acc, n = np.zeros((4, 3)), np.zeros(4)
    for i in range(d.ncon):
        c = d.contact[i]
        for g in (c.geom1, c.geom2):
            if g in feet:
                acc[feet.index(g)] += c.pos; n[feet.index(g)] += 1
    out = np.full((4, 3), np.nan)
    out[n > 0] = acc[n > 0] / n[n > 0, None]
    return out


def other_contacts(m, d):
    """Number of contacts that involve a geom other than a foot sphere (trunk, hip, thigh, calf on the floor)."""
    feet = set(sim.index(m).feet)
    return sum(1 for i in range(d.ncon) if d.contact[i].geom1 not in feet and d.contact[i].geom2 not in feet)


def mujoco_clearance(m, d, bodies=('base',)):
    """Smallest MuJoCo geom distance (mj_geomDistance) from the floor to the collision geoms of the named bodies."""
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in bodies]
    geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in ids and m.geom_contype[g] and g not in sim.index(m).feet]
    return float(min(mujoco.mj_geomDistance(m, d, floor, g, 1.0, None) for g in geoms))


def leg_bodies(m):
    return tuple(f'{l}_{p}' for l in LEGS for p in ('hip', 'thigh', 'calf'))


# ---- the posture controller and API ------------------------------------------------------------------------------------------

class Posture:
    """Posture control of the Go2 on the floor. Holds the reference: body position p (world), body roll-pitch-yaw, the four
    foot targets P (4, 3, world) and the load weights w (4,) used by force_distribution. Each call plans the whole motion
    (minimum-jerk from the current reference, IK every 2 ms) and raises Rejected before moving if any sample is unreachable or
    unsafe, then simulates it. self.log() returns everything measured so far as arrays.
        pose = Posture(m, d)                       # robot standing: anchors the stance at the current feet
        pose.set_body_pose(rpy=(0, 0.1, 0), T=1)  # pitch 0.1 rad
        pose.shift_com(xy, T=1); pose.lift_foot('FL', 0.06); pose.place_foot('FL', xy_new)"""

    def __init__(self, m, d, kp=KP, kd=KD, rec=None):
        self.m, self.d, self.kp, self.kd, self.rec = m, d, kp, kd, rec
        self.idx, self.L = sim.index(m), s02.legs(m)
        self.b = m.dof_damping[self.idx.qvel].copy()
        self.mb, self.cb = float(m.body_mass[self.idx.base]), m.body_ipos[self.idx.base].copy()
        self.M = self.mb + sum(L.mass.sum() for L in self.L)
        self.g = m.opt.gravity.copy()
        self.limit = m.actuator_ctrlrange[self.idx.act, 1].copy()
        self._log = []
        self.anchor()

    # -- state ------------------------------------------------------------------------------------------------------------
    def q(self):
        return self.d.qpos[self.idx.qpos].copy()

    def anchor(self):
        """Take the current feet as the stance: P = p_base + R fk(q). This is the one read of the base position."""
        p, _, R = sim.base_pose(self.d)
        q = self.q()
        self.P = np.array([p + R @ L.fk(q[3 * i:3 * i + 3]) for i, L in enumerate(self.L)])
        self.z0 = self.P[:, 2].copy()                 # foot-centre height of each foot standing (soft contact)
        self.p, self.rpy, self.w, self.q_ref = p.copy(), R_to_rpy(R), np.ones(4), q

    def com_body(self, q):
        """Whole-robot centre of mass in the base frame from the MJCF masses: (m_base c_base + sum m_b c_b(q)) / M."""
        s = self.mb * self.cb
        for i, L in enumerate(self.L):
            for (o, _, Rb), mb, cb in zip(L.bodies(q[3 * i:3 * i + 3]), L.mass, L.com):
                s = s + mb * (o + Rb @ cb)
        return s / self.M

    # -- planning ---------------------------------------------------------------------------------------------------------
    def ik(self, p, rpy, P, q_prev):
        R = rpy_to_R(rpy)
        q = np.empty(12)
        for i, L in enumerate(self.L):
            f = R.T @ (P[i] - p)
            qi = s02.ik_closest(L, f, q_prev[3 * i:3 * i + 3])
            if qi is None:
                raise Rejected(s02.ik_checked(L, f)[1] or 'ik', f'{L.name} foot at {np.round(f, 4)} m in the body frame')
            q[3 * i:3 * i + 3] = qi
        return q

    def com_world(self, p, rpy, P=None):
        """Predicted CoM (world) if the body were at (p, rpy) with the feet at P."""
        q = self.ik(p, rpy, self.P if P is None else P, self.q_ref)
        return p + rpy_to_R(rpy) @ self.com_body(q)

    def plan(self, T, p=None, rpy=None, P=None, w=None):
        """Minimum-jerk from the current reference to the given targets (None keeps that part) over T s."""
        N = max(1, int(round(T / DT)))
        s = min_jerk(N)[:, None]
        lerp = lambda a, b: a + s * (np.asarray(b, float) - a) if b is not None else np.tile(a, (N, 1))  # noqa: E731
        PP = self.P + s[:, :, None] * ((self.P if P is None else np.asarray(P, float)) - self.P)
        return self.path(lerp(self.p, p), lerp(self.rpy, rpy), PP, lerp(self.w, w))

    def path(self, pp, rr, PP, ww):
        """Reference arrays sampled every DT (N rows) -> joint reference by IK, checked sample by sample before any motion."""
        N = len(pp)
        Q, q = np.empty((N, 12)), self.q_ref
        for k in range(N):
            c = trunk_clearance(pp[k], rpy_to_R(rr[k]))
            if c < MIN_CLEARANCE:
                raise Rejected('clearance', f'predicted trunk clearance {c:.4f} m < {MIN_CLEARANCE} m')
            q = Q[k] = self.ik(pp[k], rr[k], PP[k], q)
        return dict(Q=Q, p=pp, rpy=rr, P=PP, w=ww, support=np.ones(N))

    # -- execution --------------------------------------------------------------------------------------------------------
    def torque(self, qd, dqd, w, support, kp, kd):
        d = self.d
        q, dq = d.qpos[self.idx.qpos], d.qvel[self.idx.qvel]
        R = sim.base_pose(d)[2]
        c = R @ self.com_body(q)
        dxy = np.array([(R @ L.fk(q[3 * i:3 * i + 3]))[:2] for i, L in enumerate(self.L)]) - c[:2]
        f = force_distribution(dxy, w, support * self.M * -self.g[2]) if support > 0 else np.zeros(4)
        tau = kp * (qd - q) + kd * (dqd - dq) + self.b * dqd
        gb = R.T @ self.g
        for i, L in enumerate(self.L):
            qi = q[3 * i:3 * i + 3]
            L.g = gb                                   # LegModel.gravity uses L.g, the gravity vector in the base frame
            tau[3 * i:3 * i + 3] += L.gravity(qi) - L.jacobian(qi).T @ (R.T @ [0.0, 0.0, f[i]])
        return tau

    def execute(self, plan, kp=None, kd=None):
        kp, kd = kp or self.kp, kd or self.kd
        Q = plan['Q']
        dQ = np.diff(np.vstack([self.q_ref, Q]), axis=0) / DT
        t0, N = self.d.time, len(Q)
        log = []

        def ctrl(m, d):
            k = min(int(round((d.time - t0) / DT)), N - 1)
            return self.torque(Q[k], dQ[k], plan['w'][k], plan['support'][k], kp, kd)

        def step(m, d):
            if self.rec is not None:
                self.rec(m, d)
            log.append(self.sample())
        sim.run(self.m, self.d, ctrl, N * DT, on_step=step)
        self.q_ref = Q[-1].copy()
        for k in ('p', 'rpy', 'P', 'w'):
            if plan.get(k) is not None:
                setattr(self, k, plan[k][-1].copy())
        rec = {k: np.array([s[k] for s in log]) for k in log[0]}
        rec.update({f'ref_{k}': plan[k] if plan.get(k) is not None else np.full((N,) + np.shape(getattr(self, k)), np.nan)
                    for k in ('p', 'rpy', 'P', 'w')}, ref_q=Q)
        self._log.append(rec)
        return rec

    def sample(self):
        m, d = self.m, self.d
        p, _, R = sim.base_pose(d)
        return dict(t=d.time, p=p, rpy=R_to_rpy(R), q=d.qpos[self.idx.qpos].copy(), tau=d.ctrl[self.idx.act].copy(),
                    F=sim.foot_forces(m, d), cp=contact_points(m, d), feet=sim.foot_positions(m, d),
                    com=d.subtree_com[self.idx.base].copy(), n_other=other_contacts(m, d))

    def log(self, start=0):
        """All moves since move number `start`, concatenated: t, p, rpy, q, tau, F, cp, feet, com, n_other, ref_*."""
        L = self._log[start:]
        return {k: np.concatenate([x[k] for x in L]) for k in L[0]}

    # -- the API ----------------------------------------------------------------------------------------------------------
    def move(self, T, p=None, rpy=None, P=None, w=None, dry_run=False):
        plan = self.plan(T, p, rpy, P, w)
        return plan if dry_run else self.execute(plan)

    def hold(self, T):
        return self.move(T)

    def follow(self, p=None, rpy=None):
        """Track explicit body trajectories (N, 3) sampled every DT, e.g. a sine; the missing one is held."""
        N = len(p if p is not None else rpy)
        hold = lambda a: np.tile(a, (N, 1))  # noqa: E731
        plan = self.path(hold(self.p) if p is None else np.asarray(p, float), hold(self.rpy) if rpy is None else np.asarray(rpy, float),
                         np.tile(self.P, (N, 1, 1)), hold(self.w))
        return self.execute(plan)

    def set_body_pose(self, p=None, rpy=None, T=1.0, dry_run=False):
        """Body position (world, m) and roll-pitch-yaw (rad); None keeps the current reference. Feet stay where they are."""
        return self.move(T, p=p, rpy=rpy, dry_run=dry_run)

    def body_for_com(self, xy, P=None):
        """Body position that puts the predicted CoM projection at xy (fixed-point iteration; the legs move the CoM too)."""
        p = self.p.copy()
        for _ in range(6):
            p[:2] += np.asarray(xy) - self.com_world(p, self.rpy, P)[:2]
        return p

    def shift_com(self, xy, T=1.0, min_margin=MIN_MARGIN, dry_run=False):
        """Translate the body so the CoM projection moves to xy (world, m). Rejected if xy is closer than min_margin to the
        edge of the polygon of the loaded feet."""
        st = self.w > 0
        mg = support_margin(self.P[st], xy)
        if mg < min_margin:
            raise Rejected('margin', f'predicted margin {mg:.4f} m < {min_margin} m')
        return self.move(T, p=self.body_for_com(xy), dry_run=dry_run)

    def margin(self, legs=None, xy=None):
        """Predicted margin of the CoM (current reference, or xy) for the given stance legs (default: loaded feet)."""
        st = self.w > 0 if legs is None else np.isin(LEGS, legs)
        return support_margin(self.P[st], self.com_world(self.p, self.rpy)[:2] if xy is None else xy)

    def com_into_triangle(self, leg, margin=0.04, T=1.0):
        """Shift the CoM the shortest distance that leaves it `margin` inside the triangle of the other three feet."""
        tri = self.P[np.array(LEGS) != leg, :2]
        c = self.com_world(self.p, self.rpy)[:2]
        from scipy.optimize import minimize
        cons = {'type': 'ineq', 'fun': lambda x: support_margin(tri, x) - margin}
        xy = minimize(lambda x: np.sum((x - c) ** 2), tri.mean(0), constraints=[cons], method='SLSQP').x
        return self.shift_com(xy, T, min_margin=margin - 1e-4), xy

    def lift_foot(self, leg, height, T_unload=0.5, T_lift=0.6, min_margin=MIN_MARGIN):
        """Unload `leg` (its weight w goes 1 -> 0, the other feet take its force), then raise it by `height` (m)."""
        i = LEGS.index(leg)
        others = [l for l in LEGS if l != leg]
        mg = self.margin(others)
        if mg < min_margin:
            raise Rejected('margin', f'CoM {mg:.4f} m inside the {"-".join(others)} triangle, need {min_margin} m')
        w = self.w.copy(); w[i] = 0.0
        a = self.move(T_unload, w=w)
        P = self.P.copy(); P[i, 2] += height
        return a, self.move(T_lift, P=P)

    def place_foot(self, leg, xy, T_move=0.6, T_lower=0.6, T_load=0.5):
        """Move the lifted foot above xy (world), lower it to its standing height, and load it again."""
        i = LEGS.index(leg)
        P = self.P.copy(); P[i, :2] = xy
        a = self.move(T_move, P=P)
        P = P.copy(); P[i, 2] = self.z0[i]
        b = self.move(T_lower, P=P)
        w = self.w.copy(); w[i] = 1.0
        return a, b, self.move(T_load, w=w)

    def joint_move(self, q1, T, kp=None, kd=None, support=(1.0, 1.0)):
        """Joint-space minimum-jerk move from the current joint reference to q1 (used to rest and to stand up), with the
        weight in the force feed-forward ramped from support[0] to support[1] times m g. Call anchor() afterwards."""
        N = max(1, int(round(T / DT)))
        s = min_jerk(N)
        Q = self.q_ref + s[:, None] * (np.asarray(q1, float) - self.q_ref)
        plan = dict(Q=Q, w=np.ones((N, 4)), support=support[0] + s * (support[1] - support[0]))
        return self.execute(plan, kp, kd)


def rest(m, d, T=3.0, rec=None):
    """Place the robot in Q_REST on the floor (feet touching) and let it settle under a soft joint PD (kp 40, kd 2, no weight
    feed-forward). Returns the Posture."""
    sim.place_on_ground(m, d, Q_REST)
    pose = Posture(m, d, rec=rec)
    pose.joint_move(Q_REST, T, kp=40.0, kd=2.0, support=(0.0, 0.0))
    return pose


def stand_up(pose, T=1.5, settle=0.5):
    """From rest to Q_STAND in joint space over T s, weight feed-forward ramped 0 -> m g, then settle and anchor the stance."""
    pose.joint_move(Q_STAND, T, support=(0.0, 1.0))
    pose.joint_move(Q_STAND, settle)
    pose.anchor()
    return pose


def standing(rec=None):
    """New model on the floor, rested and stood up: (m, d, pose)."""
    m, d = sim.load()
    pose = rest(m, d, rec=rec)
    return m, d, stand_up(pose)
