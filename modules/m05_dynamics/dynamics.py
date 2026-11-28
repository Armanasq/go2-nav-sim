"""Rigid-body dynamics of the Go2 from first principles (Featherstone's spatial vector algebra).

Spatial vectors are 6-vectors in Featherstone's order: motion [omega; v], force [n; f], both in body coordinates.
The tree is read from an MJCF: per body its parent, the fixed offset in the parent (body_pos, body_quat), at most one joint
(a hinge at the body origin, or a free joint on the root) and its inertia (mass, COM, principal inertia and axes).
MuJoCo is used only to read those parameters; nothing here calls a MuJoCo or Pinocchio dynamics function.

Algorithms (Featherstone 2008: RNEA table 5.1, CRBA table 6.2, ABA table 7.1):
  rnea(q, v, a)        tau = M(q) a + h(q, v)                inverse dynamics, O(n)
  crba(q)              M(q)                                   joint-space mass matrix, composite rigid bodies
  fd(q, v, tau)        a = M^-1 (tau - h)                     forward dynamics via CRBA + Cholesky
  aba(q, v, tau)       a                                      forward dynamics, articulated bodies, O(n)
  contact_kkt(...)     (a, lambda)                            dynamics with rigid point contacts, KKT system

Generalised coordinates ("native"):
  q = MuJoCo qpos layout ([p_world, quat w x y z, joints] when floating, joints only when fixed)
  v = [v_base (body frame), omega_base (body frame), dq]      (= Pinocchio's free-flyer convention)
MuJoCo's qvel = [v_base (world frame), omega_base (body frame), dq] differs in the linear part; to_mj_* / from_mj_* convert.
Rotor inertia (MJCF armature) is added to the joint-space diagonal: M + diag(armature). Joint damping and friction loss
are not rigid-body terms and are left to the caller."""
import numpy as np
import mujoco
from common import axis_angle_R, quat_to_R


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def crm(v):
    """Motion cross product matrix: v x m."""
    S = np.zeros((6, 6)); S[:3, :3] = S[3:, 3:] = skew(v[:3]); S[3:, :3] = skew(v[3:])
    return S


def crf(v):
    """Force cross product matrix: v x* f = -crm(v)^T f."""
    return -crm(v).T


def plucker(R, p):
    """^B X_A for frame B with origin p and axes R (columns), both expressed in A."""
    X = np.zeros((6, 6)); Rt = R.T
    X[:3, :3] = X[3:, 3:] = Rt; X[3:, :3] = -Rt @ skew(p)
    return X


def spatial_inertia(m, c, Ic):
    """Spatial inertia at the body origin of a body with mass m, COM c and rotational inertia Ic about the COM."""
    C = skew(c); I = np.zeros((6, 6))
    I[:3, :3] = Ic + m * C @ C.T; I[:3, 3:] = m * C; I[3:, :3] = m * C.T; I[3:, 3:] = m * np.eye(3)
    return I


S_FREE = np.block([[np.zeros((3, 3)), np.eye(3)], [np.eye(3), np.zeros((3, 3))]])   # [v; w] (native) -> [w; v] (spatial)


class Tree:
    """Kinematic tree of all bodies of an MJCF model below the world. Joint kinds: 'free' (root only), 'hinge', None (welded)."""

    def __init__(self, m):
        self.m, self.nb = m, m.nbody - 1                     # body k of the tree is MuJoCo body k + 1
        self.parent, self.kind, self.axis, self.Xt, self.Rt, self.pt, self.I = [], [], [], [], [], [], []
        self.dofs, self.qadr = [], []
        nv = 0
        for b in range(1, m.nbody):
            self.parent.append(m.body_parentid[b] - 1)       # -1 = world
            R, p = quat_to_R(m.body_quat[b]), m.body_pos[b].copy()
            self.Rt.append(R); self.pt.append(p); self.Xt.append(plucker(R, p))
            Ri = quat_to_R(m.body_iquat[b])
            self.I.append(spatial_inertia(m.body_mass[b], m.body_ipos[b], Ri @ np.diag(m.body_inertia[b]) @ Ri.T))
            nj = m.body_jntnum[b]; assert nj <= 1, 'one joint per body'
            if nj == 0:
                self.kind.append(None); self.axis.append(None); self.dofs.append(np.arange(0)); self.qadr.append(None)
                continue
            j = m.body_jntadr[b]; t = m.jnt_type[j]
            assert np.allclose(m.jnt_pos[j], 0), 'joint expected at the body origin'
            if t == mujoco.mjtJoint.mjJNT_FREE:
                assert self.parent[-1] == -1, 'free joint only on a root body'
                self.kind.append('free'); self.axis.append(None); self.dofs.append(np.arange(nv, nv + 6)); nv += 6
            else:
                assert t == mujoco.mjtJoint.mjJNT_HINGE
                self.kind.append('hinge'); self.axis.append(m.jnt_axis[j].copy()); self.dofs.append(np.array([nv])); nv += 1
            self.qadr.append(m.jnt_qposadr[j])
        self.nv = nv
        self.armature = np.array([m.dof_armature[m.jnt_dofadr[m.body_jntadr[b + 1]] + k]
                                  for b in range(self.nb) for k in range(len(self.dofs[b]))])
        self.gravity = m.opt.gravity.copy()
        self.floating = 'free' in self.kind

    # ---- kinematics ----------------------------------------------------------------------------------------------------
    def S(self, i):
        if self.kind[i] == 'free':
            return S_FREE
        if self.kind[i] == 'hinge':
            return np.concatenate([self.axis[i], np.zeros(3)])[:, None]
        return np.zeros((6, 0))

    def joint_transform(self, i, q):
        """(R, p) of body i in its parent: fixed offset, then the joint."""
        if self.kind[i] == 'free':
            a = self.qadr[i]
            return quat_to_R(q[a + 3:a + 7]), q[a:a + 3].copy()
        R, p = self.Rt[i], self.pt[i]
        if self.kind[i] == 'hinge':
            R = R @ axis_angle_R(self.axis[i], q[self.qadr[i]])
        return R, p

    def kinematics(self, q):
        """Per body: ^iX_parent (6x6), world rotation and position."""
        X, Rw, pw = [], [], []
        for i in range(self.nb):
            R, p = self.joint_transform(i, q)
            X.append(plucker(R, p))
            lam = self.parent[i]
            if lam < 0:
                Rw.append(R); pw.append(p)
            else:
                Rw.append(Rw[lam] @ R); pw.append(pw[lam] + Rw[lam] @ p)
        return X, Rw, pw

    def _a0(self, gravity):
        """Spatial acceleration of the world in world coordinates: -g (Featherstone's trick for gravity)."""
        a0 = np.zeros(6)
        if gravity:
            a0[3:] = -self.gravity
        return a0

    # ---- inverse dynamics ----------------------------------------------------------------------------------------------
    def rnea(self, q, v, a, gravity=True, f_ext=None):
        """tau = M(q) a + h(q, v). f_ext: optional {body: spatial force in body coordinates} applied to the body."""
        X, _, _ = self.kinematics(q)
        V, A, F = [None] * self.nb, [None] * self.nb, [None] * self.nb
        for i in range(self.nb):
            S, d = self.S(i), self.dofs[i]; vJ = S @ v[d]
            lam = self.parent[i]
            vp, ap = (np.zeros(6), self._a0(gravity)) if lam < 0 else (V[lam], A[lam])
            V[i] = X[i] @ vp + vJ
            A[i] = X[i] @ ap + S @ a[d] + crm(V[i]) @ vJ
            F[i] = self.I[i] @ A[i] + crf(V[i]) @ self.I[i] @ V[i]
            if f_ext and i in f_ext:
                F[i] = F[i] - f_ext[i]
        tau = np.zeros(self.nv)
        for i in reversed(range(self.nb)):
            tau[self.dofs[i]] = self.S(i).T @ F[i]
            if self.parent[i] >= 0:
                F[self.parent[i]] = F[self.parent[i]] + X[i].T @ F[i]
        return tau + self.armature * a

    def bias(self, q, v):
        """h(q, v) = C(q, v) v + g(q)."""
        return self.rnea(q, v, np.zeros(self.nv))

    def gravity_torque(self, q):
        return self.rnea(q, np.zeros(self.nv), np.zeros(self.nv))

    # ---- mass matrix ---------------------------------------------------------------------------------------------------
    def crba(self, q):
        X, _, _ = self.kinematics(q)
        Ic = [I.copy() for I in self.I]
        for i in reversed(range(self.nb)):
            if self.parent[i] >= 0:
                Ic[self.parent[i]] += X[i].T @ Ic[i] @ X[i]
        M = np.zeros((self.nv, self.nv))
        for i in range(self.nb):
            di = self.dofs[i]
            if not len(di):
                continue
            Fi = Ic[i] @ self.S(i)
            M[np.ix_(di, di)] = self.S(i).T @ Fi
            j = i
            while self.parent[j] >= 0:
                Fi = X[j].T @ Fi; j = self.parent[j]
                dj = self.dofs[j]
                if len(dj):
                    M[np.ix_(di, dj)] = Fi.T @ self.S(j); M[np.ix_(dj, di)] = M[np.ix_(di, dj)].T
        return M + np.diag(self.armature)

    # ---- forward dynamics ----------------------------------------------------------------------------------------------
    def fd(self, q, v, tau):
        """a = M^-1 (tau - h) by Cholesky of the CRBA mass matrix."""
        L = np.linalg.cholesky(self.crba(q))
        y = np.linalg.solve(L, tau - self.bias(q, v))
        return np.linalg.solve(L.T, y)

    def aba(self, q, v, tau, gravity=True):
        X, _, _ = self.kinematics(q)
        V, c, IA, pA = [None] * self.nb, [None] * self.nb, [None] * self.nb, [None] * self.nb
        for i in range(self.nb):
            vJ = self.S(i) @ v[self.dofs[i]]; lam = self.parent[i]
            V[i] = X[i] @ (np.zeros(6) if lam < 0 else V[lam]) + vJ
            c[i] = crm(V[i]) @ vJ
            IA[i] = self.I[i].copy(); pA[i] = crf(V[i]) @ self.I[i] @ V[i]
        U, Dinv, u = [None] * self.nb, [None] * self.nb, [None] * self.nb
        k = np.cumsum([0] + [len(d) for d in self.dofs])
        for i in reversed(range(self.nb)):
            S = self.S(i); d = self.dofs[i]
            if len(d):
                U[i] = IA[i] @ S
                Dinv[i] = np.linalg.inv(S.T @ U[i] + np.diag(self.armature[k[i]:k[i + 1]]))
                u[i] = tau[d] - S.T @ pA[i]
                Ia = IA[i] - U[i] @ Dinv[i] @ U[i].T
                pa = pA[i] + Ia @ c[i] + U[i] @ Dinv[i] @ u[i]
            else:
                Ia, pa = IA[i], pA[i] + IA[i] @ c[i]
            lam = self.parent[i]
            if lam >= 0:
                IA[lam] = IA[lam] + X[i].T @ Ia @ X[i]; pA[lam] = pA[lam] + X[i].T @ pa
        qdd = np.zeros(self.nv); A = [None] * self.nb
        for i in range(self.nb):
            lam = self.parent[i]
            ap = X[i] @ (self._a0(gravity) if lam < 0 else A[lam]) + c[i]
            d = self.dofs[i]
            if len(d):
                qdd[d] = Dinv[i] @ (u[i] - U[i].T @ ap)
                A[i] = ap + self.S(i) @ qdd[d]
            else:
                A[i] = ap
        return qdd

    # ---- points on bodies ----------------------------------------------------------------------------------------------
    def body_jacobians(self, q):
        """Per body the 6 x nv Jacobian of its spatial velocity in body coordinates; plus world poses."""
        X, Rw, pw = self.kinematics(q)
        J = []
        for i in range(self.nb):
            lam = self.parent[i]
            Ji = np.zeros((6, self.nv)) if lam < 0 else X[i] @ J[lam]
            Ji[:, self.dofs[i]] += self.S(i)
            J.append(Ji)
        return J, Rw, pw

    def point_jacobian(self, q, body, p_world):
        """3 x nv Jacobian of the world-frame velocity of the point p_world fixed to tree body `body`."""
        J, Rw, pw = self.body_jacobians(q)
        r = Rw[body].T @ (p_world - pw[body])
        return Rw[body] @ (J[body][3:] - skew(r) @ J[body][:3])

    def point_bias_acceleration(self, q, v, body, p_world):
        """J_dot v: world-frame acceleration of the point when a = 0 and gravity is off."""
        X, Rw, pw = self.kinematics(q)
        V, A = [None] * self.nb, [None] * self.nb
        for i in range(self.nb):
            vJ = self.S(i) @ v[self.dofs[i]]; lam = self.parent[i]
            vp, ap = (np.zeros(6), np.zeros(6)) if lam < 0 else (V[lam], A[lam])
            V[i] = X[i] @ vp + vJ; A[i] = X[i] @ ap + crm(V[i]) @ vJ
        r = Rw[body].T @ (p_world - pw[body]); w, vo = V[body][:3], V[body][3:]
        return Rw[body] @ (A[body][3:] + np.cross(A[body][:3], r) + np.cross(w, vo + np.cross(w, r)))

    # ---- contact-constrained dynamics ----------------------------------------------------------------------------------
    def contact_kkt(self, q, v, tau, points):
        """Rigid point contacts, no slip:  M a + h = tau + Jc^T lam,  Jc a + Jc_dot v = 0.
        tau: nv generalised forces; points: [(body, p_world)]. Returns (a, lam (k, 3) world-frame contact forces, Jc)."""
        M, h = self.crba(q), self.bias(q, v)
        Jc = np.vstack([self.point_jacobian(q, b, p) for b, p in points])
        gam = np.concatenate([self.point_bias_acceleration(q, v, b, p) for b, p in points])
        k = Jc.shape[0]
        K = np.block([[M, -Jc.T], [Jc, np.zeros((k, k))]])
        sol = np.linalg.solve(K, np.concatenate([tau - h, -gam]))
        return sol[:self.nv], sol[self.nv:].reshape(-1, 3), Jc

    def contact_schur(self, q, v, tau, points):
        """Same solution by the Schur complement: lam = (Jc M^-1 Jc^T)^-1 (-gam - Jc M^-1 (tau - h)). Used as a check."""
        M, h = self.crba(q), self.bias(q, v)
        Jc = np.vstack([self.point_jacobian(q, b, p) for b, p in points])
        gam = np.concatenate([self.point_bias_acceleration(q, v, b, p) for b, p in points])
        Mi_J, a0 = np.linalg.solve(M, Jc.T), np.linalg.solve(M, tau - h)
        lam = np.linalg.solve(Jc @ Mi_J, -gam - Jc @ a0)
        return a0 + Mi_J @ lam, lam.reshape(-1, 3)


# ---- MuJoCo <-> native conversions (floating base) -----------------------------------------------------------------------
# MuJoCo qvel = T v with T = diag(R, I, I); power is invariant, so tau_mj = T^-T tau = diag(R, I, I) tau and
# M_mj = T^-T M T^-1. MuJoCo qacc is the derivative of qvel: its linear part is R (v_dot_b + w x v_b).

def _R(qpos):
    return quat_to_R(qpos[3:7])


def from_mj_vel(qpos, qvel):
    v = qvel.copy(); v[:3] = _R(qpos).T @ qvel[:3]
    return v


def from_mj_acc(qpos, qvel, qacc):
    R = _R(qpos); a = qacc.copy(); vb = R.T @ qvel[:3]
    a[:3] = R.T @ qacc[:3] - np.cross(qvel[3:6], vb)
    return a


def to_mj_acc(qpos, v, a):
    R = _R(qpos); out = a.copy()
    out[:3] = R @ (a[:3] + np.cross(v[3:6], v[:3]))
    return out


def to_mj_force(qpos, tau):
    out = tau.copy(); out[:3] = _R(qpos) @ tau[:3]
    return out


def from_mj_force(qpos, tau):
    out = tau.copy(); out[:3] = _R(qpos).T @ tau[:3]
    return out


def to_mj_matrix(qpos, M):
    D = np.eye(M.shape[0]); D[:3, :3] = _R(qpos)
    return D @ M @ D.T


def to_pin_q(qpos):
    """Pinocchio free-flyer configuration: quaternion x y z w."""
    return np.concatenate([qpos[:3], qpos[4:7], qpos[3:4], qpos[7:]])


# ---- Lagrangian two-link leg (thigh + calf in the sagittal plane, hip abduction at zero) ---------------------------------

class TwoLink:
    """Closed-form M, C, g of the planar thigh-calf chain (README, Theory). Parameters read from the MJCF for one leg.
    Coordinates: x forward, z up in the thigh joint frame; rotations about +y; theta = (q_thigh, q_calf)."""

    def __init__(self, m, leg='FL'):
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)  # noqa: E731
        t, c = bid(f'{leg}_thigh'), bid(f'{leg}_calf')
        assert np.allclose(m.body_quat[c], [1, 0, 0, 0]) and np.allclose(m.body_pos[c][:2], 0)
        self.l1 = -m.body_pos[c][2]
        self.m1, self.m2 = m.body_mass[t], m.body_mass[c]
        self.c1, self.c2 = m.body_ipos[t][[0, 2]].copy(), m.body_ipos[c][[0, 2]].copy()
        Iyy = lambda b: (quat_to_R(m.body_iquat[b]) @ np.diag(m.body_inertia[b]) @ quat_to_R(m.body_iquat[b]).T)[1, 1]  # noqa: E731
        self.I1, self.I2 = Iyy(t), Iyy(c)
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_thigh_joint')
        self.arm = m.dof_armature[m.jnt_dofadr[j]]
        self.g = -m.opt.gravity[2]

    def k(self, th2):
        return self.l1 * (self.c2[0] * np.sin(th2) - self.c2[1] * np.cos(th2))

    def dk(self, th2):
        return self.l1 * (self.c2[0] * np.cos(th2) + self.c2[1] * np.sin(th2))

    def M(self, th):
        m1, m2, c1, c2 = self.m1, self.m2, self.c1, self.c2; k = self.k(th[1])
        M22 = self.I2 + m2 * c2 @ c2
        M12 = M22 + m2 * k
        M11 = self.I1 + m1 * c1 @ c1 + self.I2 + m2 * (self.l1 ** 2 + c2 @ c2 + 2 * k)
        return np.array([[M11, M12], [M12, M22]]) + self.arm * np.eye(2)

    def C(self, th, dth):
        """Coriolis and centrifugal matrix from the Christoffel symbols; C(th, dth) dth is the velocity term."""
        b = self.m2 * self.dk(th[1])
        return np.array([[b * dth[1], b * (dth[0] + dth[1])], [-b * dth[0], 0.0]])

    def gvec(self, th):
        (c1x, c1z), (c2x, c2z), g = self.c1, self.c2, self.g; t1, t12 = th[0], th[0] + th[1]
        g2 = self.m2 * g * (-c2x * np.cos(t12) - c2z * np.sin(t12))
        g1 = self.m1 * g * (-c1x * np.cos(t1) - c1z * np.sin(t1)) + self.m2 * g * self.l1 * np.sin(t1) + g2
        return np.array([g1, g2])

    def tau(self, th, dth, ddth):
        return self.M(th) @ ddth + self.C(th, dth) @ dth + self.gvec(th)
