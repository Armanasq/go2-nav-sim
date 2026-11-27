"""Single-leg kinematics of the Go2, from first principles.

The chain base -> hip -> thigh -> calf -> foot is read from the MJCF as fixed offsets (position + quaternion of each body in its
parent) and one hinge per body (axis in the body frame, located at the body origin). Forward kinematics composes
    T_base_foot(q) = prod_i [ R(quat_i), p_i ] [ Rot(axis_i, q_i), 0 ]  ·  [ I, p_foot ]
The geometric Jacobian of the foot position is J_i = z_i x (p_foot - o_i), with z_i the joint axis and o_i the joint origin, both
in the base frame. Inverse kinematics is damped least squares with joint-limit clamping.
Nothing here calls MuJoCo's or Pinocchio's kinematics; they are used only to check this code (validate.py, test_kinematics.py)."""
import numpy as np
import mujoco
from common import axis_angle_R, quat_to_R


class Leg:
    def __init__(self, model, leg):
        self.name = leg; self.links = []
        for part in ('hip', 'thigh', 'calf'):
            b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'{leg}_{part}')
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{part}_joint')
            assert np.allclose(model.jnt_pos[j], 0), 'joint expected at the body origin'
            self.links.append((model.body_pos[b].copy(), quat_to_R(model.body_quat[b]), model.jnt_axis[j].copy()))
        g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg)                    # foot sphere, child of the calf
        self.foot = model.geom_pos[g].copy()
        self.lo = np.array([model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{p}_joint')][0] for p in ('hip', 'thigh', 'calf')])
        self.hi = np.array([model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{p}_joint')][1] for p in ('hip', 'thigh', 'calf')])
        # Geometry the closed form assumes (asserted, not hoped for): hip about x, thigh and knee about y, no rotated bodies,
        # thigh offset purely lateral, calf offset purely down, foot in the leg's sagittal plane.
        (ph, Rh, ah), (pt, Rt, at), (pc, Rc, ac) = self.links
        assert np.allclose(ah, [1, 0, 0]) and np.allclose(at, [0, 1, 0]) and np.allclose(ac, [0, 1, 0])
        assert all(np.allclose(R_, np.eye(3)) for R_ in (Rh, Rt, Rc)) and np.allclose(pt[[0, 2]], 0) and np.allclose(pc[:2], 0)
        assert np.isclose(self.foot[1], 0)
        self.hip, self.d, self.l1 = ph, pt[1], -pc[2]                 # hip origin in base, lateral offset, thigh length
        self.l2 = np.hypot(*self.foot[[0, 2]])                         # knee -> foot centre
        self.gamma = np.arctan2(-self.foot[0], -self.foot[2])          # angle of that segment from the calf's -z axis

    def ik_closed_form(self, target):
        """All joint solutions within the limits that put the foot at `target` (base frame), ordered foot-below-hip first.
        Derivation in README.md: hip abduction from the lateral offset, knee from the law of cosines, thigh from the angle of the
        sagittal foot vector. Empty array when the target is unreachable."""
        x, y, z = np.asarray(target, float) - self.hip
        r2 = y * y + z * z - self.d ** 2                               # squared distance in the leg plane, minus the offset
        if r2 < 0:
            return np.empty((0, 3))
        sols = []
        for sz in (-np.sqrt(r2), np.sqrt(r2)):                         # foot below / above the hip in the leg plane
            q1 = np.arctan2(z, y) - np.arctan2(sz, self.d)
            D = (x * x + sz * sz - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
            if abs(D) > 1 + 1e-12:
                continue
            for sgn in (-1, 1):                                        # knee branch: Go2 knees bend backwards, q3 < 0
                q3 = -self.gamma + sgn * np.arccos(np.clip(D, -1, 1))
                vx = self.foot[0] * np.cos(q3) + self.foot[2] * np.sin(q3)
                vz = -self.l1 - self.foot[0] * np.sin(q3) + self.foot[2] * np.cos(q3)
                q2 = np.arctan2(x, sz) - np.arctan2(vx, vz)
                q = np.array([q1, q2, q3])
                q = q + 2 * np.pi * np.ceil((self.lo - q) / (2 * np.pi))   # smallest 2*pi shift at or above the lower limit
                if np.all(q <= self.hi + 1e-12):
                    sols.append(q)
        return np.array(sols).reshape(-1, 3)

    def det_jacobian(self, q):
        """det J = -l1 l2 sin(q3 + gamma) s_z, with s_z the foot's coordinate along the leg-plane 'down' direction (README)."""
        a = q[1] + q[2]
        sz = -self.l1 * np.cos(q[1]) - self.foot[0] * np.sin(a) + self.foot[2] * np.cos(a)
        return -self.l1 * self.l2 * np.sin(q[2] + self.gamma) * sz

    def frames(self, q):
        """Joint origins o_i, joint axes z_i (base frame) and the foot position."""
        R, p = np.eye(3), np.zeros(3); origins, axes = [], []
        for (pos, Rb, axis), qi in zip(self.links, q):
            p = p + R @ pos; R = R @ Rb                       # fixed offset of this body in its parent
            origins.append(p.copy()); axes.append(R @ axis)
            R = R @ axis_angle_R(axis, qi)                    # rotate about the hinge
        return np.array(origins), np.array(axes), p + R @ self.foot

    def fk(self, q):
        return self.frames(q)[2]

    def jacobian(self, q):
        o, z, f = self.frames(q)
        return np.stack([np.cross(z[i], f - o[i]) for i in range(3)], 1)          # 3x3, foot position w.r.t. q

    def ik(self, target, q0, tol=1e-6, max_iter=50, damping=1e-3):
        """Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e, clamped to the joint limits.
        Returns (q, iterations, final error norm, converged)."""
        q = np.clip(np.array(q0, float), self.lo, self.hi)
        for it in range(1, max_iter + 1):
            e = target - self.fk(q)
            if np.linalg.norm(e) < tol:
                return q, it - 1, float(np.linalg.norm(e)), True
            J = self.jacobian(q)
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(3), e), self.lo, self.hi)
        e = float(np.linalg.norm(target - self.fk(q)))
        return q, max_iter, e, e < tol

    def ik_multistart(self, target, q0, tol=1e-6, thighs=(-1.0, 2.2, 3.1)):
        """Joint-limit clamping creates local minima (every single-seed failure in validate.py ended on a limit). Retry from seeds
        spread over the thigh range when the first solve fails. Returns (q, total iterations, error, converged)."""
        q, it, e, ok = self.ik(target, q0, tol)
        total = it
        for th in thighs:
            if ok:
                break
            q, it, e, ok = self.ik(target, np.array([q0[0], th, q0[2]]), tol); total += it
        return q, total, e, ok
