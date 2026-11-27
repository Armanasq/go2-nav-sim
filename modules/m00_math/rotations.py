"""Rotations and rigid transforms in plain NumPy (module 00). Derivations are in README.md; scipy and Pinocchio check this code
in test_rotations.py and validate.py, they are not called here.

Conventions: quaternions (w, x, y, z) as in MuJoCo; roll-pitch-yaw (phi, theta, psi) with R = Rz(psi) Ry(theta) Rx(phi), the URDF
and Pinocchio convention; angular velocity omega in the body frame unless stated."""
import pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common import axis_angle_R, quat_to_R  # noqa: E402,F401  (Rodrigues and quaternion -> R live in common.py)


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def Rx(a):
    c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def exp_so3(w):
    """Rotation vector w = theta * a -> R (Rodrigues). Below 1e-12 rad the first-order term I + [w] is exact to rounding."""
    th = np.linalg.norm(w)
    return np.eye(3) + skew(w) if th < 1e-12 else axis_angle_R(w / th, th)


def log_so3(R):
    """R -> rotation vector, theta in [0, pi]. trace R = 1 + 2 cos theta and vee(R - R^T) = 2 sin theta a, so
    theta = atan2(|v|, tr R - 1) (arccos alone loses digits near 0 and pi). Above pi/2 the skew part is small, so the axis comes
    from the symmetric part instead: a a^T = (sym(R) - cos theta I) / (1 - cos theta), with its sign taken from v."""
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    th = np.arctan2(np.linalg.norm(v), np.trace(R) - 1)
    if th < 1e-12:
        return v / 2
    if th < np.pi / 2:
        return th / (2 * np.sin(th)) * v
    B = ((R + R.T) / 2 - np.cos(th) * np.eye(3)) / (1 - np.cos(th)); k = int(np.argmax(np.diag(B)))
    a = B[:, k] / np.sqrt(B[k, k])
    return th * (a if a @ v >= 0 else -a)


def R_to_quat(R):
    """Shepperd's method: take the largest of 4w^2 = 1 + tr R, 4x^2 = 1 + 2 R00 - tr R, ... to avoid dividing by a small number.
    Returns (w, x, y, z) with w >= 0."""
    t = np.trace(R); cands = [t, R[0, 0], R[1, 1], R[2, 2]]; k = int(np.argmax(cands))
    if k == 0:
        w = np.sqrt(1 + t) / 2; q = [w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)]
    else:
        i, j, l = k - 1, k % 3, (k + 1) % 3                    # cyclic (i, j, l)
        v = np.zeros(3); v[i] = np.sqrt(1 + 2 * R[i, i] - t) / 2
        v[j] = (R[j, i] + R[i, j]) / (4 * v[i]); v[l] = (R[l, i] + R[i, l]) / (4 * v[i])
        q = [(R[l, j] - R[j, l]) / (4 * v[i]), *v]
    q = np.array(q)
    return q if q[0] >= 0 else -q


def rpy_to_R(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)


def R_to_rpy(R, eps=1e-9):
    """ZYX extraction: R20 = -sin(theta), R21/R22 = tan(phi), R10/R00 = tan(psi). At theta = +-pi/2 only psi -+ phi is defined;
    set psi = 0 and read phi from R01, R11 (gimbal lock)."""
    theta = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if np.hypot(R[0, 0], R[1, 0]) < eps:
        return np.array([np.arctan2(-R[2, 0] * R[0, 1], R[1, 1]), theta, 0.0])
    return np.array([np.arctan2(R[2, 1], R[2, 2]), theta, np.arctan2(R[1, 0], R[0, 0])])


def rpy_rates(rpy, omega):
    """Euler-angle rates (phi_dot, theta_dot, psi_dot) from body angular velocity; singular at cos(theta) = 0."""
    phi, theta, _ = rpy; s, c = np.sin(phi), np.cos(phi)
    psi_dot = (s * omega[1] + c * omega[2]) / np.cos(theta)
    return np.array([omega[0] + np.sin(theta) * psi_dot, c * omega[1] - s * omega[2], psi_dot])


def se2_compose(a, b):
    """(x, y, theta) poses: T_a T_b."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], np.arctan2(np.sin(a[2] + b[2]), np.cos(a[2] + b[2]))])


def se2_inverse(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]])


def se3(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def se3_inverse(T):
    R, t = T[:3, :3], T[:3, 3]
    return se3(R.T, -R.T @ t)


# Camera optical frame (z forward, x right, y down) in the camera link frame (x forward, y left, z up): URDF rpy(-pi/2, 0, -pi/2)
R_LINK_OPTICAL = rpy_to_R(-np.pi / 2, 0, -np.pi / 2)
