"""Lie-group perturbation tools in plain NumPy (module 00, advanced level). Derivations are in README.md; Pinocchio and finite
differences check this code in test_lie.py and validate.py, they are not called here.

SO(3): right and left Jacobians and their inverses, with Exp(w + dw) ~= Exp(w) Exp(Jr(w) dw) ~= Exp(Jl(w) dw) Exp(w) and
Log(Exp(w) Exp(d)) ~= w + Jr(w)^-1 d. SE(2): Exp/Log with poses stored as (x, y, theta), the right perturbation T Exp(d)."""
import pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from rotations import se2_compose, skew  # noqa: E402

SMALL = 1e-5      # rad; below this the closed forms lose digits to cancellation and the Taylor series is exact to ~1e-16


def jr_so3(w):
    """Right Jacobian Jr(w) = I - (1 - cos t)/t^2 [w] + (t - sin t)/t^3 [w]^2, t = |w|."""
    t = np.linalg.norm(w); W = skew(w)
    if t < SMALL:
        return np.eye(3) - W / 2 + W @ W / 6
    return np.eye(3) - 2 * np.sin(t / 2)**2 / t**2 * W + (t - np.sin(t)) / t**3 * W @ W     # 1 - cos t = 2 sin^2(t/2)


def jl_so3(w):
    """Left Jacobian Jl(w) = Jr(-w) = Jr(w)^T."""
    return jr_so3(-np.asarray(w, float))


def jr_inv_so3(w):
    """Jr(w)^-1 = I + [w]/2 + (1/t^2 - cot(t/2)/(2t)) [w]^2. cot(t/2) replaces (1 + cos t)/sin t, which is 0/0 at t = pi."""
    t = np.linalg.norm(w); W = skew(w)
    if t < SMALL:
        return np.eye(3) + W / 2 + W @ W / 12
    return np.eye(3) + W / 2 + (1 / t**2 - 1 / (2 * t * np.tan(t / 2))) * W @ W


def jl_inv_so3(w):
    return jr_inv_so3(-np.asarray(w, float))


def _V2(phi):
    """SE(2) translation Jacobian V(phi) = (1/phi) [[sin, -(1 - cos)], [1 - cos, sin]]; I + phi/2 [1]x near 0."""
    if abs(phi) < SMALL:
        return np.array([[1 - phi**2 / 6, -phi / 2], [phi / 2, 1 - phi**2 / 6]])
    s, c = np.sin(phi), np.cos(phi)
    return np.array([[s, -(1 - c)], [1 - c, s]]) / phi


def se2_exp(xi):
    """Tangent vector (rho_x, rho_y, phi) -> pose (x, y, theta): t = V(phi) rho, theta = phi."""
    return np.array([*(_V2(xi[2]) @ xi[:2]), xi[2]])


def se2_log(T):
    """Pose (x, y, theta), theta in (-pi, pi] -> (rho_x, rho_y, phi) with rho = V(phi)^-1 t."""
    return np.array([*np.linalg.solve(_V2(T[2]), T[:2]), T[2]])


def se2_plus(T, xi):
    """Right retraction T (+) xi = T Exp(xi), the update the Gauss-Newton and LM steps in optim.py use."""
    return se2_compose(T, se2_exp(xi))
