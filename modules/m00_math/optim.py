"""Nonlinear least squares from scratch (module 00, advanced level): Gauss-Newton and Levenberg-Marquardt on a manifold, and the
two problems the README works through, SE(2) point-set registration and Go2 leg kinematic calibration. Derivations are in README.md;
scipy.optimize.least_squares and MuJoCo check this code in test_optim.py and validate.py, they are not called here.

Both solvers minimise F(x) = 1/2 |r(x)|^2. fun(x) returns (r, J) with J = dr/dh at h = 0 for the update x <- plus(x, h), so the
same code runs on vectors (plus = +) and on SE(2) with the right perturbation T Exp(h) (lie.se2_plus)."""
from typing import Any

import numpy as np
import numpy.typing as npt


def _add(x, h):
    return x + h


def gauss_newton(fun, x0, plus=_add, max_iter=100, tol_grad=1e-10, tol_step=1e-12) -> dict[str, Any]:
    """Undamped Gauss-Newton: solve (J^T J) h = -J^T r, take the full step. No line search, so it can overshoot or diverge; that is
    kept on purpose, the README shows when. Returns dict(x, xs and cost per iterate, step, status, iters, nfev)."""
    x = np.array(x0, float); r, J = fun(x); cost = [0.5 * r @ r]; steps = []; xs = [x.copy()]; status = 'max_iter'
    for k in range(max_iter):
        g = J.T @ r
        if np.abs(g).max() <= tol_grad:
            status = 'grad'; break
        try:
            h = np.linalg.solve(J.T @ J, -g)
        except np.linalg.LinAlgError:
            status = 'singular'; break
        x = plus(x, h); r, J = fun(x); cost.append(0.5 * r @ r); steps.append(float(np.linalg.norm(h))); xs.append(x.copy())
        if not np.isfinite(cost[-1]):
            status = 'diverged'; break
        if np.linalg.norm(h) <= tol_step * (np.linalg.norm(x) + tol_step):
            status = 'step'; break
    return dict(x=x, xs=xs, cost=cost, step=steps, status=status, iters=len(steps), nfev=len(cost))


def levenberg_marquardt(fun, x0, plus=_add, max_iter=200, tol_grad=1e-10, tol_step=1e-12, tau=1e-3) -> dict[str, Any]:
    """Levenberg-Marquardt with Nielsen's damping update (Madsen, Nielsen and Tingleff 2004, ch. 3):
    (J^T J + mu I) h = -g; gain ratio rho = (F(x) - F(x (+) h)) / (L(0) - L(h)), L(0) - L(h) = 1/2 h^T (mu h - g).
    rho > 0: accept, mu *= max(1/3, 1 - (2 rho - 1)^3), nu = 2. Otherwise reject, mu *= nu, nu *= 2.
    cost holds F at each accepted iterate; mu the damping used for each trial step."""
    x = np.array(x0, float); r, J = fun(x); F = 0.5 * r @ r; cost = [F]; mus = []; steps = []; xs = [x.copy()]
    A = J.T @ J; g = J.T @ r; mu = tau * np.diag(A).max(); nu = 2.0; status = 'max_iter'; nfev = 1; k = 0
    while k < max_iter:
        if np.abs(g).max() <= tol_grad:
            status = 'grad'; break
        k += 1
        h = np.linalg.solve(A + mu * np.eye(len(g)), -g); mus.append(float(mu))
        if np.linalg.norm(h) <= tol_step * (np.linalg.norm(x) + tol_step):
            status = 'step'; break
        xn = plus(x, h); rn, Jn = fun(xn); nfev += 1; Fn = 0.5 * rn @ rn
        rho = (F - Fn) / (0.5 * h @ (mu * h - g))
        if np.isfinite(Fn) and rho > 0:
            x, r, J, F = xn, rn, Jn, Fn; A = J.T @ J; g = J.T @ r
            mu *= max(1 / 3, 1 - (2 * rho - 1) ** 3); nu = 2.0; cost.append(F); steps.append(float(np.linalg.norm(h))); xs.append(x.copy())
        else:
            mu *= nu; nu *= 2
    return dict(x=x, xs=xs, cost=cost, step=steps, mu=mus, status=status, iters=k, nfev=nfev)


# ---- SE(2) point-set registration --------------------------------------------------------------------------------------------

J2 = np.array([[0.0, -1.0], [1.0, 0.0]])          # [1]x in 2-D: d/dphi R(phi) = R(phi) J2


def se2_apply(T, P):
    c, s = np.cos(T[2]), np.sin(T[2])
    return P @ np.array([[c, -s], [s, c]]).T + T[:2]


def registration_problem(P, Q, weights=None):
    """Residuals r_i = R p_i + t - q_i (known correspondences) and their Jacobian for the right perturbation T Exp(h),
    J_i = [R, R J2 p_i]. weights(r_i norms) -> w_i turns it into one IRLS step of a robust kernel."""
    P, Q = np.asarray(P, float), np.asarray(Q, float)

    def fun(T):
        c, s = np.cos(T[2]), np.sin(T[2]); R = np.array([[c, -s], [s, c]])
        r = P @ R.T + T[:2] - Q
        J = np.zeros((len(P), 2, 3)); J[:, :, :2] = R; J[:, :, 2] = P @ (R @ J2).T
        if weights is not None:
            sw = np.sqrt(weights(np.linalg.norm(r, axis=1)))[:, None]; r = r * sw; J = J * sw[:, :, None]
        return r.ravel(), J.reshape(-1, 3)
    return fun


def registration_closed_form(P, Q):
    """2-D Procrustes (Arun et al. 1987, Umeyama 1991, restricted to the plane): theta = atan2(sum cross, sum dot) of the
    centred sets, t = q_bar - R p_bar. The exact minimiser of the same cost, used to check the iterative solvers."""
    pb, qb = P.mean(0), Q.mean(0); A, B = P - pb, Q - qb
    th = np.arctan2(np.sum(A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]), np.sum(A * B))
    c, s = np.cos(th), np.sin(th)
    return np.array([*(qb - np.array([[c, -s], [s, c]]) @ pb), th])


def cauchy_weights(c):
    """Cauchy kernel rho(s) = c^2/2 log(1 + s^2/c^2): IRLS weight w = rho'(s)/s = 1 / (1 + s^2/c^2)."""
    return lambda s: 1.0 / (1.0 + (s / c) ** 2)


# ---- Go2 leg kinematic calibration -------------------------------------------------------------------------------------------

def _rx(a) -> npt.NDArray[np.float64]:
    c, s, o, z = np.cos(a), np.sin(a), np.ones_like(a), np.zeros_like(a)
    return np.stack([o, z, z, z, c, -s, z, s, c], -1).reshape(-1, 3, 3)


def _ry(a) -> npt.NDArray[np.float64]:
    c, s, o, z = np.cos(a), np.sin(a), np.ones_like(a), np.zeros_like(a)
    return np.stack([c, z, s, z, o, z, -s, z, c], -1).reshape(-1, 3, 3)


def leg_fk(q, p, hip, dy, fx):
    """Foot centre in the base frame for encoder angles q (N, 3) and parameters p = (d_hip, d_thigh, d_calf, l1, l2):
    joint zero offsets d (rad, true angle = encoder + d), thigh length l1, calf length l2 (m). Known geometry: hip origin `hip`,
    lateral thigh offset dy, foot x offset fx in the calf frame. Chain: Rx(q1) [ (0,dy,0) + Ry(q2) ( (0,0,-l1) + Ry(q3) (fx,0,-l2) ) ].
    Returns foot (N, 3) and the Jacobian (N, 3, 5) of foot w.r.t. p: joint columns z_i x (f - o_i), length columns the unit
    vector along each link."""
    a = np.atleast_2d(q) + np.asarray(p[:3]); ex, ey, ez = np.eye(3)
    R1 = _rx(a[:, 0]); R12 = R1 @ _ry(a[:, 1]); R123 = R12 @ _ry(a[:, 2])
    o2 = hip + R1 @ (dy * ey); o3 = o2 - p[3] * R12 @ ez
    f = o3 + R123 @ np.array([fx, 0.0, -p[4]]); y_ax = R1 @ ey             # thigh and knee axis in the base frame
    Jac = np.stack([np.cross(ex, f - hip), np.cross(y_ax, f - o2), np.cross(y_ax, f - o3), -R12 @ ez, -R123 @ ez], -1)
    return f, Jac


def calibration_problem(q_enc, y, hip, dy, fx):
    """Residuals r_i = foot(q_enc_i; p) - y_i for measured foot positions y (N, 3), stacked to 3N."""
    def fun(p):
        f, Jac = leg_fk(q_enc, p, hip, dy, fx)
        return (f - y).ravel(), Jac.reshape(-1, 5)
    return fun
