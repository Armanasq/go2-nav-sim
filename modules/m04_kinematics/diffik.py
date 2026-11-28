"""Differential inverse kinematics for one Go2 leg: unconstrained damped least squares and a constrained QP.

Each control tick solves for a joint step dq that moves the foot by the task displacement e:
    min_dq  ||J dq - e||^2 + lambda^2 ||dq||^2      s.t.   lo <= dq <= hi
with lo, hi the tighter of the position limits (q_lo - q, q_hi - q) and the velocity limits (+-qd_max dt). Without the bounds
the minimiser is the damped least-squares step dq = J^T (J J^T + lambda^2 I)^-1 e. The QP is solved by the primal active-set
method below, written for general A x <= b; no QP library is used. scipy.optimize.lsq_linear is used only to check it."""
import numpy as np

# Joint speed limits [rad/s] (hip, thigh, knee), from the <limit velocity=...> tags of the Unitree Go2 URDF
# (modules/m02_description/go2_gazebo.urdf). The MJCF carries no speed limits.
QD_MAX = np.array([30.1, 30.1, 20.06])


def qp_active_set(H, g, A, b, x0, max_iter=100, tol=1e-12):
    """min 1/2 x^T H x + g^T x  s.t.  A x <= b, H positive definite, x0 feasible. Primal active set
    (Nocedal and Wright, Numerical Optimization, 2nd ed., ch. 16). Returns (x, multipliers (m,), iterations)."""
    x = np.array(x0, float); n, m = len(x), len(b)
    assert np.all(A @ x <= b + 1e-9), 'x0 must be feasible'
    W = []                                                         # working set: constraints held as equalities
    for it in range(1, max_iter + 1):
        Aw = A[W].reshape(len(W), n)
        K = np.block([[H, Aw.T], [Aw, np.zeros((len(W), len(W)))]])
        sol = np.linalg.solve(K, np.concatenate([-(H @ x + g), np.zeros(len(W))]))
        p, lam = sol[:n], sol[n:]
        if np.linalg.norm(p) < tol * (1 + np.linalg.norm(x)):
            if len(W) == 0 or lam.min() >= -tol:                   # KKT point: stationary and every multiplier >= 0
                y = np.zeros(m); y[W] = lam
                return x, y, it
            W.pop(int(np.argmin(lam)))                             # drop the constraint that is pulling the wrong way
            continue
        alpha, block = 1.0, None                                   # longest step along p that stays feasible
        for i in range(m):
            ap = A[i] @ p
            if i not in W and ap > tol:
                a_i = (b[i] - A[i] @ x) / ap
                if a_i < alpha:
                    alpha, block = max(a_i, 0.0), i
        x = x + alpha * p
        if block is not None:
            W.append(block)
    raise RuntimeError('active set did not converge')


def box(lo, hi):
    """lo <= x <= hi as A x <= b."""
    n = len(lo)
    return np.vstack([np.eye(n), -np.eye(n)]), np.concatenate([hi, -np.asarray(lo)])


def step_bounds(L, q, dt, margin=0.0, qd_max=QD_MAX):
    """Bounds on one joint step: stay inside [q_lo + margin, q_hi - margin] and below qd_max * dt."""
    lo = np.maximum(L.lo + margin - q, -qd_max * dt)
    hi = np.minimum(L.hi - margin - q, qd_max * dt)
    return np.minimum(lo, 0.0), np.maximum(hi, 0.0)               # outside the band: may not move further out, dq = 0 feasible


def dls_step(J, e, lam):
    return J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(3), e)


def qp_step(J, e, lam, lo, hi):
    """Constrained step: H = 2 (J^T J + lambda^2 I), g = -2 J^T e, box bounds. Returns (dq, iterations)."""
    A, b = box(lo, hi)
    x, _, it = qp_active_set(2 * (J.T @ J + lam ** 2 * np.eye(3)), -2 * J.T @ e, A, b, np.clip(np.zeros(3), lo, hi))
    return x, it


def adaptive_damping(sigma_min, lam, eps, lam_max):
    """lambda^2 = lam^2 + lam_max^2 (1 - (sigma_min / eps)^2) inside the band sigma_min < eps, lam outside
    (variable damping of Chiaverini, Siciliano and Egeland 1994, on top of the constant floor lam)."""
    return np.sqrt(lam ** 2 + lam_max ** 2 * max(0.0, 1 - (sigma_min / eps) ** 2))


def track(L, p_ref, q0, dt, method, lam=0.01, margin=0.0, adapt=None):
    """Follow the foot reference p_ref (N, 3), base frame, sampled every dt, starting at q0. Closed-loop kinematic control: at
    tick k the task displacement is e = p_ref[k] - fk(q_{k-1}). method: 'pinv' (lambda = 0, no bounds), 'dls' (no bounds),
    'dls_clip' (DLS step, then clipped to the step bounds), 'qp'. adapt=(eps, lam_max) raises the damping near singularities
    (adaptive_damping). Returns dict of arrays q (N, 3), p (N, 3), qp_iters (N,)."""
    q = np.array(q0, float); Q, P, its = [], [], []; lam0 = lam
    for k in range(len(p_ref)):
        J = L.jacobian(q); e = p_ref[k] - L.fk(q); it = 0
        if adapt:
            lam = adaptive_damping(np.linalg.svd(J, compute_uv=False)[-1], lam0, *adapt)
        if method == 'pinv':
            dq = np.linalg.pinv(J) @ e
        elif method == 'dls':
            dq = dls_step(J, e, lam)
        elif method == 'dls_clip':
            dq = np.clip(dls_step(J, e, lam), *step_bounds(L, q, dt, margin))
        elif method == 'qp':
            dq, it = qp_step(J, e, lam, *step_bounds(L, q, dt, margin))
        else:
            raise ValueError(method)
        q = q + dq; Q.append(q.copy()); P.append(L.fk(q)); its.append(it)
    return dict(q=np.array(Q), p=np.array(P), qp_iters=np.array(its))


def reach_radius(L):
    """Largest hip-origin-to-foot distance inside the knee limit: sqrt(|s|^2 + d^2) with |s| from the law of cosines at the knee
    limit nearest straight (README, workspace boundary). Valid while the hip and thigh limits do not bind."""
    c = np.cos(L.hi[2] + L.gamma)
    return np.sqrt(L.l1 ** 2 + L.l2 ** 2 + 2 * L.l1 * L.l2 * c + L.d ** 2)


def fold_radius(L):
    """Smallest hip-origin-to-foot distance inside the knee limit: the same law of cosines at the fully folded knee limit
    (README, workspace boundaries). Valid while the hip and thigh limits do not bind."""
    c = np.cos(L.lo[2] + L.gamma)
    return np.sqrt(L.l1 ** 2 + L.l2 ** 2 + 2 * L.l1 * L.l2 * c + L.d ** 2)


def distance_to_workspace(L, p):
    """Lower bound on the distance from target p (base frame) to the reachable set, from the three boundaries the README derives:
    the outer sphere of radius reach_radius and the inner sphere of radius fold_radius about the hip origin (knee limits), and
    the cylinder |(y, z)| = |d| about the abduction axis (s_z = 0). Exact when the hip and thigh limits do not bind at the
    nearest reachable point and the target violates one boundary at a time."""
    r = np.asarray(p) - L.hip; rho = np.linalg.norm(r, axis=-1)
    outer = np.maximum(rho - reach_radius(L), 0)
    folded = np.maximum(fold_radius(L) - rho, 0)
    inner = np.maximum(abs(L.d) - np.linalg.norm(r[..., 1:], axis=-1), 0)
    return np.maximum(np.maximum(outer, folded), inner)
