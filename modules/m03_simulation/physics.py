"""Module 03 (simulation): analytic answers and first-principles models of what a rigid-body simulator does in one step.

Everything here is written out from the equations in README.md; MuJoCo is only used to build the small test models (MJCF
strings) and to read model parameters. validate.py runs the experiments and compares MuJoCo against these functions.
  integrators   explicit Euler, semi-implicit Euler (MuJoCo 'Euler'), Euler with implicit damping, classical RK4
  analytic      free fall and its semi-implicit Euler error, oscillator energy factors per step, incline threshold and
                sliding acceleration, saturated joint response
  contact       MuJoCo's soft-constraint model: solref -> stiffness and damping, solimp -> impedance d(r), regulariser R,
                reference acceleration, steady penetration and friction creep
  models        MJCF builders for the test rigs"""
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

G = 9.81
MINIMP, MAXIMP = 1e-4, 0.9999          # MuJoCo clamps the impedance to this interval (mjMINIMP, mjMAXIMP)


# ---- integrators for x'' = a(x, v), one step ---------------------------------------------------------------------------

def step_explicit_euler(x, v, h, acc):
    return x + h * v, v + h * acc(x, v)


def step_semi_implicit_euler(x, v, h, acc):
    """Velocity first, then position with the new velocity: what MuJoCo's 'Euler' does when nothing is damped."""
    v1 = v + h * acc(x, v)
    return x + h * v1, v1


def step_euler_implicit_damping(x, v, h, f, mass, b):
    """MuJoCo 'Euler' with joint damping b: the damping force is taken at the new velocity,
    (m + h b) v1 = m v + h (f(x) - b v) + h b v  ->  v1 = v + h (f(x) - b v) / (m + h b)."""
    v1 = v + h * (f(x) - b * v) / (mass + h * b)
    return x + h * v1, v1


def step_rk4(x, v, h, acc):
    k1x, k1v = v, acc(x, v)
    k2x, k2v = v + h / 2 * k1v, acc(x + h / 2 * k1x, v + h / 2 * k1v)
    k3x, k3v = v + h / 2 * k2v, acc(x + h / 2 * k2x, v + h / 2 * k2v)
    k4x, k4v = v + h * k3v, acc(x + h * k3x, v + h * k3v)
    return x + h / 6 * (k1x + 2 * k2x + 2 * k3x + k4x), v + h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)


def integrate(step, x0, v0, h, n, *args):
    """Trajectory (n+1, 2) of [x, v] from repeated calls to step(x, v, h, *args)."""
    out = np.empty((n + 1, 2)); out[0] = x0, v0
    x, v = x0, v0
    for k in range(n):
        x, v = step(x, v, h, *args)
        out[k + 1] = x, v
    return out


# ---- analytic answers --------------------------------------------------------------------------------------------------

def free_fall(t, z0=0.0, v0=0.0, g=G):
    return z0 + v0 * t - 0.5 * g * t ** 2, v0 - g * t


def free_fall_semi_implicit_offset(t, h, g=G):
    """Position error of semi-implicit Euler at t = n h under constant gravity: z_n - z(t_n) = -g h t / 2 (exact)."""
    return -0.5 * g * h * t


def free_fall_semi_implicit_energy_error(t, h, mass, g=G):
    """Energy error of the same scheme: (1/2) m v_n^2 + m g z_n - E_0 = -m g^2 h t / 2 (exact)."""
    return -0.5 * mass * g ** 2 * h * t


def oscillator_energy_factor(z, method):
    """E_{k+1} / E_k per step for x'' = -w^2 x with z = w h, E = (v^2 + w^2 x^2) / 2.
    explicit Euler: 1 + z^2 for every state. RK4: |R(iz)|^2 = 1 - z^6/72 + z^8/576 for every state."""
    if method == 'explicit_euler':
        return 1 + z ** 2
    if method == 'rk4':
        return 1 - z ** 6 / 72 + z ** 8 / 576
    raise ValueError(method)


def symplectic_modified_energy(x, v, w, h):
    """Quantity semi-implicit Euler conserves exactly on the oscillator: (v^2 + w^2 x^2 - h w^2 x v) / 2."""
    return 0.5 * (v ** 2 + w ** 2 * x ** 2 - h * w ** 2 * x * v)


def symplectic_energy_band(z):
    """max |E/E_0 - 1| of semi-implicit Euler started from rest (v0 = 0): E/E_0 lies in [1/(1+z/2), 1/(1-z/2)]."""
    return (z / 2) / (1 - z / 2)


def incline_threshold(mu):
    """Steepest angle a block holds on a plane with Coulomb friction mu: tan(theta*) = mu."""
    return np.arctan(mu)


def incline_acceleration(theta, mu, g=G):
    """Acceleration along the slope: g (sin theta - mu cos theta) when sliding, 0 when it holds."""
    return np.maximum(g * (np.sin(theta) - mu * np.cos(theta)), 0.0)


def pyramid_friction_ratio(phi):
    """MuJoCo's pyramidal cone (condim 3) has edges n +- mu t1, n +- mu t2: its tangential cross-section is the square
    |f1| + |f2| <= mu f_n, so the friction limit in direction phi from t1 is mu / (|cos phi| + |sin phi|)."""
    return 1.0 / (abs(np.cos(phi)) + abs(np.sin(phi)))


def saturated_response(t, inertia, tau, b, q0=0.0, dq0=0.0):
    """Joint under constant net torque tau and viscous damping b:  I q'' = tau - b q'.
    q'(t) = tau/b + (dq0 - tau/b) e^{-t/T},  q(t) = q0 + tau t / b + (dq0 - tau/b) T (1 - e^{-t/T}),  T = I / b."""
    T, vinf = inertia / b, tau / b
    e = np.exp(-t / T)
    return q0 + vinf * t + (dq0 - vinf) * T * (1 - e), vinf + (dq0 - vinf) * e


def saturated_response_discrete(n, h, inertia, tau, b, q0=0.0):
    """The same joint stepped by MuJoCo's Euler (damping implicit): v_{k+1} = (v_k + h tau / I) / (1 + h b / I) from rest gives
    v_n = (tau/b)(1 - rho^n), rho = 1 / (1 + h b / I), and q_n = q0 + h sum_{k=1..n} v_k = q0 + h (tau/b)(n - rho (1 - rho^n) / (1 - rho))."""
    rho, vinf = 1 / (1 + h * b / inertia), tau / b
    n = np.asarray(n, float)
    return q0 + h * vinf * (n - rho * (1 - rho ** n) / (1 - rho)), vinf * (1 - rho ** n)


def pd_saturated_reference(t_end, inertia, b, frictionloss, kp, kd, q_des, tau_max, q0, dq0=0.0, rtol=1e-11):
    """Reference for the bench: I q'' = clip(kp (q_des - q) - kd q', +-tau_max) - b q' - frictionloss sign(q'), integrated by
    scipy (DOP853) up to t_end or to the first stop (q' = 0 after moving), whichever comes first. Returns the OdeResult."""
    def f(t, y):
        tau = np.clip(kp * (q_des - y[0]) - kd * y[1], -tau_max, tau_max)
        return [y[1], (tau - b * y[1] - frictionloss * np.sign(y[1] if y[1] != 0 else q_des - y[0])) / inertia]

    def stop(t, y):
        return y[1] - 1e-9 * np.sign(q_des - q0)
    stop.terminal, stop.direction = True, -np.sign(q_des - q0)
    return solve_ivp(f, (0, t_end), [q0, dq0], method='DOP853', rtol=rtol, atol=1e-13, dense_output=True, events=stop,
                     max_step=1e-4)


def inertia_about_axis(mass, com, I_com, axis, armature=0.0):
    """Moment of inertia of a rigid body about the line through the origin along the unit `axis`, plus motor armature:
    a^T I_c a + m (|c|^2 - (a.c)^2) + armature (parallel-axis theorem)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis); c = np.asarray(com, float)
    return float(a @ I_com @ a + mass * (c @ c - (a @ c) ** 2) + armature)


def fit_order(h, err):
    """Slope of log(err) against log(h), least squares: the observed order of accuracy."""
    return float(np.polyfit(np.log(h), np.log(err), 1)[0])


# ---- MuJoCo's soft contact, written out -------------------------------------------------------------------------------

def impedance(r, solimp):
    """d(r) from solimp = (d0, dwidth, width, midpoint, power): a sigmoid in x = |r| / width from d0 at x = 0 to dwidth at
    x >= 1. y(x) = x^p / mid^(p-1) for x <= mid, 1 - (1-x)^p / (1-mid)^(p-1) above; d = d0 + y (dwidth - d0)."""
    d0, dw, width, mid, p = solimp
    d0, dw = np.clip(d0, MINIMP, MAXIMP), np.clip(dw, MINIMP, MAXIMP)
    x = abs(r) / width if width > 0 else 1.0
    if x >= 1:
        y = 1.0
    elif p == 1:
        y = x
    elif x <= mid:
        y = x ** p / mid ** (p - 1)
    else:
        y = 1 - (1 - x) ** p / (1 - mid) ** (p - 1)
    return float(np.clip(d0 + y * (dw - d0), MINIMP, MAXIMP))


def stiffness_damping(solref, solimp, h=0.002, refsafe=True):
    """(K, B) of the reference acceleration a_ref = -B v - K d r. Standard solref = (timeconst, dampratio) > 0:
    K = 1 / (dmax^2 tc^2 zeta^2), B = 2 / (dmax tc), with dmax = dwidth and, under refsafe, tc >= 2 h.
    Direct solref = (-stiffness, -damping): K = stiffness / dmax^2, B = damping / dmax."""
    dmax = float(np.clip(solimp[1], MINIMP, MAXIMP))
    if solref[0] > 0:
        tc = max(solref[0], 2 * h) if refsafe else solref[0]
        return 1 / (dmax ** 2 * tc ** 2 * solref[1] ** 2), 2 / (dmax * tc)
    return -solref[0] / dmax ** 2, -solref[1] / dmax


def reference_acceleration(r, v, solref, solimp, h=0.002):
    K, B = stiffness_damping(solref, solimp, h)
    return -B * v - K * impedance(r, solimp) * r


def regulariser(r, solimp, diag_a):
    """R = (1 - d) / d * A_hat: the softness the solver adds to the constraint's diagonal (A_hat = MuJoCo's diagonal
    approximation of J M^-1 J^T, the body's inverse weight for a contact normal)."""
    d = impedance(r, solimp)
    return (1 - d) / d * diag_a


def inverse_weight(J_com, M):
    """MuJoCo's body_invweight0 for translation: mean of the diagonal of J M^-1 J^T, J the body-COM Jacobian at qpos0."""
    return float(np.trace(J_com @ np.linalg.solve(M, J_com.T)) / 3)


def steady_penetration(f_normal, diag_a, solref, solimp, margin=0.0, h=0.002, refsafe=True):
    """Penetration depth (m, positive into the surface) at which a resting contact carries f_normal.
    At rest J a = 0 and v = 0, so for a constraint inside its cone a_ref = R f, i.e. K d(u) u = (1 - d(u)) / d(u) A f,
    with u = depth + margin the violation. Solved for u by bracketing (the left side minus the right grows with u)."""
    K, _ = stiffness_damping(solref, solimp, h, refsafe)

    def g(u):
        d = impedance(u, solimp)
        return K * d * d * u - (1 - d) * diag_a * f_normal
    u = brentq(g, 0.0, 1.0, xtol=1e-15, rtol=1e-14)
    return u - margin


def creep_speed(f_tangential, diag_a, r, solref, solimp, impratio=1.0, h=0.002):
    """Steady sliding speed of a contact that should hold (elliptic cone, friction inside the cone): the friction row has
    zero violation, so 0 = a_ref - R_t f_t = -B v - R_t f_t, R_t = R_n / impratio (equal tangential mu). |v| = R_t f_t / B."""
    _, B = stiffness_damping(solref, solimp, h)
    return regulariser(r, solimp, diag_a) / impratio * f_tangential / B


# ---- MJCF test rigs ----------------------------------------------------------------------------------------------------

def _attrs(solref, solimp):
    s = ''
    if solref is not None:
        s += f' solref="{solref[0]} {solref[1]}"'
    if solimp is not None:
        s += ' solimp="' + ' '.join(str(x) for x in solimp) + '"'
    return s


def sphere_xml(mass=1.0, radius=0.05, z0=1.0, h=0.002, integrator='Euler', floor=True, solref=None, solimp=None):
    """Free sphere, optionally above a plane; frictionless contact (condim 1) with the same solref/solimp on both geoms,
    so MuJoCo's parameter mixing returns them unchanged."""
    a = _attrs(solref, solimp)
    plane = f'<geom type="plane" size="2 2 .1" condim="1"{a}/>' if floor else ''
    return f'''<mujoco><option timestep="{h}" integrator="{integrator}"/><worldbody>{plane}
  <body pos="0 0 {z0}"><freejoint/><geom type="sphere" size="{radius}" mass="{mass}" condim="1"{a}/></body>
</worldbody></mujoco>'''


def oscillator_xml(mass=1.0, k=100.0, b=0.0, h=0.01, integrator='Euler'):
    """Mass on a slide joint with a spring (stiffness k) and a damper (b), no gravity, no armature."""
    return f'''<mujoco><option timestep="{h}" integrator="{integrator}" gravity="0 0 0"/><worldbody>
  <body><joint name="x" type="slide" axis="1 0 0" stiffness="{k}" damping="{b}"/>
    <geom type="sphere" size="0.05" mass="{mass}" contype="0" conaffinity="0"/></body>
</worldbody></mujoco>'''


def slider_xml(mu, cone='elliptic', impratio=1.0, h=0.002, mass=1.0, solref=None, solimp=None):
    """Point-like block: a sphere on three slide joints (it cannot roll or tip), on a plane, Coulomb friction mu
    (condim 3, same mu on both geoms). Gravity is tilted by the experiment to make the plane an incline."""
    a = _attrs(solref, solimp)
    return f'''<mujoco><option timestep="{h}" cone="{cone}" impratio="{impratio}"/><worldbody>
  <geom type="plane" size="5 5 .1" condim="3" friction="{mu} 0 0"{a}/>
  <body pos="0 0 0.05"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>
    <geom type="sphere" size="0.05" mass="{mass}" condim="3" friction="{mu} 0 0"{a}/></body>
</worldbody></mujoco>'''


def knee_bench_xml(m_go2, leg='FL', h=0.002, gravity=False):
    """The Go2 calf on a bench: thigh clamped, knee hinge with the Go2's joint parameters (damping, armature, friction loss,
    range) and motor (ctrlrange), calf mass, centre of mass and inertia copied from the compiled Menagerie model."""
    import mujoco
    b = mujoco.mj_name2id(m_go2, mujoco.mjtObj.mjOBJ_BODY, f'{leg}_calf')
    j = mujoco.mj_name2id(m_go2, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_calf_joint')
    a = mujoco.mj_name2id(m_go2, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{leg}_calf')
    dof = m_go2.jnt_dofadr[j]
    f = lambda v: ' '.join(f'{x:.17g}' for x in np.ravel(v))  # noqa: E731
    g = '0 0 -9.81' if gravity else '0 0 0'
    return f'''<mujoco><compiler angle="radian"/><option timestep="{h}" gravity="{g}"/><worldbody>
  <body name="calf"><inertial pos="{f(m_go2.body_ipos[b])}" quat="{f(m_go2.body_iquat[b])}" mass="{m_go2.body_mass[b]:.17g}"
      diaginertia="{f(m_go2.body_inertia[b])}"/>
    <joint name="knee" axis="{f(m_go2.jnt_axis[j])}" damping="{m_go2.dof_damping[dof]:.17g}" armature="{m_go2.dof_armature[dof]:.17g}"
      frictionloss="{m_go2.dof_frictionloss[dof]:.17g}" range="{f(m_go2.jnt_range[j])}" limited="true"/>
    <geom type="capsule" fromto="0 0 0 0 0 -0.213" size="0.012" contype="0" conaffinity="0" mass="0"/></body>
</worldbody><actuator><motor name="knee" joint="knee" ctrlrange="{f(m_go2.actuator_ctrlrange[a])}" ctrllimited="true"/></actuator>
</mujoco>'''
