#!/usr/bin/env python3
"""Experiment for module 00, all three levels.
  fundamental, intermediate: rotations.py against scipy and Pinocchio on random samples, and the worked numbers (yaw from the gyro,
      camera optical frame, tilt height error).
  advanced: SO(3) right/left Jacobians against finite differences and Pinocchio; Gauss-Newton and Levenberg-Marquardt (optim.py)
      on SE(2) point-set registration and on Go2 leg kinematic calibration, against the closed form and scipy.optimize.least_squares.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--n 100000] [--no-media]"""
import argparse, json, math, pathlib, sys, time
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import pinocchio as pin  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from rotations import (R_LINK_OPTICAL, R_to_quat, R_to_rpy, Ry, exp_so3, log_so3, quat_to_R, rpy_rates, rpy_to_R,  # noqa: E402
                       se2_compose, se2_inverse, se3, se3_inverse, skew)
from lie import SMALL, jl_so3, jl_inv_so3, jr_inv_so3, jr_so3, se2_exp, se2_log, se2_plus  # noqa: E402
from optim import (calibration_problem, cauchy_weights, gauss_newton, leg_fk, levenberg_marquardt,  # noqa: E402
                   registration_closed_form, registration_problem, se2_apply)

FIG, MEDIA = HERE / 'figures', HERE / 'media'

# Acceptance thresholds for the advanced level, fixed before the first full run (MODULE_SPEC.md). (key, op, threshold)
ACCEPT = {
    'jr_vs_fd': ('<', 1e-8), 'jl_vs_fd': ('<', 1e-8), 'jr_inv_vs_fd': ('<', 1e-7), 'jr_inv_times_jr_minus_I': ('<', 1e-12),
    'jl_equals_R_jr': ('<', 1e-14), 'jr_vs_pinocchio_Jexp3': ('<', 1e-14), 'jr_inv_vs_pinocchio_Jlog3': ('<', 1e-10),
    'jacobian_jump_at_series_switch': ('<', 1e-14), 'first_order_slope_with_jr': ('range', (1.9, 2.1)),
    'first_order_slope_without_jr': ('range', (0.9, 1.1)),
    'se2_exp_log_roundtrip': ('<', 1e-14), 'registration_jacobian_vs_fd': ('<', 1e-8),
    'registration_gn_vs_closed_form': ('<', 1e-9), 'registration_lm_vs_closed_form': ('<', 1e-9),
    'registration_scipy_vs_closed_form': ('<', 1e-9), 'registration_gn_success_all_angles': ('>=', 1.0),
    'registration_lm_success_all_angles': ('>=', 1.0), 'registration_gn_max_iters': ('<=', 10),
    'registration_cauchy_20pct_outliers_rot_err_deg': ('<', 0.5), 'registration_cauchy_20pct_outliers_trans_err_m': ('<', 0.02),
    'calib_fk_vs_mujoco_m': ('<', 1e-12), 'calib_jacobian_vs_fd': ('<', 1e-8), 'calib_gn_vs_scipy': ('<', 1e-8),
    'calib_lm_vs_scipy': ('<', 1e-8), 'calib_cost_rel_diff_vs_scipy': ('<', 1e-10), 'calib_max_err_in_sd': ('<', 3.0),
    'calib_mean_nees': ('range', (4.5, 5.5)), 'calib_lm_success_at_0p5rad': ('>=', 1.0), 'calib_gn_success_at_0p5rad': ('>=', 1.0),
    'calib_lm_not_worse_than_gn': ('>=', 1.0), 'calib_rms_over_predicted_sd_max': ('<', 1.5),
    'media_mp4_under_6MB': ('>=', 1.0),
}


def check(key, val):
    op, thr = ACCEPT[key]
    return bool(thr[0] <= val <= thr[1]) if op == 'range' else bool({'<': val < thr, '<=': val <= thr, '>=': val >= thr}[op])


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


# ---- fundamental + intermediate (validated earlier, unchanged) ---------------------------------------------------------------

def fundamentals(n):
    rng = np.random.default_rng(0); rots = Rotation.random(n, random_state=0); err = dict.fromkeys(
        ['quat_to_R_vs_scipy', 'R_to_quat_vs_scipy', 'rpy_to_R_vs_pinocchio', 'R_to_rpy_vs_pinocchio', 'exp_vs_pinocchio',
         'log_vs_pinocchio', 'log_vs_scipy', 'se3_inverse_vs_pinocchio', 'rpy_rates_vs_pinocchio', 'gimbal_lock_rpy_roundtrip',
         'R_link_optical_vs_pinocchio'], 0.0)

    def upd(k, e):
        err[k] = max(err[k], float(np.abs(e).max()))
    for r in rots:
        x, y, z, w = r.as_quat(); R = r.as_matrix(); rv = r.as_rotvec(); t = rng.normal(size=3)
        upd('quat_to_R_vs_scipy', quat_to_R([w, x, y, z]) - R)
        upd('R_to_quat_vs_scipy', R_to_quat(R) - np.sign(w) * np.array([w, x, y, z]))
        rpy = pin.rpy.matrixToRpy(R)
        upd('rpy_to_R_vs_pinocchio', rpy_to_R(*rpy) - pin.rpy.rpyToMatrix(rpy))
        upd('R_to_rpy_vs_pinocchio', R_to_rpy(R) - rpy)
        upd('exp_vs_pinocchio', exp_so3(rv) - pin.exp3(rv))
        upd('log_vs_pinocchio', log_so3(R) - pin.log3(R)); upd('log_vs_scipy', log_so3(R) - rv)
        upd('se3_inverse_vs_pinocchio', se3_inverse(se3(R, t)) - pin.SE3(R, t).inverse().homogeneous)
        if abs(rpy[1]) < 1.5:                                       # rates are singular at |pitch| = pi/2
            rates = rng.normal(size=3)
            upd('rpy_rates_vs_pinocchio', rpy_rates(rpy, pin.rpy.computeRpyJacobian(rpy, pin.LOCAL) @ rates) - rates)
    # Gimbal lock: at pitch +-pi/2 only phi -+ psi is defined; R_to_rpy sets psi = 0 and must still reconstruct R.
    rg = np.random.default_rng(6)
    for pitch in (np.pi / 2, -np.pi / 2):
        for _ in range(1000):
            R = rpy_to_R(rg.uniform(-np.pi, np.pi), pitch, rg.uniform(-np.pi, np.pi)); rpy = R_to_rpy(R)
            assert rpy[2] == 0.0
            upd('gimbal_lock_rpy_roundtrip', rpy_to_R(*rpy) - R)
    upd('R_link_optical_vs_pinocchio', R_LINK_OPTICAL - pin.rpy.rpyToMatrix(-np.pi / 2, 0, -np.pi / 2))
    for k, v in err.items():
        print(f'{k:28s} max error {v:.1e}')
    # Yaw from omega_z alone during a steady turn at constant roll and pitch: omega_z = cos(phi) cos(theta) psi_dot.
    yaw = {}
    for roll_deg, pitch_deg in ((1.0, 0.0), (2.1, 0.0), (3.6, 0.0), (3.6, 2.0), (10.0, 0.0)):
        phi, th = np.deg2rad([roll_deg, pitch_deg]); dt = 1e-3; ts = np.arange(0, 2 * np.pi, dt)     # 1 rad/s, one full turn
        Rs = [rpy_to_R(phi, th, t) for t in ts]
        wz = [Rotation.from_matrix(R0.T @ R1).as_rotvec()[2] / dt for R0, R1 in zip(Rs[:-1], Rs[1:])]
        integ = np.rad2deg(ts[-1] - np.sum(wz) * dt); k = f'roll {roll_deg} pitch {pitch_deg}'
        yaw[k] = dict(predicted_deg_per_turn=float(np.rad2deg(2 * np.pi * (1 - np.cos(phi) * np.cos(th)))),
                      integrated_deg_per_turn=float(integ), relative=float(1 - np.cos(phi) * np.cos(th)))
        print(f'yaw from omega_z, {k}: {yaw[k]["predicted_deg_per_turn"]:.3f} deg/turn predicted, {integ:.3f} integrated')
    h = 3 * np.sin(np.deg2rad(3.6))
    return dict(n=n, max_error=err, yaw_from_omega_z=yaw, R_link_optical=R_LINK_OPTICAL.round(12).tolist(), height_shift_3m_3p6deg_m=h)


# ---- advanced: SO(3) Jacobians -----------------------------------------------------------------------------------------------

def random_rotvecs(rng, n, tmax=np.pi - 1e-3):
    a = rng.normal(size=(n, 3)); a /= np.linalg.norm(a, axis=1, keepdims=True)
    return a * rng.uniform(0, tmax, (n, 1))


def fd(f, x, eps=1e-6):
    return np.stack([(f(x + eps * e) - f(x - eps * e)) / (2 * eps) for e in np.eye(len(x))], 1)


BERNOULLI = [1, -1 / 2, 1 / 6, 0, -1 / 30, 0, 1 / 42, 0, -1 / 30, 0, 5 / 66]


def series_jr(w):
    """Jr(w) = sum_k (-[w])^k / (k+1)!, 11 terms: exact to rounding for |w| <= 1e-2."""
    W = -skew(w); return sum(np.linalg.matrix_power(W, k) / math.factorial(k + 1) for k in range(11))


def series_jr_inv(w):
    """Jr(w)^-1 = sum_k B_k (-[w])^k / k! with the Bernoulli numbers B_k (B_1 = -1/2)."""
    W = -skew(w); return sum(BERNOULLI[k] * np.linalg.matrix_power(W, k) / math.factorial(k) for k in range(11))


def jacobians(n):
    rng = np.random.default_rng(1); W = random_rotvecs(rng, n); e = dict.fromkeys(
        ['jr_vs_fd', 'jl_vs_fd', 'jr_inv_vs_fd', 'jr_inv_times_jr_minus_I', 'jl_equals_R_jr', 'jr_vs_pinocchio_Jexp3',
         'jr_inv_vs_pinocchio_Jlog3'], 0.0)

    def upd(k, v):
        e[k] = max(e[k], float(np.abs(v).max()))
    worst = dict(jr=(0.0, 0.0), jri=(0.0, 0.0))
    for w in W:
        R = exp_so3(w); Jr = jr_so3(w); Jri = jr_inv_so3(w); t = np.linalg.norm(w)
        for k, v in (('jr', np.abs(Jr - pin.Jexp3(w)).max()), ('jri', np.abs(Jri - pin.Jlog3(R)).max())):
            worst[k] = max(worst[k], (float(v), float(t)))
        upd('jr_vs_fd', Jr - fd(lambda x: log_so3(R.T @ exp_so3(x)), w))                 # Exp(w + d) = Exp(w) Exp(Jr d)
        upd('jl_vs_fd', jl_so3(w) - fd(lambda x: log_so3(exp_so3(x) @ R.T), w))          # Exp(w + d) = Exp(Jl d) Exp(w)
        upd('jr_inv_vs_fd', Jri - fd(lambda d: log_so3(R @ exp_so3(d)), np.zeros(3)))  # Log(Exp(w) Exp(d)) = w + Jr^-1 d
        upd('jr_inv_times_jr_minus_I', Jri @ Jr - np.eye(3)); upd('jl_equals_R_jr', jl_so3(w) - R @ Jr)
        upd('jr_vs_pinocchio_Jexp3', Jr - pin.Jexp3(w)); upd('jr_inv_vs_pinocchio_Jlog3', Jri - pin.Jlog3(R))
    a = W[0] / np.linalg.norm(W[0]); lo, hi = a * SMALL * (1 - 1e-9), a * SMALL * (1 + 1e-9)
    e['jacobian_jump_at_series_switch'] = float(max(np.abs(f(lo) - f(hi)).max() for f in (jr_so3, jr_inv_so3)))
    # First-order accuracy: |Log(Exp(w)^T Exp(w + eps d)) - eps Jr d| ~ eps^2, against ~eps when Jr is replaced by I.
    w = np.array([0.6, -0.9, 0.5]); dvec = np.array([0.3, 0.8, -0.52]); dvec /= np.linalg.norm(dvec); R = exp_so3(w)
    eps = np.logspace(-5, -0.5, 19)
    err_jr = np.array([np.linalg.norm(log_so3(R.T @ exp_so3(w + s * dvec)) - s * jr_so3(w) @ dvec) for s in eps])
    err_id = np.array([np.linalg.norm(log_so3(R.T @ exp_so3(w + s * dvec)) - s * dvec) for s in eps])
    sl = slice(0, 12)                                                                    # eps <= 1e-2, before higher orders
    e['first_order_slope_with_jr'] = float(np.polyfit(np.log(eps[sl]), np.log(err_jr[sl]), 1)[0])
    e['first_order_slope_without_jr'] = float(np.polyfit(np.log(eps[sl]), np.log(err_id[sl]), 1)[0])
    e['n'] = n; e['first_order_w'] = w.tolist(); e['first_order_w_norm_rad'] = float(np.linalg.norm(w))
    e['jr_vs_pinocchio_worst_angle_rad'] = worst['jr'][1]; e['jr_inv_vs_pinocchio_worst_angle_rad'] = worst['jri'][1]
    # Diagnostic added after the Pinocchio rows missed: an independent reference, the Taylor series summed to 11 terms, on
    # 2000 rotation vectors with |w| in [1e-4, 1e-2] rad, where those misses occurred.
    tt = np.logspace(-4, -2, 2000); a_ = rng.normal(size=(2000, 3))
    Ws = a_ / np.linalg.norm(a_, axis=1, keepdims=True) * tt[:, None]
    dg = dict.fromkeys(['jr_vs_series', 'pin_Jexp3_vs_series', 'jr_inv_vs_series', 'pin_Jlog3_vs_series'], 0.0)
    for ws in Ws:                                     # not `w`: w is the first-order test vector returned for the figure
        S, Si = series_jr(ws), series_jr_inv(ws)
        for k, v in (('jr_vs_series', jr_so3(ws) - S), ('pin_Jexp3_vs_series', pin.Jexp3(ws) - S),
                     ('jr_inv_vs_series', jr_inv_so3(ws) - Si), ('pin_Jlog3_vs_series', pin.Jlog3(exp_so3(ws)) - Si)):
            dg[k] = max(dg[k], float(np.abs(v).max()))
    big = W[np.linalg.norm(W, axis=1) > 1e-2]
    dg['pin_Jexp3_vs_jr_above_1e-2rad'] = float(max(np.abs(jr_so3(w) - pin.Jexp3(w)).max() for w in big))
    dg['pin_Jlog3_vs_jr_inv_above_1e-2rad'] = float(max(np.abs(jr_inv_so3(w) - pin.Jlog3(exp_so3(w))).max() for w in big))
    e['small_angle_diagnostic'] = dg
    # Worked example: gyro x bias b during a 90 deg yaw turn at 1 rad/s. Exp((omega + b) T) = Exp(omega T) Exp(Jr(omega T) b T).
    om, T, b = np.array([0.0, 0.0, 1.0]), np.pi / 2, np.array([0.01, 0.0, 0.0])
    exact = log_so3(exp_so3(om * T).T @ exp_so3((om + b) * T))
    e['bias_turn_example'] = dict(first_order=(jr_so3(om * T) @ b * T).tolist(), exact=exact.tolist(), naive=(b * T).tolist(),
                                  first_order_err=float(np.linalg.norm(exact - jr_so3(om * T) @ b * T)),
                                  naive_err=float(np.linalg.norm(exact - b * T)))
    for k, v in e.items():
        print(f'SO(3) Jacobians {k:34s} {v}')
    return e, dict(eps=eps, err_jr=err_jr, err_id=err_id, w=w)


# ---- advanced: SE(2) registration --------------------------------------------------------------------------------------------

T_TRUE = np.array([0.4, -0.25, np.deg2rad(35.0)])
SIGMA_SCAN = 0.01                                                                        # m, point noise


def make_scan(rng, outlier_frac=0.0):
    """A 2-D scan of a 4 m x 3 m room with a 0.6 m box, in the scan frame (P), and the same points in the map frame (Q) with
    noise: Q = T_TRUE P + N(0, SIGMA_SCAN^2). A fraction of Q is replaced by uniform points (wrong correspondences)."""
    walls = [((-2, -1.5), (2, -1.5)), ((2, -1.5), (2, 1.5)), ((2, 1.5), (-2, 1.5)), ((-2, 1.5), (-2, -1.5)),
             ((0.6, 0.2), (1.2, 0.2)), ((1.2, 0.2), (1.2, 0.8)), ((1.2, 0.8), (0.6, 0.8)), ((0.6, 0.8), (0.6, 0.2))]
    P = np.concatenate([np.linspace(a, b, 12 if np.hypot(*np.subtract(b, a)) > 1 else 5, endpoint=False) for a, b in walls])
    Q = se2_apply(T_TRUE, P) + SIGMA_SCAN * rng.normal(size=P.shape)
    bad = np.zeros(len(P), bool)
    if outlier_frac:
        bad[rng.choice(len(P), int(round(outlier_frac * len(P))), replace=False)] = True
        Q[bad] = rng.uniform(-2.5, 2.5, (bad.sum(), 2))
    return P, Q, bad


def pose_err(a, b):
    return max(np.abs(a[:2] - b[:2]).max(), abs(wrap(a[2] - b[2])))


def registration():
    rng = np.random.default_rng(2); P, Q, _ = make_scan(rng); f = registration_problem(P, Q); cf = registration_closed_form(P, Q)
    out = {}
    out['se2_exp_log_roundtrip'] = float(max(np.abs(se2_exp(se2_log(T)) - T).max() for T in
                                             np.c_[rng.uniform(-3, 3, (1000, 2)), rng.uniform(-np.pi + 1e-9, np.pi, 1000)]))
    T0 = np.array([0.1, 0.2, 0.3])
    out['registration_jacobian_vs_fd'] = float(np.abs(f(T0)[1] - fd(lambda h: f(se2_plus(T0, h))[0], np.zeros(3))).max())
    x0 = np.array([0.0, 0.0, np.deg2rad(170.0)])
    G = gauss_newton(f, x0, se2_plus); L = levenberg_marquardt(f, x0, se2_plus)
    sc_cost = []

    def sfun(x):
        r = f(x)[0]; sc_cost.append(0.5 * r @ r); return r
    S = least_squares(sfun, x0, jac=lambda x: _jac_vec(P, x), method='lm',
                      xtol=1e-15, ftol=1e-15, gtol=1e-15)
    out.update(registration_gn_vs_closed_form=pose_err(G['x'], cf), registration_lm_vs_closed_form=pose_err(L['x'], cf),
               registration_scipy_vs_closed_form=pose_err(S.x, cf), registration_gn_iters_170deg=G['iters'],
               registration_lm_iters_170deg=L['iters'], registration_lm_accepted_steps_170deg=len(L['step']),
               registration_lm_final_cost_170deg=float(L['cost'][-1]), registration_lm_mu_per_trial_170deg=L['mu'],
               registration_scipy_nfev_170deg=int(S.nfev),
               registration_scipy_residual_calls_170deg=len(sc_cost),
               registration_closed_form_err_vs_truth_m=float(np.abs(cf[:2] - T_TRUE[:2]).max()),
               registration_closed_form_err_vs_truth_deg=float(np.rad2deg(abs(wrap(cf[2] - T_TRUE[2])))))
    # Basin: every initial heading on a 5 deg grid, translation guess at the origin.
    th0 = np.arange(-180, 180, 5.0); basin = dict(theta0_deg=th0.tolist(), gn_iters=[], lm_iters=[], gn_ok=[], lm_ok=[])
    for th in th0:
        x0b = np.array([0.0, 0.0, np.deg2rad(th)])
        for name, solver in (('gn', gauss_newton), ('lm', levenberg_marquardt)):
            r = solver(f, x0b, se2_plus); basin[f'{name}_iters'].append(r['iters']); basin[f'{name}_ok'].append(bool(pose_err(r['x'], cf) < 1e-9))
    out.update(registration_gn_success_all_angles=float(np.mean(basin['gn_ok'])),
               registration_lm_success_all_angles=float(np.mean(basin['lm_ok'])),
               registration_gn_max_iters=int(max(basin['gn_iters'])), registration_lm_max_iters=int(max(basin['lm_iters'])))
    i = int(np.argmax(basin['gn_iters'])); slow = gauss_newton(f, np.array([0.0, 0.0, np.deg2rad(th0[i])]), se2_plus)
    out.update(registration_gn_slowest_start_deg=float(th0[i]),
               registration_gn_headings_from_slowest_start_deg=[round(float(np.rad2deg(x[2])), 2) for x in slow['xs']])
    # Outliers: 20 % wrong correspondences, plain least squares against Cauchy IRLS (c = 5 sigma), both GN from the identity.
    Po, Qo, bad = make_scan(np.random.default_rng(3), 0.2)
    ls = gauss_newton(registration_problem(Po, Qo), np.zeros(3), se2_plus)
    ca = gauss_newton(registration_problem(Po, Qo, cauchy_weights(5 * SIGMA_SCAN)), ls['x'], se2_plus, max_iter=200, tol_grad=1e-9)
    for name, r in (('ls', ls), ('cauchy', ca)):
        out[f'registration_{"plain" if name == "ls" else "cauchy"}_20pct_outliers_rot_err_deg'] = float(np.rad2deg(abs(wrap(r['x'][2] - T_TRUE[2]))))
        out[f'registration_{"plain" if name == "ls" else "cauchy"}_20pct_outliers_trans_err_m'] = float(np.linalg.norm(r['x'][:2] - T_TRUE[:2]))
    out['registration_cauchy_iters'] = ca['iters']; out['registration_cauchy_status'] = ca['status']
    for k, v in out.items():
        print(f'SE(2) registration {k:46s} {v}')
    return out, dict(P=P, Q=Q, cf=cf, G=G, L=L, S=S, sc_cost=sc_cost, basin=basin, Po=Po, Qo=Qo, bad=bad, ls=ls, ca=ca)


def _jac_vec(P, x):
    """Jacobian of the residual w.r.t. the plain vector (x, y, theta), for scipy, which adds steps: [I, J2 R p]."""
    c, s = np.cos(x[2]), np.sin(x[2]); J = np.zeros((len(P), 2, 3)); J[:, 0, 0] = J[:, 1, 1] = 1
    J[:, 0, 2] = -s * P[:, 0] - c * P[:, 1]; J[:, 1, 2] = c * P[:, 0] - s * P[:, 1]
    return J.reshape(-1, 3)


# ---- advanced: Go2 FL leg kinematic calibration ------------------------------------------------------------------------------

P_TRUE = np.array([np.deg2rad(2.0), np.deg2rad(-3.0), np.deg2rad(4.0), 0.213, 0.213])    # encoder zero offsets, l1, l2 (MJCF)
P_NOMINAL = np.array([0.0, 0.0, 0.0, 0.20, 0.20])                                         # the initial guess
SIGMA_MOCAP = 1e-3                                                                         # m, foot position noise
N_POSES = 40


class FLLeg:
    """Known geometry and the MuJoCo 'motion capture' of the FL foot: set joint angles, mj_kinematics, read the foot geom."""

    def __init__(self, m, d):
        self.m, self.d = m, d
        self.jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'FL_{p}_joint') for p in ('hip', 'thigh', 'calf')]
        self.adr = m.jnt_qposadr[self.jid]; self.lo, self.hi = m.jnt_range[self.jid].T
        self.geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
        self.hip = m.body_pos[b].copy(); self.dy = float(m.body_pos[b + 1][1]); self.fx = float(m.geom_pos[self.geom][0])
        assert np.isclose(m.body_pos[b + 2][2], -P_TRUE[3]) and np.isclose(m.geom_pos[self.geom][2], -P_TRUE[4])

    def measure(self, q):
        """True joint angles (N, 3) -> foot centres in the base frame, base at the origin with identity orientation."""
        out = []
        for qq in np.atleast_2d(q):
            self.d.qpos[:] = 0; self.d.qpos[3] = 1; self.d.qpos[self.adr] = qq; mujoco.mj_kinematics(self.m, self.d)
            out.append(self.d.geom_xpos[self.geom].copy())
        return np.array(out)

    def poses(self, rng, n=N_POSES, half_range=None):
        """True joint angles: uniform over the joint limits, or within +-half_range of the standing pose, clipped to the limits."""
        if half_range is None:
            return rng.uniform(self.lo, self.hi, (n, 3))
        return np.clip(sim.Q_STAND[:3] + rng.uniform(-half_range, half_range, (n, 3)), self.lo, self.hi)

    def problem(self, q_true, rng):
        y = self.measure(q_true) + SIGMA_MOCAP * rng.normal(size=(len(q_true), 3))
        return calibration_problem(q_true - P_TRUE[:3], y, self.hip, self.dy, self.fx)


def predicted_sd(fun, p):
    J = fun(p)[1]
    return SIGMA_MOCAP * np.sqrt(np.diag(np.linalg.inv(J.T @ J))), J


def calib_ok(p, ref):
    return bool(np.abs(wrap(p[:3] - ref[:3])).max() < 1e-6 and np.abs(p[3:] - ref[3:]).max() < 1e-6)


def calibration(m, d):
    leg = FLLeg(m, d); out = {}; rng = np.random.default_rng(4)
    q = leg.poses(rng, 2000)
    out['calib_fk_vs_mujoco_m'] = float(np.abs(leg_fk(q, np.r_[0, 0, 0, P_TRUE[3:]], leg.hip, leg.dy, leg.fx)[0] - leg.measure(q)).max())
    q = leg.poses(rng); f = leg.problem(q, rng)
    out['calib_jacobian_vs_fd'] = float(np.abs(f(P_NOMINAL)[1] - fd(lambda p: f(p)[0], P_NOMINAL)).max())
    G = gauss_newton(f, P_NOMINAL); L = levenberg_marquardt(f, P_NOMINAL); sc_cost = []

    def sfun(p):
        r = f(p)[0]; sc_cost.append(0.5 * r @ r); return r
    S = least_squares(sfun, P_NOMINAL, jac=lambda p: f(p)[1], method='lm', xtol=1e-15, ftol=1e-15, gtol=1e-15)
    sd, _ = predicted_sd(f, S.x); err = S.x - P_TRUE
    out.update(calib_gn_vs_scipy=float(np.abs(G['x'] - S.x).max()), calib_lm_vs_scipy=float(np.abs(L['x'] - S.x).max()),
               calib_cost_rel_diff_vs_scipy=float(max(abs(G['cost'][-1] - S.cost), abs(L['cost'][-1] - S.cost)) / S.cost),
               calib_gn_iters=G['iters'], calib_lm_iters=L['iters'], calib_scipy_nfev=int(S.nfev), calib_scipy_residual_calls=len(sc_cost),
               calib_estimate=G['x'].tolist(), calib_error=err.tolist(), calib_predicted_sd=sd.tolist(),
               calib_max_err_in_sd=float(np.max(np.abs(err) / sd)),
               calib_rms_residual_m=float(np.sqrt(2 * S.cost / (3 * N_POSES))))
    # Monte Carlo: same poses, new noise; normalised estimation error squared e^T P^-1 e should follow chi^2 with 5 dof.
    nees = []
    for _ in range(200):
        fk = leg.problem(q, rng); r = gauss_newton(fk, P_NOMINAL); e = r['x'] - P_TRUE; J = fk(r['x'])[1]
        nees.append(float(e @ (J.T @ J) @ e / SIGMA_MOCAP**2))
    out['calib_mean_nees'] = float(np.mean(nees)); out['calib_nees_trials'] = len(nees)
    # Basin: initial joint offsets uniform in +-a (all three), lengths +-3 cm around the truth, 60 trials per a.
    # A start counts as 'reference' when it ends on scipy's parameters, 'mirrored' when it ends on the same cost with a negative
    # link length (an equivalent parameterisation, README), 'local' on a higher stationary cost, 'not converged' otherwise.
    mags = [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0]; NB = 60
    basin = dict(mag_rad=mags, trials=NB, gn=[], lm=[], gn_min_cost=[], lm_min_cost=[], gn_kinds={}, lm_kinds={})
    for a in mags:
        ok = dict(gn=0, lm=0); mc = dict(gn=0, lm=0)
        for _ in range(NB):
            p0 = P_TRUE + np.r_[rng.uniform(-a, a, 3), rng.uniform(-0.03, 0.03, 2)]
            for name, solver in (('gn', gauss_newton), ('lm', levenberg_marquardt)):
                r = solver(f, p0); same_cost = abs(r['cost'][-1] - S.cost) < 1e-9 * S.cost
                kind = ('reference' if calib_ok(r['x'], S.x) else 'mirrored' if same_cost and min(r['x'][3:]) < 0 else
                        'local' if r['status'] == 'grad' and np.isfinite(r['cost'][-1]) else 'not converged')
                ok[name] += kind == 'reference'; mc[name] += kind in ('reference', 'mirrored')
                basin[f'{name}_kinds'][kind] = basin[f'{name}_kinds'].get(kind, 0) + 1
        for name in ('gn', 'lm'):
            basin[name].append(ok[name] / NB); basin[f'{name}_min_cost'].append(mc[name] / NB)
    out.update(calib_basin=basin, calib_gn_success_at_0p5rad=basin['gn'][1], calib_lm_success_at_0p5rad=basin['lm'][1],
               calib_lm_not_worse_than_gn=float(all(b >= a - 1e-12 for a, b in zip(basin['gn'], basin['lm']))))
    # Excitation: poses within +-r of standing. Predicted sd at the truth against the RMS error over 30 noise draws.
    ranges = [0.02, 0.05, 0.1, 0.2, 0.5, 1.0]; exc = dict(half_range_rad=ranges, pred_sd=[], rms_err=[], cond=[])
    for hr in ranges:
        qe = leg.poses(np.random.default_rng(5), half_range=hr); errs = []
        for _ in range(30):
            fe = leg.problem(qe, rng); errs.append(levenberg_marquardt(fe, P_NOMINAL)['x'] - P_TRUE)
        sdp, J = predicted_sd(fe, P_TRUE)
        exc['pred_sd'].append(sdp.tolist()); exc['rms_err'].append(np.sqrt(np.mean(np.square(errs), 0)).tolist())
        exc['cond'].append(float(np.linalg.cond(J.T @ J)))
    ratio = np.array(exc['rms_err']) / np.array(exc['pred_sd'])
    out.update(calib_excitation=exc, calib_rms_over_predicted_sd_max=float(ratio.max()), calib_rms_over_predicted_sd_min=float(ratio.min()))
    for k, v in out.items():
        if not isinstance(v, (dict, list)):
            print(f'leg calibration {k:36s} {v}')
    return out, dict(leg=leg, q=q, f=f, G=G, L=L, S=S, sc_cost=sc_cost, nees=nees)


# ---- figures -----------------------------------------------------------------------------------------------------------------

def fig_perturbation_diagram():
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.9), gridspec_kw=dict(width_ratios=[1, 1.15]))
    ax = axs[0]; ax.axis('off'); ax.set_aspect('equal'); ax.set_xlim(-1.25, 1.35); ax.set_ylim(-0.25, 1.35)
    t = np.linspace(0.15, np.pi - 0.15, 200); ax.plot(np.cos(t), np.sin(t), color=fs.C['ink'], lw=1.4)
    ax.text(0, 0.45, 'group manifold\n(SO(3), SE(2))', color=fs.C['muted'], fontsize=8.5, ha='center')
    a0 = 1.2; X = np.array([np.cos(a0), np.sin(a0)]); tg = np.array([-np.sin(a0), np.cos(a0)])
    ax.plot(*np.c_[X - 0.55 * tg, X + 0.75 * tg], color=fs.C['plan'], lw=1.0, ls='--')
    ax.text(*(X + 0.78 * tg + [0.0, 0.05]), 'tangent space at $X$', color=fs.C['plan'], fontsize=8.5, ha='center')
    fs.arrow(ax, X, X + 0.6 * tg, None, color=fs.C['plan'], lw=1.6); ax.text(*(X + 0.32 * tg + [0.02, 0.06]), '$h$', color=fs.C['plan'])
    a1 = a0 + 0.6; Y = np.array([np.cos(a1), np.sin(a1)]); tt = np.linspace(a0, a1, 50)
    ax.plot(np.cos(tt) * 0.995, np.sin(tt) * 0.995, color=fs.C['est'], lw=2.4)
    ax.plot(*X, 'o', color=fs.C['ink']); ax.plot(*Y, 'o', color=fs.C['est'])
    ax.text(*(X + [0.05, -0.12]), '$X$', fontsize=10); ax.text(*(Y + [-0.42, -0.16]), r'$X \oplus h = X\,\mathrm{Exp}(h)$', color=fs.C['est'])
    ax.set_title('Right perturbation: steps live in the tangent space')
    ax = axs[1]; ax.axis('off'); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    fs.box(ax, (0.05, 0.78), 0.4, 0.14, 'residual $r(X)$,\nJacobian $J = \\partial r / \\partial h$', fontsize=8.5)
    fs.box(ax, (0.55, 0.78), 0.42, 0.14, 'solve $(J^\\top J + \\mu I)\\,h = -J^\\top r$\n(GN: $\\mu = 0$)', fontsize=8.5)
    fs.box(ax, (0.55, 0.45), 0.42, 0.14, 'trial $X\' = X\\,\\mathrm{Exp}(h)$\ngain ratio $\\varrho$', fontsize=8.5)
    fs.box(ax, (0.05, 0.12), 0.4, 0.14, 'accept: $X \\leftarrow X\'$\n$\\mu \\leftarrow \\mu\\max(1/3, 1-(2\\varrho-1)^3)$', fontsize=8.5, color=fs.C['good'])
    fs.box(ax, (0.55, 0.12), 0.42, 0.14, 'reject: keep $X$\n$\\mu \\leftarrow \\nu\\mu,\\ \\nu \\leftarrow 2\\nu$', fontsize=8.5, color=fs.C['bad'])
    fs.arrow(ax, (0.45, 0.85), (0.55, 0.85)); fs.arrow(ax, (0.76, 0.78), (0.76, 0.59))
    fs.arrow(ax, (0.62, 0.45), (0.3, 0.26), r'$\varrho > 0$', color=fs.C['good'], offset=(-18, 0))
    fs.arrow(ax, (0.82, 0.45), (0.82, 0.26), r'$\varrho \leq 0$', color=fs.C['bad'], offset=(18, 0))
    fs.arrow(ax, (0.12, 0.26), (0.12, 0.78), 'next iteration', color=fs.C['muted'], offset=(-6, 0), ha='right')
    fs.arrow(ax, (0.97, 0.19), (0.99, 0.19), None, color=fs.C['bad']); ax.annotate('', xy=(0.97, 0.52), xytext=(0.99, 0.19),
                                                                                  arrowprops=dict(arrowstyle='-|>', color=fs.C['bad'], lw=1.0, connectionstyle='arc3,rad=0.5'))
    ax.set_title('Levenberg-Marquardt iteration (Gauss-Newton: $\\mu = 0$, every step accepted)')
    fs.save(fig, FIG / 'perturbation_and_lm_diagram')


def fig_calibration_diagram(leg):
    q_true = np.array([0.0, 0.9, -1.6]); p = P_TRUE; q_enc = q_true - p[:3]
    def chain(q, l1, l2):
        knee = Ry(q[1]) @ np.array([0, 0, -l1]); foot = knee + Ry(q[1] + q[2]) @ np.array([leg.fx, 0, -l2])
        return np.array([0, 0]), knee[[0, 2]], foot[[0, 2]]
    fig, ax = fs.diagram(6.4, 4.4, (-0.36, 0.2), (-0.38, 0.06))
    h, k, f = chain(q_true, p[3], p[4]); hn, kn, fn = chain(q_enc, P_NOMINAL[3], P_NOMINAL[4])
    ax.plot(*np.c_[h, k, f], '-o', color=fs.C['gt'], lw=5, alpha=0.6, ms=6, solid_capstyle='round', label='true leg (MuJoCo)')
    ax.plot(*np.c_[hn, kn, fn], '--o', color=fs.C['est'], lw=1.5, ms=4, label='model at the initial guess')
    ax.plot(*f, 'o', ms=9, mfc='none', mec=fs.C['ink']); fs.label(ax, f, 'measured foot $y_i$ (motion capture, 1 mm noise)', xytext=(-30, -8), ha='right')
    fs.arrow(ax, fn, f, None, color=fs.C['bad'], lw=1.2); fs.label(ax, (fn + f) / 2, 'residual $r_i$', xytext=(18, 6), color=fs.C['bad'])
    ax.plot([0, 0], [0, -0.12], color=fs.C['muted'], lw=0.7, ls=':')
    ax.text(0.01, -0.05, r'$q_2 = \hat q_2 + \delta_2$', fontsize=8.5); ax.text(k[0] - 0.008, k[1] - 0.035, r'$q_3 = \hat q_3 + \delta_3$', fontsize=8.5, ha='right')
    ax.text(*((h + k) / 2 + [0.012, 0.0]), '$l_1$', fontsize=10); ax.text(*((k + f) / 2 + [0.012, 0.0]), '$l_2$', fontsize=10)
    fs.frame_axes(ax, (0.12, -0.08), length=0.05, names=('x', 'z'))
    ax.text(-0.35, 0.035, 'FL leg, sagittal plane (hip abduction $q_1$ and its offset $\\delta_1$ act out of the page)', fontsize=8, color=fs.C['muted'])
    ax.legend(loc='upper left', bbox_to_anchor=(0.0, 0.92), fontsize=8)
    ax.set_title('Kinematic calibration: encoder zero offsets $\\delta$ and link lengths $l_1, l_2$')
    fs.save(fig, FIG / 'calibration_diagram')


def fig_jacobian(J):
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    ax.loglog(J['eps'], J['err_id'], 'o-', color=fs.C['bad'], label=r'$\mathrm{Exp}(w+\epsilon d) \approx \mathrm{Exp}(w)\mathrm{Exp}(\epsilon d)$')
    ax.loglog(J['eps'], J['err_jr'], 'o-', color=fs.C['est'], label=r'$\mathrm{Exp}(w+\epsilon d) \approx \mathrm{Exp}(w)\mathrm{Exp}(\epsilon J_r d)$')
    ax.loglog(J['eps'], 0.5 * J['eps']**2, ':', color=fs.C['plan'], label=r'slope 2 reference')
    ax.set(xlabel=r'perturbation size $\epsilon$ [rad]', ylabel='first-order error [rad]',
           title=f'Right Jacobian at |w| = {np.linalg.norm(J["w"]):.2f} rad: error drops as $\\epsilon^2$ instead of $\\epsilon$')
    ax.legend(); fs.save(fig, FIG / 'so3_jacobian_first_order')


def fig_registration(R):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 4.0), gridspec_kw=dict(width_ratios=[1, 1.1]))
    ax = axs[0]; ax.set_aspect('equal')
    x0 = R['G']['xs'][0]
    ax.plot(*R['Q'].T, 'o', color=fs.C['gt'], ms=5, label='map points $q_i$')
    ax.plot(*se2_apply(x0, R['P']).T, '.', color=fs.C['plan'], ms=4, label='scan at the initial guess (170°)')
    ax.plot(*se2_apply(R['G']['x'], R['P']).T, '.', color=fs.C['est'], ms=4, label='scan after Gauss-Newton')
    ax.set(xlabel='x [m]', ylabel='y [m]', title=f'SE(2) registration, {len(R["P"])} points, σ = 1 cm'); ax.legend(loc='upper left', fontsize=7.5)
    ax = axs[1]; Fs = R['S'].cost
    for key, col, lab in (('G', fs.C['est'], 'Gauss-Newton (per iteration)'), ('L', fs.C['est2'], 'Levenberg-Marquardt (per accepted step)')):
        ax.semilogy(np.maximum(np.array(R[key]['cost']) - Fs, 1e-17), 'o-', color=col, label=lab)
    ax.semilogy(np.maximum(np.array(R['sc_cost']) - Fs, 1e-17), 'x--', color=fs.C['plan'],
                label=f"scipy least_squares 'lm' (per residual call: {len(R['sc_cost'])} calls, nfev = {R['S'].nfev})")
    ax.set(xlabel='iteration / residual call', ylabel=r'$F - F^\ast$ [m$^2$]', title='Convergence from a 170° heading error'); ax.legend(fontsize=7.5)
    fs.save(fig, FIG / 'se2_registration')
    fig, ax = plt.subplots(figsize=(6.4, 3.2)); b = R['basin']
    ax.plot(b['theta0_deg'], b['gn_iters'], 'o-', color=fs.C['est'], ms=3, label='Gauss-Newton')
    ax.plot(b['theta0_deg'], b['lm_iters'], 's-', color=fs.C['est2'], ms=3, label='Levenberg-Marquardt')
    i = int(np.argmax(b['gn_iters'])); th_w = b['theta0_deg'][i]
    fs.label(ax, (th_w, b['gn_iters'][i]), f'start {th_w:.0f}° = truth − 180°:\nthe cost has its maximum here', xytext=(30, -8), fontsize=8)
    ax.set(xlabel='initial heading [deg] (truth 35°)', ylabel='iterations to converge',
           title=f'Every initial heading converges: GN {100 * np.mean(b["gn_ok"]):.0f} %, LM {100 * np.mean(b["lm_ok"]):.0f} % of {len(b["gn_ok"])} starts')
    ax.legend(); fs.save(fig, FIG / 'se2_registration_basin')
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 4.0), sharey=True)
    for ax, key, title in ((axs[0], 'ls', 'plain least squares'), (axs[1], 'ca', 'Cauchy kernel, IRLS (c = 5 cm)')):
        x = R[key]['x']; e = np.rad2deg(abs(wrap(x[2] - T_TRUE[2]))); et = np.linalg.norm(x[:2] - T_TRUE[:2])
        ax.set_aspect('equal'); ax.plot(*R['Qo'][~R['bad']].T, 'o', color=fs.C['gt'], ms=5, label='map points (correct)')
        ax.plot(*R['Qo'][R['bad']].T, 'x', color=fs.C['bad'], ms=5, label='wrong correspondences (20 %)')
        ax.plot(*se2_apply(x, R['Po']).T, '.', color=fs.C['est'] if key == 'ca' else fs.C['bad'], ms=4, label='aligned scan')
        ax.set(xlabel='x [m]', title=f'{title}: {e:.2f}° / {100 * et:.1f} cm error')
    axs[0].set_ylabel('y [m]'); axs[0].legend(loc='upper left', fontsize=7.5); fs.save(fig, FIG / 'se2_registration_outliers')


# Calibration parameters are all estimates, so none gets the pass, ground-truth or plan colour (MODULE_SPEC): joint offsets in
# C['est'], link lengths in neutral C['ink'], each told apart by marker and line style. (colour, marker, line style)
PSTYLE = [(fs.C['est'], 'o', '-'), (fs.C['est'], 's', '--'), (fs.C['est'], '^', ':'), (fs.C['ink'], 'D', '-'), (fs.C['ink'], 'v', '--')]


def fig_calibration(C, res):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.8)); Fs = C['S'].cost; ax = axs[0]
    for key, col, lab in (('G', fs.C['est'], 'Gauss-Newton'), ('L', fs.C['est2'], 'Levenberg-Marquardt')):
        ax.semilogy(np.maximum(np.array(C[key]['cost']) - Fs, 1e-18), 'o-', color=col, label=lab)
    ax.semilogy(np.maximum(np.array(C['sc_cost']) - Fs, 1e-18), 'x--', color=fs.C['plan'],
                label=f"scipy 'lm' (per residual call: {len(C['sc_cost'])} calls, nfev = {C['S'].nfev})")
    ax.set(xlabel='iteration / residual call', ylabel=r'$F - F^\ast$ [m$^2$]', title='FL leg calibration, 40 poses, 1 mm noise'); ax.legend(fontsize=7.5)
    ax = axs[1]; xs = np.array(C['G']['xs']); sd = np.array(res['calib_predicted_sd'])
    for i, lab in enumerate([r'$\delta_1$', r'$\delta_2$', r'$\delta_3$', r'$l_1$', r'$l_2$']):
        ax.semilogy(np.abs(xs[:, i] - P_TRUE[i]) / sd[i], PSTYLE[i][1] + PSTYLE[i][2], color=PSTYLE[i][0], label=lab, mfc='none')
    for y in (1, 3):
        ax.axhline(y, color=fs.C['muted'], ls=':', lw=0.8); ax.text(len(xs) - 1, y * 1.12, f'{y}σ', fontsize=7.5, color=fs.C['muted'], ha='right')
    ax.set(xlabel='Gauss-Newton iteration', ylabel='|error vs MJCF truth| / predicted σ',
           title='Errors reach the noise level after 2 iterations'); ax.legend(fontsize=7.5, ncol=3)
    fs.save(fig, FIG / 'calibration_convergence')
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.7)); b = res['calib_basin']; ax = axs[0]
    for name, col, mk, lab in (('gn', fs.C['est'], 'o', 'Gauss-Newton'), ('lm', fs.C['est2'], 's', 'Levenberg-Marquardt')):
        ax.plot(b['mag_rad'], 100 * np.array(b[name]), mk + '-', color=col, label=f'{lab}: reference solution')
        ax.plot(b['mag_rad'], 100 * np.array(b[f'{name}_min_cost']), mk + ':', color=col, mfc='none', label=f'{lab}: any global minimum')
    ax.set(xlabel='initial joint-offset error, uniform in ±a [rad]', ylabel='starts [%]', ylim=(0, 105),
           title=f'Basin of convergence ({b["trials"]} starts per a)'); ax.legend(fontsize=7, loc='lower left')
    e = res['calib_excitation']; hr = np.array(e['half_range_rad']); ps = np.array(e['pred_sd']); rm = np.array(e['rms_err'])
    ax = axs[1]
    for i, lab in ((0, r'$\delta_1$'), (1, r'$\delta_2$'), (2, r'$\delta_3$')):
        col, mk, ls = PSTYLE[i]
        ax.loglog(hr, np.rad2deg(ps[:, i]), ls, color=col, label=f'{lab} predicted σ'); ax.loglog(hr, np.rad2deg(rm[:, i]), mk, color=col, mfc='none', label=f'{lab} RMS, 30 runs')
    ax.set(xlabel='pose spread, ±r around standing [rad]', ylabel='joint offset error [deg]', title='Offsets vs pose spread'); ax.legend(fontsize=7, ncol=2)
    ax = axs[2]
    for i, lab in ((3, r'$l_1$'), (4, r'$l_2$')):
        col, mk, ls = PSTYLE[i]
        ax.loglog(hr, 1000 * ps[:, i], ls, color=col, label=f'{lab} predicted σ'); ax.loglog(hr, 1000 * rm[:, i], mk, color=col, mfc='none', label=f'{lab} RMS, 30 runs')
    ax.set(xlabel='pose spread, ±r around standing [rad]', ylabel='link length error [mm]', title='Lengths vs pose spread'); ax.legend(fontsize=7)
    fs.save(fig, FIG / 'calibration_basin_and_excitation')


# ---- videos ------------------------------------------------------------------------------------------------------------------

def video_registration(R):
    """Matplotlib animation of the Gauss-Newton iterates from a 170 deg heading error, each step drawn along the group geodesic
    X_k Exp(s h_k), s in [0, 1]. Encoded by ffmpeg (H.264, yuv420p, 960x540)."""
    from matplotlib.animation import FFMpegWriter
    xs = R['G']['xs']; P, Q = R['P'], R['Q']; Fs = R['S'].cost
    frames = []
    for k in range(len(xs) - 1):
        h = se2_log(se2_compose(se2_inverse(xs[k]), xs[k + 1]))
        frames += [(k, s, se2_plus(xs[k], s * h)) for s in np.linspace(0, 1, 16)] + [(k + 1, 1.0, xs[k + 1])] * 6
    frames = [(0, 0.0, xs[0])] * 15 + frames + [(len(xs) - 1, 1.0, xs[-1])] * 25
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 5.4), dpi=100, gridspec_kw=dict(width_ratios=[1.2, 1]))
    ax = axs[0]; ax.set_aspect('equal'); ax.set_xlim(-3, 3); ax.set_ylim(-3, 3)
    ax.plot(*Q.T, 'o', color=fs.C['gt'], ms=5, label='map points'); sc, = ax.plot([], [], '.', color=fs.C['est'], ms=6, label='scan at $X$')
    ttl = ax.set_title(''); ax.set(xlabel='x [m]', ylabel='y [m]'); ax.legend(loc='upper left', fontsize=8)
    ax2 = axs[1]; c = np.maximum(np.array(R['G']['cost']) - Fs, 1e-17); ax2.semilogy(c, 'o-', color=fs.C['est'])
    mk, = ax2.semilogy([0], [c[0]], 'o', ms=12, mfc='none', mec=fs.C['bad'], mew=2)
    ax2.set(xlabel='Gauss-Newton iteration', ylabel=r'$F - F^\ast$ [m$^2$]', title='cost')
    w = FFMpegWriter(fps=20, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '23'])
    path = MEDIA / 'se2_gauss_newton.mp4'; MEDIA.mkdir(parents=True, exist_ok=True)
    with w.saving(fig, str(path), dpi=100):
        for k, s, X in frames:
            sc.set_data(*se2_apply(X, P).T); mk.set_data([k], [c[k]])
            ttl.set_text(f'iteration {k}: x = {X[0]:+.3f} m, y = {X[1]:+.3f} m, θ = {np.rad2deg(X[2]):+.1f}°'); w.grab_frame()
    fig.savefig(MEDIA / 'se2_gauss_newton.png', dpi=100); plt.close(fig)
    return path


def video_calibration(m, d, C):
    """The FL leg moving through 10 of the 40 calibration poses (base held 0.45 m above the floor, kinematic playback), 30 fps."""
    leg = C['leg']; poses = C['q'][:10]; qs = [sim.Q_STAND[:3]] + list(poses) + [sim.Q_STAND[:3]]
    path = MEDIA / 'calibration_poses.mp4'; dt = 1 / 30
    with sim.Recorder(m, path, width=960, height=540, fps=30, distance=1.15, azimuth=225, elevation=-12) as rec:
        if not rec.ok:
            return None
        t = 0.0; ix = sim.index(m)
        for j, (a, b) in enumerate(zip(qs[:-1], qs[1:])):
            traj = sim.min_jerk(np.asarray(a), np.asarray(b), 0.6)
            for s in np.arange(0, 0.9, dt):
                q = np.array(sim.Q_STAND); q[:3] = traj(s); d.qpos[:] = 0; d.qpos[2] = 0.45; d.qpos[3] = 1; d.qpos[ix.qpos] = q
                d.time = t; mujoco.mj_forward(m, d); rec(m, d); t += dt
            if j == 1:
                rec.still(d, MEDIA / 'calibration_poses.png')                  # held at the second calibration pose
        rec.close()
    return path


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=100000); ap.add_argument('--n-jac', type=int, default=20000)
    ap.add_argument('--no-media', action='store_true'); ap.add_argument('--json', default=str(HERE / 'results.json')); a = ap.parse_args()
    t0 = time.time(); res = fundamentals(a.n)
    jac, Jd = jacobians(a.n_jac); reg, Rd = registration()
    m, d = sim.load(); cal, Cd = calibration(m, d)
    res['advanced'] = dict(so3_jacobians=jac, se2_registration=reg, leg_calibration=cal)
    fig_perturbation_diagram(); fig_calibration_diagram(Cd['leg']); fig_jacobian(Jd); fig_registration(Rd); fig_calibration(Cd, cal)
    flat = {**jac, **reg, **cal}
    if not a.no_media:
        vids = [video_registration(Rd), video_calibration(m, d, Cd)]
        sizes = {p.name: p.stat().st_size for p in vids if p is not None and p.exists()}
        res['media_bytes'] = sizes; flat['media_mp4_under_6MB'] = float(bool(sizes) and all(s < 6e6 for s in sizes.values()))
    res['checks'] = {k: dict(measured=flat[k], op=op, threshold=thr, passed=check(k, flat[k])) for k, (op, thr) in ACCEPT.items() if k in flat}
    print('\nacceptance:')
    for k, c in res['checks'].items():
        print(f'  {"PASS" if c["passed"] else "MISS"}  {k:50s} {c["measured"]:.3g}  ({c["op"]} {c["threshold"]})')
    res['wall_time_s'] = round(time.time() - t0, 1)
    pathlib.Path(a.json).write_text(json.dumps(res, indent=2) + '\n')
    print(f'wrote {a.json} in {res["wall_time_s"]} s')


if __name__ == '__main__':
    main()
